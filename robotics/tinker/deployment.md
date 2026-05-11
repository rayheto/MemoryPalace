# Tinker Sim2Real 部署管线

从训练好的 PyTorch 模型到 Odroid C4 机载计算机，再到 STM32 底层电机控制的完整链路。

---

## 硬件架构总览

```
                    ┌──────────────────────────┐
                    │      Odroid C4 (Linux)    │
                    │   ARM Cortex-A55, 4GB RAM │
                    │   PREEMPT_RT 实时内核      │
                    │   IP: 192.168.1.113       │
                    ├──────────────────────────┤
                    │ navigation_task  (UDP)    │ ←─ OCU/ROS 外部指令
                    │ control_task     (500Hz)  │ ←─ TVM RL 推理 + VMC
                    │      ↕ SPI 共享内存       │
                    └──────────────────────────┘
                              │ SPI (2kHz)
                    ┌──────────────────────────┐
                    │   STM32F4 协处理器         │
                    │   hardware_task           │
                    ├──────────────────────────┤
                    │ CAN1/CAN2 (2Mbps)         │ ←─ 14 无刷电机 + 编码器
                    │ SBUS                      │ ←─ 遥控接收机
                    │ MEMS IMU (板载)            │
                    │ 电池电压监测                │
                    └──────────────────────────┘
```

**关键澄清**: 电机通信协议是 **CAN bus (MIT Mini Cheetah 协议)**，NOT EtherCAT。EtherCAT 注释出现在 `ethercat.sh` 中，属于另一个硬件版本/其他机器人的配置，Tinker 当前实际使用的是 CAN + STM32 架构。

---

## Odroid C4 系统配置

### 硬件规格

| 参数 | 规格 |
|---|---|
| SoC | Amlogic S905X3 |
| CPU | 4× Cortex-A55 @ 2.016GHz, ARMv8-A, Neon + Crypto |
| GPU | Mali-G31 MP2 @ 650MHz, OpenGL ES 3.0 / Vulkan 1.0 |
| 内存 | 4GB DDR4-2640 |
| 接口 | 40Pin GPIO, HDMI 4K/60Hz, SPI, UART, USB |
| 支持系统 | Ubuntu 20.04 / Android / CoreELEC |
| 性能对比 | Pi 3 的 2-10 倍；弱于 Odroid N2 (S922X)；强于 Raspberry Pi 4 |

### RT 实时补丁安装

Odroid C4 必须安装 PREEMPT_RT 实时补丁，保证与 STM32 载板的 SPI 实时通讯。RT 补丁需要 **5.10 内核版本**（RT 补丁不支持 5.11+）。

**安装步骤:**

1. **下载预构建 RT 镜像** (5.10 内核 + RT 补丁):
   ```
   下载地址: http://pdl.atnet.at/images/raw/arm64/focal/
   选择 1218 版本（5.10 内核）
   ```
   官方论坛参考: `forum.odroid.com/viewtopic.php?t=38133`

2. **烧录到 SD 卡**，插入 Odroid C4 启动

3. **验证 RT 内核:**
   ```bash
   uname -r                    # 应显示 5.10.x-rt*
   cat /sys/kernel/realtime    # 应返回 1
   ```

4. **配置 SSH 免密登录:**
   ```bash
   ssh-copy-id -i ~/.ssh/id_rsa.pub odroid@192.168.1.113
   ```

### 部署到 Odroid

```bash
cd /home/ubuntu/cll/git_tinker/OmniBotSeries-Tinker-main/OmniBotHub/Linux/

# 部署编译好的 task 二进制
./download_tinker.sh
# 实际执行: scp ./build/*task* odroid@192.168.1.113:/home/odroid/Tinker

# 部署 YAML 参数文件
./download_param_tinker.sh
# 实际执行: scp ./control_task2/Param_Tinker14_Zero/param_* odroid@192.168.1.113:/home/odroid/Tinker/Param
```

Odroid 上电后自动启动 (`/etc/rc.local`):
```bash
sleep 10
cd /home/odroid/Tinker

sudo ./hardware_task_tinker &     # 1. STM32 固件 (CAN电机通信 + IMU + 遥控)
sleep 1
sudo ./control_task_tinker &      # 2. 实时控制 (VMC/RL推理 + 状态估计)
sleep 1
sudo ./navigation_task_tinker &   # 3. 通信桥接 (OCU/ROS UDP 接口)
```

---

## 三个 Task 详细职责

### 1. hardware_task — STM32F4 底层固件 (运行于 MCU)

| 功能 | 说明 |
|---|---|
| **电机通信** | CAN1 + CAN2 (2Mbps), MIT Mini Cheetah 协议, 14 个无刷电机 |
| **编码器读取** | 14 个磁编码器 (通过 CAN 回传) |
| **IMU 读取** | 板载 MEMS IMU (加速度计 + 陀螺仪 + 磁力计) |
| **遥控接收** | SBUS 协议 (遥控发射机) + ESP32 无线控制器 |
| **电池监测** | 电池电压 AD 采样 |
| **与 Linux 通信** | SPI 全双工共享内存 (key=`MEM_SPI=0x0001`, 2048 字节缓冲) |

### 2. control_task — 实时运动控制 (运行于 Linux, 500Hz)

| 功能 | 说明 |
|---|---|
| **RL 推理** | TVM 加载 `policy_arm64_cpu.so`，推理频率 ~35.7Hz |
| **VMC 力控制** | 虚拟模型控制 (Virtual Model Control)，位置-力混合控制 |
| **状态估计** | 身体姿态、地面平面、CoM 位置/速度、GRF 估计、负载估计 |
| **足端检测** | 通过估计 GRF 判断触地/离地 |
| **QP 力分配** | qpOASES 求解器，最优 GRF 分配 (摩擦锥约束) |
| **步态状态机** | IDLE / TROT / STAND_RC / RECOVER / G_RL / G_KICK / SOFT |
| **正逆运动学** | 5 连杆或 3-DOF 串行链，雅可比矩阵计算 |
| **UDP 服务** | RL 推理 (port 10000), 遥测 (port 7070) |

### 3. navigation_task — 通信桥接 (运行于 Linux)

| 功能 | 说明 |
|---|---|
| **OCU 接口** | UDP port 8889, 接收地面站遥控指令 |
| **ROS/SDK 接口** | UDP port 8888, 接收高层指令 (速度/姿态/步态切换) |
| **ARM 控制接口** | UDP port 9000, 手臂舵机控制 |
| **数据记录** | 日志写入 `/home/odroid/Tinker/Data/` (姿态/CoM/GRF/关节角/力矩) |
| **参数转发** | IMU 标定参数、OCU 参数转发到 control_task |

---

## 任务间通信协议

```
OCU (地面站) ──UDP:8889──→ navigation_task
ROS/SDK ──────UDP:8888──→ navigation_task
                             │ 共享内存 (MEM_CONTROL=4999, 2048B)
                             ▼
ESP32 ────────UDP:7070──→ control_task
                             │ SPI 共享内存 (MEM_SPI=0x0001, 2048B)
                             ▼
                       hardware_task (STM32)
                             │ CAN1/CAN2 (2Mbps)
                             ▼
                       14× 无刷电机驱动器
```

### 共享内存协议

- **读写仲裁**: `int flag` = 0 表示 control_task 可写，= 1 表示可读
- **数据序列化**: `setDataFloat_mem()` / `floatFromData_spi()` 手动编解码 4 字节浮点数
- **SPI 数据内容**: IMU 姿态/角速度、14 电机位置/速度/力矩、14 舵机位置、遥控状态、电池电压
- **控制数据内容**: 电机指令/反馈、CoM 状态、地面姿态、步态状态、遥控指令

---

## 控制循环频率

| 线程 | 周期 | 频率 | 职责 |
|---|---|---|---|
| Thread_T1 (主控制) | 2000μs | **500Hz** | 步态 + 力控制 + 状态估计 |
| Thread_Mem_Servo (SPI) | 500μs | **2000Hz** | STM32 共享内存 I/O |
| Thread_T5 (遥控) | 5000μs | **200Hz** | 遥控指令处理 |
| Thread_UDP_RL_Tinker | 2000μs | **500Hz** | RL 策略 UDP 交换 |
| Thread_Mem_Navigation | 10000μs | **100Hz** | 导航共享内存 |
| Thread_UDP_OCU (导航) | 5000μs | **200Hz** | OCU 遥控 UDP |
| Thread_UDP_SDK | 5000μs | **200Hz** | ROS SDK 指令 |
| Thread_Record (日志) | 5000μs | **200Hz** | 数据记录 |

> 实时机制: `SCHED_FIFO` + `pthread_setschedparam`，设置了 CPU affinity (CPU3)。循环超时 > 5ms 打印警告。

---

## 电机通信协议 (CAN)

### MIT Mini Cheetah 协议

```
CAN 帧 (标准):
  ID: 0x001-0x00E (14 个电机)
  Data[0-1]: 位置 (uint16)
  Data[2-3]: 速度 (uint16)
  Data[4-5]: KP (uint16)
  Data[6-7]: KD (uint16)
  Data[8-9]: 力矩 (uint16)
```

### 缩放常量

| 参数 | 范围 | 说明 |
|---|---|---|
| P_MIN / P_MAX | ±180° | 位置限幅 |
| V_MIN / V_MAX | ±2000 | 速度限幅 |
| T_MIN / T_MAX | ±10.0 Nm | 力矩限幅 |
| KP_MIN / KP_MAX | 0.0 ~ 10.0 | 刚度范围 |
| KD_MIN / KD_MAX | 0.0 ~ 10.0 | 阻尼范围 |

电机工作在**力矩模式** (`MOTOR_MODE_T`)，control_task 计算前馈力矩 + KP/KD/刚度后经 SPI→STM32→CAN 下发。

---

## 实机 RL 推理流程

```
Odroid C4 运行循环 (dt = 0.028s, RL推理 ~35.7Hz):
  1. hardware_task (STM32): CAN 读取 14 电机位置/速度 + IMU
  2. SPI → control_task:
     a. 构造观测向量 (本体感觉 + 指令 + 历史帧)
     b. TVM 推理 (policy_arm64_cpu.so)
     c. 输出动作 = action_scale * net_output + def_act
     d. PD 控制: τ = kp*(q_target - q) - kd*q_dot
     e. 阻抗外环 (Virtual Model Control, 由 imp_param 控制)
     f. QP 力分配 (qpOASES, 摩擦锥约束)
  3. SPI → STM32 → CAN → 力矩指令写入电机
```

### RL 步态参数

```yaml
rl_gait:
  net_run_dt: 0.028      # 策略推理周期 (~35.7 Hz)
  action_scale: 0.25     # 动作缩放因子
  def_act0:  0            # 左 Hip Yaw
  def_act1: -0.05         # 左 Hip Roll
  def_act2:  0.56         # 左 Hip Pitch
  def_act3: -1.12         # 左 Knee
  def_act4:  0.57         # 左 Ankle
  # def_act7-13: 右侧镜像
```

---

## 控制参数速查

### 阻抗控制 (imp_param)

| 参数 | 值 | 说明 |
|---|---|---|
| kp_q00-kp_q04 | 10.0 | 位置刚度 |
| kd_q00-kd_q03 | 0.5 | 速度阻尼 |
| kd_q04 | 0.2 | 踝关节阻尼 (较小) |
| stiff_q00-q04 | 1.4 | 刚度乘子 |
| imp_x_kp | 0.01 | X 方向阻抗 |
| imp_z_kp | 0.03 | Z 方向(高度)阻抗 |

### VMC 站立参数 (备用)

| 参数 | 值 | 说明 |
|---|---|---|
| kp_pit / kp_rol / kp_yaw | 40 / 30 / 35 | 姿态 PID |
| kp_pos_z | 2500 | 高度刚度 (非常高) |
| ground_mu | 0.5 | 摩擦系数估计 |

### 运动学/动力学参数

| 参数 | 值 |
|---|---|
| L1 (大腿) | 0.12 m |
| L2 (小腿) | 0.14 m |
| L3 (踝) | 0.075 m |
| 总质量 | 12.0 kg |
| 站立高度 | 0.306 m |
| 惯量 Ix/Iy/Iz | 0.04 / 0.075 / 0.06 |
| 最大速度 X | 0.32 m/s |
| 最大速度 Y | 0.18 m/s |
| 最大旋转 | 65 deg/s |
| 支撑相/摆动相 | 0.45s / 0.45s |

---

## 传感器

| 传感器 | 接口 | 说明 |
|---|---|---|
| MEMS IMU | STM32 板载 | 3 轴加速度 + 3 轴陀螺 + 3 轴磁力计 |
| USB IMU | USB (备用) | `imu_usb_enable: 0` 默认禁用 |
| 电机编码器 | CAN 回传 | 14 磁编码器 |
| 遥控接收机 | SBUS | 无线遥控 + ESP32 控制器 |
| 电池电压 | STM32 ADC | 电压监测 |

### IMU 标定参数

```yaml
# param_hardware.yaml
imu_set_att: [0, 0, 0]                              # 安装姿态
gyro_bias: [0.0823, -0.4672, 0.0492]                # 陀螺仪零偏
```

---

## 模型导出链

```
IsaacGym 训练 (PyTorch .pt)
    │
    ├─→ torch.jit.script() → model_jitt.pt (TorchScript)
    │       │
    │       ├─→ pt2onnx.py → ONNX (.onnx)
    │       │
    │       └─→ pt2tvm.py → TVM 编译
    │               ├─→ policy_x64_cpu.so  (x86 PC 测试)
    │               └─→ policy_arm64_cpu.so (Odroid C4 部署)
    │
    └─→ sim2sim_tinker.py (MuJoCo 验证)
            │
            └─→ sim2sim_tinker_udp.py → UDP → 实机通信协议测试
```

### 模型导出命令

```bash
# TorchScript
python train.py  # 自动保存 model_jitt.pt

# ONNX
python pt2onnx.py

# TVM (Odroid C4 部署)
python pt2tvm.py
# 生成 policy_arm64_cpu.so → 复制到 Odroid C4
```

---

## 外部接口总览

| 接口 | 端口 | 协议 | 方向 | 用途 |
|---|---|---|---|---|
| OCU 地面站 | UDP 8889 | 自定义 | PC→nav | 遥控/监控 |
| ROS/SDK | UDP 8888 | 自定义 | ROS→nav | 高层指令 |
| ARM 控制 | UDP 9000 | 自定义 | →nav | 手臂舵机 |
| ESP32 无线 | UDP 7070 | 自定义 | ESP32→ctrl | 无线遥控 |
| RL 推理 | UDP 10000 | 自定义 | ↔ctrl | RL 策略交换 |
| 调试面板 | UDP 8001 | 自定义 | PC→ctrl | py_ui.py |

---

## TVM 工具链

TVM (Apache TVM) 将 PyTorch 模型编译为目标平台的优化机器码:
- 输入: TorchScript 模型
- 输出: `.so` 动态库 (C++ 通过 `tvm2.h` 直接调用)
- 优势: 无需 PyTorch 运行时，ARM Cortex-A55 上 autotune 优化
- 脚本: `pt2tvm.py` (在 LocomotionWithNP3O 和 OmniBotCtrl 中都有)

---

## 库与依赖

| 库 | 用途 |
|---|---|
| qpOASES | QP 求解器 (最优 GRF 分配，摩擦锥约束) |
| Eigen3 | 矩阵运算 (雅可比/旋转矩阵/运动学) |
| yaml-cpp | 配置文件解析 (`param_robot.yaml`, `param_gait.yaml`) |
| TVM Runtime | 离线 NN 推理 |
| pthreads | 多线程 + 实时优先级 (SCHED_FIFO) |
| Linux shm | 进程间共享内存 (sys/shm.h) |
| Linux SPI | SPI 底层接口 (linux/spi/spidev.h) |
| POSIX sockets | UDP 通信 |

---

## Sim2Real 注意事项

1. **观测一致性**: sim2real 必须确保观测构建方式与训练完全一致 (关节角度范围、归一化因子、历史帧数)
2. **PD 增益匹配**: 仿真中的 PD 增益必须与实机一致
3. **通信延迟**: 训练时用 action lag 域随机化模拟实机延迟，实机实际延迟应在此范围内
4. **IMU 零偏**: `param_hardware.yaml` 中有预标定的陀螺仪零偏值，首次使用需重新标定
5. **CAN 同步**: 14 电机通过 CAN 总线同步，由 STM32 的 2000Hz SPI 轮询驱动
6. **RT 超时检测**: 控制循环超过 5ms 会打印警告，表示实时性可能不足
