# control_task 源码分析

实时运动控制器，运行于 Odroid C4 Linux (PREEMPT_RT)。

**路径**: `OmniBotHub/Linux/control_task2/`

---

## 线程架构

`main()` 创建 5 个 pthread 线程 (加上 navigation_task 的 SPI/Nav 线程共 7 个):

| 线程 | 函数 | 周期 | 频率 | 职责 |
|---|---|---|---|---|
| Thread_T1 | 主控制 | `usleep(2000)` | **500Hz** | 传感器→状态估计→步态→力控制 |
| Thread_T5 | 遥控处理 | `usleep(5000)` | **200Hz** | RC/SDK 指令 → 速度/姿态目标 |
| Thread_Mem_Servo | SPI I/O | `usleep(500)` | **2000Hz** | STM32 共享内存双向交换 |
| Thread_Mem_Navigation | Nav 共享内存 | `usleep(10000)` | **100Hz** | 与 navigation_task 数据交换 |
| Thread_UDP_RL_Tinker | RL UDP | `usleep(2000)` | **500Hz** | RL 策略观测/动作交换 (port 10000) |
| Thread_UDP_OCU | ESP32 遥测 | `usleep(10000)` | **100Hz** | ESP32 OCU 无线遥控 (port 7070) |
| Thread_UDP_MAP | MAP 服务 | `usleep(50000)` | **20Hz** | 地图/里程计 UDP (port 6666) |

实时机制: `SCHED_FIFO` + `pthread_setschedparam`，优先级设为 (max+min)/2。Thread_T1 循环超时 > 5ms 打印警告。

---

## 主控制循环 (Thread_T1, 500Hz)

```cpp
while(1) {
    T = Get_Cycle_T(31);                          // 测量循环时间
    subscribe_imu_to_webot(&robotwb, T);           // IMU 姿态更新
    readAllMotorPos(&robotwb, leg_dt[1]);          // 读取 14 电机角度
    locomotion_sfm(leg_dt[1]);                     // 步态状态机
    if(vmc_all.gait_mode == G_RL)
        force_control_and_dis_rl(leg_dt[1]);       // RL 力控制 (TROT)
    else
        force_control_and_dis_stand(leg_dt[1]);    // 站立阻抗控制
    if(T > 0.005) printf("OT-S::sys_dt=%f\n", T); // 超时警告
    usleep(2000);
}
```

---

## 步态状态机 (`locomotion_sfm.cpp`)

全局变量 `gait_ww` (类型 `Gait_Mode`) 和 `vmc_all.gait_mode` 管理状态:

### 状态枚举

| 状态 | 值 | 说明 |
|---|---|---|
| IDLE | 0 | 空闲, 无输出 |
| TROT | 1 | 对角小跑步态 |
| STAND_RC | 2 | 遥控站立 |
| RECOVER | 3 | 倒地恢复 |
| G_RL | 4 | RL 策略控制 |
| G_KICK | 5 | 踢腿动作 |
| SOFT | 6 | 柔性/软模式 |
| STAND | 7 | 纯站立 |

### 状态转换

- `IDLE` → `STAND_RC`: 遥控使能 (`ocu.key_x`)
- `STAND_RC` → `TROT`: 遥控走步 (`ocu.key_a`)
- `STAND_RC` → `G_RL`: RL 激活 (`Gait_RL_Active()`)
- `RECOVER` → `STAND_RC`: 恢复完成 (自起立)
- `G_RL` → `STAND_RC`: RL 停止/故障

### 参数加载 (`vmc_param_init()`)

从 `param_robot.yaml` 读取:
- 连杆长度: L1(大腿)=0.12m, L2(小腿)=0.14m, L3(踝)=0.075m
- 身体尺寸: H=0.306m, W=0.09m
- 质量: 12kg, 惯量 Ix=0.04, Iy=0.075, Iz=0.06
- 运动学边界计算 (MIN_Z, MAX_Z, MIN_X, MAX_X)

---

## RL 推理 (`gait_src/rl.cpp`)

### 两种模式

1. **UDP 模式** (`Thread_UDP_RL_Tinker`, main.cpp): 观测/动作通过 UDP port 10000 与外部 Python 进程交换 — 用于 sim2sim 桥接和远端推理
2. **TVM 本地推理模式** (`Gait_RL_Update`, rl.cpp): TVM 加载 `.so` 模型在 Odroid C4 本地推理

### TVM 推理流程

```cpp
// 模型加载
Gait_RL_Active(rst):
  switch(rst):
    1 → tvm.init("Model/Trot/policy_arm64_cpu.so", 39, 390, 10)   // trot
    2 → tvm.init("Model/Stand/policy_arm64_cpu.so", 39, 390, 10)  // stand
    3-6 → dance/jump models

// 推理循环 (≈62.5Hz, net_run_dt=0.016)
Gait_RL_Update(dt):
  // 1. 构造 39D 观测向量 obs1:
  //    [0-2]:   omega (角速度 × 0.25)
  //    [3-5]:   euler angles (× 1.0)
  //    [6-8]:   cmd (vx×2.0, vy×2.0, vyaw×0.25)
  //    [9-18]:  (q - default_q) × 1.0   (10 关节位置误差)
  //    [19-28]: dq × 0.05                 (10 关节速度)
  //    [29-38]: last_action               (10 上一动作)
  //
  // 2. 10帧历史滑窗: obs10.tail<39>() = obs1
  // 3. tvm.in1 = obs1, tvm.in2 = obs10
  // 4. tvm.run() → tvm.out (10D action)
  
  // 动作后处理:
  action = actionOld * 0.2 + tvm.out * 0.8   // 低通滤波
  action = CLAMP(action, -5, 5)
  q_target = action × action_scale + default_action  // 缩放+偏移
```

### UDP RL 协议 (sim2sim 桥接)

**Request** (control_task → Python, 每次策略步):

| 字段 | 类型 | 维度 | 说明 |
|---|---|---|---|
| trigger | float32 | 1 | 1=新策略步, 0=重复 |
| command | float32 | 4 | vx, vy, vyaw, reserved |
| eu_ang | float32 | 3 | roll, pitch, yaw (rad) |
| omega | float32 | 3 | 角速度 (rad/s) |
| q | float32 | 10 | 关节位置 (rad) |
| dq | float32 | 10 | 关节速度 (rad/s) |
| q_init | float32 | 10 | 默认关节角 (rad) |

**Response** (Python → control_task):

| 字段 | 类型 | 维度 |
|---|---|---|
| q_exp | float32 | 10 |
| dq_exp | float32 | 10 |
| tau_exp | float32 | 10 |

---

## 力/阻抗控制 (`vmc_src/force_imp_controller.cpp`)

两个控制函数:

### `force_control_and_dis_stand()` — 站立模式

阻抗控制: 计算期望足端力 → 雅可比转置 → 关节力矩

### `force_control_and_dis_rl()` — RL 模式 (TROT)

混合位置-力控制:
- 支撑腿: 力控制 (阻抗)
- 摆动腿: 位置控制 (PD 跟踪)
- QP 力分配: qpOASES 求解最优 GRF (摩擦锥约束)

---

## SPI 共享内存协议 (`src/memory_share.cpp`)

### 结构体

```cpp
struct shareMemory {
    int flag;                    // 0=可写(control), 1=可读(STM32)
    unsigned char szMsg[2048];   // 前半读, 后半写 (全双工)
};
// 键值: MEM_SPI = 0x0001
```

### SPI 读取 (STM32 → control_task, 每帧 ~500B)

| 偏移 | 字段 | 类型 |
|---|---|---|
| 0 | att[3] (pitch, roll, yaw deg) | float×3 |
| 12 | att_rate[3] (deg/s) | float×3 |
| 24 | acc_b[3] (机体加速度 m/s²) | float×3 |
| 36 | acc_n[3] (世界加速度) | float×3 |
| 48 | imu_mems_connect | char |
| 49 | att_usb[3], att_rate_usb[3], acc_b_usb[3] | float×9 |
| 85 | q[14], dq[14], tau[14] (关节角/速度/力矩) | float×42 |
| 253 | connect[14] (bit: node/motor/ready) | char×14 |
| 267 | q_servo[14], tau_servo[14] (舵机) | float×28 |
| 379 | bat_v | float |

### SPI 写入 (control_task → STM32, 每帧 ~500B)

| 偏移 | 字段 | 类型 |
|---|---|---|
| 512 | en_motor, en_servo, reset_q, reset_err, calib | char×8 |
| 520 | q_set[14], q_reset[14], tau_ff[14], kp[14], kd[14], stiff[14] | float×84 |
| 856 | q_set_servo[14]... | float×56 |

### 导航共享内存 (control_task ↔ navigation_task)

键值: `MEM_CONTROL = 4999`

control_task 写入 (→ navigation):
- 姿态 (att_now, att_tar), 速度 (dcom_n), CoM 位置, GRF 估计
- 14 电机 q_set/q_now/tau_now

control_task 读取 (← navigation):
- 遥控指令 (rc_spd, rc_rate, 按钮)
- SDK 指令 (exp_spd_o, exp_att_o, exp_pos_o)
- IMU 标定参数

---

## CAN 协议定义 (`inc/can.h`)

### MIT Mini Cheetah 协议常量

| 常量 | 值 | 说明 |
|---|---|---|
| P_MIN/P_MAX | ±180° | 位置限幅 |
| V_MIN/V_MAX | ±2000 | 速度限幅 |
| T_MIN/T_MAX | ±10.0 Nm | 力矩限幅 |
| KP_MIN/KP_MAX | 0.0 ~ 10.0 | 刚度范围 |
| KD_MIN/KP_MAX | 0.0 ~ 10.0 | 阻尼范围 |

### CAN ID 分配

| ID 范围 | 用途 |
|---|---|
| 0-19 | 系统参数下发 |
| 20-39 | 力矩指令下发 |
| 40-59 | 状态反馈 |
| 60-79 | 位置反馈 |
| 80-99 | 速度反馈 |
| 100-119 | MIT 模式反馈 |
| 140-159 | 力矩/电流指令 |
| 160-179 | 位置指令 |
| 180-199 | 速度指令 |
| 200-219 | MIT 模式指令 |

### 电机数据结构 (`_LEG_MOTOR_ALL`)

14 个关节: 每条腿 7 个 (q00-q06 左, q10-q16 右)，实际 q05/q06/q15/q16 为预留

control_task 输出 (经 SPI→STM32→CAN 下发):
- `q_set[14]`: 目标位置 (deg)
- `tau_ff[14]`: 前馈力矩 (Nm), 乘 soft_weight
- `kp[14]`: 位置刚度, `kd[14]`: 速度阻尼, `stiff[14]`: 刚度乘子

---

## 遥控处理 (`rc_process()`, 200Hz)

三路指令源优先级:

1. **ESP32 无线遥控** (`ocu.esp32_connect=true`): UDP port 7070, 接收 `_msg_ocu_tinker` 结构体
2. **SDK/ROS 指令** (`sdk.sdk_mode=1`): 速度/姿态/位置指令
3. **SBUS 遥控器** (经 STM32→SPI): 摇杆 (±1 normalized)

ESP32 协议头: `0xFA 0xFB`

### 指令类型

| gait_mode (ESP32) | 行为 |
|---|---|
| 99 | 断电 (`ocu.key_ud=-1`) |
| 1 | 上电 + STAND_RC (`ocu.key_x=1`) |
| 10 | RL Walk (`ocu.key_b=1`) |
| 11 | RL Stand (`ocu.key_a=1`) |

### 速度指令处理

```
遥控模式: tar_spd = RC_mapped × MAX_SPD × LPF(0.68)
SDK 模式: tar_spd = sdk.cmd × LPF(1.0)
无指令:   tar_spd → 0 × LPF(0.86)
```

### SDK 中断检测

RC 摇杆偏移 > 0.1 → 中断 SDK → 切回遥控。2 秒无操作 → 重新启用 SDK。

---

## 关键全局变量

| 变量 | 类型 | 说明 |
|---|---|---|
| `vmc_all` | VMC_ALL | 主控制状态 (步态/姿态/速度/参数) |
| `robotwb` | robotTypeDef | 机器人本体状态 (IMU/腿/CoM) |
| `leg_motor_all` | _LEG_MOTOR_ALL | 14 电机状态+指令 |
| `arm_motor_all` | _LEG_MOTOR_ALL | 14 舵机状态+指令 |
| `ocu` | _OCU | 遥控器状态 (按钮/摇杆/模式) |
| `sdk` | _SDK | ROS/SDK 外部指令 |
| `spi_rx` / `spi_tx` | _SPI_RX / _SPI_TX | SPI 收发缓冲 |
| `nav_rx` / `nav_tx` | _NAV_RX / _NAV_TX | 导航共享内存 |

---

## 编译依赖

| 库 | 用途 |
|---|---|
| qpOASES | QP 求解器 (最优 GRF 分配) |
| Eigen3 | 矩阵运算 (雅可比, 旋转矩阵) |
| yaml-cpp | 配置文件解析 |
| TVM Runtime | 本地 RL 推理 (可选, `RL_USE_TVM`) |
| pthreads | 多线程 + SCHED_FIFO |
| Linux SHM | 共享内存 (sys/shm.h) |
| Linux SPI | SPI 设备接口 (linux/spi/spidev.h) |
| POSIX sockets | UDP 通信 |

---

## 安全机制

1. **SPI 连接丢失检测**: `mem_loss_cnt > 1.0s` → 力矩清零, kp/kd=0, en_motor=0
2. **RL 连接丢失检测**: `loss_rl > 100` → `rl_connect=0`
3. **循环超时检测**: `T > 5ms` → 打印警告
4. **非 RL 模式: 观测清零**: 避免错误的状态估计输入 RL 策略
5. **ESP32 断开检测**: 500 次超时无数据 → `esp32_connect=0`
6. **SDK 遥控中断**: RC 摇杆活动 → 自动切回手动控制
