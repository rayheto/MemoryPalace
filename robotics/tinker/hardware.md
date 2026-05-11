# Tinker 硬件规格

基于 `/home/ubuntu/cll` 中的 URDF、CAD 文件、参数配置实物提取。

---

## 版本对比

| 参数 | Tinker V1 (`tinker/`) | Tinker V2 (`TinkerV2_URDF/`) | 实机 (param_robot.yaml) |
|---|---|---|---|
| **DOF** | 10 (5×2 legs) | 10 (5×2 legs) | 14 (10 legs + 4 arms) |
| **总质量** | ~4.4 kg | ~7.5-8 kg | 12 kg |
| **站立高度** | — | — | 0.306 m (髋到地) |
| **大腿长度 L1** | — | — | 0.12 m |
| **小腿长度 L2** | — | — | 0.14 m |
| **踝长 L3** | — | — | 0.075 m |
| **状态** | SolidWorks 导出 | 重新设计 | 实际部署参数 |

---

## V2 关节参数 (来自 URDF)

每条腿 5 个关节，从髋到踝：

| 序号 | 关节名 | 类型 | 轴线 | 范围 (rad) | 力矩 (Nm) |
|---|---|---|---|---|---|
| J0 | `J_L0` / `J_R0` | Hip Yaw | Z | ±0.7 | 12 |
| J1 | `J_L1` / `J_R1` | Hip Roll | -X | -0.38~+0.46 | 20 |
| J2 | `J_L2` / `J_R2` | Hip Pitch | Y/-Y | ±1.57 | 20 |
| J3 | `J_L3` / `J_R3` | Knee | Y/-Y | -2.35~0 / 0~2.35 | 20 |
| J4 | `J_L4_ankle` / `J_R4_ankle` | Ankle | -Y/Y | ±1.2 | 12 |

**V1 vs V2 关键差异:**
- V2 髋 Pitch 范围更大: ±1.57 vs -1.57~+0.57 rad
- V2 膝盖范围更大，基本质量翻倍
- V2 关节命名改为 `J_L0-J_L4` / `J_R0-J_R4`

---

## 实机关节参数 (来自 param_robot.yaml)

14 DOF 系统 (q00-q06 左腿, q10-q16 右腿)，实际仅 q00-q04/q10-q14 有力矩，q05/q06/q15/q16 为 0 (预留手臂):

| 关节 | 默认角(deg) | 最小角(deg) | 最大角(deg) | 力矩(Nm) |
|---|---|---|---|---|
| q00/q10 (Hip Yaw) | 0 | -22 | +22 | 6 |
| q01/q11 (Hip Roll) | -4.3/+4.3 | -18 | +18 | 12 |
| q02/q12 (Hip Pitch) | 56 | -36 | +74 | 12 |
| q03/q13 (Knee) | -96 | -142 | 0 | 12 |
| q04/q14 (Ankle) | 45 | -40 | +65 | 6 |
| q05-q06/q15-q16 | 0 | 0 | 0 | 0 (预留) |

---

## 电机通信架构

**实际使用 CAN + STM32 架构**，NOT EtherCAT。`ethercat.sh` 中的 EtherCAT 注释属于其他硬件版本/其他机器人的配置。

```
Odroid C4 (Linux, PREEMPT_RT)
    │ SPI (2kHz, 共享内存 MEM_SPI=0x0001, 2048B)
    ▼
STM32F4 协处理器 (hardware_task)
    ├── CAN1 (2Mbps)  → 左腿 7 电机 (Index 1-7)
    ├── CAN2 (2Mbps)  → 右腿 7 电机 (Index 8-14)
    ├── SBUS          → 遥控接收机
    ├── I2C/SPI       → 板载 MEMS IMU
    └── ADC           → 电池电压
```

### CAN 电机协议

采用 **MIT Mini Cheetah 协议**，每条 CAN 消息控制一个电机:

| 字段 | 位宽 | 说明 |
|---|---|---|
| 位置 | uint16 | P_MIN=-180°, P_MAX=+180° |
| 速度 | uint16 | V_MIN=-2000, V_MAX=+2000 |
| KP | uint16 | KP_MIN=0.0, KP_MAX=10.0 |
| KD | uint16 | KD_MIN=0.0, KD_MAX=10.0 |
| 力矩 | uint16 | T_MIN=-10.0Nm, T_MAX=+10.0Nm |

电机工作在**力矩模式**，control_task 计算前馈力矩 + KP/KD 后经 SPI→STM32→CAN 下发。

---

## 机械 CAD 文件位置

都在 `git_tinker/OmniBotSeries-Tinker-main/OmniBotSeries/`:

| 目录 | 内容 | 文件数 |
|---|---|---|
| `切割/` | 碳板/金属切割件 (大腿、小腿、踝板、机体、支架) | 10 STEP |
| `机加工/` | CNC 机加工件 | — |
| `打印/` | 3D 打印件 (脖子、灯头、外壳、电池支架、天线) | ~15 STEP |
| `TinkerV2_URDF_Head.STEP` | V2 头部总成 | 1 STEP |

---

## PCB 文件

在 `OmniBotTele/PCB/` — **仅遥控器**:

| 文件 | 格式 | 说明 |
|---|---|---|
| `OmniBotRemote.SchDoc` | Altium Designer | 遥控器原理图 |
| `OmniBotRemote.PcbDoc` | Altium Designer | 遥控器 PCB |
| `usb_convert.PcbDoc` | Altium Designer | USB 转接板 |

> **机器人的主控板/驱动板 PCB 不在本地仓库中**，需查看飞书文档或 Altium 工程。

---

## 机载计算机 (Odroid C4)

### 硬件规格

| 参数 | 规格 |
|---|---|
| SoC | Amlogic S905X3 |
| CPU | 4× Cortex-A55 @ 2.016GHz, ARMv8-A, Neon + Crypto |
| GPU | Mali-G31 MP2 @ 650MHz, OpenGL ES 3.0 / Vulkan 1.0 |
| 内存 | 4GB DDR4-2640 |
| 接口 | 40Pin GPIO, HDMI 4K/60Hz, SPI, UART |
| 性能 | Pi 3 的 2-10 倍；弱于 Odroid N2；强于 Raspberry Pi 4 |

### 系统配置

| 参数 | 值 |
|---|---|
| IP 地址 | 192.168.1.113 (主) |
| OS | Ubuntu 20.04 + PREEMPT_RT 实时内核 (5.10.x-rt) |
| 用户 | odroid |
| 启动方式 | `/etc/rc.local` 自动启动三个 task |
| 任务目录 | `/home/odroid/Tinker/` |
| 参数目录 | `/home/odroid/Tinker/Param/` |
| 数据日志 | `/home/odroid/Tinker/Data/` |
| SSH | `ssh-copy-id odroid@192.168.1.113` |

### 协处理器 (STM32F4)

| 参数 | 说明 |
|---|---|
| MCU | STM32F407 或兼容型号 |
| 固件 | `hardware_task_tinker` (645KB) |
| 与 Linux 通信 | SPI 全双工共享内存 (key=MEM_SPI=0x0001) |
| 电机通信 | CAN1 + CAN2 (2Mbps) |
| 传感器 | 板载 MEMS IMU + SBUS 遥控 + 电池 ADC |

---

## IMU 参数

```yaml
# param_hardware.yaml
imu_usb_enable: 0        # IMU 非 USB 接口
imu_set_att: [0, 0, 0]   # 安装姿态
gyro_bias: [0.0823, -0.4672, 0.0492]  # 陀螺仪零偏
```

---

## 硬件注意事项

1. **本地无机器人主控 PCB 原理图** — 需查阅飞书或向上游 GitHub 索取
2. **电机型号和驱动器型号** 未在本地文件中明确，需查 BOM(飞书)
3. **电源系统** (电池规格、电压、分电方式) 本地无文档
4. CAD 文件均为 STEP 格式，可用 Fusion 360/FreeCAD 打开
5. PCB 文件为 Altium Designer 格式，需 AD 或 Altium 365 Viewer 查看
