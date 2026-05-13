# SPI 通信架构全解析

## 1. 物理拓扑

```
┌──────────────────────────────────────────────────────────────────┐
│  Odroid C4 (ARM64 Linux, PREEMPT_RT)                             │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  hardware_task_tinker  (预编译二进制, 源码缺失)              │  │
│  │  ELF 64-bit ARM aarch64, dynamically linked, 645KB          │  │
│  │                                                            │  │
│  │  职责: 物理 SPI 总线驱动 + GPIO 控制                         │  │
│  │                                                            │  │
│  │  Thread_SPI() 主循环:                                      │  │
│  │    ┌──────────────────────────────────────────┐            │  │
│  │    │ 1. SPISetup()  初始化 SPI 硬件参数        │            │  │
│  │    │ 2. SPIDataRW() 全双工读写                 │            │  │
│  │    │ 3. GPIO sysfs  控制 CS 片选/中断          │            │  │
│  │    │ 4. 共享内存 IPC: shmget(0x0001)           │            │  │
│  │    └──────────────────────────────────────────┘            │  │
│  │         │                                                  │  │
│  │         │ 物理 SPI 总线                                     │  │
│  └─────────┼──────────────────────────────────────────────────┘  │
│            │                                                     │
│  ┌─────────┼──────────────────────────────────────────────────┐  │
│  │  control_task_tinker  (有源码, 586KB, 500Hz 控制循环)       │  │
│  │                                                            │  │
│  │  职责: 实时运动控制, 不直接操作 SPI 硬件                     │  │
│  │                                                            │  │
│  │  Thread_Mem_Servo() 循环:                                  │  │
│  │    ┌──────────────────────────────────────────┐            │  │
│  │    │ shmat(0x0001) → 映射共享内存               │            │  │
│  │    │ 轮询 pshm_rx->flag == 1                   │            │  │
│  │    │ memory_read()  解析传感器数据               │            │  │
│  │    │ memory_write() 写入控制指令                 │            │  │
│  │    │ pshm_rx->flag = 0                         │            │  │
│  │    └──────────────────────────────────────────┘            │  │
│  │         │                                                  │  │
│  └─────────┼──────────────────────────────────────────────────┘  │
│            │                                                     │
│  ┌─────────┼──────────────────────────────────────────────────┐  │
│  │  navigation_task_tinker  (有源码, 402KB, 通信桥接)          │  │
│  │                                                            │  │
│  │  职责: UDP 服务 (OCU/SDK/ARM/ESP32), 数据日志               │  │
│  │                                                            │  │
│  │  Thread_Mem_Navigation() 循环:                             │  │
│  │    ┌──────────────────────────────────────────┐            │  │
│  │    │ shmget(4999) 独立共享内存段                │            │  │
│  │    │ 与 control_task 交换遥控/导航/状态数据      │            │  │
│  │    └──────────────────────────────────────────┘            │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                  │
│  /boot/dtb/amlogic/  ← 设备树 (不在本仓库, 由 Odroid 镜像提供)   │
│  /dev/spidev0.0      ← SPI 设备节点 (meson-spicc 驱动)          │
│  /sys/class/gpio/     ← GPIO sysfs 接口                         │
└──────────────────────────────────────────────────────────────────┘
                          │
                          │  SPI 物理总线
                          │  MOSI/MISO/SCK/CS
                          ▼
┌──────────────────────────────────────────────────────────────────┐
│  STM32F4 微控制器 (裸机, 无 OS)                                   │
│                                                                  │
│  固件: hardware_task_tinker (另一份, STM32 裸机二进制, 645KB)      │
│                                                                  │
│  职责:                                                            │
│    - SPI 从机通信                                                 │
│    - CAN 总线 (2Mbps) → 14 个电机 (MIT Mini Cheetah 协议)         │
│    - IMU 传感器读取 (板载 MEMS + 可选 USB IMU)                     │
│    - nRF24L01 无线遥控接收                                        │
│    - 电池电压/状态监测                                             │
│    - ANO 遥测协议 (USB VCP)                                       │
│    - 共享内存: 读写同一段 shmget(0x0001)                           │
└──────────────────────────────────────────────────────────────────┘
```

> **关键纠错**: `hardware_task_tinker` 并非 STM32 裸机固件，而是与 `control_task_tinker` 同级的 ARM64 Linux ELF 可执行文件，三者全部运行在 Odroid C4 上。共享内存 (`shmget`/`shmat`) 是 Linux System V IPC 机制，只能在同一台 Linux 主机上的进程间通信。

---

## 2. 启动顺序

来自 [rc.local](release/rc.local)：

```sh
cd /home/odroid/Tinker
sudo ./hardware_task_tinker &    # 1. 先启动硬件通信层
sleep 1
sudo ./control_task_tinker &     # 2. 再启动运动控制层
sleep 1
sudo ./navigation_task_tinker &  # 3. 最后启动通信桥接
```

`hardware_task_tinker` 必须最先启动，因为它是 SPI 总线初始化 + 共享内存创建的发起方。

---

## 3. SPI 通信协议（在 `hardware_task_tinker` 中实现）

由于 `hardware_task_tinker` 源码缺失，以下从二进制符号表和字符串推导：

### 3.1 SPI 设备初始化

从二进制中提取的设备路径：

| 设备路径 | 用途 |
|----------|------|
| `/dev/spidev0.0` | 主 SPI 设备 (与 STM32 通信) |
| `/dev/spidev0.1` | 备用 SPI 设备 |
| `/dev/ttyUSB0` | USB 串口 (IMU/调试) |
| `/dev/ttyACM0` | USB CDC ACM (虚拟串口) |
| `/dev/ttyS1` ~ `/dev/ttyS11` | 硬件串口 (备用) |

**SPI 初始化失败处理**（二进制中的日志字符串）：

```
init spi failed!                    → 初始化失败
Hardware::SPI Still Online at ...   → 连接正常时周期性日志
Hardware::Hardware SPI-STM32 Loss!!! → SPI 通信丢失
SPI Reopen!                         → 自动重连
```

### 3.2 二进制中的 SPI 符号

```cpp
// SPI 核心函数 (从 nm 提取)
void SPISetup();         // SPI 参数配置 (speed, mode, bits_per_word)
void SPISetupMode();     // SPI 工作模式 (CPOL/CPHA, 0~3)
void SPIDataRW();        // SPI 全双工读写 (ioctl SPI_IOC_MESSAGE)
void Thread_SPI();       // SPI 通信主线程
int  open_port(int);     // 打开 SPI/串口设备

// 全局变量
B shareMemory_spi;       // 共享内存结构 (2048 字节)
B spi_rx;                // 接收数据缓冲区
B spi_tx;                // 发送数据缓冲区
B spi_rx_buf;            // 原始接收缓冲区
B spi_tx_buf;            // 原始发送缓冲区
B spi_connect;           // SPI 连接状态标志
B spi_loss_cnt;          // 通信丢失计数器
B spi_rx_cnt;            // 接收字节计数
B spi_tx_cnt;            // 发送字节计数
B spi_tx_cnt_show;       // 显示用发送计数
```

### 3.3 GPIO 控制

```cpp
// 二进制中的日志字符串
" set gpio success"    // GPIO 配置成功 (注意前导空格, printf 格式化)
"set gpio--"           // GPIO 配置失败/释放

// 典型 Linux GPIO sysfs 操作 (推断)
// echo N > /sys/class/gpio/export
// echo out > /sys/class/gpio/gpioN/direction
// echo 1 > /sys/class/gpio/gpioN/value
```

SPI 片选 (CS) 和 STM32 中断信号通过 Linux GPIO sysfs 控制。Odroid C4 (Amlogic S905X3) 的 GPIO 由 `pinctrl` 子系统管理。

### 3.4 SPI 参数推断

MIT Mini Cheetah 电机控制要求的典型 SPI 配置：

```
SPI Mode:       0 或 3 (CPOL/CPHA)
SPI Speed:      10-20 MHz (推测, 根据 2ms 控制周期和 2048 字节帧)
Bits per word:  8
CS:             GPIO 控制 (非 SPI 硬件 CS)
```

`SPISetupMode()` 函数通过 `ioctl(fd, SPI_IOC_WR_MODE, &mode)` 和 `ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)` 配置。

---

## 4. 共享内存协议 (`control_task_tinker` 中的实现)

### 4.1 数据结构

源码位置: [inc/spi_node.h](inc/spi_node.h#L226-L231)

```c
#define MEM_SPI     0x0001   // SPI 共享内存键值
#define MEM_CONTROL 4999     // 控制共享内存键值
#define MEM_SIZE    2048     // 总帧大小 (字节)
```

共享内存段 `0x0001` 由 `hardware_task_tinker` 创建, `control_task_tinker` 附加。

#### 仲裁结构体 ([memory_share.cpp:48-53](src/memory_share.cpp#L48-L53))

```c
struct shareMemory {
    int  flag = 0;              // 0=control_task 可写, 1=hardware_task 已写
    unsigned char szMsg[2048];  // 前半 (0-1023): STM32→Odroid 传感器数据
                                // 后半 (1024-2047): Odroid→STM32 控制指令
};
```

### 4.2 仲裁协议

```
时间轴 →

hardware_task (SPI 主循环):
  │
  ├─ SPIDataRW() 从 STM32 读取传感器数据
  ├─ 序列化写入 szMsg[0..1023]
  ├─ flag = 1   ← 通知 control_task 数据就绪
  │
  ├─ 轮询 flag == 0   ← 等待 control_task 写完控制指令
  ├─ 从 szMsg[1024..2047] 读取控制指令
  ├─ SPIDataRW() 发送给 STM32
  └─ 循环

control_task (Thread_Mem_Servo, 500us 轮询):
  │
  ├─ 轮询 pshm_rx->flag == 1   ← 等待新传感器数据
  ├─ pthread_mutex_lock(&lock)
  ├─ 拷贝 szMsg[0..1023] → mem_read_buf
  ├─ memory_read()  解析传感器数据
  ├─ memory_write() 生成控制指令
  ├─ 拷贝 mem_write_buf → szMsg[1024..2047]
  ├─ pthread_mutex_unlock(&lock)
  ├─ flag = 0   ← 通知 hardware_task 控制指令就绪
  └─ 循环
```

### 4.3 半双工全帧架构

**关键设计**: 整个 2048 字节帧在一次 SPI 传输中完成，分为两个半区：

```
szMsg 字节布局:
┌─────────────────────────┬──────────────────────────┐
│ 0                      1024                        2047
│◄── STM32 → Odroid ────►│◄── Odroid → STM32 ──────►│
│      (传感器数据)        │      (控制指令)           │
│                         │                          │
│  flag == 1 时          │  flag == 0 时            │
│  control_task 读取      │  control_task 写入       │
└─────────────────────────┴──────────────────────────┘
```

---

## 5. 读帧格式 (STM32/ESP32 → control_task, szMsg[0..1023])

数据解析代码: [memory_read()](src/memory_share.cpp#L220-L360)

### 5.1 序列化规则

所有 `float` 以大端序编码（在 ARM64 小端序机器上解码）:

```cpp
// 解码 (小端序机器读取大端序数据)
static float floatFromData_spi(unsigned char *data, int *anal_cnt)
{
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);   // byte[3] → 最高字节
    i |= (*(data + *anal_cnt + 2) << 16);   // byte[2]
    i |= (*(data + *anal_cnt + 1) << 8);    // byte[1]
    i |= (*(data + *anal_cnt + 0));         // byte[0] → 最低字节
    *anal_cnt += 4;
    return *(float *)&i;
}

// 编码 (大端序)
static void setDataFloat_mem(float f, int *anal_cnt)
{
    int i = *(int *)&f;
    mem_write_buf[(*anal_cnt)++] = ((i >> 24) & 0xFF);   // byte[3]
    mem_write_buf[(*anal_cnt)++] = ((i >> 16) & 0xFF);   // byte[2]
    mem_write_buf[(*anal_cnt)++] = ((i >> 8) & 0xFF);    // byte[1]
    mem_write_buf[(*anal_cnt)++] = (i & 0xFF);           // byte[0]
}
```

### 5.2 读帧字段映射表 (szMsg[0..])

| 偏移(B) | 长度 | 字段 | 类型 | 说明 |
|---------|------|------|------|------|
| 0 | 12 | `att[3]` | float×3 | 姿态欧拉角 PITr/ROLr/YAWr (度) |
| 12 | 12 | `att_rate[3]` | float×3 | 角速度 (度/秒) |
| 24 | 12 | `acc_b[3]` | float×3 | 机体坐标系加速度 (m/s²) |
| 36 | 12 | `acc_n[3]` | float×3 | 世界坐标系加速度 (m/s²) |
| 48 | 1 | `imu_mems_connect` | char | 板载 IMU 连接标志 |
| 49 | 12 | `att_usb[3]` | float×3 | 外接 USB-IMU 姿态 (备用) |
| 61 | 12 | `att_rate_usb[3]` | float×3 | 外接 USB-IMU 角速度 |
| 73 | 12 | `acc_b_usb[3]` | float×3 | 外接 USB-IMU 加速度 |
| 85 | 56 | `q[0..13]` | float×14 | 14 个电机关节位置 (度) |
| 141 | 56 | `dq[0..13]` | float×14 | 14 个电机关节速度 (度/秒) |
| 197 | 56 | `tau[0..13]` | float×14 | 14 个电机关节力矩 (Nm), 乘以 `mem_connect` |
| 253 | 14 | `connect[0..13]` | byte×14 | 电机连接状态复合编码 |
| 267 | 56 | `q_servo[0..13]` | float×14 | 14 个舵机位置 (度, 手臂用) |
| 323 | 14 | `connect_servo[0..13]` | byte×14 | 舵机连接状态 |
| 337 | 4 | — | — | (可能是 dq_servo, 代码中未读) |
| 341 | 56 | `tau_servo[0..13]` | float×14 | 14 个舵机力矩 (Nm) |
| 397 | 4 | `bat_v` | float | 电池电压 (V) |
| 401 | 1 | `ocu.connect` | char | 遥控器连接标志 |
| 402 | 1 | `ocu.key_st` | char | 遥控器 Start 键 |
| 403 | 1 | `ocu.key_back` | char | 遥控器 Back 键 |
| 404 | 1 | `ocu.key_lr` | char | 遥控器 左右键 (255=-1) |
| 405 | 1 | `ocu.key_ud` | char | 遥控器 上下键 (255=-1) |
| 406 | 1 | `ocu.key_x` | char | 遥控器 X 键 |
| 407 | 1 | `ocu.key_a` | char | 遥控器 A 键 |
| 408 | 1 | `ocu.key_b` | char | 遥控器 B 键 |
| 409 | 1 | `ocu.key_y` | char | 遥控器 Y 键 |
| 410 | 1 | `ocu.key_ll` | char | 遥控器 左肩键 |
| 411 | 1 | `ocu.key_rr` | char | 遥控器 右肩键 |
| 412 | 4 | `ocu.rc_spd_w[0]` | float | 遥控器世界X 速度指令 |
| 416 | 4 | `ocu.rc_spd_w[1]` | float | 遥控器世界Y 速度指令 |
| 420 | 4 | `ocu.rc_att_w[0]` | float | 遥控器俯仰 姿态指令 |
| 424 | 4 | `ocu.rc_att_w[1]` | float | 遥控器横滚 姿态指令 |
| 428 | 4 | `ocu.rate_yaw_w` | float | 遥控器偏航 角速度指令 |
| | | | **总计 ~432 字节 + 未使用填充** |

### 5.3 电机连接状态的比特编码

[memory_share.cpp:264-266](src/memory_share.cpp#L264-L266)

```cpp
temp = charFromData_spi(mem_read_buf, &mem_read_cnt) * mem_connect;
spi_rx.connect[i]        = temp / 100 % 10;    // 百位: 节点在线
spi_rx.connect_motor[i]  = (temp - ...*100)/10; // 十位: 电机在线
spi_rx.ready[i]          = temp % 10;           // 个位: 预备就绪
```

### 5.4 IMU 数据融合策略

[memory_share.cpp:247-256](src/memory_share.cpp#L247-L256)

```cpp
// 板载 MEMS IMU 连接时, 若外接 USB-IMU 数据有效 (在阈值范围内),
// 则优先使用 USB-IMU 的姿态/角速度/加速度数据覆盖板载数据
if (spi_rx.imu_mems_connect) {
    for(int i=0; i<3; i++) {
        if (fabs(spi_rx.att_usb[0]) < 180 &&
            fabs(spi_rx.att_usb[1]) < 180 &&
            fabs(spi_rx.att_usb[2]) < 360) {
            spi_rx.att[i]      = spi_rx.att_usb[i];
            spi_rx.att_rate[i] = spi_rx.att_rate_usb[i];
            spi_rx.acc_b[i]    = spi_rx.acc_b_usb[i];
        }
    }
}
```

### 5.5 姿态零偏补偿

[memory_share.cpp:335-340](src/memory_share.cpp#L335-L340)

```cpp
robotwb.IMU_now_o.pitch = spi_rx.att[0] + vmc_all.att_measure_bias[PITr];
if (vmc_all.side_flip)
    robotwb.IMU_now_o.roll = spi_rx.att[1] + vmc_all.att_measure_bias_flip[ROLr];
else
    robotwb.IMU_now_o.roll = spi_rx.att[1] + vmc_all.att_measure_bias[ROLr];
robotwb.IMU_now_o.yaw = spi_rx.att[2];
```

---

## 6. 写帧格式 (control_task → STM32, szMsg[1024..2047])

控制指令生成: [memory_write()](src/memory_share.cpp#L362-L459)

### 6.1 写帧字段映射表 (szMsg[1024..])

| 偏移(B) | 长度 | 字段 | 类型 | 说明 |
|---------|------|------|------|------|
| 1024 | 1 | `en_motor` | char | 14 个 BLDC 电机总使能 |
| 1025 | 1 | `en_servo` | char | 14 个舵机总使能 |
| 1026 | 1 | `reser_q` | char | 全部关节复位标志 |
| 1027 | 1 | `reset_err` | char | 错误复位标志 |
| 1028 | 1 | `Acc_CALIBRATE` | char | 加速度计标定使能 |
| 1029 | 1 | `Gyro_CALIBRATE` | char | 陀螺仪标定使能 |
| 1030 | 1 | `Mag_CALIBRATE` | char | 磁力计标定使能 |
| 1031 | 1 | `beep_state` | char | 蜂鸣器状态 |
| 1032 | 56 | `q_set[0..13]` | float×14 | 14 电机目标位置 (度) |
| 1088 | 56 | `q_reset[0..13]` | float×14 | 14 电机复位后初始位置 |
| 1144 | 56 | `tau_ff[0..13]` | float×14 | 14 电机前馈力矩 (Nm), 乘以 `soft_weight` |
| 1200 | 56 | `kp[0..13]` | float×14 | 14 电机位置刚度, 乘以 `soft_weight` |
| 1256 | 56 | `kd[0..13]` | float×14 | 14 电机速度阻尼, 乘以 `soft_weight` |
| 1312 | 56 | `stiff[0..13]` | float×14 | 14 电机刚度乘子 |
| 1368 | 14 | `reser_q_div[0..13]` | byte×14 | 14 电机单独复位标志 |
| 1382 | 56 | `q_set_servo[0..13]` | float×14 | 14 舵机目标位置 (度) |
| 1438 | 56 | `q_reset_servo[0..13]` | float×14 | 14 舵机复位位置 |
| 1494 | 56 | `tau_ff_servo[0..13]` | float×14 | 14 舵机前馈力矩, 乘以 `soft_weight` |
| 1550 | 56 | `kp_servo[0..13]` | float×14 | 14 舵机位置刚度 (当前未使用) |
| 1606 | 56 | `kd_servo[0..13]` | float×14 | 14 舵机速度阻尼 (当前未使用) |
| 1662 | 56 | `stiff_servo[0..13]` | float×14 | 14 舵机刚度乘子 |
| 1718 | 14 | `reser_q_div_servo[0..13]` | byte×14 | 14 舵机单独复位标志 |
| | | | **总计 ~1732 字节** |

### 6.2 力矩/增益缩放的 soft_weight 机制

所有力矩和 PD 增益在发送前乘以 `vmc_all.param.soft_weight`：

```cpp
spi_tx.tau_ff[id] = leg_motor_all.set_t[id] * mem_connect * vmc_all.param.soft_weight;
spi_tx.kp[id]     = leg_motor_all.kp[id]      * vmc_all.param.soft_weight;
spi_tx.kd[id]     = leg_motor_all.kd[id]      * vmc_all.param.soft_weight;
```

`soft_weight` 通常为 0~1，用于在软启动/软停止时渐进施加控制力，防止电机突变。

---

## 7. 通信健康监控

### 7.1 连接丢失检测

```cpp
// [memory_share.cpp:660-670]
mem_loss_cnt += sys_dt;
if (mem_loss_cnt > 1.0 && mem_connect == 1) {  // 持续 1s 以上无数据
    mem_connect = 0;
    // 将所有电机指令清零, 进入安全状态
    for (int i = 0; i < 14; i++) {
        spi_tx.q_set[i]   = spi_rx.q[i];   // 保持当前位置
        spi_tx.tau_ff[i]  = 0;              // 清零力矩
        spi_tx.kp[i] = spi_tx.ki[i] = spi_tx.kd[i] = spi_tx.en_motor = 0;
    }
    printf("Control::Memery Hardware Loss!!!\n");
}
```

### 7.2 SPI 层重连 (hardware_task_tinker)

从二进制字符串可知 `hardware_task_tinker` 端有独立的健康检测：

```
Hardware::Hardware SPI-STM32 Loss!!!   → SPI 层检测到通信丢失
SPI Reopen!                             → 自动尝试重新打开 /dev/spidev*
Hardware::SPI Still Online at ...       → 周期性心跳日志
```

---

## 8. 关键设计要点

### 8.1 为什么要拆分出 hardware_task?

| 原因 | 说明 |
|------|------|
| **实时性隔离** | SPI 硬件 I/O 的延迟不稳定, 与 500Hz 控制循环分离, 防止 SPI 阻塞影响控制周期 |
| **权限隔离** | 硬件访问 (`/dev/spidev*`, `/sys/class/gpio/`) 需要 `sudo`, 控制层不需要 |
| **故障隔离** | SPI 重连/异常处理不影响控制状态机, 硬件层崩溃不拖垮运动控制 |
| **进程分工** | `hardware_task`: "我在跟硬件打交道" / `control_task`: "我在做运动控制数学" |

### 8.2 为什么用共享内存而不是 Unix Socket?

- **零拷贝**: 数据直接在共享内存段中读写, 无需在进程间序列化/反序列化传输
- **极低延迟**: 共享内存是访问 RAM 的速度, ~100ns 级; socket 需要内核参与, ~10us 级
- **物理 SPI 隔离开销**: 共享内存跑在 Odroid C4 本地 RAM (~100ns)，SPI 总线的物理延迟 (微秒级) 由 `hardware_task_tinker` 独立承担，不拖慢 500Hz 控制循环

### 8.3 共享内存安全

```cpp
#define EN_THREAD_LOCK 1

#if EN_THREAD_LOCK
    pthread_mutex_lock(&lock);       // 读/写期间加锁
#endif
    memory_read();
    memory_write();
#if EN_THREAD_LOCK
    pthread_mutex_unlock(&lock);
#endif
```

---

## 9. 设备树 (从 Odroid C4 镜像提取)

设备树来源: `ubuntu-20.04-server-odroidc4-20201218.img.xz` → `/boot/dtbs/5.10.0-odroid-arm64/amlogic/meson-sm1-odroid-c4.dtb`

内核版本: `Linux 5.10.0-odroid-arm64`

### 9.1 SPI0 控制器节点 (已启用, 生成 /dev/spidev0.0)

```dts
spi@13000 {                                          // spicc0 控制器, 基址 0xffd13000
    compatible = "amlogic,meson-g12a-spicc";
    reg = <0x00 0x13000 0x00 0x44>;
    interrupts = <0x00 0x51 0x04>;                   // 中断号 81
    clocks = <0x02 0x17>, <0x02 0x102>;
    clock-names = "core", "pclk";
    status = "okay";                                  // ← 已启用

    pinctrl-names = "default", "gpio_periphs";
    pinctrl-0 = <&spicc0_x_pins>;                    // 数据引脚: SCK/MOSI/MISO
    pinctrl-1 = <&spicc0_ss0_x_pins>;                // 片选引脚: CS0

    num_chipselect = <1>;                            // 1 个片选信号
    cs-gpios = <&gpio 0x4b 1>;                       // CS 由GPIO控制, ACTIVE_LOW
                                                      // 0x4b = 75 = GPIOX.12

    spidev@0 {
        status = "okay";                              // ← 导出 /dev/spidev0.0
        compatible = "linux,spidev";
        spi-max-frequency = <100000000>;              // 100 MHz
        reg = <0>;
    };
};
```

### 9.2 SPI1 控制器节点 (未启用)

```dts
spi@15000 {                                          // spicc1 控制器, 基址 0xffd15000
    compatible = "amlogic,meson-g12a-spicc";
    reg = <0x00 0x15000 0x00 0x44>;
    interrupts = <0x00 0x5a 0x04>;                   // 中断号 90
    clocks = <0x02 0x1d>, <0x02 0x105>;
    clock-names = "core", "pclk";
    #address-cells = <1>;
    #size-cells = <0>;
    status = "disabled";                              // ← 未启用
};
```

### 9.3 SPI Flash 控制器 (未启用)

```dts
spi@14000 {                                          // spifc 控制器 (SPI Flash专用)
    compatible = "amlogic,meson-gxbb-spifc";
    status = "disabled";                              // ← 未启用
    reg = <0x00 0x14000 0x00 0x80>;
    #address-cells = <1>;
    #size-cells = <0>;
    clocks = <0x02 0x0a>;
};
```

### 9.4 引脚复用配置 (pinmux)

```dts
// SPI 数据 + 时钟引脚组 (SCK, MOSI, MISO)
spicc0-x {
    phandle = <0x2a>;

    mux {
        groups = "spi0_mosi_x", "spi0_miso_x", "spi0_clk_x";
        function = "spi0";
        drive-strength-microamp = <4000>;            // 4mA 驱动强度
        bias-disable;                                 // 无内部上/下拉
    };
};

// SPI 片选引脚组 (CS0)
spicc0-ss0-x {
    phandle = <0x2b>;

    mux {
        groups = "spi0_ss0_x";                       // 仅 CS0 一根线
        function = "spi0";
        drive-strength-microamp = <4000>;
        bias-disable;
    };
};
```

### 9.5 物理管脚映射表

Amlogic S905X3 的 `spi0_*_x` 组全部位于 GPIOX bank，"x" 后缀表示 GPIOX 组。这是芯片内部硬连线的，不可更改。

| 设备树 Pin Group | Amlogic 引脚 | 芯片内部索引 | Odroid C4 40pin 排针 | 功能 |
|------------------|-------------|-------------|---------------------|------|
| `spi0_clk_x` | GPIOX.9 | 54 | Pin **19** | SCK (时钟) |
| `spi0_mosi_x` | GPIOX.10 | 55 | Pin **21** | MOSI (主机输出) |
| `spi0_miso_x` | GPIOX.11 | 56 | Pin **23** | MISO (主机输入) |
| `spi0_ss0_x` | GPIOX.12 | 57 (cs-gpios 用 75) | Pin **24** | CS0 (片选, 低有效) |

> **注意**: `cs-gpios` 中 `0x4b` (十进制 75) 是 periphs-pinctrl 域内的引脚序号，与 Linux 系统 `/sys/class/gpio/` 导出的全局编号不同。最终全局 GPIO 编号由内核 `gpio-ranges = <0x19 0x00 0x00 0x56>` 属性映射决定。应用层通过 `/dev/spidev0.0` 访问，无需关心 GPIO 号。

### 9.6 CS 片选的内核自动管理

```dts
cs-gpios = <&gpio 0x4b 1>;   // 参数: <GPIO控制器, 引脚序号, 有效电平标志>
                               // 0x1f = periphs-pinctrl GPIO 控制器
                               // 0x4b = 75 = periphs域内 GPIOX.12
                               // 0x01 = GPIO_ACTIVE_LOW (低电平有效)
```

当 `hardware_task_tinker` 调用 `ioctl(fd, SPI_IOC_MESSAGE, &tr)` 时，内核 `meson-g12a-spicc` 驱动自动完成：

```
1. gpiod_set_value(cs_gpio, 0)   ──→ CS 拉低 (选中 STM32)
2. spi_transfer_one_message()     ──→ 在 SCK/MOSI/MISO 上传输数据
3. gpiod_set_value(cs_gpio, 1)   ──→ CS 拉高 (释放总线)
```

**应用层代码完全不需要知道 GPIO 号**，这是 Linux SPI 子系统的标准抽象。

### 9.7 "set gpio" 日志的真实身份

从 `hardware_task_tinker` 二进制中提取的 GPIO 相关日志：

```
set PCB direction success       ← 外部 IMU/BLE 模块 AT 指令
get PCB direction: 0x[%02x]     ← 读取模块安装方向
set gpio success                 ← 模块上 GPIO 引脚功能配置
set gpio--                       ← 模块配置失败
```

这些**不是** Linux 内核 GPIO 操作。它们是对外部传感器模块 (通过 `/dev/ttyUSB0` 或 `/dev/ttyS*` 串口) 发送的 AT 配置指令，与 SPI 通信无关。证据：
- 同类日志有 `set BLE name`、`set PowerDownVoltage`、`set BaudRate`、`set accelRange` —— 典型的 IMU/BLE 模块 AT 指令集
- 二进制中不存在 `/sys/class/gpio/`、`/dev/gpiochip` 等 Linux GPIO sysfs 路径字符串
- SPI 的 CS 由内核驱动自动管理，无需应用层干预

---

## 10. 第二个共享内存通道 (MEM_CONTROL = 4999)

这是 `control_task_tinker` 与 `navigation_task_tinker` 之间的独立通道, 传送遥控指令、导航数据、SDK 外部控制命令。详细格式见 [protocols.md](protocols.md)。两个通道物理上互不干扰：

```
hardware_task ←── shm(0x0001) ──→ control_task ←── shm(4999) ──→ navigation_task
   (SPI)                           (控制)                          (UDP/日志)
```

---

## 11. SPI 帧校验 (hardware_task_tinker 内部)

从二进制字符串推断：

```
spi sum err=%d sum_cal=0x%X sum=0x%X !!    → 校验和错误
spi head err!!                               → 帧头错误
```

表明 SPI 帧包含**帧头**(magic bytes) 和**校验和**, 但具体格式因 `hardware_task_tinker` 源码缺失而不可知。

---

## 12. 源码文件索引

```
control_task2/
├── inc/spi_node.h              # 共享内存结构体定义 (SPI_RX/TX, NAV_RX/TX, MEMS)
├── src/memory_share.cpp        # 共享内存读写 + 数据序列化/反序列化 (734行)
├── src/comm.c                  # 共享内存创建/销毁工具 (createshm/getshm/destroyshm)
├── src/main.cpp                # 主入口, 线程管理
├── vmc_src/hardware_interface.cpp  # 将 SPI 数据映射到控制变量 (robotwb, vmc_all)
└── release/Build/hardware_task_tinker  # 预编译硬件驱动 (645KB, 无源码)
```
