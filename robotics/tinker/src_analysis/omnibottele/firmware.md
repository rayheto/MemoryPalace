# 固件源码分析

STM32 遥控器固件 + ESP32 显示器固件。

**路径**: `OmniBotTele/STM32/` (遥控器), `OmniBotTele/ESP32/` (显示器)

> **注意**: 机器人端 STM32 电机控制器固件 (`hardware_task_tinker`) 仅有 645KB 编译后二进制，无源码。

---

## STM32 遥控器固件

### 硬件

| 项目 | 规格 |
|---|---|
| MCU | STM32F103ZE (Cortex-M3, 512KB Flash) |
| 工程 | Keil MDK v5, 项目名 `RobTele` |
| 编译器 | ARMCC 5.06 |

### 外设

| 外设 | GPIO/接口 | 用途 |
|---|---|---|
| 2× 模拟摇杆 | ADC1 DMA (PA0-PA4, 5ch) | 4 轴 + 电池电压 |
| 6× 按钮 | PB6-PB9, PA5-PA6 | 模式选择/确认 |
| MPU9250 9轴IMU | SPI1 (CS1) | 遥控器姿态感知 |
| nRF24L01 2.4GHz | SPI1 (CS2), 2Mbps, ch44 | 32B帧 双向无线 |
| OLED 128×64 | 软 SPI (PA/PB) | 状态显示 |
| 蜂鸣器 | TIM3 CH4 PWM | 音频反馈 |
| USB VCP | USART1 @ 256kbps | PC 通信 |
| 4× LED | PB12-PB15 | 状态指示 |

### 主循环 (TIM4 任务调度器)

`TIM4` 1ms 中断生成 `flag_ms[]` 数组，`main()` 中的循环轮询:

| 周期 | 任务 | 函数 |
|---|---|---|
| 2ms | 摇杆 ADC 读取 | `getFlyDataADCValue()` |
| 5ms | 测量计时 | — |
| 10ms | 摇杆数据发送到上位机 | `Send_To_Wio()` |
| 20ms | 计时 | — |
| 25ms | 按键扫描 | `KEY_Scan()` |
| 50ms | — | — |

### nRF24L01 无线协议

无线链路: 2.4GHz, 2Mbps, 通道 44, 32 字节帧。

**发送** (遥控 → 机器人):
- 4 通道: THROTTLE, YAW, ROLL, PITCH (1000-2000us)
- 5 AUX 通道
- 按钮状态 + 标定标志 + PID 读写标志 + 校验和

**接收** (机器人 → 遥控, 8 种包类型):

| ID | 内容 |
|---|---|
| 0x01 | 姿态 (roll/pitch/yaw), 速度 (x/y/z), 位置, GPS, 锁定/模式, 电池 |
| 0x02 | 调试值 (9 int16) + RC 通道 (5 int16) |
| 0x03-06 | PID 参数 (6 组×3 值) |
| 0x07 | 任务数据 (目标 att/spd/pos) |
| 0x08 | GPS 经纬度 (double) |

连接丢失检测: `Nrf_Check_Event()` 轮询 > 200 次无应答 → "lost"

### ANO 遥测协议 (USB VCP)

**帧格式**:
- 下行 (PC→STM32): `0xAA 0xAF <func_id> <len> <data> <checksum>`
- 上行 (STM32→PC): `0xAA 0xAA <func_id> <len> <data> <checksum>`

**支持的遥测类型** (上行, 轮询发送):
- 状态 (att/高度/飞行模式/解锁)
- 传感器 (acc/gyro/mag 原始值)
- RC 数据 (10 通道)
- 电机 PWM (8 电机)
- 电源 (电压/电流)
- 速度/位置/GPS
- PID 参数 (6 组)
- 用户自定义 debug

**支持的下行指令** (PC→STM32):
- PID 参数写入 (func_id 0x10-0x15, 6 组 × 3 套 × 3 值)
- 标定指令 (0x01: acc/gyro/mag cal, 0x21-0x26: 6 面加速度 3D 标定)
- PID 读请求 (0x02)

### IMU 驱动 (mpu9250.c)

- 配置: accel ±8g, gyro ±2000dps, 磁力计 AK8963 (经 MPU 内部 I2C)
- 读取: `MPU9250_ReadValue()` SPI 突发 22 字节
- 标定: `MPU6050_Data_Offset()` 平均 50 样本 → Flash 存储 (0x0801F000)
- 滤波: 移动平均 (可配置 taps)
- 转换: 原始 gyro × 0.06103 = deg/s

### 关键源文件清单

| 文件 | 职责 |
|---|---|
| `User/main.c` | 主循环 + 任务调度 |
| `User/adc.c` | 摇杆 ADC → RC 脉宽 (1000-2000us) |
| `User/key.c` | 6 按钮扫描 + 4 LED 控制 |
| `User/mpu9250.c` | MPU9250 SPI 驱动 + 标定 + 滤波 |
| `User/rc_mine.c` | nRF24L01 无线协议编解码 (8 包类型) |
| `User/data_transfer.c` | ANO 遥测协议编解码 + PID/标定指令 |
| `User/usart.c` | USB VCP 发送 |
| `User/beep/beep.c` | 蜂鸣器 PWM 音乐 (3 八度) |
| `User/OLED/oled.c` | 128×64 OLED 驱动 + GUI 基元 |
| `User/FLASH.c` | 标定参数 Flash 存储 |
| `User/timer.c` | TIM4 1ms ISR |
| `User/nrf.c/h` | nRF24L01 SPI 底层驱动 |
| `User/fatfs/` | FATFS 文件系统 (SD 卡日志) |
| `ocu.h` | OCU 结构体定义 (STM32 + ESP32 共用) |

---

## ESP32 显示器固件

### 硬件

| 项目 | 规格 |
|---|---|
| 开发板 | ALIENTEK ESP32S3 BOX |
| MCU | ESP32-S3 |
| 显示 | 2.4" LCD 触摸 (CHSC5XXX) |
| 构建 | ESP-IDF |

### 当前状态

**基础 LVGL 演示模板**，来自 Alientek 官方示例。包含:
- LVGL GUI + FreeType 字体 (SPI Flash XBF 格式)
- LCD + 触摸驱动
- SD 卡 + NVS
- WiFi SmartConfig (代码存在但已注释)
- UART + I2C (XL9555 IO 扩展)

**未实现**机器人专用功能: UDP 通信、数据中继、遥测显示等。

### 预期架构 (推测)

```
机器人 → nRF24L01 → STM32(遥控器) → UART → ESP32
                                              ├── LVGL 显示 (机器人状态)
                                              └── WiFi → UDP:3333 → PC (udp_server.py)
```

`udp_server.py` 在 STM32 目录中监听 port 3333，`ocu.h` 定义了 ESP32 和 STM32 共用的数据结构 (`_OCU`, `_NAV_RX`, gait modes 等)。

---

## 机器人端 STM32 电机控制器 (无源码)

`hardware_task_tinker` (645KB) 是 STM32F4xx 固件，负责:
- CAN1/CAN2 电机控制 (MIT 协议, 2Mbps)
- 板载 MEMS IMU 读取
- SBUS 遥控接收
- SPI 共享内存与 Odroid C4 通信 (2000Hz)

其源码未包含在本仓库中，可能闭源或需从作者单独获取。
