# OmniBotTele — 遥控系统

双向无线遥控器。STM32 作为遥测桥，ESP32 作为显示终端。

## 文件索引

| 文档 | 内容 |
|---|---|
| [firmware.md](firmware.md) | STM32F103ZE 遥控器 (nRF24L01, MPU9250, ANO 协议), ESP32 LVGL 显示, 电机固件说明 |

## 源码路径

```
OmniBotTele/
├── STM32/                       # 遥控器固件 (Keil MDK)
│   ├── User/
│   │   ├── main.c               # 主循环 + TIM4 任务调度
│   │   ├── adc.c                # 摇杆 ADC → 1000-2000us
│   │   ├── key.c                # 6 按钮 + 4 LED
│   │   ├── mpu9250.c            # 9 轴 IMU 驱动 + 标定
│   │   ├── rc_mine.c            # nRF24L01 无线协议 (8 包类型)
│   │   ├── data_transfer.c      # ANO 遥测协议
│   │   ├── usart.c              # USB VCP 发送
│   │   ├── beep/beep.c          # 蜂鸣器 PWM 音乐
│   │   ├── OLED/oled.c          # 128×64 显示驱动
│   │   ├── FLASH.c              # 标定参数存储
│   │   ├── timer.c              # TIM4 1ms ISR
│   │   ├── nrf.c/h              # nRF24L01 SPI 驱动
│   │   └── fatfs/               # SD 卡文件系统
│   ├── Project/RVMDK（uv4）/     # Keil 工程文件
│   ├── Libraries/               # STM32 标准外设库
│   ├── ocu.h                    # OCU 结构体定义
│   └── udp_server.py            # Python UDP 接收 (port 3333)
├── ESP32/                       # 显示器固件 (ESP-IDF)
│   ├── main/app_main.c          # LVGL 演示主程序
│   └── components/              # BSP 驱动 (LCD/SD/Touch/WiFi)
└── PCB/                         # 遥控器 PCB (Altium Designer)
    ├── OmniBotRemote.SchDoc     # 原理图
    ├── OmniBotRemote.PcbDoc     # PCB 版图
    └── usb_convert.*            # USB 转接板
```
