# OmniBotHub — 机载软件

Odroid C4 上运行的实时控制与通信软件。基于 Linux PREEMPT_RT，7 线程架构。

## 文件索引

| 文档 | 内容 |
|---|---|
| [control_task.md](control_task.md) | 实时控制器 (C++, 500Hz), 线程架构, 步态状态机, TVM RL 推理, SPI 共享内存, 力/阻抗控制 |
| [navigation_task.md](navigation_task.md) | 通信桥接 (C++, 6 线程), OCU/SDK/ARM UDP 服务, 数据日志 |
| [protocols.md](protocols.md) | SPI(2048B 全双工帧), CAN(MIT Mini Cheetah), 共享内存(MEM_SPI/MEM_CONTROL), UDP(5 端口) |

## 源码路径

```
OmniBotHub/Linux/
├── release/Build/              # 预编译二进制
│   ├── hardware_task_tinker     # STM32 电机固件 (645KB, 无源码)
│   ├── control_task_tinker      # 实时控制 (586KB)
│   └── navigation_task_tinker   # 通信桥接 (402KB)
├── control_task2/               # 实时控制源码
│   ├── src/                     # main, memory_share, robot_param, can, comm, sys_timer
│   ├── inc/                     # can.h, comm.h, spi_node.h, sys_time.h
│   ├── vmc_src/                 # locomotion_sfm, force_imp_controller, hardware_interface, sdk_api
│   └── gait_src/               # rl.cpp, stand.cpp, self_right.cpp
├── navigation_task2/            # 通信桥接源码
│   ├── src/                     # main, udp_ocu, udp_sdk, udp_arm, sdk_api
│   └── inc/                     # base_struct, mem_node, udp_ocu, comm, can
├── download_tinker.sh           # SCP 部署 task 到 Odroid
└── download_param_tinker.sh     # SCP 部署 YAML 参数
```
