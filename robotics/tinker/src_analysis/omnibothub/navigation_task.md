# navigation_task 源码分析

通信桥接任务，运行于 Odroid C4 Linux。负责外部接口 (OCU/ROS/SDK/ARM) 与内部 control_task 之间的双向数据转发。

**路径**: `OmniBotHub/Linux/navigation_task2/`

---

## 线程架构

`main()` 创建 6 个 UDP 服务线程:

| 线程 | 端口 | 周期 | 职责 |
|---|---|---|---|
| Thread_UDP_OCU | 8889 | 5ms (200Hz) | 地面站遥控指令接收 |
| Thread_UDP_SDK | 8888 | 5ms (200Hz) | ROS/SDK 高层指令接收 |
| Thread_UDP_ARM | 9000 | 5ms (200Hz) | 手臂舵机控制 |
| Thread_Mem | — | 10ms (100Hz) | 与 control_task 共享内存交换 |
| Thread_Record | — | 5ms (200Hz) | 数据日志记录 |
| Thread_MAP | 6666 | 50ms (20Hz) | 地图/里程计数据 (可选) |

---

## 与 control_task 共享内存协议

键值: `MEM_CONTROL = 4999`, 大小: 2048 字节

### 写入 (navigation_task → control_task)

`memory_write_c()` 序列化:

| 偏移 | 字段 | 类型 |
|---|---|---|
| 0 | connect | char |
| 1 | ocu_mode | char |
| 2-13 | rc_spd_b[3] (速度摇杆) | float×3 |
| 14-25 | rc_rate_b[3] (姿态摇杆) | float×3 |
| 26-65 | key_ud/key_lr/key_x/key_y/key_a/key_b/key_ll/key_rr/key_st/key_back | char×10 (+1 偏移编码) |
| 66 | request_gait | char |
| 67-78 | exp_spd_o[3] (SDK 期望速度) | float×3 |
| 79-90 | exp_att_o[3] (SDK 期望姿态) | float×3 |
| 91-102 | exp_datt_o[3] (SDK 期望角速度) | float×3 |
| 103-114 | exp_pos_o[3] (SDK 期望位置) | float×3 |
| 115-162 | exp_q_o[4][3] (4 腿期望关节角) | float×12 |
| 163-198 | exp_GRF_o[4][3] (4 腿期望 GRF) | float×12 |
| 199-210 | imu_pos[3] | float×3 |
| 211-222 | imu_att[3] | float×3 |
| 223-234 | gps_pos[3] | float×3 |
| 235-237 | Acc/Gyro/Mag_CALIBRATE | char×3 |

### 读取 (navigation_task ← control_task)

接收: 姿态 (att_now/tar), 速度 (dcom_n), CoM 位置, GRF 估计, 接地姿态, 14 电机完整状态 (q_set/q_now/tau), 步态/模式

---

## UDP 协议

### OCU (地面站, port 8889)

**接收** 自定义二进制帧 → 解析为 `_OCU` 结构体:
- Head1/Head2: 帧头固定值
- gait_mode: 步态指令
- att_cmd/vel_cmd/pos_cmd: 姿态/速度/位置指令
- 校准命令: 电机校零 (10/11), 手臂校零 (20/21), IMU 校准 (30)

**反馈** 机器人状态 (姿态/速度/电机位置/舵机位置/心跳)

### SDK (ROS 桥接, port 8888)

**协议**: 自定义结构体 `_SDK`:
```cpp
struct _SDK {
    float rc_spd_cmd[3];     // 速度指令 [Xr, Yr, Zr]
    float rc_att_cmd[3];     // 姿态指令 [PITr, ROLr, YAWr]
    float rc_att_rate_cmd[3]; // 姿态角速度
    float rc_pos_cmd[3];     // 位置指令
    char gait_mode;          // 步态模式
    char ros_connect;        // ROS 连接标志
};
```

`gait_mode` 特殊值:
- 100: 启用 SDK 命令
- 101: 禁用 SDK 命令
- 102: 强制安全模式

**反馈**: 机器人完整状态 (同 OCU 反馈格式)

### ARM 控制 (port 9000)

手臂舵机控制，自定义二进制协议。

---

## 数据记录 (Thread_Record)

每 5ms 写入 `/home/odroid/Tinker/Data/`:

| 文件 | 内容 |
|---|---|
| file1.txt | 姿态 (att), 期望姿态 (att_tar), 角速度 (datt) |
| file2.txt | CoM 位置/速度, GRF (4 腿 × 3 轴) |
| file3.txt | 14 关节角度, 力矩, 控制模式 |

用途: 离线调试、性能分析、故障诊断

---

## 参数配置

从 `/home/odroid/Tinker/Param/param_hardware.yaml` 读取:
- IMU 零偏 (gyro_bias)
- IMU 安装姿态 (imu_set_att)
- USB IMU 使能标志

---

## 源文件清单

| 文件 | 行数 (估) | 职责 |
|---|---|---|
| `src/main.cpp` | ~900 | 入口, 共享内存交换, 线程创建, 数据记录 |
| `src/udp_ocu.cpp` | ~400 | OCU UDP 服务 (port 8889), 地面站协议 |
| `src/udp_sdk.cpp` | ~300 | SDK/ROS UDP 服务 (port 8888) |
| `src/udp_arm.cpp` | ~200 | ARM 舵机 UDP 服务 (port 9000) |
| `src/sdk_api.cpp` | ~200 | SDK 指令解析与映射 |
| `src/sys_timer.cpp` | ~50 | 系统定时器 |
| `src/comm.c` | ~100 | 通信工具函数 |
| `inc/base_struct.h` | — | 基础数据结构定义 |
| `inc/mem_node.h` | — | 共享内存节点定义 |
| `inc/udp_ocu.h` | — | OCU 协议结构体 |
| `inc/comm.h` | — | 通信公共定义 |
| `inc/can.h` | — | CAN 协议定义 (共用) |

---

## 数据流汇总

```
外部输入:
  OCU 地面站      ──UDP:8889──→ navigation_task
  ROS/SDK         ──UDP:8888──→ navigation_task
  ARM 控制         ──UDP:9000──→ navigation_task
  control_task    ←──共享内存──→ navigation_task (姿态/电机/CoM 状态)
  
navigation_task 输出:
  → 共享内存: 遥控指令, SDK 速度/姿态/位置, IMU 标定
  → 文件: /home/odroid/Tinker/Data/ (调试日志)
  ← OCU/SDK/ARM UDP 反馈: 机器人完整状态
```
