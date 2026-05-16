# control_task_tinker — `src/` 主控入口与基础设施模块（5a）

## 概览

`src/` 是 control_task_tinker 可执行文件的**主入口与基础设施层**：负责进程初始化、线程编排、共享内存 IPC、CAN 总线驱动、YAML 配置加载、以及对接上层 VMC（`vmc_src/`）和下层伺服板（STM32）。该模块不实现控制律本身，而是把硬件 I/O、网络、配置、定时全部接到 VMC 主循环上。

| 文件 | 行数 | 功能定位 |
|------|------|----------|
| `main.cpp` | 969 | 入口；6 个并发线程编排 |
| `memory_share.cpp` | 733 | 共享内存 IPC（与 STM32 / 导航进程通信） |
| `robot_param.cpp` | 263 | YAML 配置加载到全局结构 |
| `can.cpp` | 43 | CAN 电机驱动初始化（结构体定义） |
| `sys_timer.cpp` | 55 | 微秒精度多路计时器 |
| `comm.c` | 30 | POSIX SysV IPC 共享内存包装器（C 语言） |

---

## 一、`main.cpp` — 入口与线程编排

实时控制系统的核心入口。把所有外设、IPC、网络通过**多线程**接到 VMC 主控循环上。

### 1.1 主要线程

| 线程 | 周期 / 触发 | 调度 | 主要职责 | 入口函数 |
|------|------------|------|---------|---------|
| **Thread_T1** | 500 Hz（2 ms） | `SCHED_FIFO`，高优先级 | VMC 控制主循环 — 读 IMU/电机状态 → 调用 `locomotion_sfm()` → 写关节扭矩/角度 | 见 [main.cpp](src/main.cpp) |
| **Thread_T5** | 200 Hz（5 ms） | 普通 | 遥控器/SDK 命令融合（`rc_process`），更新 `vmc_all.tar_spd` 等命令字段 | 见 [main.cpp](src/main.cpp) |
| **Thread_Mem_Servo** | 2 kHz（0.5 ms） | 高优先级 | 硬件 IPC（STM32 ↔ 主控）：通过共享内存 `MEM_SPI` 读 IMU/电机反馈、写电机指令 | 见 [main.cpp](src/main.cpp) |
| **Thread_Mem_Navigation** | 100 Hz（10 ms） | 普通 | 导航 IPC：通过共享内存 `MEM_CONTROL` 接收规划层的速度/COM 命令 | 见 [main.cpp](src/main.cpp) |
| **Thread_UDP_RL_Tinker** | 事件驱动；50 Hz observation 推送 | 普通 | RL 策略 UDP 服务器（监听端口 **10000**）；接收 10 维动作、推送观测向量 | 见 [main.cpp](src/main.cpp) |
| **Thread_UDP_OCU** | 事件驱动 | 普通 | ESP32 无线遥控器（监听端口 **7070**）接收按键/摇杆 | 见 [main.cpp](src/main.cpp) |

### 1.2 主循环（Thread_T1）调用骨架

每个 500 Hz 周期内：

1. **读取状态**：从 `vmc_all` 和 `leg_motor_all` 取 IMU、关节当前角速度等（由 Thread_Mem_Servo 异步刷新）；
2. **命令融合**：使用 Thread_T5 与 Thread_UDP_OCU 写入的 `ocu.cmd_*` 字段；
3. **状态机分派**：调用 `locomotion_sfm()` → `vmc_src/locomotion_sfm.cpp` 内部按 `vmc_all.gait_mode` 切换到 stand / RL / recovery 等步态；
4. **力/位置控制**：步态层产出 `q_set[]`，VMC 通过 `force_control_and_dis_*()` 转成电机扭矩；
5. **写回**：通过共享内存把指令交给 Thread_Mem_Servo 推送到 STM32。

### 1.3 启动顺序

main() 启动后大致流程：
1. 调用 `robot_param_read()` → 加载 YAML（[robot_param.cpp:11](src/robot_param.cpp#L11)）；
2. 创建共享内存段（`shm_servo_init()` / `shm_navigation_init()` 等，见 `memory_share.cpp`）；
3. 启动 6 个线程并设置 CPU affinity、`SCHED_FIFO` 优先级；
4. 主线程进入空循环或处理信号。

---

## 二、`memory_share.cpp` — 共享内存 IPC

实现两条独立的 IPC 通道，分别用于**硬件实时**和**导航/规划**层。

### 2.1 通道一：`MEM_SPI`（硬件层）

**用途**：与 STM32 硬件控制板（伺服板）双向交换数据。本系统的"硬件抽象"——上层无需关心 SPI/UART 协议，只通过共享内存读写。

**数据载荷（约 420 字节）**：
- **IMU 数据**（12 floats）：roll/pitch/yaw 角、角速率、加速度；
- **14 个关节状态**（每关节 12 字节）：current pos / vel / torque 等；
- **遥控器原始状态**（按键、摇杆）；
- **错误码与时间戳**。

**双缓冲 + 标志位同步**：避免读写竞争。Thread_Mem_Servo 以 2 kHz 拉新数据。

### 2.2 通道二：`MEM_CONTROL`（导航层）

**用途**：与外部导航/规划进程（一般是另一个可执行 `navigation_task`）通信。

**数据载荷（约 500 字节）**：
- 电机命令、COM 期望轨迹、GRF 期望、机身姿态命令、模式切换请求。

由 Thread_Mem_Navigation 以 100 Hz 同步。

### 2.3 实现机制

底层使用 SysV IPC（`shmget/shmat/shmctl`），封装在 `src/comm.c` 中：
- `createshm(key, size)` — 创建共享段；
- `getshm(key)` — 附加到已有段；
- `destroyshm(key)` — 销毁段。

见 [comm.c](src/comm.c)、对应头文件 [inc/comm.h](inc/comm.h)。

---

## 三、`robot_param.cpp` — YAML 配置加载

### 3.1 硬编码路径

```cpp
// src/robot_param.cpp:8-9
YAML::Node config_robot=YAML::LoadFile("/home/odroid/Tinker/Param/param_robot.yaml");
YAML::Node config_gait=YAML::LoadFile("/home/odroid/Tinker/Param/param_gait.yaml");
```

参考 [robot_param.cpp:8-9](src/robot_param.cpp#L8-L9)。**全局对象构造期**完成解析（即 main() 进入前）。

### 3.2 入口函数

`robot_param_read()`（[robot_param.cpp:11](src/robot_param.cpp#L11)）— 由 main 启动初始化阶段调用，把 YAML 数据搬到三个全局结构：

| 全局对象 | 字段类型 | 关键内容 |
|---------|---------|---------|
| `leg_motor_all` | 多组 `[14]` 数组 | `q_init/q_max/q_min/q_reset/kp/kd/stiff/q_set_servo_*` |
| `vmc_all` | 控制全局 | `default_action[14]`、`net_run_dt`、`action_scale`、`att_measure_bias[]`、`rl_commond_off[]` |
| `gait_ww` | 系统开关 | `auto_switch`、`auto_gait_time`、`auto_zmp_st_check`、`auto_mess_est` |

详见配套文档 [module-param.md](module-param.md)。

### 3.3 加载顺序（示意）

```
robot_param_read():
  17-20:  sys_param   → gait_ww
  22-24:  vmc_param   → MAX_SPD_X/Y/RAD
  27-77:  imp_param   → leg_motor_all.{stiff_init, kp[], kd[], stiff[]}
  78-92:  kin_param   → leg_motor_all.q_init[]
  111-125: kin_param  → leg_motor_all.q_reset[]
  127-141: kin_param  → leg_motor_all.q_max[]
  143-157: kin_param  → leg_motor_all.q_min[]
  160-222: servo_param → leg_motor_all.q_set_servo_*[]
  230-251: rl_gait    → vmc_all.{net_run_dt, action_scale, rl_commond_off[], default_action[]}
  255-260: vmc_param  → vmc_all.att_measure_bias{,_flip}[]
```

---

## 四、`can.cpp` — CAN 电机驱动初始化

虽名为 CAN，但本文件实际**不直接驱动 CAN 硬件**（CAN 收发由 STM32 板完成，通过共享内存接入）；它的职责是声明电机数据结构、设置电机控制模式与上限。

### 4.1 数据结构

声明在 `inc/can.h`：

- `_LEG_MOTOR`：单条腿（3 个电机，对应髋/膝/踝），包含目标扭矩、当前角度等字段；
- `_LEG_MOTOR_ALL`：14 自由度全身集合（左右腿 × 7 + 上身 servo 通道）+ kp/kd/stiff 表 + 初始/限位/复位角表。

### 4.2 电机模式初始化

```cpp
// can.cpp 中（伪代码）
for each leg (4 legs × 3 motors):
    set MOTOR_MODE_T          // 力矩控制模式
    set max torque  = 3 N·m
    set max current = 25 A
```

最终通过 `MEM_SPI` 推送给 STM32 执行真正的 CAN 发送。

---

## 五、`sys_timer.cpp` — 多路微秒计时器

实现 **100 个独立计时器**，每个支持微秒精度。

- 底层：`gettimeofday()`；
- API：复位、读取已耗时（秒/毫秒/微秒）等；
- 用途：步态切换计时（如 `timer_auto_switch`）、模式滞留计时、防抖判定。

---

## 六、`comm.c` — POSIX SysV IPC 包装

唯一一个 C 语言文件（其余都是 C++）。提供：

```c
int  createshm(key_t key, size_t size);
void *getshm(key_t key);
int  destroyshm(int shmid);
```

被 `memory_share.cpp` 调用以创建/获取硬件、导航两条共享内存段。

---

## 七、模块交互拓扑

```
                       ┌──────────────────┐
                       │   YAML 配置文件   │
                       │ /home/odroid/    │
                       │  Tinker/Param/   │
                       └────────┬─────────┘
                                │ robot_param_read()
                                ▼
        ┌─────────────────────────────────────────────────┐
        │  全局状态：vmc_all / leg_motor_all / gait_ww    │
        └──────────┬──────────────────────┬───────────────┘
                   │                      │
   ┌───────────────┴────────┐   ┌────────┴───────────┐
   │ Thread_T1 (500 Hz)     │   │ Thread_Mem_Servo   │
   │ VMC 主循环             │   │ (2 kHz, MEM_SPI)   │
   │  └─► vmc_src/          │   │  ↔ STM32 伺服板    │
   │      locomotion_sfm()  │   └─────────┬──────────┘
   └────────┬───────────────┘             │ IMU/电机反馈
            │ 状态输出                     │ 电机命令
            ▼                              ▼
   ┌──────────────────────────────────────────────────┐
   │            共享内存 (SysV IPC, comm.c)            │
   ├──────────────────────────────────────────────────┤
   │ MEM_SPI    (硬件↔主控, ~420B)                     │
   │ MEM_CONTROL (导航↔主控, ~500B)                    │
   └──────────────────────────────────────────────────┘
            ▲                              ▲
            │                              │
   ┌────────┴────────┐          ┌─────────┴─────────┐
   │ Thread_Mem_Nav  │          │ Thread_UDP_*      │
   │ (100 Hz)        │          │  - RL :10000      │
   │  ↔ 导航进程     │          │  - OCU :7070      │
   └─────────────────┘          └───────────────────┘
```

---

## 八、关键端口与外部接口

| 接口 | 协议 | 端口 / 路径 | 对端 | 频率 |
|------|------|------------|------|------|
| MEM_SPI | SysV IPC 共享内存 | 内核 key | STM32 伺服板进程 | 2 kHz |
| MEM_CONTROL | SysV IPC 共享内存 | 内核 key | `navigation_task` 进程 | 100 Hz |
| RL UDP | UDP | 0.0.0.0:10000 | RL 策略进程 / Sim | 推送 50 Hz；接收按需 |
| OCU UDP | UDP | 0.0.0.0:7070 | ESP32 遥控器 | 按键事件 |
| YAML 配置 | 文件 | `/home/odroid/Tinker/Param/*.yaml` | yaml-cpp 加载 | 启动一次 |

---

## 九、与其他模块的依赖关系

| 调用方向 | 调用方 | 被调方 |
|---------|--------|--------|
| `src/main.cpp` → `vmc_src/locomotion_sfm.cpp` | 主循环 | VMC 状态机 |
| `src/robot_param.cpp` → 全局对象 | 启动初始化 | 被所有模块共享读 |
| `src/memory_share.cpp` ↔ STM32 / 导航进程 | IPC | 硬件 I/O 抽象 |
| `vmc_src/`、`gait_src/` ← `inc/can.h` | 头文件包含 | 电机数据结构 |
| 所有模块 ← `inc/sys_time.h` | 头文件包含 | 计时器 |

`src/` 是**唯一**直接接触线程、网络套接字、共享内存系统调用的模块；其他模块全部走全局变量。
