# control_task_tinker 完整模块架构分析

## 项目概览

双足/四足机器人实时运动控制系统，目标平台 Linux arm64（NVIDIA Jetson），C++11/O3 编译。基于上海交通大学 Nabo 框架。

---

## 一、构建产物与依赖关系图

```
                    ┌─────────────┐
                    │  Eigen3     │  (仅头文件，不编译)
                    │ loong_utility/eigen3/ │
                    └─────────────┘
                           │ 被所有模块引用

     ┌──────────────────┬──┴───┬──────────────────┐
     │                  │      │                  │
     v                  v      v                  v
┌─────────┐     ┌──────────┐  ┌──────────┐  ┌──────────────┐
│  util   │     │ yaml-cpp │  │ qpOASES  │  │ tvm_runtime  │
│ STATIC  │     │ STATIC   │  │ STATIC   │  │  (预编译.so) │
└────┬────┘     └────┬─────┘  └────┬─────┘  └──────┬───────┘
     │               │             │               │
     │               │             │               v
     │               │             │      ┌─────────────────┐
     │               │             │      │ tinker_arm64    │
     │               │             │      │  (loong_ctrl)   │
     │               │             │      │ SHARED .so      │
     │               │             │      │ tvm.cpp/tvm2.cpp│
     │               │             │      └────────┬────────┘
     │               │             │               │
     v               v             v               v
     └───────────────┴─────────────┴───────────────┘
                         │
                         v
           ┌─────────────────────────────┐
           │   control_task_tinker       │
           │   EXECUTABLE                │
           │   (6 个源码模块合编)         │
           └─────────────────────────────┘
```

### 链接关系表

| 目标 | 类型 | 链接依赖 |
|------|------|---------|
| `util` | 静态库 | 无 |
| `yaml-cpp` | 静态库 | 无 |
| `qpOASES` | 静态库 | 无 |
| `tinker_arm64` | 动态库(.so) | `util` + `libtvm_runtime.so` + `pthread` |
| `control_task_tinker` | 可执行文件 | `tinker_arm64` + `qpOASES` + `yaml-cpp` |

---

## 二、各模块详解

### 1. `util` — 基础工具库（静态库）

**源码目录：** `loong_utility/`

| 文件 | 功能 |
|------|------|
| `algorithms.cpp/.h` | 值裁剪(clamp)、边界检查 |
| `ini.cpp/.h` | INI 配置文件解析 |
| `timing.cpp/.h` | 高精度计时 |
| `eigen.h` | Eigen3 类型别名(Matrix3d, Vector3d 等) |
| `iopack.h/.hxx` | 序列化/I/O 打包 |
| `pInv.h` | 伪逆计算 |

对外无依赖，是 Nabo 框架的工具层。

---

### 2. `tinker_arm64` — 机器人控制核心库（动态库）

**源码目录：** `loong_ctrl/src/`

| 文件 | 功能 |
|------|------|
| `rl/tvm.cpp/.h` | TVM 运行时封装类 `tvmClass`，加载/执行 TVM 编译的神经网络模型 |
| `rl/tvm2.cpp/.h` | TVM 运行时封装类 `tvm2Class`，第二版接口，RL 步态实际使用 |
| `rl/thread_loop.h` | 线程循环基础类模板，用于周期性实时控制线程 |
| `manager/nabo_config.h` | Nabo 框架全局配置（机器人常量：5-DOF腿、14电机、关节索引等） |

**依赖：**
- 链接 `util`（静态）
- 链接 `libtvm_runtime.so`（预编译，TVM 推理运行时）
- 链接 `pthread`（系统）
- 头文件引用 `pinocchio/`、`boost_parts/`（预编译在 `loong_ctrl/third_party/` 中）

**角色：** "OpenLoong" 框架的核心——将 PyTorch 训练好的 RL 策略模型通过 TVM 编译后，在此模块中加载并在实时控制循环中推理。

---

### 3. `qpOASES` — QP 求解器（静态库）

**源码目录：** `qpOASES/src/`（19个 .cpp）

在线 Active-Set 策略的二次规划求解器(v3.2.0)。用于足式机器人足底力分配的 QP 优化。

无外部依赖，完全自包含。

---

### 4. `yaml-cpp` — YAML 解析器（静态库）

**源码目录：** `yaml-cpp/src/`

解析 `param_robot.yaml`（机器人物理参数）和 `param_gait.yaml`（步态控制参数）。

无外部依赖。

---

### 5. `control_task_tinker` — 主可执行文件

合编 6 个子模块，所有 .cpp 直接编译进可执行文件。

#### 5a. `src/` — 主控入口与基础设施

| 文件 | 功能 |
|------|------|
| `main.cpp` | **入口**，线程管理（控制线程、伺服线程、MPC 线程、导航线程） |
| `can.cpp` | CAN 总线通信（与电机驱动器通信） |
| `comm.c` | 通信协议（C 语言） |
| `memory_share.cpp` | 共享内存 IPC（进程间通信） |
| `robot_param.cpp` | 加载 YAML 配置（调用 yaml-cpp） |
| `sys_timer.cpp` | 系统定时器 |

#### 5b. `math_src/` — 数学计算层

| 文件 | 功能 |
|------|------|
| `common_math.cpp` | 通用数学函数 |
| `kin_math.cpp` | **运动学**（正/逆运动学，最大文件约36K） |
| `fliter_math.cpp` | 滤波器 |
| `RT_math.cpp` | Simulink Coder 生成的实时数学代码 |
| `bezier_math.cpp` | Bezier 曲线（足端轨迹规划） |
| `imp_math.cpp` | **阻抗控制**数学 |
| `eso.cpp` | **扩展状态观测器** (Extended State Observer) |
| `eso_RL.cpp` | RL 专用的 ESO 变体 |
| `rtGetInf/NaN/rt_nonfinite.cpp` | Simulink RT 代码生成工具函数 |

#### 5c. `vmc_src/` — 运动控制层

| 文件 | 功能 |
|------|------|
| `locomotion_sfm.cpp` | **VMC 运动状态机**（最大核心文件约40K） |
| `hardware_interface.cpp` | 硬件/IMU/传感器接口（约35K） |
| `force_imp_controller.cpp` | 力/阻抗混合控制器 |
| `sdk_api.cpp` | SDK 远程控制 API（大部分为空桩） |

#### 5d. `gait_src/` — 步态行为层

| 文件 | 功能 |
|------|------|
| `stand.cpp` | 站立步态 |
| `rl.cpp` | **RL 强化学习步态**（调用 `tvm2.h`，条件编译 `RL_USE_TVM`） |
| `self_right.cpp` | 自恢复行为（摔倒后站起） |

#### 5e. `Param_Tinker14/` 等 — 配置文件目录

| 目录 | 用途 |
|------|------|
| `Param_Tinker14/` | Tinker14 机器人参数（param_robot.yaml, param_gait.yaml） |
| `Param_Tinker12/` | Tinker12 变体 |
| `Param_Tinker14_Zero/` | Tinker14 Zero 变体 |
| `Param_Taitan14_Zero/` | Taitan14 变体 |

配置在运行时从 `/home/odroid/Tinker/Param/` 加载（硬编码路径），不参与编译。

#### 5f. `mpc_locomotion/` & `vision_location/` — 预留空目录

目录存在但无源码，为未来 MPC 运动控制和视觉定位功能预留。

---

## 三、完整调用层次

```
main.cpp (线程管理)
  │
  ├─ robot_param.cpp  ←── yaml-cpp  ←── .yaml 配置文件
  ├─ can.cpp          ←── CAN 总线 → 电机驱动器
  ├─ memory_share.cpp ←── 共享内存 → 其他进程
  │
  └─ 控制循环 ──→ locomotion_sfm.cpp (VMC 状态机)
                      │
                      ├─→ kin_math.cpp (运动学计算)
                      ├─→ bezier_math.cpp (足端轨迹)
                      ├─→ imp_math.cpp (阻抗控制)
                      ├─→ eso.cpp / eso_RL.cpp (状态估计)
                      ├─→ force_imp_controller.cpp (力控)
                      ├─→ stand.cpp / rl.cpp / self_right.cpp (步态)
                      │       │
                      │       └─→ tvm2.cpp (RL推理) ──→ libtvm_runtime.so
                      │
                      └─→ hardware_interface.cpp (传感器读取)
```

---

## 四、外部预编译依赖

位置：`loong_ctrl/third_party/lib_lin_arm64/`

| 库 | 用途 |
|----|------|
| `libtvm_runtime.so` | Apache TVM 运行时（神经网络推理） |
| `libpinocchio.a` | 刚体动力学库（正/逆运动学、动力学） |
| `libEiQP.a` | 高效 QP 求解器（备选） |
| `liburdfdom_model.a` | URDF 模型解析 |
| `libquill.a` | 异步日志库 |
| `libcuda.so` / `libcudart.so` (桩) | CUDA STUB（Jetson Tegra 平台） |

系统依赖：`libpthread.so`、`libdl.so`

---

## 五、目录树总览

```
control_task2/
  CMakeLists.txt                    <-- 顶层构建
  src/                              <-- 主入口 + 基础设施 (6 .cpp)
  inc/                              <-- src/ 对应的头文件
  math_src/                         <-- 数学计算实现 (11 .cpp)
  math_inc/                         <-- 数学头文件
  vmc_src/                          <-- VMC 运动控制 (4 .cpp)
  vmc_inc/                          <-- VMC 头文件
  gait_src/                         <-- 步态行为 (3 .cpp)
  model/                            <-- 神经网络模型文件 (.pt, .so)
  loong_ctrl/                       <-- "OpenLoong" 控制框架
    src/manager/                    <-- nabo_config.h
    src/rl/                         <-- TVM RL 推理封装 (tvm.cpp, tvm2.cpp, thread_loop.h)
    third_party/include/            <-- 头文件: boost_parts, pinocchio, tvm
    third_party/lib_lin_arm64/      <-- 预编译 arm64 库 + cuda 桩库
  loong_utility/                    <-- 工具库 (algorithms, ini, timing, iopack, pInv, eigen)
    eigen3/                         <-- Eigen3 unsupported 模块
  qpOASES/                          <-- qpOASES QP 求解器 v3.2.0
  yaml-cpp/                         <-- yaml-cpp YAML 解析器 v0.7.0
  eigen/                            <-- Eigen3 完整源码 (不编译)
  Param_Tinker14/                   <-- Tinker14 机器人配置
  Param_Tinker14_Zero/              <-- Tinker14 Zero 变体配置
  Param_Tinker12/                   <-- Tinker12 变体配置
  Param_Taitan14_Zero/              <-- Taitan14 变体配置
  mpc_locomotion/                   <-- 空目录 (未来 MPC)
  vision_location/                  <-- 空目录 (未来视觉定位)
```
