# control_task_tinker — `mpc_locomotion/` & `vision_location/` 预留模块（5f）

## 现状：仅 CMake 占位，仓库内**目录与源码均不存在**

CMakeLists.txt 中两段引用：

```cmake
# CMakeLists.txt:73
include_directories(
     "./build"
     "./inc"
     "./math_inc"
     "./vmc_inc"
     "./mpc_locomotion"      # ← 头文件搜索路径（目录不存在）
     "/usr/local/include"
)

# CMakeLists.txt:85-86
AUX_SOURCE_DIRECTORY(mpc_locomotion DIR_SRCS5)
AUX_SOURCE_DIRECTORY(vision_location DIR_SRCS6)

# CMakeLists.txt:88
ADD_EXECUTABLE(${PROJECT_NAME}
    ${DIR_SRCS} ${DIR_SRCS1} ${DIR_SRCS2} ${DIR_SRCS3}
    ${DIR_SRCS4} ${DIR_SRCS5} ${DIR_SRCS6})
```

参考 [CMakeLists.txt:73](CMakeLists.txt#L73)、[CMakeLists.txt:85-88](CMakeLists.txt#L85-L88)。

由于 `AUX_SOURCE_DIRECTORY` 对**不存在或为空**的目录返回空列表，且 CMake 在 `add_executable` 不要求所有路径预先存在，构建系统不会因此失败——这两段在当前版本是**无害的预占位（dead reservation）**。

`build/.cmake/api/v1/reply/target-control_task_tinker-*.json` 中 source paths 含 `"./mpc_locomotion"`，但实际编译单元为零，可由 `build/build.ninja` 中无任何 `mpc_locomotion/*.cpp.o` 目标验证。

---

## 一、`mpc_locomotion/` — MPC 运动控制（预留）

**预期功能**（基于命名与项目体系推断）：

| 候选职责 | 说明 |
|----------|------|
| 模型预测控制器 | 基于 SRBD/质心动力学（Single Rigid Body Dynamics）的滚动时域优化 |
| QP 求解 | 使用本仓库自带的 `qpOASES/`（静态库已编入主可执行文件，[CMakeLists.txt:94](CMakeLists.txt#L94)），用于足端力分配 |
| 替代 / 协同 VMC | 当前主控由 `vmc_src/locomotion_sfm.cpp` 的虚拟模型控制驱动；MPC 模块预计提供更优的足端反力分配方案，可与 VMC 共存或在特定步态模式下接管 |
| 衔接 | 与 `vmc_inc/gait_math.h` 中的雅可比/坐标变换、`math_src/RT_math.cpp`（Simulink Coder 代码）联动 |

**佐证**：
1. 仓库根目录已携带 [qpOASES/](qpOASES/) 子项目，链接到主可执行文件但目前仅用于潜在 MPC——`vmc_src/` 内并未调用其求解器；
2. `loong_ctrl/` 核心库（动态库 `tinker_arm64.so`）已链接 `libtvm_runtime.so`，参见 [CMakeLists.txt:64-66](CMakeLists.txt#L64-L66)，整套基础设施（QP、TVM）已具备实现 MPC 的条件；
3. `vmc_inc/` 中的 `base_struct.h` 含 `RobotData / VMC` 状态字段（机身位姿、足端力期望），可直接被 MPC 状态空间复用。

---

## 二、`vision_location/` — 视觉定位（预留）

**预期功能**：

| 候选职责 | 说明 |
|----------|------|
| 视觉里程计 / SLAM | 摄像头/深度相机驱动 + 位姿估计 |
| 地形感知 | 用于 RL 步态或 MPC 中的地形地图生成 |
| 与 navigation_task 衔接 | `navigation_task.md` 描述了独立的导航进程；本目录可能是其客户端或互补，把视觉结果通过 IPC 注入 `vmc_all` |
| 标志/二维码定位 | 室内场景下基于固定标志的定位 |

**仓库内可对接位置**：
- `src/memory_share.cpp` 已实现导航命令共享内存（`MEM_CONTROL`），视觉模块可作为生产者；
- 当前 `vmc_all` 状态结构未见视觉融合字段，新增需修改 `vmc_inc/base_struct.h`。

---

## 三、与现役模块的对照

```
                   [当前已实现]                    [预留 / 未来]
                                                 ┌────────────────┐
                                                 │ vision_location │
                                                 │ 视觉定位        │
                                                 └────────┬───────┘
                                                          │ pose / map
                                                          ▼
   src/  ──状态──►  vmc_src/  ─────────────────────► (vmc_all)
   (IMU/CAN/IPC)    (VMC 主控)                          ▲
                       │                                │ 期望足端力
                       ▼                                │
                   gait_src/                    ┌────────────────┐
                   (步态行为)                    │ mpc_locomotion  │
                                                 │ MPC 力分配      │
                                                 └─────┬───────────┘
                                                       │  qpOASES (已就绪)
                                                       ▼
                                                 GRF 期望 → vmc_src
```

---

## 四、结论与启用门槛

启用任一目录需要的最小步骤：

1. 在仓库根创建 `mpc_locomotion/` 或 `vision_location/` 目录；
2. 放入 `.cpp/.h` 源文件——CMake 无需修改（已 `AUX_SOURCE_DIRECTORY` 收录）；
3. 在 `vmc_src/locomotion_sfm.cpp` 主状态机的 `gait_mode` 分支添加调用钩子；
4. 视需要扩展 `base_struct.h` 中 `vmc_all` 状态字段。

依赖项已就绪：qpOASES（MPC）、Eigen3、libtvm_runtime.so（深度学习推理）、共享内存 IPC 框架。

> 与 [module-architecture.md](module-architecture.md) §5f 的判断一致：「目录存在但无源码，为未来 MPC 运动控制和视觉定位功能预留」——实际现状是**目录不存在**，仅 CMake 中存在占位。
