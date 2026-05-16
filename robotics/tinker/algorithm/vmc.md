# 虚拟模型控制（VMC, Virtual Model Control）

> **主要参考**：[飞书文档 SvrYwWVnGiwH7ikjb5EcQYBOnab](https://gvtdwawnc78.feishu.cn/wiki/SvrYwWVnGiwH7ikjb5EcQYBOnab)
> **代码对应**：本仓库 [vmc_src/](vmc_src/) 模块，详见 [module-vmc.md](MemoryPalace/robotics/tinker/src_analysis/omnibothub/module/module-vmc.md)。

---

## 一、原理与定义

VMC 方法最初由 **Jerry E. Pratt** 提出，用于双足机器人的运动控制 [97]。该方法将机器人系统的空间运动简化为一个**刚性浮动体**的位置和姿态变化。底座与地面的互动可以用**虚拟弹簧阻尼模型**来模拟，用以调节底座体的运动；当忽略所有腿的支撑效果时，不论底座体的加速度影响，控制力均分配至支撑腿的脚端作为地面反作用力（GRF）。各关节的控制力可根据每条腿的**力雅可比矩阵**计算得出 [98]。

### 核心思想速记

```
   "底座是浮动刚体" + "腿是无质量支撑棍" + "弹簧阻尼挂在浮动刚体上"
       ──► 浮动刚体期望位姿与实际位姿之差通过虚拟弹簧产生虚拟力
       ──► 虚拟力按力分配规则下发到各支撑腿足端
       ──► 足端力通过雅可比转置 τ = Jᵀ·F 变为关节扭矩
```

### 控制律（典型形式）

```
F_virtual = Kp · (x_des − x) + Kd · (ẋ_des − ẋ) + F_gravity_compensation
F_foot_i  = α_i · F_virtual                      (按支撑相分配)
τ_joint_i = J_iᵀ · F_foot_i                      (按腿的雅可比转置投影)
```

其中：
- `Kp/Kd` 即虚拟弹簧的刚度/阻尼系数（类比 PID 的 P/D）；
- `x` 涵盖浮动基的位置 + 姿态 6 维（或者面向高度+俯仰+横滚等子集）；
- `α_i` 由步态相（支撑/摆动）和摩擦锥约束决定；
- `J_i` 是第 i 条腿足端到关节空间的雅可比。

---

## 二、性能与适用性

在计算机器人控制力时，通过调整虚拟弹簧的刚度和阻尼系数，可以实现**不同的跟踪效果**和**抗干扰能力**。VMC **无需分层控制器**，仅需关节扭矩即可实现四足机器人的速度和高度控制。此方法考虑了身体和脚的广义坐标，包括它们的配置、位置和速度，而无需复杂的动力学计算 [99]。该方法已在 **StarlETH 机器人** [100] 和 **HyQ 机器人** [99] 上成功应用。

### 工程优点

VMC 是一种**直观的基于模型的控制算法**，通过调整控制参数来解耦机器人的运动变量，无需考虑动力学计算的复杂关系，从而实现良好的力控特性。这种方法简单且计算效率高 [101]。虚拟弹簧的刚度和阻尼系数类似于 PID 控制的比例和微分参数。因此，当机器人配置出现显著偏差时，需要较大的控制力矩。然而，在脚部摩擦锥的约束下，力矩可能会迅速饱和，影响机器人的稳定性和持久性。

### 局限

VMC 的主要缺点在于，**虚拟物理组件无法充分描述系统的动态特性**，尤其是在快速动态运动能力方面存在明显局限 [102]。**惯性和传感器噪声**也会加重这些限制。此外，VMC 仅关注**系统当前时刻的控制**，与预测控制方法相比，在复杂地形（如楼梯）上的动态性能和鲁棒性较差 [103]。

### 对比速查

| 维度 | VMC | MPC | RL |
|------|-----|-----|-----|
| 模型依赖 | 弱（仅刚体 + 弹簧阻尼） | 强（SRBD/全身动力学） | 端到端学习 |
| 计算开销 | **极低** | 高（实时 QP） | 中（推理） |
| 静态性能 | 好 | 好 | 视训练 |
| 动态性能 | 中（脚部饱和受限） | **好** | 好 |
| 复杂地形 | 弱 | **强** | 强 |
| 调参方式 | Kp/Kd 直观 | 权重矩阵 | reward + 训练 |
| 部署难度 | 极低 | 中（QP 求解器+建模） | 中（模型部署） |

---

## 三、Tinker 中的 VMC 实现

Tinker 在 `control_task_tinker` 主可执行文件里独立维护了一个 `vmc_src/` 模块，整体走"VMC 主控 + 可切换 RL 推理步态"路线，其中 VMC 作为站立、姿态保持、自起恢复的底层控制律，RL 步态接管行走/跳跃等动态行为。

### 3.1 模块组成

| 文件 | 角色 |
|------|------|
| [vmc_src/locomotion_sfm.cpp](vmc_src/locomotion_sfm.cpp) | VMC 主状态机（约 1104 行） |
| [vmc_src/hardware_interface.cpp](vmc_src/hardware_interface.cpp) | IMU/编码器读入 + 姿态融合 |
| [vmc_src/force_imp_controller.cpp](vmc_src/force_imp_controller.cpp) | 力 / 阻抗混合控制器（控制律核心） |

### 3.2 控制律实现

[vmc_src/force_imp_controller.cpp:42-105](vmc_src/force_imp_controller.cpp#L42-L105) 完整地实现了上文公式：

```
τ_joint = Jᵀ · ( F_cmd_world                ← 上层力期望（站立/支撑相）
                + Kp · (p_des − p_now)       ← 笛卡尔空间位置 PD
                + Kd · (ṗ_des − ṗ_now) )     ← 弹簧+阻尼形式
```

- `Kp/Kd` 来自 YAML 配置的 `imp_param.imp_x/y/z_kp/kd`，详见 [module-param.md §3d](MemoryPalace/robotics/tinker/src_analysis/omnibothub/module/module-param.md)；
- `J` 来自 [math_src/kin_math.cpp::cal_jacobi_new()](math_src/kin_math.cpp#L420)；
- 力分配 `α_i` 通过 `vmc[i].ground`（接地标志）与摩擦锥参数 `imp_param.ground_mu` 共同决定。

### 3.3 教科书 VMC vs MIT 版 VMC：**Tinker 没有 QP，也没有权重矩阵 `S`**

VMC 在文献里其实存在**两个版本**，它们的差别集中在"力分配"那一步：

| 版本 | 力分配方式 | 是否需要权重 `S` | 代表实现 |
|------|------------|------------------|----------|
| **教科书 / Pratt 原版 VMC** | 按相位手工 50/50 平分、单腿 100% | ❌ 不需要 | StarlETH 早期、**Tinker** |
| **MIT Balance Controller**（"VMC + QP" 增强版） | 在线 QP：`min ‖A·F−b‖²_S + α‖F‖²` s.t. 摩擦锥 | ✅ **核心参数** | MIT Cheetah 3 |

教科书 VMC 靠"工程直觉规则"决定哪条腿出多少力（哪条腿接地就给它分多少比例），**没有最优化问题**，自然就没有目标函数权重 `S`。

#### Tinker 走的是教科书版

证据链：
1. `qpOASES` 被链接进了可执行文件（[CMakeLists.txt](CMakeLists.txt)），但 `vmc_src/` 目录下 **grep 不到任何 `QProblem` / `qpOASES` 调用**——QP 库纯属占位；
2. [vmc_src/force_imp_controller.cpp:42-105](vmc_src/force_imp_controller.cpp#L42-L105) 的控制律是 `τ = Jᵀ·(F_cmd + Kp·Δp + Kd·Δṗ)`，**没有 6×N 的 A 矩阵**——每条腿独立计算自己的足端力，再按 `vmc[i].ground` 标志门控；
3. `param_gait.yaml` 里找不到任何 `w_x / w_z / w_att / Q_force / R_force` 之类的二次型权重项。

#### 那"哪个轴更重要"在 Tinker 里如何表达

在没有 QP 的前提下，Tinker 用**离线调好的标量 PD 增益**实现同样的轴向偏好——把 QP 的**在线权衡**降级为**调参期手动权衡**：

| QP 里 `S` 要做的事 | Tinker VMC 用什么代替 | 位置 |
|--------------------|----------------------|------|
| 让 Z 向支撑力优先满足 | `imp_z_kp = 0.03` >> `imp_x/y_kp = 0.01`（3 倍） | [param_gait.yaml:112-114](Param_Tinker14/param_gait.yaml#L112-L114) |
| 让姿态(pitch/roll) 优先于水平位置 | `kp_pit = 40 / kp_rol = 30` vs `kp_pos_x = 20` | [param_gait.yaml:134-148](Param_Tinker14/param_gait.yaml#L134-L148) |
| 高度跟踪权重远大于水平 | `kp_pos_z = 2500` vs `kp_pos_x = 20`（**125 倍**） | [param_gait.yaml:155](Param_Tinker14/param_gait.yaml#L155) |
| 摩擦锥不等式约束 | `ground_mu = 0.5`，仅作为缩放标量，**不当约束** | [param_gait.yaml:159](Param_Tinker14/param_gait.yaml#L159) |

`kp_pos_z = 2500` vs `kp_pos_x = 20` 这个 **125 倍的差距** 正是 Tinker 在表达"垂直方向必须紧跟、水平方向可以软"——与 QP 仿真里 `w_z >> w_x` 是同一个思想，只是手段不同：

- **QP 版**：在线解 `min‖A·F − b‖²_S`，`S` 显式做加权；
- **Tinker 版**：把这个权重"焊死"在 PD 比例里，**跳过最优化**。

#### 一句话总结

> 教科书 VMC 没有 QP，也就没有 `S`；MIT 版 VMC 才有 QP 和 `S`；**Tinker 用的是教科书版**。
> Tinker 仓库里链接的 `qpOASES` 当前不被 VMC 调用，**预留给将来的 [mpc_locomotion/](mpc_locomotion/)** ——届时 QP + `S` 才会真正出现在控制回路里。

---

### 3.4 状态机融合

[vmc_src/locomotion_sfm.cpp:482](vmc_src/locomotion_sfm.cpp#L482) 主循环按 `vmc_all.gait_mode` 分派到具体步态，VMC 控制律在以下模式下激活：

| `gait_mode` | 步态 | VMC 角色 |
|-------------|------|----------|
| `STAND_RC` | 站立 | **纯 VMC**（虚拟弹簧维持位姿） |
| `RECOVER` | 自起 | **VMC + 多阶段关节 PD** |
| `G_RL` | RL 行走 | VMC 退化为关节级阻抗，让 RL 神经网络主导 |
| `G_KICK` | 踢球 | VMC 维持支撑腿，摆动腿走规划 |
| `SOFT` | 软着陆 | VMC 慢慢卸力 |

### 3.5 摩擦锥与饱和保护

如理论分析所述，VMC 在脚部摩擦锥约束下扭矩易饱和。Tinker 的处理：
- 启动时通过 [robot_param.cpp:127-141](src/robot_param.cpp#L127-L141) 加载 `tau_max` 限制每关节扭矩；
- `imp_param.ground_mu = 0.5`（默认）参与力分配以避免横向滑动；
- 检测到严重姿态偏差（roll > 45°）时切到 `RECOVER`，而不是继续硬怼 VMC（[locomotion_sfm.cpp:992](vmc_src/locomotion_sfm.cpp#L992)）。

---

## 四、调参指南（针对 Tinker `imp_param`）

依据上文"刚度/阻尼系数类比 PID 的 P/D"原则，调参顺序：

1. **关节级 PD**：先调 `kp_qXY`、`kd_qXY`、`stiff_qXY`，让单腿 PD 跟踪稳定（YAML `imp_param.kp_q* / kd_q* / stiff_q*`）。
2. **足端阻抗**：再调 `imp_x/y/z_kp` 与 `imp_x/y/z_kd`，控制笛卡尔空间的"软硬"。
   - 站立时 Z 方向 kp 偏大（0.03）抵抗扰动；
   - X/Y 方向 kp 偏小（0.01）允许滑动调整。
3. **力前馈**：`fb_x/y/z_kp` 在支撑相提供前馈，减小跟踪误差。
4. **遥控前馈**：`imp_x/y_fp = 0.25`，让命令瞬时响应而不仅靠 P 误差。
5. **姿态外环**（`stand_param`）：`kp/ki/kd_pit/rol/yaw`，PD 主导，I 弱化（防积分爆冲）。

各字段详细对应见 [module-param.md §3d](MemoryPalace/robotics/tinker/src_analysis/omnibothub/module/module-param.md)。

---

## 五、何时不该用 VMC

依据上文 [102][103] 论证：

| 场景 | 建议 |
|------|------|
| 高度动态（冲跳、Bound、Pronk） | 改 MPC 或 RL；VMC 的脚部力会快速饱和 |
| 复杂离散地形（楼梯、踏石） | 改 MPC（带 footstep 规划）或 RL（带视觉） |
| 高速大幅转向 | VMC + 偏置补偿 或 RL（Tinker 当前路线） |
| 大负载变化 | VMC + 自适应/L1 控制 或 MPC（带在线辨识） |
| 静态站立、姿态稳定 | **VMC 最适合**，无需任何升级 |

Tinker 的实践选择：**站立/恢复用 VMC，行走/动态用 RL**，二者通过 `gait_mode` 状态机切换——这避开了 VMC 动态性能弱与 RL 静态性能不稳的双重短板。

---

## 六、引用文献编号说明

文中 `[97]–[103]` 的编号沿用了主参考飞书文档的索引：

- [97] Pratt J. E. 等，VMC 原始论文（双足机器人）
- [98] 力雅可比映射相关推导
- [99] HyQ 机器人 VMC 应用
- [100] StarlETH 机器人 VMC 应用
- [101]–[103] VMC 与 MPC、RL 的对比综述

完整书目以原飞书文档为准（本机无法直接抓取，访问需登录）。

---

## 七、扩展阅读

- 本仓库源码层细节：[module-vmc.md](MemoryPalace/robotics/tinker/src_analysis/omnibothub/module/module-vmc.md)
- 控制律对应参数表：[module-param.md](MemoryPalace/robotics/tinker/src_analysis/omnibothub/module/module-param.md)
- 腿型与工作空间（决定雅可比形式）：[leg-mechanism-types.md](MemoryPalace/robotics/tinker/mechanical/leg-mechanism-types.md)
- 整体模块架构：[module-architecture.md](MemoryPalace/robotics/tinker/src_analysis/omnibothub/module-architecture.md)
