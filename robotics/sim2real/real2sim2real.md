---
tags:
  - robotics
  - sim2real
  - real2sim2real
  - digital-twin
---

# Real2Sim2Real

Real2Sim2Real 将 Sim2Real 从开环流程变成闭环流程。它不是只从仿真训练然后部署，而是先用真实世界生成数字资产或真实反馈，再回过头优化仿真器，最后重新训练或微调策略并迁移回现实。

![[assets/rsr-three-stage-flow.svg|697]]

## 三阶段流程

1. Real2Sim：从真实场景采集数据，进行三维重建、参数识别和接触模型估计。
2. Simulation Training：在更新后的仿真器或数字孪生中大规模训练策略。
3. Sim2Real：将优化后的策略部署到真实系统。

典型做法包括使用 NeRF、VR 扫描或三维重建构建高保真数字孪生环境，然后在虚拟世界中训练策略，最后迁移到真实场景。

## 真实场景生成的数字资产

真实场景生成的数字资产可以用于：

- 构建高保真数字孪生仿真环境。
- 反推仿真器中的建模结构，例如动力学参数和接触模型。
- 影响策略学习过程，例如改变奖励、损失或探索策略。
- 构建可重复迭代的建模框架。

## 仿真器参数反推

RSR 可以把物理参数当作可优化变量，用真实数据反向修正仿真器。

![[assets/rsr-modeling-pipeline.svg|691]]

一个典型管线包括：

- Simulation Dataset Generation：给定物理参数、动作和上一时刻状态，通过仿真器生成仿真数据。
- Surrogate Modelling：用神经网络拟合仿真器输入到输出的映射。
- Gradient-based Refinement：将物理参数作为可优化变量，通过真实数据误差进行梯度更新。

符号含义：

- $f, p, d$：friction, stiffness, damping coefficients，即摩擦、刚度和阻尼系数。
- $P_t$：仿真器数据。
- $S_t$：真机数据。

## 适配到 lerobot-rlinf

[rayheto/lerobot-rlinf](https://github.com/rayheto/lerobot-rlinf) 已经具备 RSR 的雏形：`openpi`/π₀.₅ 提供基础策略，`leisaac` + Isaac Lab 提供 SO-101 `pick_orange` 仿真环境，`eval_run.py` 可以把 rollout 写成 LeRobot v2.1 数据集，`src/diagnostics` 可以比较参考集和候选集，`src/rl/simple` 则在冻结 π₀.₅ 上训练小型残差 PPO 头。

因此，参数反推不应该先改策略，而应该先把真实 rollout 当作校准信号，反向更新 Isaac/LeIsaac 仿真参数：

$$
\theta_{sim}^{*}=\arg\min_{\theta_{sim}}\mathcal{L}
(\text{rollout}_{sim}(\pi_{0.5}, \theta_{sim}),\text{rollout}_{real})
$$

### 需要反推的参数

优先选择会直接影响 `pick_orange` 失败模式的参数，不要一开始就把整个场景都设为可学习：

- 接触参数：橘子、盘子、桌面、夹爪之间的摩擦系数、恢复系数、接触 offset、静摩擦/动摩擦比例。
- 物体参数：橘子质量、惯量、半径、质心偏移，盘子和桌面的碰撞体近似。
- 执行器参数：关节 PD 增益、速度/力矩限制、gripper 闭合阈值、动作延迟、命令滤波。
- 时间参数：`decimation`、sim fps、dataset fps、policy action chunk 执行节奏。
- 观测参数：front/wrist 相机外参、分辨率、畸变、曝光和图像延迟。

这些参数和项目现有问题是对齐的：当前诊断指出候选 rollout 有 episode length inflation 和 state coverage divergence，说明策略容易陷入示教未覆盖的低速 OOD 状态。若真实系统里同样出现“抓取中间态停留”或“橘子被推出分布外”，优先反推接触、夹爪和动作延迟，而不是直接加大 PPO。

### 数据接口

项目已有 LeRobot v2.1 数据集落盘能力，可以把它扩展成 Real2Sim 校准数据格式：

- 保留现有列：`observation.state`、`action`、front/wrist 视频、episode metadata。
- 新增真机列：`observation.ee_pose`、`observation.object_pose`、`observation.plate_pose`、`observation.joint_vel`、`timestamp_wallclock`。
- 对齐单位：继续沿用项目里的 leisaac radians ↔ lerobot motor degrees 转换边界，避免把“单位错配”误当成动力学误差。
- 对齐频率：真实数据和仿真 rollout 都落到同一 dataset fps，例如 30 Hz。

若暂时拿不到完整物体位姿，最低配也要记录事件时间：首次接近橘子、首次闭合夹爪、橘子首次离桌、首次放置、episode 结束原因。

### 损失函数

不要只用 success rate 做反推目标，success 太稀疏。更稳的损失可以由几部分组成：

$$
\mathcal{L}
=w_s\mathcal{L}_{state}
+w_o\mathcal{L}_{object}
+w_e\mathcal{L}_{event}
+w_d\mathcal{L}_{diagnostic}
+w_a\mathcal{L}_{action}
$$

- $\mathcal{L}_{state}$：真实和仿真关节状态转移误差，例如 $\Delta q_t$、$\Delta \dot q_t$。
- $\mathcal{L}_{object}$：橘子、盘子、末端执行器相对位姿误差。
- $\mathcal{L}_{event}$：grasp/place/rest/fail 事件发生时间差。
- $\mathcal{L}_{diagnostic}$：复用 `src/diagnostics` 的 episode length、state coverage、action smoothness 等指标。
- $\mathcal{L}_{action}$：同一 π₀.₅ 策略下，仿真动作后果是否接近真实动作后果。

这相当于把现有 diagnostics 从“评估报告”升级为“仿真器参数优化目标”。

### 实现路径

1. 固定策略：先冻结 π₀.₅ 和 residual head，用同一策略分别跑真实系统与 Isaac Lab。
2. 扩展 rollout 记录：让真实 rollout 和 `eval_run.py` 生成的仿真 rollout 拥有相同 schema。
3. 暴露 `sim_params.yaml`：集中管理可调仿真参数，例如摩擦、质量、PD gain、delay、camera extrinsic。
4. 注入环境：在 `IsaaclabPickOrangeEnv._make_env_function()` 载入 leisaac env 后，根据 `sim_params.yaml` 修改 scene asset、material、actuator 或 camera 配置。
5. 外循环搜索：先用 CMA-ES、Bayesian optimization 或 grid/coarse-to-fine 搜索 $\theta_{sim}$，因为 Isaac/PhysX 不是端到端可微。
6. 候选评估：每组参数跑固定 episode 数，写出 LeRobot dataset，调用 `python -m src.diagnostics` 产出指标。
7. 选择最优仿真器：用最接近真实 rollout 的参数版本重新训练 residual PPO。
8. 闭环迭代：将新策略部署到真实系统，再收集真实 rollout，继续更新仿真器。

### 最小可行版本

最小版本不需要完整 differentiable simulation，只需要三件事：

- `real_rollout_dataset/`：真机采集的 LeRobot 数据集，至少包含 state、action、视频、episode metadata。
- `sim_params.yaml`：只开放 5-10 个高影响参数，例如橘子摩擦、桌面摩擦、夹爪 effort、action delay、grasp threshold。
- `fit_sim_params.py`：外循环脚本，负责采样参数、运行 Isaac eval、调用 diagnostics、根据综合 loss 选最优参数。

先把仿真器校准到“错误方式像真实系统”，再做 residual PPO，收益会比直接在偏差很大的仿真器里继续强化学习更可靠。

## 用真实表现优化策略学习

![[assets/rsr-policy-loop.svg|665]]

广义 RSR 不只优化仿真器参数，也可以让真实系统表现影响策略训练过程。比如在策略训练循环中设计自适应损失函数，动态平衡任务完成和数据探索。

可表示为：

$$
a_{k,t} = \arg\min \mathcal{L}(a_{k,t}) = \mathcal{L}_{task}(a_t) + \mathcal{L}_{sr}(a_{k,t})
$$

其中 $\mathcal{L}_{task}$ 关注任务完成，$\mathcal{L}_{sr}$ 关注真实到仿真的闭环校准或探索需求。

## 适用场景

- 真实数据昂贵但可以少量采集。
- 任务需要高保真场景几何或接触模型。
- 真实场景长期复用，值得构建数字孪生。
- 希望把真实系统反馈纳入仿真器和策略学习闭环。
