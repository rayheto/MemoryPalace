---
tags:
  - robotics
  - sim2real
  - domain-randomization
  - domain-adaptation
---
# 域随机化与域自适应

域随机化和域自适应都在处理分布差异，但方向不同：域随机化是在训练前主动扩大仿真分布，域自适应是在看到目标域后对齐源域和目标域分布。

## 域随机化

Domain Randomization 的基本假设是：只要仿真分布足够宽，真实世界就会落在训练分布内部。模型在大量随机环境中训练后，会学到任务真正需要的鲁棒特征。

![[assets/domain-randomization-domains.svg|666]]

![[assets/randomized-sim-to-real.svg|672]]

可随机化对象：

- 视觉：颜色、纹理、材质、光照、相机姿态、背景、噪声、模糊。
- 几何：物体尺寸、形状、位置、姿态、障碍物布局。
- 动力学：质量、摩擦、阻尼、刚度、重力、执行器增益、控制延迟。
- 任务：初始状态、目标状态、扰动、奖励参数。

域随机化的作用可以从三个角度理解：

![[assets/randomization-effects.svg|675]]

- 数据增广：随机化后的仿真数据覆盖更大的分布，尽量包住真实数据所在区域。
- 特征强化：随机化非关键因素，迫使模型学习与任务相关的稳定特征，而不是记住颜色、材质、背景等伪相关因素。
- 数学问题变更：从单一数据集上的经验风险最小化，变成跨任务或跨分布的期望风险最小化。

对应形式：

$$
\theta^\* = \arg\min_\theta \mathcal{L}_\theta(D)
$$

变为：

$$
\theta^\* = \arg\min_\theta \mathbb{E}_{D \sim p(D)}[\mathcal{L}_\theta(D)]
$$

优点：

- 不要求精确还原真实环境。
- 适合零样本迁移。
- 对视觉策略和强化学习策略都常用。

风险：

- 随机化范围太窄，真实世界仍然落在分布外。
- 随机化范围太宽，训练任务会变难，策略可能保守。
- 随机化变量如果和任务无关，可能浪费样本效率。

## lerobot-rlinf 中已有的域随机化

`lerobot-rlinf` 自身主要是 `openpi` / π₀.₅、`leisaac` / Isaac Lab、LeRobot 数据集格式和 PPO 后训练之间的胶水层。项目目前没有在 `src/` 下单独实现一套完整的域随机化配置层，但它继承并使用了 LeIsaac `LeIsaac-SO101-PickOrange-v0` 环境里已经写好的 reset-level 随机化。

当前已经生效的随机化来自 `third_party/leisaac` 的 `pick_orange_env_cfg.py`：

| 随机化域 | 对象 | 范围 | 粒度 |
| --- | --- | --- | --- |
| 任务初始状态 | `Orange001`、`Orange002`、`Orange003` | `x, y ∈ [-0.03, 0.03] m`，`z = 0` | 每次 reset |
| 任务初始状态 | `Plate` | `x, y ∈ [-0.03, 0.03] m`，`z = 0` | 每次 reset |
| 视觉几何 | `front` 相机 | `x, y, z ∈ [-0.025, 0.025] m` | 每次 reset |
| 视觉几何 | `front` 相机 | `roll, pitch, yaw ∈ [-2.5°, 2.5°]` | 每次 reset |

实现方式是 LeIsaac 的 `domain_randomization(...)` helper 把若干 `EventTerm` 挂到 `env_cfg.events` 上：

- 物体位姿随机化使用 `mdp.reset_root_state_uniform`。
- 相机位姿随机化使用 `enhance_mdp.randomize_camera_uniform`。
- 这些 event 的 `mode="reset"`，所以它们只在 episode/reset 边界采样，不是 step 级传感噪声或在线扰动。

`lerobot-rlinf` 的 `IsaaclabPickOrangeEnv._make_env_function()` 会从 Isaac Lab registry 加载这个 LeIsaac env cfg，然后设置 `seed`、`num_envs`、`episode_length_s`、`decimation`、front/wrist 相机分辨率和 aux observation。它没有清掉上游 `events`，所以这些 reset-level 随机化会继续保留。

### 不属于域随机化的部分

项目中还有几类“随机”或“扰动”容易和域随机化混在一起，但语义不同：

- `ResidualGaussianPolicy` 里的 `r ~ N(mu, sigma)` 是 PPO residual head 的探索噪声。它改变策略采样，不改变环境分布。
- `pick_orange_ppo.yaml` 里的 `noise_level`、`flow_sde` 属于 actor/action 采样配置，不是物体、相机、动力学或传感域随机化。
- `OOD KNN penalty`、`survival_cost`、`dense_lift` 是 reward shaping / 后训练约束，用来把策略拉回示教流形或提供稀疏奖励之外的梯度。
- `eval.py` / `eval_run.py` 里的 `prefetch`、`prefetch_latency_ms` 是推理时序实验，默认关闭；它不是训练时的 action delay randomization。
- `decimation = 2` 是为了让 Isaac 外层 step 对齐 SFT 数据集的 30 fps，不是随机化。

这意味着当前项目的真实域随机化覆盖面很窄：它主要覆盖“桌面上橘子/盘子的初始布局变化”和“front 相机安装误差”。它还没有覆盖 wrist 相机、光照、材质、摩擦、质量、执行器响应、通信延迟或动作延迟。

### 当前随机化的价值

这套已有随机化虽然简单，但和 `pick_orange` 的数据分布是对齐的：

- 橘子和盘子的 xy reset jitter 能让策略看到更宽的初始任务状态，减少对单一摆放位置的记忆。
- front 相机 6DoF 小扰动能迫使视觉策略不要完全依赖固定相机投影。
- 所有随机化都在 reset 时发生，便于复现、录制和与 LeRobot 数据集格式对齐。

同时，它没有明显增加接触和控制难度，这对 60 条示教上做 SFT 和残差 PPO 后训练是保守的。当前阶段更像是“轻量随机化 + 目标域后训练”，而不是完整的鲁棒 sim2real 训练。

## lerobot-rlinf 的后续域随机化设计方向

下一步不应该直接把所有变量都随机化，而应该把 LeIsaac 已有的 reset event 升级为项目可控、可记录、可消融的配置层。目标是让训练分布、评测分布和真实部署分布之间有可追踪关系。

### 配置显式化

当前随机化藏在 LeIsaac env cfg 里。后续可以在 `pick_orange_ppo.yaml` 和 `src/rl/simple/config.py` 中增加显式配置：

```yaml
domain_randomization:
  enabled: true
  profile: "train_mild"
  task:
    orange_xy_jitter_m: 0.03
    plate_xy_jitter_m: 0.03
  vision:
    front_camera_pos_jitter_m: 0.025
    front_camera_rot_jitter_deg: 2.5
```

第一版先复刻 LeIsaac 当前行为，确认打开/关闭随机化可以由本仓库配置控制。之后再扩展新域。这样做的好处是：训练、评测、消融不再依赖“上游 cfg 里隐含了什么”。

### 记录随机化参数

`eval_run.py` 已经能把 rollout 写成 LeRobot v2.1 dataset。后续应把每个 episode 实际采样到的随机化参数也写入 sidecar metadata，例如：

```text
dataset/
  data/
  videos/
  meta/
  dr_params.jsonl
```

`dr_params.jsonl` 至少记录 `episode_index`、`seed`、profile、橘子/盘子位姿偏移、相机位姿偏移、success/fail/timeout、grasp/place/rest 事件时间。这样 diagnostics 才能回答“失败是否集中发生在某类随机化条件下”。

### 扩展优先级

更合理的扩展顺序是从低风险到高风险：

| 优先级 | 方向        | 建议                                                    |
| --- | --------- | ----------------------------------------------------- |
| P0  | 显式接管已有 DR | 把橘子/盘子 xy jitter 和 front camera jitter 暴露到本仓库 config  |
| P0  | 初始状态扩展    | 增加机器人 reset joint jitter、橘子 yaw/roll/pitch 小扰动        |
| P1  | wrist 相机  | 给 wrist camera 加小范围外参扰动，因为 π₀.₅ 实际消费 wrist 图像         |
| P1  | 时序扰动      | 实现 action delay queue、action repeat jitter、chunk 边界扰动 |
| P1  | 视觉外观      | 轻量 brightness/contrast/noise/blur，不要一开始改到破坏 π₀.₅ 图像先验 |
| P2  | 接触物理      | 橘子/桌面/盘子/夹爪摩擦、restitution、contact offset、橘子质量         |
| P2  | 执行器       | PD gain、速度/力矩限制、夹爪 effort/close threshold、命令滤波        |

对 `pick_orange` 来说，优先补 wrist camera 和 action delay 很重要。前者对应视觉定位域差，后者对应 chunk action 策略的时序脆弱性。接触物理也重要，但它容易显著改变任务难度，最好放在已有策略稳定后再逐步加。

### 训练与评测 profile

至少保留四类 profile：

| Profile | 用途 | 特点 |
| --- | --- | --- |
| `eval_nominal` | 对齐历史 baseline | 关闭新增随机化，保留确定 seed |
| `train_mild` | 默认后训练 | 复刻当前 LeIsaac 轻量 DR，再加少量 wrist/timing |
| `train_full` | 鲁棒性训练后期 | 视觉、时序、接触、执行器逐步打开 |
| `eval_sweep` | 泛化诊断 | 单变量 sweep，定位脆弱域 |

每次新增一个随机化域，都应该同时跑 `eval_nominal` 和 `eval_sweep`。如果 sweep 变好但 nominal 大幅下降，通常说明范围过宽或 curriculum 太快。

### 和 diagnostics 闭环

域随机化是否有效，不应该只看 success rate。`lerobot-rlinf` 已经有 LeRobot dataset 输出和 diagnostics，因此可以把随机化 profile 当成实验对象：

- `EXP_03_Episode-Length Inflation`：随机化后是否明显拖慢 episode。
- `EXP_05_State Coverage Divergence`：策略是否仍然跑出示教 state manifold。
- `EXP_04_Action Smoothness`：时序随机化是否引入高频抖动或 chunk 边界伪迹。
- `EXP_02_Compounding Error`：接触或执行器扰动是否放大关节轨迹弧长。
- `EXP_01_Mode Averaging`：过强随机化是否让策略退化成保守均值动作。

最终目标不是“随机化越多越好”，而是找到一组能覆盖真实系统误差、又不压垮 SFT/RL 后训练样本效率的分布。后续 Real2Sim2Real 做系统辨识时，也可以把真机估计出的摩擦、相机外参、延迟和执行器响应范围回填到同一套 `domain_randomization` 配置里。

## 域自适应

Domain Adaptation 关注源域和目标域的数据分布对齐。它通常假设源域数据充足且有标签，目标域数据无标签或少标签。

![[assets/domain-adaptation-alignment.svg|687]]

目标是学习域不变特征，减少源域和目标域之间的分布差异，从而提升模型在新域上的泛化能力。

常见方法：

- 分布对齐：最小化 MMD、CORAL、JS divergence 等统计距离。
- 对抗学习：训练特征提取器欺骗域判别器，使源域和目标域特征不可区分。
- 自监督或伪标签：用目标域无标签数据生成辅助任务或伪标签。
- 微调：利用少量真实数据调整模型。

## lerobot-rlinf 中的域自适应

`lerobot-rlinf` 里的域自适应不是单独实现一个 MMD、CORAL 或对抗域判别器，而是通过 π₀.₅ 多任务基础模型的目标域后训练来完成。

π₀.₅ 本身已经具备跨任务、跨场景的视觉-动作先验；`lerobot-rlinf` 做的是把这个通用先验适配到 SO-101 `pick_orange` 目标域：

1. **SFT 阶段**：在 `LightwheelAI/leisaac-pick-orange` 的 LeRobot 数据集上做 LoRA 监督微调，让 π₀.₅ 的多任务先验对齐到固定 robot embodiment、相机命名、关节单位、语言 prompt 和 pick-orange 行为分布。
2. **评测闭环**：`eval.py` / `eval_run.py` 在 LeIsaac 环境中执行策略，并把 rollout 重新写成 LeRobot v2.1 数据集，使目标域表现可以和参考示教集对齐比较。
3. **诊断阶段**：`src/diagnostics` 通过 episode length、state coverage、action smoothness 等指标判断 SFT 后策略是否偏离示教流形。
4. **RL 后训练阶段**：`src/rl/simple` 冻结 π₀.₅，用小型 residual Gaussian head 做 PPO，把策略从“能模仿目标域”继续推向“在目标域闭环执行中更可靠”。

因此，这里的域自适应可以理解为：

$$
\pi_{\text{general}}(\text{multi-task}) 
\xrightarrow{\text{SFT on LeRobot pick-orange}}
\pi_{\text{target}}
\xrightarrow{\text{residual PPO + diagnostics}}
\pi_{\text{closed-loop target}}
$$

这和域随机化的角色不同：域随机化扩大仿真训练分布，域自适应则把已有的通用模型能力压到目标任务分布上。当前 `lerobot-rlinf` 更强的一侧其实是域自适应与后训练闭环，而不是大规模物理/视觉随机化。

## 二者区别

| 方法 | 发生时机 | 需要真实目标域数据 | 核心动作 |
| --- | --- | --- | --- |
| Domain Randomization | 训练前或训练中 | 不一定需要 | 扩大仿真分布 |
| Domain Adaptation | 目标域可见后 | 通常需要无标签或少标签数据 | 对齐源域和目标域表示 |
