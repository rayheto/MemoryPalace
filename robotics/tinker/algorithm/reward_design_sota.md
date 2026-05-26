# 足式机器人 RL 奖励函数设计：前沿类型、设计准则与实验分析方法

本文档综述 2024–2025 年足式机器人强化学习领域奖励函数设计的前沿趋势、核心设计准则和实验分析方法。适合为 Tinker 项目的 reward 优化提供方法论参考。

---

## 目录

1. [奖励函数的核心分类体系](#1-奖励函数的核心分类体系)
2. [前沿趋势 (2024–2025)](#2-前沿趋势-20242025)
3. [奖励函数设计准则](#3-奖励函数设计准则)
4. [奖励函数的数学形式选择](#4-奖励函数的数学形式选择)
5. [实验分析方法](#5-实验分析方法)
6. [Tinker 当前 reward 的对标分析](#6-tinker-当前-reward-的对标分析)

---

## 1. 奖励函数的核心分类体系

通过对 150+ 篇足式机器人 RL 论文的系统梳理，奖励函数可以按功能归为以下类别：

### 1.1 按物理目标分类

| 大类 | 子类 | 典型 reward | 代表性工作 |
|---|---|---|---|
| **任务跟踪 (Task)** | 速度跟踪、位置跟踪、朝向跟踪 | $\exp(-\|\mathbf{v}-\mathbf{c}\|^2)$ | ETH Zurich legged_gym, ANYmal |
| **姿态稳定 (Posture)** | 躯干姿态、高度、朝向 | $\exp(-\|\mathbf{g}_{\text{proj},xy}\|^2)$ | 几乎所有 locomotion 工作 |
| **步态周期 (Gait)** | 足部离地时间、接触序列、相位 | $f(t_{\text{air}}, T_{\text{cycle}})$ | "Learning to Walk in Minutes", Rudin et al. |
| **能量效率 (Energy)** | 力矩、功率、CoT | $\sum \tau^2, \sum \|\tau \cdot \dot{q}\|$ | "Adaptive Energy Regularization", 2024 |
| **平滑性 (Smoothness)** | 动作变化、加速度、jerk | $\|\mathbf{a}_t - \mathbf{a}_{t-1}\|^2$ | Sim-to-real 关键项 |
| **安全约束 (Safety)** | 关节限位、力矩限位、碰撞 | $\mathbb{1}[\text{violation}]$ | N3PO, CPO, P3O |
| **足部行为 (Foot)** | 滑动、clearance、落脚点、旋转 | 接触力 × 速度 | "Rapid Locomotion", Margolis et al. |
| **风格/模仿 (Style)** | 参考轨迹跟踪、运动先验 | $\|\mathbf{q} - \mathbf{q}_{\text{ref}}\|^2$ | AMP, "Motion Priors Reimagined" |

### 1.2 按数学形式分类

| 形式 | 典型表达式 | 梯度特性 | 使用场景 |
|---|---|---|---|
| **平方惩罚** | $r = -\|x\|^2$ | 远离目标时梯度大 | 正则化（力矩、速度） |
| **指数奖励** | $r = e^{-\sigma\|x\|^2}$ | 有上界（最大 1.0），近目标时梯度大 | 跟踪任务（速度、姿态） |
| **线性惩罚** | $r = -\|x\|$ | 梯度恒定 | 足底滑动 |
| **分段常数** | $r \in \{-2, -1, 0, 1.2\}$ | 不连续，需注意梯度 | 接触状态检测 |
| **hinge/ReLU** | $r = -\max(0, x - x_{\text{th}})$ | 仅在超出阈值时激活 | 软约束（限位） |
| **逻辑/barrier** | $r = -\log(x_{\text{max}} - x)$ | 接近边界时爆炸 | 硬约束替代 |

### 1.3 按优化范式分类

| 范式 | 奖励形式 | 约束形式 | 代表算法 |
|---|---|---|---|
| **无约束 Reward** | 全部通过 reward | 无显式约束 | PPO (legged_gym) |
| **Reward + Penalty** | 鼓励目标行为 | 惩罚不期望行为（同 reward 管道） | PPO with negative weights |
| **Reward + Constraint (Lagrangian)** | 最大化 reward | 独立 constraint costs | N3PO, CPO, P3O, IPO |
| **Reward + Safety Layer** | 最大化 reward | post-hoc safety filter | "Safe Reinforcement Learning for Legged Locomotion" |
| **Multi-Objective RL** | 多个独立 objective | Pareto 前沿搜索 | MORL, "Not Only Rewards but Also Constraints" |

---

## 2. 前沿趋势 (2024–2025)

### 2.1 从"堆叠 reward"到"约束优先"

**核心论文**："Not Only Rewards but Also Constraints: Applications on Legged Robot Locomotion" (IEEE T-RO, 2024, KAIST)

**思想**：传统做法是设计 20-30 个 reward 项并手工调整权重。新范式将大部分"不期望的行为"转为**约束 (constraints)**，通过 Lagrangian 乘子法或 barrier 函数优化，只保留 1–2 个核心任务 reward。

**优势**：
- 权重调优从 O(n²) 降为 O(1)（只需调约束的 k 值和 d 值）
- 约束语义清晰："关节不能超限"比"惩罚关节位置偏差"更精确
- 策略不会因为过度惩罚而牺牲任务性能

**与 Tinker 的关联**：Tinker 的 N3PO 已迈出了这一步——5 个 cost 函数通过 Lagrangian 乘子与 reward 协同优化。但仍有 26 个 reward 项，可以进一步做"reward→constraint"迁移。

### 2.2 能量极简主义

**核心论文**："Adaptive Energy Regularization for Autonomous Gait Transition and Energy-Efficient Quadruped Locomotion" (2024)

**思想**：用一个能量项替代复杂的多目标 reward。单一的能量正则化项可以根据速度自适应调节权重，**自发涌现**步态切换（低速 walk → 高速 trot）而无需显式的步态周期 reward。

**公式示例**：

$$r = r_{\text{track}} - \alpha(v) \cdot \sum |\tau_j \cdot \dot{q}_j|$$

其中 $\alpha(v)$ 是速度自适应的权重函数。

**实验结果**：在 ANYmal-C 和 Go1 上，速度跟踪和能效**均优于**复杂的多 term reward。

### 2.3 Barrier 函数风格奖励

**核心论文**："A Learning Framework for Diverse Legged Robot Locomotion Using Barrier-Based Style Rewards" (ICRA 2025, KAIST)

**思想**：用放宽的对数 barrier 函数 $\log(x_{\text{max}} - x)$（或其近似）作为软约束来引导步态风格。相比传统的硬编码接触序列，barrier 允许在训练过程中**柔性调整**。

**成果**：单一策略实现 4.67 m/s gallop、3.6 m/s bipedal run、58cm 越障（KAIST HOUND, 45kg 四足）。

### 2.4 LLM/VLM 自动生成 Reward

**核心论文**："Video2Reward: Generating Reward Function from Videos for Legged Robot Behavior Learning" (ECAI 2024)

**思想**：从视频演示中提取关键点轨迹 → LLM 生成 reward 函数代码 → 迭代视觉反馈优化。

**优势**：
- 无需手动设计 reward
- 行为切换只需更换输入视频
- 相比纯文本描述生成，提升了 37.6%

**局限**：目前仅适用于简单行为（步行、跑步），复杂动态技能仍在探索。

### 2.5 层次化 Reward 分解

**核心论文**："Motion Priors Reimagined: Adapting Flat-Terrain Skills for Complex Quadruped Mobility" (CoRL 2025)

**思想**：两阶段层次化 RL：
1. **低层**：用动物运动数据预训练运动先验（motion imitation reward）
2. **高层**：添加残差修正（task-specific reward）

这减少了"精心调优速度/位置跟踪 reward"的需求。

### 2.6 周期性奖励组合

**思想**（多篇 2024 工作）：将不同的步态统一到同一个周期性 reward 框架下。通过调整概率周期函数的参数（频率、相位、占空比）即可从 standing 切换到 walking → hopping → running → skipping。

**公式框架**：

$$r_{\text{gait}} = \sum_f p_f(t) \cdot r_{\text{contact}, f} + (1 - p_f(t)) \cdot r_{\text{swing}, f}$$

其中 $p_f(t)$ 是足 $f$ 的期望接触概率（周期函数）。

---

## 3. 奖励函数设计准则

### 3.1 核心原则

| 原则 | 说明 | 反例 |
|---|---|---|
| **1. 语义清晰** | 每项 reward 应对应一个明确的物理目标 | 混合"速度+姿态"的复合项 |
| **2. 梯度连续** | reward 函数应几乎处处可导，避免平坦/悬崖 | 阈值二值化（无梯度过渡） |
| **3. 上界有限** | 用指数函数而非平方函数做正向奖励，防止 reward explosion | $r = -\|x\|^2$ 作正奖励（无上界） |
| **4. 尺度校准** | 不同 reward 项的数值量级应在同一数量级（contributing equally before weighting） | 一项量级 1e-3，另一项量级 1e3 |
| **5. 非对抗性** | reward 项之间应协同而非相互矛盾 | 同时奖励"高速运动"和"零力矩" |
| **6. 最小充分性** | 用最少的 reward 项完成任务，新增项必须有消融实验证明必要性 | 盲目堆叠 30+ reward |
| **7. 稀疏奖励谨慎** | 稀疏奖励需要辅助 shaping 或 curriculum，否则收敛极慢 | 仅在 episode 结束时给一个 binary |

### 3.2 权重调优策略

**方法一：逐步堆叠 (Incremental)**

1. 从**单一 task reward**（如速度跟踪）开始训练
2. 观察行为缺陷 → 添加一个 regularization 项
3. 调该项权重直到行为改善，但不破坏 task reward
4. 重复 2-3 直到行为满意

**方法二：约束优先 (Constraint-first, 推荐)**

1. 核心任务用 reward（1-2 项）
2. 安全/硬性要求用 cost/constraint
3. 行为引导用 barrier 函数而非 reward penalty
4. 仅在 constraint 无法表达时才新增 reward

**方法三：自动调参 (Auto-tuning)**

- **Population-Based Training (PBT)**：多个 agent 并行训练，定期交换权重
- **Bayesian Optimization**：在高斯过程指导下搜索权重空间
- **Meta-RL / Learned reward**：训练一个网络学习 reward 权重

**权重经验法则**：

| 项类型 | 初始权重建议 | 调优方向 |
|---|---|---|
| Task (速度跟踪) | 1.0–5.0 | 看 episode return 是否增长 |
| Posture (姿态) | 0.1–2.0 | 看机器人是否持续倾斜 |
| Energy (力矩) | 1e-5–1e-3 | 看 CoT 是否合理降低 |
| Smoothness (动作) | 0.001–0.1 | 看 sim-to-real gap |
| Gait (步态) | 1.0–5.0 | 看是否形成周期步态 |
| Termination | -1.0 至 -10.0 | 看 episode length 是否增长 |

### 3.3 常见陷阱

1. **Reward hacking**：策略找到 reward 函数的漏洞（如通过高频抖动增大某项 reward），需要在 sim 中可视化行为
2. **Reward imbalance**：某项 reward 权重过大压制其他项 → 用 episode log 中各项的数值诊断
3. **Dead reward**：reward 函数的值域太窄（如 exp(-100x)），在初始探索阶段全为 0 → 梯度消失
4. **Exploding reward**：无上界的平方项在高速运动时暴涨 → 导致 value function 不收敛
5. **Negative transfer**：某项 reward 学到的行为在 sim2real 中崩溃 → 需要 domain randomization + 该项 reward 的消融实验

---

## 4. 奖励函数的数学形式选择

### 4.1 正向奖励（鼓励行为）

**推荐：指数函数** $r = e^{-\sigma \cdot \text{error}^2}$

- 上界为 1.0（reward 不会无限增长）
- 误差小（接近目标）时梯度大，误差大时梯度小但非零
- $\sigma$ 控制锐度：$\sigma$ 越大，对目标附近的偏差越敏感

**不推荐：平方函数** $r = -\text{error}^2$ 作为正奖励

- 无上界 → 远离目标时 reward 极端负 → 价值函数震荡

### 4.2 负向惩罚（抑制行为）

**推荐：平方或 hinge**

- 平方：$r = -\|x\|^2$（行为量越大惩罚越重）
- Hinge：$r = -\max(0, x - x_{\text{th}})$（仅在超出阈值时惩罚）

**不推荐：指数作为惩罚**

- $r = -e^{\|x\|}$ 在 $x$ 大时梯度爆炸

### 4.3 接触/离散事件

**推荐：分段常数 + 时间积分**

- 如 `no_jump`：{-2, -0.5, 0, 1.0}
- 如 `feet_air_time`：$(t_{\text{air}} - T_{\text{cycle}}) \cdot \mathbb{1}[\text{contact}]$（只在离散事件时触发）

### 4.4 Barrier 函数（前沿替代）

当需要软约束时，barrier 函数优于 hinge 惩罚：

$$r_{\text{barrier}} = \frac{1}{t} \log(x_{\text{max}} - x) \quad \text{或} \quad r_{\text{barrier}} = -\frac{1}{(x_{\text{max}} - x)^2}$$

Barrier 在接近边界时梯度 → ∞，提供强保证；远离边界时几乎为 0（不干扰主任务）。

---

## 5. 实验分析方法

### 5.1 单次训练的监控指标

| 指标 | 健康趋势 | 预警信号 |
|---|---|---|
| **Mean episode reward** | 单调上升，后期平稳 | 剧烈震荡、长期不变 |
| **Mean episode length** | 快速上升到 max 的 80%+ | 一直很短（< 20% max）→ 跌倒频繁 |
| **Value function loss** | 早期上升（ep length 增长），后期缓慢下降 | 持续暴涨 → reward 不稳定 |
| **Surrogate loss** | 小幅波动，通常为负 | 持续为正 → PPO 不使用 clip |
| **Action noise std** | 单调下降 | 卡在 init 值不动 → 学习未启动；太快降到 0.05 → 早熟 |
| **KL divergence** | 在 desired_kl 附近波动 | 持续远超 desired_kl → 学习不稳定 |

### 5.2 逐项 Reward 分析

**基本方法**：

1. **排序各项贡献**：每 N 个 iteration 打印 `Mean episode rew_<name>`，按绝对值排序
2. **趋势分析**：对每项画 (iteration, value) 曲线，观察：
   - 应该上升的正项是否上升？
   - 应该接近 0 的负项绝对值是否在减小？
   - 有没有突然的跳变（可能对应 curriculum 变化）？
3. **主导项检测**：如果一项占总 reward 的 90%+，其他项基本无效 → 需要重新平衡权重

**示例分析（Tinker iter 21999）**：

```
tracking_lin_vel   2.18459  ← 绝对主导
tracking_ang_vel   1.71578  ← 第二主导
orientation_eular  0.83483  ← 第三
no_jump            0.55564  ← 有效的步态引导
...
dof_vel           -0.20951  ← 最大惩罚项
```

如果发现某个预期有效的 reward 持续 ≈ 0（如 `stumble = 0.00000`），可能是：
- reward 函数的激活条件太严格（从未触发）
- reward 的值域在初始状态就饱和（如 `exp(-100x)` 在 x=0.1 时已 ≈ 0）
- reward 信号被其他项淹没（权重太小）

### 5.3 消融实验 (Ablation Study)

**标准流程**：

1. **Full model**：所有 reward 项，作为 baseline
2. **Leave-one-out**：逐个移除 reward 项（设 weight=0），重新训练，对比：
   - Episode return 变化
   - 特定行为指标（如 CoT、滑移距离、跌倒率）
   - 可视化 gait pattern 有无变化
3. **Single-term test**：仅用一项 task reward（如 tracking_lin_vel），看策略能学会什么
4. **Weight sweep**：对关键项进行权重扫描（0.1×, 0.5×, 1×, 2×, 5×, 10×），找最优区间

**消融实验矩阵**（推荐格式）：

| 移除项 | Episode Return | Ep Length | CoT | 步态质量 | 结论 |
|---|---|---|---|---|---|
| None (Full) | baseline | baseline | baseline | baseline | — |
| - tracking_lin_vel | -80% | -30% | — | 不移动 | **必需** |
| - orientation | -15% | -10% | — | 倾斜增大 | 重要 |
| - dof_vel | +5% | 0% | +20% | 更抖动 | 影响能效 |

### 5.4 Reward Landscape 分析

**方法**：在训练好的策略参数附近，对 reward 函数的输入参数做 grid search：

1. 固定策略，对关键状态变量（如基座速度误差、姿态误差）进行 sweep
2. 绘制 $r = f(\text{state\_error})$ 的曲面
3. 检查：
   - 是否有平坦区域（梯度消失）？
   - 是否有悬崖（梯度爆炸）？
   - 最优区域是否与策略实际到达的状态分布重合？

### 5.5 相关性分析

计算各 reward 项之间以及 reward 项与 cost 项之间的 Pearson/Spearman 相关性：

$$\rho_{ij} = \frac{\text{Cov}(r_i, r_j)}{\sigma_{r_i} \sigma_{r_j}}$$

**解读**：
- 高度正相关（> 0.8）：两项可能在激励同一种行为 → 可以合并
- 高度负相关（< -0.8）：两项可能冲突 → 需要平衡或移除一项
- 零相关（≈ 0）：两项编码独立的行为目标

### 5.6 Curriculum 对 Reward 的影响

如果使用 curriculum（命令速度递增、地形难度递增等），需要分析：

- **Curriculum 阶段切换时**各 reward 项的变化 — 正常应该看到 tracking 类 reward 短暂下降后恢复
- **同一 curriculum 阶段内** reward 是否在提升 — 说明策略在适应当前难度
- **不同 curriculum 阶段间** reward 的绝对值不可直接比较（难度不同）

### 5.7 Cost/Constraint 分析

对 NP3O 的 cost 系统：

1. **Cost value 是否准确**：对比 `cost_value_loss` 和 `value_loss`，前者显著更大说明 cost 预测更困难
2. **Viol loss 何时激活**：如果全是 0，约束没有在起作用。如果间歇性激活，约束在边界处生效
3. **K-value 轨迹**：观察 Lagrangian 乘子是否收敛。单调增长说明约束一直被违反
4. **每个约束的 episode mean** 是否在 d_value 以下

---

## 6. Tinker 当前 reward 的对标分析

### 6.1 与国际前沿的差距

| 维度 | 国际前沿 (2024-2025) | Tinker 当前 | 差距 |
|---|---|---|---|
| **Reward 项数量** | 趋向极简 (2-10 项) | 26 项 | 偏多 |
| **约束使用** | 主流 (N3PO, CPO) | 已使用 N3PO (5 项) | 对齐 |
| **Reward 形式** | 指数为主 + barrier | 指数 + 平方 + 分段 | 对齐 |
| **自适应权重** | 开始出现 (速度自适应) | 固定权重 | 可探索 |
| **自动 reward 生成** | 实验阶段 (LLM/Video) | 手工设计 | 前沿探索 |
| **层次化** | Motion prior + task reward | 单一层次 | 可探索 |
| **消融实验** | 多数论文标配 | 未系统进行 | 建议补充 |

### 6.2 具体优化建议

1. **合并冗余项**：
   - `tracking_lin_vel` + `tracking_ang_vel` + `track_vel_hard` + `low_speed` 四个速度跟踪项可合并为 1-2 项
   - `torques` + `powers` 可合并（powers 更物理，建议保留 powers、移除 torques）

2. **将 reward 转为 cost**：
   - `hip_pos` 同时作为 reward（scale=-1）和 cost（k=0.1），可统一为 cost 形式
   - `lin_vel_z`, `ang_vel_xy` 这类"不要有"的行为更适合做 constraint

3. **探索能量自适应权重**：
   - powers/torques 的权重可根据 CoT 目标或速度自适应调节

4. **添加步态风格项**：
   - 当前步态通过 `no_jump` + `feet_air_time` + `stand_2leg` 间接实现
   - 可探索 barrier 风格函数或周期性接触奖励以获得更干净的 trot gait

5. **系统化消融实验**：
   - 对 26 项 reward 做 leave-one-out 实验
   - 目标：将 26 项精简到 10-15 项，同时不损失性能

---

## 参考文献

1. Kim et al., "Not Only Rewards but Also Constraints: Applications on Legged Robot Locomotion," IEEE T-RO, 2024.
2. Kim et al., "A Learning Framework for Diverse Legged Robot Locomotion Using Barrier-Based Style Rewards," ICRA, 2025.
3. "Adaptive Energy Regularization for Autonomous Gait Transition and Energy-Efficient Quadruped Locomotion," 2024.
4. Zhang et al., "Motion Priors Reimagined: Adapting Flat-Terrain Skills for Complex Quadruped Mobility," CoRL, 2025.
5. "Video2Reward: Generating Reward Function from Videos for Legged Robot Behavior Learning," ECAI, 2024.
6. Allred et al., "From Walking to Parkour: A Structured Survey of RL for Dynamic Skills in Legged Robots," 2025.
7. Rudin et al., "Learning to Walk in Minutes Using Massively Parallel Deep RL," CoRL, 2022.
8. Margolis et al., "Rapid Locomotion via RL," RSS, 2024.
9. Schulman et al., "Proximal Policy Optimization Algorithms," 2017.
10. Schulman et al., "High-Dimensional Continuous Control Using Generalized Advantage Estimation," ICLR, 2016.
