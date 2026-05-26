# 训练过程中奖励函数参与权重调整的完整机制

本文档详细解释 Tinker 双足机器人训练中，奖励函数如何从环境步的标量值，经过 GAE 优势估计、PPO surrogate loss、NP3O 约束惩罚，最终影响神经网络权重更新的完整数据流。

适用代码库：`OmniBotCtrl/OmniBotCtrl`，算法 NP3O（N-step PPO with Constraint Optimization）。

---

## 1. 总览：从 reward 到 weight update 的五个阶段

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ 1. Reward     │ ──→ │ 2. GAE       │ ──→ │ 3. Advantage  │ ──→ │ 4. Surrogate  │ ──→ │ 5. Gradient   │
│    Computation│     │    Returns    │     │    + Ratio    │     │    Loss       │     │    Descent    │
└──────────────┘     └──────────────┘     └──────────────┘     └──────────────┘     └──────────────┘
    环境步级别                轨迹级别                小批次级别               Loss 级别                参数级别
```

---

## 2. 阶段一：Reward 的合成（`compute_reward()`）

### 2.1 数据流

文件：[legged_robot.py:1144-1161](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1144)

```python
def compute_reward(self):
    self.rew_buf[:] = 0.
    for i in range(len(self.reward_functions)):
        name = self.reward_names[i]
        rew = self.reward_functions[i]() * self.reward_scales[name]
        self.rew_buf += rew
        self.episode_sums[name] += rew
    if self.cfg.rewards.only_positive_rewards:
        self.rew_buf[:] = torch.clip(self.rew_buf[:], min=0.)
    if "termination" in self.reward_scales:
        rew = self._reward_termination() * self.reward_scales["termination"]
        self.rew_buf += rew
        self.episode_sums["termination"] += rew
```

**关键细节**：

| 步骤 | 说明 |
|---|---|
| 预处理 | `_prepare_reward_function()` 将所有 `reward_scales[name]` 乘以 `self.dt`（控制周期），即真实权重 = 配置权重 × dt |
| 合成方式 | **线性加权和**：$r_t = \sum_i w_i \cdot r_{i,t}$，其中 $w_i$ 是 `reward_scales[name]`，$r_{i,t}$ 是 `_reward_<name>()` |
| `only_positive_rewards` | Trot 配置**未启用**（默认 False），所以负向奖励直接保留 |
| termination | **最后单独加**，不受 only_positive_rewards 裁剪影响 |

### 2.2 奖励函数权重转换

配置文件 `tinker_constraint_him_trot.py:177-208` 中的 `scales`（如 `tracking_lin_vel = 2.5`）会在 `_prepare_reward_function()` 中乘以 `dt`：

$$w_i^{\text{effective}} = w_i^{\text{config}} \times dt$$

其中 $dt = \text{decimation} \times \text{sim\_dt} = 4 \times 0.005s = 0.02s$。

所以 $w_{\text{tracking\_lin\_vel}}^{\text{effective}} = 2.5 \times 0.02 = 0.05$。

**为什么乘 dt？** 保证 reward 的尺度不随控制频率变化。假设 $r_t$ 是瞬时值，累积一 episode 为 $\sum r_t \cdot dt \approx \int r(t)dt$。

---

## 3. 阶段二：环境步执行与 GAE 回报计算

### 3.1 从 step 到 storage

文件：[on_constraint_policy_runner.py:143-150](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/runner/on_constraint_policy_runner.py#L143)

```python
for i in range(self.num_steps_per_env):  # num_steps_per_env = 24
    actions = self.alg.act(obs, critic_obs, infos)
    obs, privileged_obs, rewards, costs, dones, infos = self.env.step(actions)
    self.alg.process_env_step(rewards, costs, dones, infos)
```

`process_env_step()` 在 [np3o.py:112-122](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/algorithm/np3o.py#L112)：

```python
def process_env_step(self, rewards, costs, dones, infos):
    self.transition.rewards = rewards.clone()
    self.transition.costs = costs.clone()
    # Timeout bootstrapping：超时截断时不把截断当终止
    if 'time_outs' in infos:
        self.transition.rewards += self.gamma * torch.squeeze(
            self.transition.values * infos['time_outs'].unsqueeze(1), 1)
    self.storage.add_transitions(self.transition)
```

**关键点：timeout bootstrapping**——当 episode 因为超时（max_episode_length）而非真实跌倒终止时，用当前价值函数估计引导 reward：
$$r_t^{\text{bootstrapped}} = r_t + \gamma \cdot V(s_{t+1}) \cdot \mathbb{1}[\text{timeout}]$$

这避免了把"时间到了"当作"任务失败"。

### 3.2 GAE (Generalized Advantage Estimation)

文件：[np3o.py:126-132](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/algorithm/np3o.py#L126)

```python
def compute_returns(self, last_critic_obs):
    last_values = self.actor_critic.evaluate(last_critic_obs).detach()
    self.storage.compute_returns(last_values, self.gamma, self.lam)
```

GAE 在 `RolloutStorageWithCost.compute_returns()` 中实现，使用标准公式：

$$\hat{A}_t = \delta_t + (\gamma\lambda)\delta_{t+1} + \dots + (\gamma\lambda)^{T-t-1}\delta_{T-1}$$

其中 TD 误差：
$$\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)$$

回报（returns）：
$$R_t = \hat{A}_t + V(s_t)$$

**参数**（来自 `tinker_constraint_him_trot.py`）：
- $\gamma = 0.98$：折扣因子
- $\lambda = 0.95$：GAE 的 TD-$\lambda$ 参数。越接近 1，$A_t$ 越依赖长期回报但方差更大

### 3.3 Cost 的独立 GAE

```python
def compute_cost_returns(self, obs):
    last_cost_values = self.actor_critic.evaluate_cost(obs).detach()
    self.storage.compute_cost_returns(last_cost_values, self.gamma, self.lam)
```

Cost 有**独立的价值网络**和 GAE 计算管道，完全平行于 reward。这意味着策略网络学习"一条从 rewards 提取信号的路径"和"一条从 costs 提取信号的路径"，两者在最终 loss 中汇合。

---

## 4. 阶段三：从 mini-batch 到 Surrogate Loss

### 4.1 Mini-batch 采样

文件：[np3o.py:195-196](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/algorithm/np3o.py#L195)

```python
# num_learning_epochs = 5, num_mini_batches = 4
# total_updates_per_iteration = 5 * 4 = 20
generator = self.storage.mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)
```

每个 PPO 迭代：
- 收集 $24 \times 1024 = 24576$ 个 transitions
- 进行 $5 \times 4 = 20$ 次梯度更新
- 每次更新在一个 mini-batch（$\frac{24576}{4} = 6144$ samples）上进行

### 4.2 Surrogate Loss（策略梯度核心）

文件：[np3o.py:134-140](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/algorithm/np3o.py#L134)

```python
def compute_surrogate_loss(self, actions_log_prob_batch, old_actions_log_prob_batch, advantages_batch):
    ratio = torch.exp(actions_log_prob_batch - old_actions_log_prob_batch)
    surrogate = -advantages_batch * ratio
    surrogate_clipped = -advantages_batch * torch.clamp(ratio, 1.0 - self.clip_param, 1.0 + self.clip_param)
    surrogate_loss = torch.max(surrogate, surrogate_clipped).mean()
    return surrogate_loss
```

数学表达：

$$\mathcal{L}^{\text{surr}} = \mathbb{E}_t\left[\max\left(-A_t \cdot r_t(\theta),\; -A_t \cdot \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon)\right)\right]$$

其中：
- $r_t(\theta) = \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{\text{old}}}(a_t|s_t)}$：新旧策略的概率比
- $A_t$：GAE 优势函数（**这是 reward 参与权重调整的关键入口**）
- $\epsilon = 0.2$：clip 参数

**Reward 如何影响梯度**：
- 如果 $A_t > 0$（动作比预期好），loss 推动 $r_t(\theta)$ 增大（加强该动作概率）
- 如果 $A_t < 0$（动作比预期差），loss 推动 $r_t(\theta)$ 减小（削弱该动作概率）
- clip 机制防止单步更新过大

**Reward 的信号链**：
$$r_t \xrightarrow{\text{GAE}} A_t \xrightarrow{\times(-\log\pi)} \nabla_\theta\mathcal{L} \xrightarrow{\text{Adam}} \Delta\theta$$

## 5. 阶段四：完整 Loss 函数

文件：[np3o.py:241-249](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/algorithm/np3o.py#L241)

```python
main_loss = surrogate_loss + self.cost_viol_loss_coef * viol_loss
combine_value_loss = self.cost_value_loss_coef * cost_value_loss + self.value_loss_coef * value_loss
entropy_loss = -self.entropy_coef * entropy_batch.mean()
loss = main_loss + combine_value_loss + entropy_loss
```

完整 loss：

$$\mathcal{L} = \underbrace{\mathcal{L}^{\text{surr}}}_{\text{reward 信号}} + \underbrace{w_{\text{viol}} \cdot \mathcal{L}^{\text{viol}}}_{\text{cost 约束信号}} + \underbrace{\mathcal{L}^{\text{value}}}_{\text{reward critic}} + \underbrace{w_{\text{cost\_v}} \cdot \mathcal{L}^{\text{cost\_value}}}_{\text{cost critic}} + \underbrace{(-w_{\text{ent}} \cdot H)}_{\text{探索}}$$

其中：
- $\mathcal{L}^{\text{value}} = \max((V - R)^2, (V_{\text{clip}} - R)^2)$：**reward 价值网络**
- $\mathcal{L}^{\text{cost\_value}}$：cost 价值网络，同公式但用 cost returns
- $\mathcal{L}^{\text{viol}}$：NP3O 约束违反损失（见下节）
- $w_{\text{ent}} = 0.001$

### 5.1 各 Loss 项如何参与权重更新

| Loss 项 | 影响的参数 | Reward 是否参与 |
|---|---|---|
| `surrogate_loss` | Actor（策略网络） | **是**——通过 $A_t$ 进入 |
| `value_loss` | Critic（价值网络） | **是**——监督信号是 GAE returns $R_t$ |
| `cost_value_loss` | Cost Critic | 否——监督信号是 cost returns |
| `viol_loss` | Actor | 否——通过 cost advantages 进入 |
| `entropy_loss` | Actor | 否——仅依赖策略分布 |
| `imitation_loss` (可选) | Actor | 否——监督信号是 Barlow Twins 自监督 |

### 5.2 不同 reward 项的权重如何影响学习

每个 reward 项 $r_i$ 的权重 $w_i$ 决定了三个层面：

1. **信号强度**：$w_i \cdot r_{i,t}$ 直接改变 $r_t$ 的大小 → 改变 $\delta_t$ → 改变 $A_t$ → 改变梯度强度
2. **主导性竞争**：由于 $r_t$ 是线性求和，权重大的项在梯度中贡献更大比例
3. **符号作用**：正权重（$w_i > 0$）是鼓励，负权重（$w_i < 0$）是惩罚。策略会朝"增大正项 + 减小负项"的方向优化

---

## 6. 阶段五：NP3O 的 Cost 约束机制（viol_loss）

### 6.1 Lagrangian 乘子法

文件：[np3o.py:168-182](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/algorithm/np3o.py#L168)

```python
def compute_viol(self, actions_log_prob_batch, old_actions_log_prob_batch,
                 cost_advantages_batch, cost_volation_batch):
    cost_surrogate_loss = self.compute_cost_surrogate_loss(...)
    cost_volation_loss = cost_volation_batch.mean()  # cost returns over threshold
    cost_loss = cost_surrogate_loss + cost_volation_loss
    cost_loss = torch.sum(self.k_value * F.relu(cost_loss))
    return cost_loss
```

这实现了 Lagrangian 约束优化：
$$\mathcal{L}^{\text{viol}} = \sum_i k_i \cdot \text{ReLU}\left(\underbrace{\mathcal{L}^{\text{cost\_surr}}_i}_{\text{策略对 cost 的影响}} + \underbrace{\bar{C}_i}_{\text{cost 均值}}\right)$$

其中 $k_i$ 是 Lagrange 乘子（`costs.scales` 中的值），$\bar{C}_i$ 是 constraints 的 episode 均值。

### 6.2 K-value 的渐进收紧

```python
def update_k_value(self, i):
    self.k_value = torch.min(torch.ones_like(self.k_value), self.k_value * (1.0004**i))
```

每迭代 k_value 乘以 $1.0004^i$，最多不超过 1，实现软性渐进收紧。

### 6.3 Reward 和 Cost 的协同

Reward 最大化 vs Cost 约束形成了一个**带约束的优化问题**：
$$\max_\theta \mathbb{E}[R] \quad \text{s.t.} \quad \mathbb{E}[C_i] \leq d_i$$

通过 Lagrangian 转化为无约束问题：
$$\max_\theta \min_{k \geq 0} \mathbb{E}[R] - \sum_i k_i \cdot (\mathbb{E}[C_i] - d_i)$$

**实际梯度流**：
- Actor 参数同时收到来自 reward 的**推力**（增大 $R$）和来自 cost 的**拉力**（减小 $C_i$）
- 当 cost 超过阈值 $d_i$ 时，viol_loss > 0，拉力激活
- 当 cost 在阈值内，ReLU 输出 0，拉力关闭

### 6.4 Tinker Trot 配置的 Active Cost 约束

来自 `tinker_constraint_him_trot.py:301-325`：

| Cost | k_value | d_value | 含义 |
|---|---|---|---|
| `pos_limit` | 0.1 | 0.0 | 关节位置不允许超限 |
| `torque_limit` | 0.1 | 0.0 | 关节力矩不允许超限 |
| `dof_vel_limits` | 0.1 | 0.0 | 关节速度不允许超限 |
| `feet_air_time` | 0.1 | 0.1 | 支撑相不短于目标-0.1s |
| `hip_pos` | 0.1 | 0.0 | 髋侧展角不偏离默认值 |

---

## 7. 自适应 KL 学习率调度

文件：[np3o.py:207-219](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/algorithm/np3o.py#L207)

```python
if self.desired_kl != None and self.schedule == 'adaptive':
    with torch.inference_mode():
        kl = torch.sum(torch.log(sigma_batch / old_sigma_batch + 1.e-5)
            + (torch.square(old_sigma_batch) + torch.square(old_mu_batch - mu_batch))
            / (2.0 * torch.square(sigma_batch)) - 0.5, axis=-1)
        kl_mean = torch.mean(kl)
        if kl_mean > self.desired_kl * 2.0:
            self.learning_rate = max(1e-5, self.learning_rate / 1.5)
        elif kl_mean < self.desired_kl / 2.0 and kl_mean > 0.0:
            self.learning_rate = min(1e-2, self.learning_rate * 1.5)
```

**作用**：如果策略变化太大（KL > 0.02），降低学习率；如果策略变化太小（KL < 0.005），增大学习率。这间接控制了 reward 信号对参数的影响速度。

---

## 8. 完整的数据流示意图

```
Config (tinker_constraint_him_trot.py)
  │
  ├── rewards.scales = {tracking_lin_vel: 2.5, tracking_ang_vel: 2.0, ...} (26项)
  │   └── × dt = 0.02 ──→ effective weights
  │
  └── costs.scales = {pos_limit: 0.1, ...} (5项) ──→ k_values

                    ┌─────────────────────────────────┐
                    │  compute_reward() 每步调用        │
                    │                                  │
                    │  r_t = Σ w_i × _reward_i()       │
                    │  rew_buf: [num_envs]              │
                    └──────────────┬──────────────────┘
                                   │
                    ┌──────────────▼──────────────────┐
                    │  process_env_step()              │
                    │                                  │
                    │  rewards → storage (buffer)      │
                    └──────────────┬──────────────────┘
                                   │ (24 steps collected)
                    ┌──────────────▼──────────────────┐
                    │  compute_returns()  GAE          │
                    │                                  │
                    │  A_t = Σ (γλ)^k δ_{t+k}          │
                    │  R_t = A_t + V(s_t)              │
                    │  cost returns (独立计算)           │
                    └──────────────┬──────────────────┘
                                   │ (mini-batch sampling, 5 epochs × 4 mini-batches)
                    ┌──────────────▼──────────────────┐
                    │  update() - Loss 计算             │
                    │                                  │
                    │  surrogate_loss = max(           │
                    │    -A_t × ratio,                 │
                    │    -A_t × clip(ratio, 0.8, 1.2)  │
                    │  )                               │
                    │  value_loss = clip_MSE(V, R)     │
                    │  viol_loss = Σ k × ReLU(         │
                    │    cost_surr + cost_returns      │
                    │  )                               │
                    │  cost_value_loss = clip_MSE(Vc,Rc)│
                    │  entropy_loss = -w_e × H(π)      │
                    │                                  │
                    │  L = surrogate + w_v×viol         │
                    │    + value + cost_value          │
                    │    + entropy                     │
                    └──────────────┬──────────────────┘
                                   │
                    ┌──────────────▼──────────────────┐
                    │  optimizer.step()                │
                    │                                  │
                    │  θ_actor += η × ∂L/∂θ_actor     │
                    │  θ_critic += η × ∂L/∂θ_critic   │
                    │  θ_cost_critic += η × ∂L/∂θ_cost │
                    └──────────────────────────────────┘
```

---

## 9. 实际日志中 reward 数值的解读

以用户提供的日志片段（iteration 21999）为例：

```
Mean reward: 99.00
```

这是 `rewbuffer`（最近 100 episode 的 total reward 均值）。注意这里的 reward 是**累积和** $\sum_{t} r_t$（不是平均值），所以与 episode 长度正相关。

各分项 `Mean episode rew_<name>` 是**单 episode 平均**（除以 `max_episode_length_s`），即 $\frac{1}{T_{\text{max}}} \sum_{t} w_i \cdot r_{i,t}$。

**解读关键关系**：
- `rew_tracking_lin_vel = 2.18459` 是最大贡献项 → 策略主要被线速度跟踪奖励驱动
- `rew_orientation_eular = 0.83483` 是第二大贡献项 → 姿态维持很重要
- 负项如 `rew_dof_vel = -0.20951` 表示策略被惩罚关节速度，绝对值越大越受约束
- `rew_termination = -0.00036` 接近 0 → 跌倒很少，episode 基本能跑满

---

## 10. 关键参数速查表

| 参数 | 值 | 含义 |
|---|---|---|
| $\gamma$ | 0.98 | 折扣因子 |
| $\lambda$ (GAE) | 0.95 | GAE 平滑参数 |
| $\epsilon$ (clip) | 0.2 | PPO clip 范围 |
| $w_{\text{ent}}$ | 0.001 | 熵系数 |
| num_steps_per_env | 24 | 每次 rollout 长度 |
| num_learning_epochs | 5 | 每批数据重复学习次数 |
| num_mini_batches | 4 | mini-batch 分割数 |
| lr (initial) | 1e-4 | 初始学习率 |
| adaptive KL target | 0.01 | 目标 KL 散度 |
| cost_viol_loss_coef | 1.0 | violation loss 权重 |
| cost_value_loss_coef | 1.0 | cost critic loss 权重 |
| value_loss_coef | 1.0 | reward critic loss 权重 |
