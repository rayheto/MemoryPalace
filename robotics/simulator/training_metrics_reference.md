# PPO 训练输出参数完全解读

> 源项目：`ModelBasedFootstepPlanning-IROS2024`（IsaacGym + 自实现 PPO）
> 目标：把训练终端打印的每一项数字，从代码里反向考据成精确含义

---

## 0. 一帧典型输出长这样

```
####################################################################################################
                                 Learning iteration 33/5000

                                 Computation: 111406 steps/s (collection: 0.802s, learning 0.080s)
                         Value function loss: 0.0747
                              Surrogate loss: -0.0048
                       Mean action noise std: 0.64
                                 Mean reward: 8.40
                         Mean episode length: 149.21
             Mean episode rew_actuation_rate: -0.2964
                ...
                Mean episode rew_termination: -0.2000
----------------------------------------------------------------------------------------------------
                             Total timesteps: 3342336
                              Iteration time: 0.88s
                                  Total time: 29.58s
                                         ETA: 4322.0s
```

打印位置：`learning/runners/on_policy_runner.py:224-285`（`log_wandb` 函数）。

---

## 1. 性能与进度（4 项）

| 字段 | 代码定义 | 真实含义 |
|---|---|---|
| `Learning iteration X/Y` | `locs['it']` / `tot_iter` | 当前 PPO 迭代序号 / 总迭代数 |
| `Computation: N steps/s` | `num_steps_per_env * num_envs / (collection_time + learn_time)` | 整轮每秒采集的 (env × 时间步) 数。注意是 **rollout + 学习** 的整体吞吐 |
| `collection: A s` | rollout 循环耗时 | 与 sim 交互所用时间；瓶颈是 GPU sim |
| `learning: B s` | `self.alg.update()` 耗时 | PPO 梯度更新；瓶颈是网络规模和 batch |
| `Total timesteps` | `tot_timesteps += num_steps_per_env * num_envs` 累加 | 训练以来所有 env 合计的步数 |
| `Iteration time` | `collection_time + learn_time` | 本轮总耗时 |
| `Total time` | 累加 `iteration_time` | 累计训练墙钟 |
| `ETA` | `(tot_time / (it+1)) × (num_iterations − it)` = 平均每轮耗时 × 剩余轮数 | **预估剩余训练时间**（不是总耗时也不是已耗时）。随训练推进单调下降 |

---

## 2. PPO 损失（3 项核心）

代码位置：`learning/algorithms/ppo.py:119-182`。两个 loss 都是 `num_mini_batches × num_learning_epochs` 次小批更新的平均。

### 2.1 Surrogate loss（actor）

```python
ratio = exp(log_prob_new − log_prob_old)
surrogate         = −A · ratio
surrogate_clipped = −A · clamp(ratio, 1±clip_param)
surrogate_loss    = max(surrogate, surrogate_clipped).mean()
```

PPO clipped surrogate（Schulman 2017）。代码里取 `max` 是因为目标函数整体取了负号（minimize）。

**判断准则**：
- ±0.01~±0.1 是正常波动
- 绝对值持续放大 → 策略在剧烈变化，可能学崩
- 长期 ≈0 → 策略不再更新

#### 2.1.x 本质推导：surrogate 这个名字从哪来

**起源问题**：经典策略梯度
$$
\nabla J(\theta) = \mathbb{E}_{a\sim\pi_\theta}[\, A(s,a)\nabla\log\pi_\theta\,]
$$
期望必须在**当前策略 πθ** 下取，意味着改一次 θ 就要重新采样，IsaacGym 一轮 4096×24 = 10 万 transition 只用一次太浪费。

**重要性采样救场**：用 πold 采样估计 πθ 期望，加比率修正：
$$
L^{\text{IS}}(\theta) = \mathbb{E}_{\text{old}}\big[\, r(\theta)\cdot A \,\big],\quad r(\theta)=\frac{\pi_\theta}{\pi_{\text{old}}}
$$
这就是 "surrogate"（代理）—— 它**代替**真实目标 J，让你用旧数据多次 SGD（PPO 默认 `num_learning_epochs × num_mini_batches = 5 × 4 = 20` 次更新）。

**致命陷阱**：r 漂移 → 比率爆炸 → 策略崩。TRPO 用 KL 约束（难算），PPO 直接砍：

$$
L^{\text{CLIP}} = \mathbb{E}[\,\min(r\cdot A,\ \text{clip}(r,1\pm\epsilon)\cdot A)\,]
$$

**取 min 是悲观策略**（只在"反方向"夹梯度）：

| 情况 | 不裁剪 r·A | 裁剪 | min 取哪个 |
|---|---|---|---|
| A>0, r 偏大（想加大概率） | 大正 | 被夹 | 夹后的 → 防过激 |
| A>0, r 偏小 | 小正 | 同 | — |
| A<0, r 偏大（想加大概率但坏动作）| 大负 | 同大负 | 大负 → 严罚 |
| A<0, r 偏小（在缩小概率，好的方向）| 小负 | 被夹 | 小负 → 不奖励过度退缩 |

**代码 → 论文的负号变换**：论文 maximize `min(r·A, clip·A)`，代码里 minimize，所以加负号后用 `max(-x,-y) = -min(x,y)` 等价改写。

#### 2.1.y 打印数字怎么读

| 现象 | 含义 |
|---|---|
| 略负（~-0.01） | 正常。advantage 已标准化（mean≈0），少数好动作贡献负 loss（= L^CLIP 正） |
| 越来越负 | L^CLIP ↑，策略持续改进 |
| 围绕 0 抖动 | 学不动 / 已收敛 |
| 突然大正大负 | 极端 advantage，可能不稳 |

**关键认知**：这是 surrogate 目标，**不是回归 loss，不要 minimize 到 0**。符号和动态反映"策略往哪个方向改、改得有多狠"。

#### 2.1.z 与 Value loss 的本质区别

- **Value loss**：critic 的回归 loss，目标是预测准 returns，**应该单调下降**
- **Surrogate loss**：actor 的策略改进代理量，**符号和大小都反映训练动态**，没有"理想值"

### 2.2 Value function loss（critic）

```python
value_clipped = target_v + clamp(value − target_v, ±clip_param)
value_loss = max((value − returns)², (value_clipped − returns)²).mean()
```

PPO 裁剪版 critic MSE。`use_clipped_value_loss=True` 默认开启，取裁剪和非裁剪两者**更大**的（更保守的估计）。

**判断准则**：
- 前期高（>1），后期应单调下降并收敛到小数
- 持续高 → reward 方差太大，critic 学不动

### 2.3 Mean action noise std

```python
mean_std = self.alg.actor_critic.std.mean()
```

actor 输出的高斯策略 std 向量（**可学参数**），沿动作维取平均。

**判断准则**：
- 初始 ≈1.0，正常训练应缓慢下降到 0.1~0.3
- 长期 >0.8 → 没收敛
- <0.05 → 几乎不再探索，过早收敛

### 2.4 隐藏机制：自适应 KL → LR

`learning/algorithms/ppo.py:135-147` 还跑了一个 KL 触发的 LR 调度：

```
if kl > desired_kl * 2:   lr = max(1e-5, lr / 1.5)
elif kl < desired_kl/2:   lr = min(1e-2, lr * 1.5)
```

这个**不打印**，但默默在跑。`Loss/learning_rate` 在 wandb 里有记录。

---

## 3. 整体表现（2 项）

代码位置：`learning/runners/on_policy_runner.py:142-146, 250-251`。

`rewbuffer` 和 `lenbuffer` 都是 `deque(maxlen=100)`，每次有 env `done` 时把它累计的 reward 和 length 推进去。

| 字段 | 真实含义 |
|---|---|
| `Mean reward` | **最近 100 个完结 episode** 的累计 reward 平均（不是滚动 100 步）|
| `Mean episode length` | 最近 100 个 episode 的**控制步数**平均 |

### 关于"控制步" vs "秒"

- `env.step()` 调用一次 = 一个**控制步**
- 一个控制步对应 `dt × decimation = 0.001 × 10 = 0.01 s = 10 ms` 仿真时间
- 配置在 `gym/envs/humanoid/humanoid_controller_config.py:212` 和 `gym/envs/base/legged_robot_config.py:209`
- `episode_length_s = 5` → 上限 500 控制步 = 5 秒仿真时间

所以 `Mean episode length = 149.21` ≈ **1.5 秒仿真时间**，到 500 才是不摔倒、撑满整 episode。

---

## 4. 各项 reward `rew_*`（最关键）

### 4.1 显示的数字到底是什么单位？

三处代码联动：

**`base_task.py:166`** — 启动时把所有 reward 权重一次性乘 dt：
```python
self.reward_weights[name] *= self.dt
```

**`base_task.py:187-188`** — 每步累加：
```python
rew = self.reward_weights[name] * self.eval_reward(name)  # = w·dt·raw_value
self.episode_sums[name] += rew
```

**`legged_robot.py:187`** — 重置时除以 `T_max`：
```python
self.extras["episode"]['rew_' + key] =
    torch.mean(self.episode_sums[key][env_ids]) / self.max_episode_length_s
```

合起来：

$$
\text{display}(\text{rew}_x) = \frac{\overline{\sum_t w_x \cdot dt \cdot \text{raw}_t}}{T_{\max}}
$$

如果 episode 撑满 `T_max`：display ≈ `w · mean(raw)`，即权重 × 时间平均 raw 信号。

如果中途摔倒：display 会按比例缩水（因为分子小，分母仍 `T_max`）。

**注意**：`termination` 类项**不会**乘 dt（见 `base_task.py:169-173`）。

### 4.2 正向跟踪类（越大越好）

权重见 `gym/envs/humanoid/humanoid_controller_config.py:312-334`。

辅助函数（`gym/envs/base/legged_robot.py:1010-1020`）：
```python
_neg_exp(x, a)    = exp(−(x/a)        /σ)    # σ=0.25
_negsqrd_exp(x,a) = exp(−(x/a)²       /σ)
```

| 名称 | 权重 w | raw 公式 | raw 最大 | 显示满分 | 代码 |
|---|---|---|---|---|---|
| `tracking_lin_vel_world` | **4.0** | `Σᵢ exp(−(eᵢ/(1+\|cmdᵢ\|))²/σ)`, e=cmd_vxy−base_vxy, **xy sum** | 2 | **8** | `humanoid_controller.py:798` |
| `base_heading` | 3.0 | `exp(−\|err\|/(π/2)/σ)` | 1 | 3 | `humanoid_controller.py:786` |
| `contact_schedule` | **3.0** | `(c_R−c_L)·sched · 3·exp(−offset/σ)` | 3 | **9** | `humanoid_controller.py:823` |
| `base_height` | 1.0 | `exp(−(0.62−h)²/σ)` | 1 | 1 | `humanoid_controller.py:781` |
| `base_z_orientation` | 1.0 | `exp(−(\|proj_grav_xy\|/0.2)²/σ)` | 1 | 1 | `humanoid_controller.py:793` |
| `joint_regularization` | 1.0 | 4 个 yaw/abad 关节 negsqrd_exp 的平均 | 1 | 1 | `humanoid_controller.py:806` |

> 注意：`tracking_lin_vel_world` 末尾的 `.sum(dim=1)` 让 raw 上限 = 2（xy 各贡献 1）；`contact_schedule` 内部又乘了 k=3，raw 上限 = 3。其他都是 1。**显示满分 = w × raw_max**，前提是 episode 撑满 T_max。

### 4.3 惩罚类（负值，越接近 0 越好）

| 名称 | 权重 w | raw 公式（已带负号）| 代码 |
|---|---|---|---|
| `dof_pos_limits` | **10** | `−Σ max(0, q−q_max) + max(0, q_min−q)` | `legged_robot.py:968` |
| `actuation_rate` | 1e-3 | `−Σ ((aₜ−aₜ₋₁)/dt)²` | `legged_robot.py:940` |
| `actuation_rate2` | 1e-4 | `−Σ ((aₜ−2aₜ₋₁+aₜ₋₂)/dt)²` | `legged_robot.py:948` |
| `torques` | 1e-4 | `−Σ τᵢ²` | `legged_robot.py:930` |
| `dof_vel` | 1e-3 | `−Σ q̇ᵢ²` | `legged_robot.py:935` |
| `lin_vel_z` | 1e-1 | `−vz²` | `legged_robot.py:909` |
| `ang_vel_xy` | 1e-2 | `−(ωx²+ωy²)` | `legged_robot.py:914` |
| `torque_limits` | 1e-2 | `−Σ max(0, \|τ\|−0.8·τ_max)` | `legged_robot.py:981` |

`dof_pos_limits` 权重 10 远大于其他惩罚 —— 撞硬限位是最严重的负向反馈。

### 4.4 终止项

| 名称 | 权重 w | raw 公式 | 代码 |
|---|---|---|---|
| `termination` | 1.0 | `−(reset_buf & ~timed_out)` | `legged_robot.py:963` |

提前重置（摔倒等）惩罚 −1；超时不罚。**不乘 dt**。display 值大致反映"提前死的 episode 占比 / T_max"。

---

## 5. 诊断流程（实战 checklist）

按以下顺序看，定位训练状态：

1. **`Mean episode length` → 500？**
   - 越接近上限 = 越不摔。最强先验指标
2. **`rew_termination` → 0？**
   - 接近 0 = 不再被提前死惩罚
3. **`rew_tracking_lin_vel_world` → 8？**
   - 满分 8（raw_max=2 × w=4）。这是主任务的核心
4. **`rew_contact_schedule` → 9？**
   - 满分 9（raw_max=3 × w=3）。步态成形指标
5. **某个惩罚项 |绝对值| 是不是压死了主任务？**
   - 比如 `rew_dof_pos_limits` 权重 10 最高，最容易主导
6. **`Value function loss` 是否持续下降？**
   - 不下降 → reward 方差太大或网络容量不足
7. **`Mean action noise std` 是否缓慢下降？**
   - 不降 → 没收敛；骤降到 <0.05 → 过早收敛

### 阶段性预期

| 阶段 | 表现 |
|---|---|
| iter 0~100 | reward 很低、length 短、std 高，正常 |
| iter 100~500 | length 开始爬升，先学站立平衡 |
| iter 500~1500 | length 接近上限，开始有节律踏步，tracking 上升 |
| iter 1500~3000 | tracking_lin_vel_world 显著提升，步态稳定 |
| iter 3000+ | README 说此时策略基本可用 |

### 实测：4090 D + cu118 全量训完（5000 iter）

| 指标 | 值 | 解读 |
|---|---|---|
| 总时间 | 4216 s ≈ 70 min | 4090 D，PyTorch 2.0.1 + cu118 |
| 吞吐 | ~118k steps/s | rollout 0.75s + learn 0.08s 一轮 |
| Mean episode length | 501 / 500 | 完全不摔 |
| Mean reward | 90.4 | 早期 8.4 → 11× |
| tracking_lin_vel_world | 7.22 / 8 | 90% 跟踪精度 |
| contact_schedule | 6.51 / 9 | 72%，步态成形良好 |
| base_heading | 2.52 / 3 | 84% 朝向跟踪 |
| Mean action noise std | 0.38 | 从 ~1.0 → 0.38，收敛中 |
| Value loss | 0.18 | 平稳低值 |
| Surrogate loss | -0.0048 | 略负，健康 |
| rew_termination | -0.0042 | 几乎不摔 |

---

## 6. 速查表

```
吞吐：       Computation                       (steps/s)
PPO 健康度： Value loss / Surrogate / std      (是否在合理收敛)
存活：       Mean episode length × 0.01s       (秒)
主任务：     rew_tracking_lin_vel_world        (满分 4)
步态：       rew_contact_schedule              (满分 3)
姿态：       rew_base_height/heading/orient    (各满分 1~3)
节能：       rew_torques/dof_vel/actuation*    (越接近 0 越好)
极限：       rew_dof_pos_limits                (权重 10，极度敏感)
死亡率：     rew_termination                    (越接近 0 越好)
```

---

## 7. 相关代码索引

| 内容 | 文件 |
|---|---|
| 终端 print 格式 | `learning/runners/on_policy_runner.py:224-285` |
| PPO update | `learning/algorithms/ppo.py:119-182` |
| reward 累加机制 | `gym/envs/base/base_task.py:155-197` |
| episode 重置时上报 | `gym/envs/base/legged_robot.py:180-188` |
| 通用 reward 函数 | `gym/envs/base/legged_robot.py:909-1007` |
| humanoid 专用 reward | `gym/envs/humanoid/humanoid_controller.py:781-830` |
| reward 权重配置 | `gym/envs/humanoid/humanoid_controller_config.py:312-334` |
| dt / decimation / episode_length 配置 | `gym/envs/humanoid/humanoid_controller_config.py:13-16, 209-212` |
