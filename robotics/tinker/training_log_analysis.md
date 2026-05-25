# Tinker 训练日志参数详解

本文档解释 Tinker 双足机器人 RL 训练（N3PO + PPO）日志中每一项的含义、健康趋势、以及如何把这些数据串成训练过程的诊断分析。

适用于：`train.py` 输出（基于 `LeggedRobotCfgPPO` + `OnConstraintPolicyRunner`，参考 [tinker_constraint_him_trot.py](../../../robotic/OmniBotSeries-Tinker/OmniBotCtrl/OmniBotCtrl/configs/tinker_constraint_him_trot.py)）。

---

## 1. 吞吐 / 性能

| 项目 | 含义 |
|---|---|
| `Computation: 18-19k steps/s` | 每秒模拟 + 推理的环境步数 = `num_envs × num_steps_per_env / iter_time`。**降低**通常是 GPU 抢占或 obs/reward 计算变重 |
| `collection: 1.06s` | rollout 阶段（环境前向）耗时 |
| `learning: 0.21s` | PPO 反向 + 优化耗时 |
| `Iteration time: 1.28s` | 一个完整 PPO 迭代的墙钟时间 |

**诊断**：`collection » learning` 是正常的（env 重）；如果反过来说明 batch 太大或网络太深。

---

## 2. PPO 损失函数

| 项目 | 健康趋势 |
|---|---|
| `Value function loss` | **早期上升**，后期缓慢下降。回报变长 → 价值预测难度增加。87→180 从 0.068→0.092 是正常。如果一直暴涨说明 reward 不稳定 |
| `Surrogate loss` | 经常是**负的**（PPO clip 目标）。绝对值越大表示当前步策略改动越大；接近 0 说明 KL 触发了 clip / 策略稳定 |
| `cost value function loss` | N3PO 特有：约束价值网络的 loss，越小说明约束预测越准 |
| `viol loss` | 约束违反损失。**理想是 0**；恒为 0 说明约束没被违反，cost critic 还在学习中 |

---

## 3. 策略状态

| 项目 | 含义 |
|---|---|
| `Mean action noise std: 0.49 → 0.38` | actor 高斯策略的标准差。**单调下降**是好兆头，表示自适应 KL 让策略逐步收敛、探索减少 |

- 卡在 1.0 不动 → 学习没启动
- 掉到 0.05 太快 → 早熟收敛、容易陷在局部最优

---

## 4. Episode 长度 / 总回报（**最重要的两个**）

| 项目 | 解读 |
|---|---|
| `Mean reward: 2.94 → 9.36` | 单 episode 总折扣回报，**核心指标**。3× 提升说明在学有用东西 |
| `Mean episode length: 62 → 150` | 平均存活步数（max=`episode_length_s/dt`，比如 1000）。**短 → 跌倒早**；长 → 站住了 |

**经验法则**：episode 长度先涨到 max 的 80%+，reward 才有意义比较。短命 episode 的 reward 是被"在死前蹭一点速度奖励"推高的，不代表真学到 gait。

---

## 5. 单项 reward 分量

> 每项 = `weight × ∑_episode(value)`，所以负数代表惩罚，越接近 0（对负项）或越大（对正项）越好。

### A. 任务跟踪（最关心的）

| reward | 87 → 180 | 解读 |
|---|---|---|
| `tracking_lin_vel`   | 0.16 → 0.27 | 线速度跟踪。**正向爬升**，主要 reward 来源 ✅ |
| `tracking_ang_vel`   | 0.09 → 0.18 | 角速度跟踪 ✅ |
| `track_vel_hard`     | -0.011 → -0.011 | 大速度差异时的额外 shaping，仍是惩罚说明高速误差仍存在 |
| `low_speed`          | -0.015 → -0.027 | 速度低于命令的惩罚；存活时间变长 → 累积更多 |

### B. 姿态 / 站立

| reward | 解读 |
|---|---|
| `orientation_eular` 0.022 → 0.055 | 躯干姿态对齐。涨是好的 ✅ |
| `base_height` 0.007 → 0.013 | 维持目标高度（0.30m），涨是好的 ✅ |
| `stand_2leg` 0 | 双腿同时着地 reward（命令非零时不触发） |
| `stand_still_force` 0 | 命令为零时的足底力惩罚（当前没"该站着不动"的命令） |
| `no_jump` -0.002 → -0.006 | 双足同时离地的惩罚，绝对值变大说明跳跃行为有增加 ⚠️ |

### C. 能量 / 平滑（都是惩罚项）

| reward | 含义 |
|---|---|
| `powers`, `torques`, `dof_vel`, `dof_acc` | 能耗 / 速度 / 加速度惩罚。绝对值小（接近 0）= 动作温和 |
| `action_rate`, `action_smoothness` | 相邻动作差异 / 抖动惩罚 |
| `lin_vel_z`, `ang_vel_xy` | 不期望方向的速度（z 方向跳动、绕 x/y 翻滚）惩罚 |

### D. 足部相关

| reward | 解读 |
|---|---|
| `foot_clearance` -0.0004 → -0.0005 | 抬脚高度不足惩罚，小负值 OK |
| `foot_slip`     -0.011 → -0.016 | 足底滑动惩罚，绝对值在涨说明 episode 长了累积更多 |
| `feet_rotation1/2` 0.003 → 0.005 | 足底朝向对齐 reward ✅ |
| `feet_contact_forces` -0.0014 | 接触力惩罚（>max_contact_force=120N 部分） |

### E. LIPM 集成（本次新加的）

| reward | 当前值 | **诊断** |
|---|---|---|
| `contact_schedule` | 0.00167 → **0.00120** ⬇ | 配置 scale=3.0，理论上该是主要 reward 之一。**没起来甚至下降** —— 说明 RL 还没学会按 phase 切换支撑腿 |
| `step_location`    | **0.00001** | 几乎完全是 0。摆动腿离 XCoM 目标点很远（`exp(-d/0.1)`，d>1m 就近乎 0） |

⚠️ **这是当前训练的主要问题** — 见第 8 节诊断。

---

## 6. cost 分量（N3PO 的约束部分）

cost 是**约束**不是 reward —— 数值表示约束违反程度，目标是控制在 `d_value` 阈值以内（见 `class costs.d_values`）。

| cost | 当前 | 阈值 d_value | 状态 |
|---|---|---|---|
| `dof_vel_limits` | 0.027 → 0.033 | 0.0 | 关节速度接近限位，**轻度违反** |
| `feet_air_time`  | 0.006 → 0.007 | 0.1 | 远低于阈值，OK |
| `hip_pos`        | 0.006 → 0.008 | 0.0 | 髋外展角偏离默认，轻度 |
| `pos_limit`, `torque_limit` | ~0.001 / 0 | 0.0 | OK |

`viol loss = 0` + cost 都在低位 = 约束系统目前是"软"在背景里运行。

---

## 7. 如何把这些数据串成"训练分析"

### 步骤 A：先看大局（Episode length + Mean reward）

```
ep_len 62 → 150     ↑ 2.4×       站得更稳了
reward 2.94 → 9.36  ↑ 3.2×       学有效行为
noise  0.49 → 0.38  ↓            策略在收敛
```

✅ 训练整体走在正确方向。

### 步骤 B：定位 reward 的主要贡献

把所有 reward 排序，看谁在拉动总分：

```
tracking_lin_vel  +0.265
tracking_ang_vel  +0.178
orientation       +0.055
base_height       +0.013
... LIPM 项几乎为 0
```

→ **当前训练 90% 靠 tracking_lin/ang_vel 拉**，跟原来的 trot 训练没本质区别，**LIPM 信号还没接入**。

### 步骤 C：看每项的趋势

逐项对比 87 vs 180，关注：
- **应该上升而没升或反向**的正项 → bug 信号
- **应该接近 0 而绝对值变大**的负项 → 行为有偏

在你这里：`contact_schedule 0.00167→0.00120` 反着走、`step_location ≈ 0` 都是红旗。

### 步骤 D：把约束 cost 和 reward 一起看

如果 reward 在涨但 `cost_dof_vel_limits` 也涨得很猛 → 策略在用极端动作堆 reward，sim2real 会崩。当前 cost 涨的幅度温和，OK。

---

## 8. 关于 LIPM 集成的具体诊断

**症状**：`contact_schedule` 和 `step_location` reward 几乎为 0，没有随训练增长。

**可能原因**（按可能性排序）：

1. **step_location_offset 数值过大** —— `exp(-offset / 0.1)`，offset>1m 就接近 0。早期 episode 短、机器人乱动，CoM 速度大 → XCoM 公式预测的 u 飞到几米外（smoke test 里见到过 `step_cmd L = [9.57, -2.17]`）。
2. **reward 权重相对其他项太小** —— scale=3.0 看着大，但因为 exp 项几乎为 0，实际贡献量级 1e-5。
3. **contact_schedule 的 swing 项 mask 用反了** —— 当前实现里 `~self.foot_on_motion` 是 support 脚，乘 `step_location_offset` 时 support 脚的 offset 通常很大（因为 support 脚的 step_commands 槽位是上一次的旧值），会把奖励压成噪声。

**建议下一步排查**：
1. 在 `play.py` 跑一次（带可视化），观察红/蓝六边形（目标）和实际落脚点的距离 —— 是否目标点在合理范围（机器人前方 0.1~0.3m）。
2. log `step_location_offset` 的均值（不是 reward）—— 看它具体是多少，是 5 还是 0.5。
3. 如果 offset 确实大，调整：把 `step_location` 里的 `/0.1` 改为 `/0.3` 或 `/0.5`，给 reward 一个能爬上来的坡。

---

## 附：reward 命名规律对照

reward / cost 通过 `LeggedRobot._prepare_reward_function` 自动注册：
- 配置项 `class rewards.scales.<name> = w` → 调用 `self._reward_<name>()`，乘上权重 `w`
- 配置项 `class costs.scales.<name> = k`  → 调用 `self._cost_<name>()`

日志里 `Mean episode rew_<name>` 就是该 reward 在一个 episode 上的累积平均值（带权重）。
