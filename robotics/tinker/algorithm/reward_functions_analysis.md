# Tinker 奖励函数完整分析：物理意义、数学公式与代码映射

本文档对 Tinker（Trot 配置）当前使用的 **26 个奖励函数** 和 **5 个约束函数** 进行逐一分析，每个函数包含物理意义、LaTeX 公式、源码实现、以及代码与公式的逐步对应分析。

配置来源：[tinker_constraint_him_trot.py](../../../robotic/OmniBotSeries-Tinker/OmniBotCtrl/OmniBotCtrl/configs/tinker_constraint_him_trot.py)
源码实现：[legged_robot.py](../../../robotic/OmniBotSeries-Tinker/OmniBotCtrl/OmniBotCtrl/envs/legged_robot.py)

> **符号约定**：$\mathbf{v}_b = [v_{bx}, v_{by}, v_{bz}]$ 为基座线速度，$\boldsymbol{\omega}_b = [\omega_{bx}, \omega_{by}, \omega_{bz}]$ 为基座角速度，$\mathbf{c} = [c_x, c_y, c_\omega]$ 为速度命令，$\mathbf{g}_{\text{proj}}$ 为投影重力向量。

---

## 目录

1. [任务跟踪类 (Task Tracking)](#1-任务跟踪类-task-tracking) — 4 项
2. [姿态/基座类 (Posture & Base)](#2-姿态基座类-posture--base) — 4 项
3. [正则化/能量类 (Regularization & Energy)](#3-正则化能量类-regularization--energy) — 7 项
4. [足部运动类 (Foot Motion)](#4-足部运动类-foot-motion) — 7 项
5. [接触/站立类 (Contact & Standing)](#5-接触站立类-contact--standing) — 4 项
6. [终止类 (Termination)](#6-终止类-termination) — 1 项
7. [约束函数 (Cost Functions)](#7-约束函数-cost-functions) — 5 项

---

## 1. 任务跟踪类 (Task Tracking)

### 1.1 `_reward_tracking_lin_vel` — 线速度跟踪

**配置权重**：$w = 2.5$（正，鼓励）

**物理意义**：鼓励机器人基座 xy 平面线速度尽可能接近用户命令速度。这是训练中**最重要的任务级 reward**，直接定义了"去往指定方向"的行为目标。没有这个 reward，机器人不会学到有目的的运动。

**物理公式**：

$$r_{\text{track\_lin}} = \exp\left(-\sigma \cdot \|\mathbf{c}_{xy} - \mathbf{v}_{b,xy}\|^2\right)$$

其中 $\sigma = 0.5$ 为 `tracking_sigma` 参数，控制指数衰减的锐度。

**源码**（[legged_robot.py:1959-1966](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1959)）：
```python
def _reward_tracking_lin_vel(self):
    lin_vel_error = torch.sum(torch.square(
        self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
    return torch.exp(-lin_vel_error * self.cfg.rewards.tracking_sigma)
```

**代码与公式对应分析**：

| 步骤 | 代码 | 公式 |
|---|---|---|
| 误差计算 | `self.commands[:, :2] - self.base_lin_vel[:, :2]` | $\mathbf{c}_{xy} - \mathbf{v}_{b,xy}$ |
| 平方和 | `torch.sum(torch.square(...), dim=1)` | $\|\mathbf{c}_{xy} - \mathbf{v}_{b,xy}\|^2$ |
| 指数衰减 | `torch.exp(-lin_vel_error * tracking_sigma)` | $\exp(-\sigma \cdot \|\mathbf{c}_{xy} - \mathbf{v}_{b,xy}\|^2)$ |

- 当 $\|\mathbf{c}_{xy} - \mathbf{v}_{b,xy}\|^2 = 0$（完全匹配），$r = 1.0$（最大）
- 当误差增大，$r \to 0$（指数衰减）
- $\sigma$ 越大，衰减越快（对误差越敏感）

---

### 1.2 `_reward_tracking_ang_vel` — 角速度跟踪

**配置权重**：$w = 2.0$（正，鼓励）

**物理意义**：鼓励机器人绕 z 轴（yaw）的角速度尽可能接近命令角速度。

**物理公式**：

$$r_{\text{track\_ang}} = \exp\left(-\sigma \cdot (c_\omega - \omega_{bz})^2\right)$$

**源码**（[legged_robot.py:1968-1976](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1968)）：
```python
def _reward_tracking_ang_vel(self):
    ang_vel_error = torch.square(
        self.commands[:, 2] - self.base_ang_vel[:, 2])
    return torch.exp(-ang_vel_error * self.cfg.rewards.tracking_sigma)
```

**代码与公式对应**：与 `tracking_lin_vel` 结构完全一致，只是误差从 2 维向量差变为标量差（仅 yaw 轴）。

---

### 1.3 `_reward_track_vel_hard` — 硬速度跟踪

**配置权重**：$w = 0.5$（正，鼓励）

**物理意义**：对速度跟踪的"硬"版本，使用指数奖励 + 线性惩罚的混合形式。当速度误差中等时指数项主导，当误差大时线性惩罚项介入，提供一个"有坡度的梯子"帮助策略从大误差中恢复。

**物理公式**：

$$r_{\text{hard}} = \frac{1}{2}\left[e^{-10 \cdot \|\mathbf{c}_{xy} - \mathbf{v}_{b,xy}\|} + e^{-10 \cdot |c_\omega - \omega_{bz}|}\right] - 0.2\left(\|\mathbf{c}_{xy} - \mathbf{v}_{b,xy}\| + |c_\omega - \omega_{bz}|\right)$$

**源码**（[legged_robot.py:1828-1845](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1828)）：
```python
def _reward_track_vel_hard(self):
    lin_vel_error = torch.norm(
        self.commands[:, :2] - self.base_lin_vel[:, :2], dim=1)
    lin_vel_error_exp = torch.exp(-lin_vel_error * 10)

    ang_vel_error = torch.abs(
        self.commands[:, 2] - self.base_ang_vel[:, 2])
    ang_vel_error_exp = torch.exp(-ang_vel_error * 10)

    linear_error = 0.2 * (lin_vel_error + ang_vel_error)

    return (lin_vel_error_exp + ang_vel_error_exp) / 2. - linear_error
```

**代码与公式对应**：

| 步骤 | 代码 | 公式 |
|---|---|---|
| 线速度误差范数 | `torch.norm(commands[:,:2] - base_lin_vel[:,:2], dim=1)` | $\|\mathbf{c}_{xy} - \mathbf{v}_{b,xy}\|$ |
| 角速度误差绝对值 | `torch.abs(commands[:,2] - base_ang_vel[:,2])` | $|c_\omega - \omega_{bz}|$ |
| 指数衰减项 | `torch.exp(-<error> * 10)` | $e^{-10 \cdot \text{error}}$ |
| 线性惩罚 | `0.2 * (lin_vel_error + ang_vel_error)` | $0.2 \cdot (\|\mathbf{c}_{xy} - \mathbf{v}_{b,xy}\| + |c_\omega - \omega_{bz}|)$ |

**设计意图**：指数衰减系数 10（远大于 tracking_sigma=0.5）意味着此项对"小误差"给予尖峰奖励，对"大误差"通过线性项提供持续的惩罚梯度。

---

### 1.4 `_reward_low_speed` — 低速检测/奖惩

**配置权重**：$w = 0.2$（正，鼓励/惩罚混合）

**物理意义**：根据机器人实际速度与命令速度的比例关系，给予分段式奖惩。重点在于：方向不匹配时给予严厉惩罚，速度在目标范围内时给予正向奖励。

**物理公式**（分段函数）：

$$r_{\text{low\_speed}} = \begin{cases}
-2.0 & \text{if } \text{sign}(v_{bx}) \neq \text{sign}(c_x) \quad \text{(方向不匹配)} \\
-1.0 & \text{if } |v_{bx}| < 0.5 \cdot |c_x| \quad \text{(速度过低)} \\
0.0 & \text{if } |v_{bx}| > 1.2 \cdot |c_x| \quad \text{(速度过高)} \\
1.2 & \text{otherwise} \quad \text{(速度在合理范围内)}
\end{cases}$$

当 $|c_x| \leq$ `command_dead` (0.005) 时，整个 reward 为 0。

**源码**（[legged_robot.py:1847-1877](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1847)）：
```python
def _reward_low_speed(self):
    absolute_speed = torch.abs(self.base_lin_vel[:, 0])
    absolute_command = torch.abs(self.commands[:, 0])

    speed_too_low = absolute_speed < 0.5 * absolute_command
    speed_too_high = absolute_speed > 1.2 * absolute_command
    speed_desired = ~(speed_too_low | speed_too_high)

    sign_mismatch = torch.sign(self.base_lin_vel[:, 0]) != torch.sign(self.commands[:, 0])

    reward = torch.zeros_like(self.base_lin_vel[:, 0])
    reward[speed_too_low] = -1.0
    reward[speed_too_high] = 0.
    reward[speed_desired] = 1.2
    reward[sign_mismatch] = -2.0
    return reward * (self.commands[:, 0].abs() > self.cfg.rewards.command_dead)
```

**代码与公式对应**：

`speed_too_low`, `speed_too_high`, `speed_desired` 分别对应 $|v_{bx}| < 0.5|c_x|$, $|v_{bx}| > 1.2|c_x|$, 和中间区间。`sign_mismatch` 有最高优先级（最后赋值覆盖），确保方向错误永远被惩罚。

---

## 2. 姿态/基座类 (Posture & Base)

### 2.1 `_reward_orientation_eular` — 欧拉角姿态奖励

**配置权重**：$w = 1.5$（正，鼓励）

**物理意义**：鼓励机器人保持水平躯干姿态。使用重力投影向量的范数（衡量倾斜度）+ 欧拉角偏差（roll, pitch 接近 0）。两个子项分别是绕不同空间表示的同一物理量——躯干倾斜。

**物理公式**：

$$r_{\text{orient}} = \exp\left(-10 \cdot (|\phi| + |\theta|)\right)$$

其中 $\phi$ 为 roll 角（绕 x），$\theta$ 为 pitch 角（绕 y）。

> 注：源码中实际返回值只用了 `quat_mismatch`，`orientation` 变量被计算但未使用（被注释掉的 `/2.` 平均），所以有效公式仅含欧拉角部分。

**源码**（[legged_robot.py:1795-1803](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1795)）：
```python
def _reward_orientation_eular(self):
    quat_mismatch = torch.exp(-torch.sum(torch.abs(self.base_euler_xyz[:, :2]), dim=1) * 10)
    orientation = torch.exp(-torch.norm(self.projected_gravity[:, :2], dim=1) * 20)
    return quat_mismatch  # orientation 被计算但未参与最终返回值
```

**代码与公式对应**：

| 步骤 | 代码 | 公式 |
|---|---|---|
| roll + pitch 绝对值和 | `torch.sum(torch.abs(self.base_euler_xyz[:, :2]), dim=1)` | $|\phi| + |\theta|$ |
| 指数衰减 | `torch.exp(-<sum> * 10)` | $e^{-10 \cdot (|\phi| + |\theta|)}$ |

- 当躯干完全水平（$\phi = 0, \theta = 0$），$r = 1.0$
- 系数 10 使得对倾斜非常敏感——倾斜约 0.1 rad (≈6°) 时 reward 降到 $e^{-1} \approx 0.37$

---

### 2.2 `_reward_base_height` — 基座高度

**配置权重**：$w = 0.2$（正，鼓励）

**物理意义**：鼓励机器人基座高度维持在目标值 $h_{\text{target}} = 0.30\text{m}$ 附近。通过指数衰减实现软约束。

**物理公式**：

$$r_{\text{height}} = \exp\left(-100 \cdot |h_b - h_{\text{target}}|\right)$$

**源码**（[legged_robot.py:1893-1898](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1893)）：
```python
def _reward_base_height(self):
    base_height = self._get_base_heights()
    return torch.exp(-torch.abs(base_height - self.cfg.rewards.base_height_target) * 100)
```

**代码与公式对应**：

| 步骤 | 代码 | 公式 |
|---|---|---|
| 高度误差绝对值 | `torch.abs(base_height - base_height_target)` | $|h_b - h_{\text{target}}|$ |
| 指数衰减 | `torch.exp(-<error> * 100)` | $e^{-100 \cdot |h_b - h_{\text{target}}|}$ |

- 系数 100 非常激进——偏离 1cm 时 reward 已降到 $e^{-1} \approx 0.37$，偏离 5cm 时基本为 0
- 这实际上是在执行严格的**高度约束**

---

### 2.3 `_reward_base_acc` — 基座加速度

**配置权重**：$w = 0.02$（正，鼓励）

**物理意义**：惩罚基座的突然加速/减速（jerk），鼓励平滑运动。使用前后两帧根状态的速度差分近似加速度。

**物理公式**：

$$r_{\text{acc}} = \exp\left(-3 \cdot \left\|\dot{\mathbf{v}}_{b}\right\|\right)$$

其中 $\dot{\mathbf{v}}_{b} \approx \frac{\mathbf{v}_{b,t-1} - \mathbf{v}_{b,t}}{dt}$ 是对基座线加速度和角加速度拼接的 6 维向量的近似。

**源码**（[legged_robot.py:2131-2138](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L2131)）：
```python
def _reward_base_acc(self):
    root_acc = self.last_root_vel - self.root_states[:, 7:13]
    rew = torch.exp(-torch.norm(root_acc, dim=1) * 3)
    return rew
```

**代码与公式对应**：

| 步骤 | 代码 | 公式 |
|---|---|---|
| 速度差分（6维） | `self.last_root_vel - self.root_states[:, 7:13]` | $\mathbf{v}_{b,t-1} - \mathbf{v}_{b,t} \propto \dot{\mathbf{v}}_{b}$ |
| 范数 | `torch.norm(root_acc, dim=1)` | $\|\dot{\mathbf{v}}_{b}\|$ |
| 指数衰减 | `torch.exp(-<norm> * 3)` | $e^{-3 \cdot \|\dot{\mathbf{v}}_{b}\|}$ |

---

### 2.4 ~~`_reward_orientation`~~ — 重力投影姿态（未被使用）

`_reward_orientation_eular` 被激活（scale=1.5），替代了基础的 `_reward_orientation`（gravity projection only）。Trot 配置中仅 `orientation_eular` 非零。

---

## 3. 正则化/能量类 (Regularization & Energy)

这一类 reward 全部为**负权重**（惩罚项），目标是让策略学会节能、平滑、稳定的运动，同时避免不期望的基座运动。

### 3.1 `_reward_lin_vel_z` — z 轴线速度惩罚

**配置权重**：$w = -2.0$（负，惩罚）

**物理意义**：惩罚基座在垂直方向的线速度（上下跳动）。双足行走应是水平运动为主，z 轴速度大会导致步态不稳定。

**物理公式**：

$$r_{\text{lin\_vel\_z}} = v_{bz}^2$$

**源码**（[legged_robot.py:1783-1785](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1783)）：
```python
def _reward_lin_vel_z(self):
    return torch.square(self.base_lin_vel[:, 2])
```

直接对应公式：取基座线速度的 z 分量平方。

---

### 3.2 `_reward_ang_vel_xy` — xy 轴角速度惩罚

**配置权重**：$w = -0.05$（负，惩罚）

**物理意义**：惩罚基座绕 x 轴（roll）和 y 轴（pitch）的角速度，即抑制身体的前后/左右摇晃。

**物理公式**：

$$r_{\text{ang\_vel\_xy}} = \omega_{bx}^2 + \omega_{by}^2$$

**源码**（[legged_robot.py:1787-1789](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1787)）：
```python
def _reward_ang_vel_xy(self):
    return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)
```

---

### 3.3 `_reward_torques` — 关节力矩惩罚

**配置权重**：$w = -1 \times 10^{-5}$（负，惩罚）

**物理意义**：惩罚各关节的输出力矩。力矩越小，能量消耗越低。这是能效优化中最直接的指标。

**物理公式**：

$$r_{\text{torques}} = \sum_{j=1}^{10} \tau_j^2$$

**源码**（[legged_robot.py:1900-1902](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1900)）：
```python
def _reward_torques(self):
    return torch.sum(torch.square(self.torques), dim=1)
```

> 权重极小（1e-5）的原因：力矩的平方值可达 $10^3$ 量级，$1\times 10^{-5} \times 10^3 = 0.01$，在总 reward 中作为微调项。

---

### 3.4 `_reward_powers` — 机械功率惩罚

**配置权重**：$w = -2 \times 10^{-5}$（负，惩罚）

**物理意义**：惩罚关节的瞬时机械功率。$P = \tau \cdot \omega$，这是比纯力矩更精确的能耗指标，因为大功率出现在力矩和速度都大的情况。

**物理公式**：

$$r_{\text{powers}} = \sum_{j=1}^{10} |\tau_j| \cdot |\dot{q}_j|$$

**源码**（[legged_robot.py:1904-1907](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1904)）：
```python
def _reward_powers(self):
    return torch.sum(torch.abs(self.torques) * torch.abs(self.dof_vel), dim=1)
```

**代码与公式对应**：`torch.abs(torques)` → $|\tau_j|$，`torch.abs(dof_vel)` → $|\dot{q}_j|$，两者逐元素相乘后求和。

> 注：严格的正负功率分离（$\tau \cdot \dot{q}$ 可为负）被注释掉了，当前用绝对值防止正负抵消。

---

### 3.5 `_reward_dof_vel` — 关节速度惩罚

**配置权重**：$w = -5 \times 10^{-4}$（负，惩罚）

**物理意义**：惩罚所有关节的运动速度。速度越小，运动越慢但越稳定。抑制不必要的关节高速运动。

**物理公式**：

$$r_{\text{dof\_vel}} = \sum_{j=1}^{10} \dot{q}_j^2$$

**源码**（[legged_robot.py:1909-1911](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1909)）：
```python
def _reward_dof_vel(self):
    return torch.sum(torch.square(self.dof_vel), dim=1)
```

---

### 3.6 `_reward_dof_acc` — 关节加速度惩罚

**配置权重**：$w = -2 \times 10^{-7}$（负，惩罚）

**物理意义**：惩罚关节的角加速度（速度变化率）。大的加速度意味着大的冲击力（jerk），对机械结构有害且能耗高。

**物理公式**：

$$r_{\text{dof\_acc}} = \sum_{j=1}^{10} \left(\frac{\dot{q}_{j,t-1} - \dot{q}_{j,t}}{dt}\right)^2$$

**源码**（[legged_robot.py:1913-1915](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1913)）：
```python
def _reward_dof_acc(self):
    return torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1)
```

**代码与公式对应**：`(last_dof_vel - dof_vel) / dt` 是加速度的有限差分近似 $\ddot{q}_j \approx \frac{\dot{q}_{j,t-1} - \dot{q}_{j,t}}{dt}$。

---

### 3.7 `_reward_action_smoothness` — 动作平滑性

**配置权重**：$w = -0.01$（负，惩罚）

**物理意义**：鼓励策略输出的目标关节角度变化平滑。包含三项：一阶变化（当前与上一动作的差）、二阶变化（加速度式变化）、动作幅值惩罚。平滑的动作可以减少关节冲击，是 sim-to-real transfer 的关键。

**物理公式**：

$$r_{\text{smooth}} = \underbrace{\|\mathbf{a}_t - \mathbf{a}_{t-1}\|^2}_{\text{一阶项}} + \underbrace{\|\mathbf{a}_t + \mathbf{a}_{t-2} - 2\mathbf{a}_{t-1}\|^2}_{\text{二阶项}} + \underbrace{0.05 \cdot \|\mathbf{a}_t\|_1}_{\text{幅值惩罚}}$$

**源码**（[legged_robot.py:1921-1934](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1921)）：
```python
def _reward_action_smoothness(self):
    term_1 = torch.sum(torch.square(
        self.last_actions - self.actions), dim=1)
    term_2 = torch.sum(torch.square(
        self.actions + self.last_last_actions - 2 * self.last_actions), dim=1)
    term_3 = 0.05 * torch.sum(torch.abs(self.actions), dim=1)
    return (term_1 + term_2 + term_3)
```

**代码与公式对应**：

| 项 | 代码 | 物理意义 |
|---|---|---|
| term_1 | `(last_actions - actions)^2` | 动作一阶差分（速度级平滑） |
| term_2 | `(actions + last_last_actions - 2*last_actions)^2` | 动作二阶差分（加速度级平滑） |
| term_3 | `0.05 * \|actions\|_1` | 动作幅值惩罚（偏向小幅度） |

---

## 4. 足部运动类 (Foot Motion)

### 4.1 `_reward_feet_air_time` — 摆腿时间奖励

**配置权重**：$w = 3.0$（正，鼓励）

**物理意义**：鼓励每条腿的摆动相时长接近目标周期 `cycle_time = 0.5s`。这是形成周期性步态的**核心奖励**——太短的摆腿导致急促步态，太长的摆腿导致不稳定。

**物理公式**：

$$r_{\text{air\_time}} = \sum_{f \in \text{feet}} \max(0, \ t_{\text{air}, f} - T_{\text{cycle}}) \cdot \mathbb{1}[\text{first\_contact}_f] \cdot \mathbb{1}[\|\mathbf{c}\| > \text{dead}]$$

其中 $t_{\text{air}, f}$ 是足 $f$ 的空中累计时间（从上次离地起算），奖励在**首次触地瞬间**触发（`first_contact`），奖励值 = 空中时间超出目标周期的部分。

**源码**（[legged_robot.py:1978-1992](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1978)）：
```python
def _reward_feet_air_time(self):
    contact = self.contact_forces[:, self.feet_indices, 2] > self.cfg.rewards.touch_thr
    contact_filt = torch.logical_or(contact, self.last_contacts)
    self.last_contacts = contact
    first_contact = (self.feet_air_time > 0.) * contact_filt
    self.feet_air_time += self.dt
    rew_airTime = torch.sum((self.feet_air_time - self.cfg.rewards.cycle_time) * first_contact, dim=1)
    rew_airTime *= torch.norm(self.commands[:, :3], dim=1) > self.cfg.rewards.command_dead
    self.feet_air_time *= ~contact_filt
    return rew_airTime
```

**代码与公式对应**：

| 步骤 | 代码 | 物理意义 |
|---|---|---|
| 接触检测 | `contact_forces[:, feet_indices, 2] > touch_thr` | 判断足是否触地（垂直接触力 > 6N） |
| 滤波 | `logical_or(contact, last_contacts)` | 防止 PhysX 接触检测抖动 |
| 首次触地 | `(feet_air_time > 0.) * contact_filt` | 仅在从空中转为触地的那一帧为 True |
| 空中计时 | `feet_air_time += self.dt` | 累积每足的空中时间 |
| 奖励 | `(feet_air_time - cycle_time) * first_contact` | 触地瞬间：空中时长 - 目标周期 |
| 重置 | `feet_air_time *= ~contact_filt` | 触地后将计时器归零 |

---

### 4.2 `_reward_foot_clearance` — 抬脚高度

**配置权重**：$w = -3.0$（负，惩罚）

**物理意义**：惩罚摆动脚离目标 clearance 高度的偏差。用**身体坐标系中的脚高度**与 `clearance_height_target = -0.21m`（身体坐标系中，脚在身体下方为负值）比较，并用脚的侧向速度加权——摆动时脚移动越快，高度偏差惩罚越重。

**物理公式**：

$$r_{\text{clearance}} = \sum_{f \in \text{feet}} \left(z_f^{\text{body}} - z_{\text{target}}\right)^2 \cdot \sqrt{v_{fx}^2 + v_{fy}^2} \cdot \mathbb{1}[\|\mathbf{c}\| > \text{dead}]$$

其中 $z_f^{\text{body}}$ 是脚在身体坐标系中的 z 坐标，$z_{\text{target}} = -0.21\text{m}$，$v_{fx}, v_{fy}$ 是脚在身体坐标系中的侧向速度。

**源码**（[legged_robot.py:1994-2009](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1994)）：
```python
def _reward_foot_clearance(self):
    base_height = self._get_base_heights()
    cur_footpos_translated = self.feet_pos - self.root_states[:, 0:3].unsqueeze(1)
    cur_footvel_translated = self.feet_vel - self.root_states[:, 7:10].unsqueeze(1)
    footpos_in_body_frame = torch.zeros(...)
    footvel_in_body_frame = torch.zeros(...)
    for i in range(len(self.feet_indices)):
        footpos_in_body_frame[:, i, :] = quat_rotate_inverse(self.base_quat, cur_footpos_translated[:, i, :])
        footvel_in_body_frame[:, i, :] = quat_rotate_inverse(self.base_quat, cur_footvel_translated[:, i, :])

    height_error = torch.square(footpos_in_body_frame[:, :, 2] - self.cfg.rewards.clearance_height_target)
    foot_leteral_vel = torch.sqrt(torch.sum(torch.square(footvel_in_body_frame[:, :, :2]), dim=2))
    return torch.sum(height_error * foot_leteral_vel, dim=1) * (torch.norm(self.commands[:, :3], dim=1) > ...)
```

**代码与公式对应**：

| 步骤 | 代码 | 公式 |
|---|---|---|
| 脚位姿从世界转身体系 | `quat_rotate_inverse(base_quat, footpos_world)` | 坐标变换 $\mathbf{p}_f^{\text{body}} = R_b^T \cdot (\mathbf{p}_f^{\text{world}} - \mathbf{p}_b^{\text{world}})$ |
| 高度误差 | `(footpos[:,:,2] - clearance_height_target)^2` | $(z_f^{\text{body}} - z_{\text{target}})^2$ |
| 脚侧向速度 | `sqrt(footvel[:,:,:2]^2)` | $\sqrt{v_{fx}^2 + v_{fy}^2}$ |
| 加权 | `height_error * foot_leteral_vel` | 侧向速度越大，惩罚越重 |

---

### 4.3 `_reward_foot_slip` — 足底滑动惩罚

**配置权重**：$w = -0.05$（负，惩罚）

**物理意义**：惩罚支撑脚在地面上的滑动。接触地面的脚应保持静止（相对于地面），滑动意味着摩擦力不足或步态有问题。

**物理公式**：

$$r_{\text{slip}} = \sum_{f \in \text{feet}} \sqrt{\|\mathbf{v}_{f,xy}\|} \cdot \mathbb{1}[F_{f,z} > F_{\text{thresh}}]$$

其中 $\mathbf{v}_{f,xy}$ 是足的世界坐标系水平速度（`rigid_body_states[:, feet_indices, 10:12]`），$\mathbb{1}[F_{f,z} > F_{\text{thresh}}]$ 仅在足触地时启用（接触力 > `touch_thr = 6N`）。

**源码**（[legged_robot.py:1879-1891](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1879)）：
```python
def _reward_foot_slip(self):
    contact = self.contact_forces[:, self.feet_indices, 2] > self.cfg.rewards.touch_thr
    foot_speed_norm = torch.norm(self.rigid_body_states[:, self.feet_indices, 10:12], dim=2)
    rew = torch.sqrt(foot_speed_norm)
    rew *= contact
    return torch.sum(rew, dim=1)
```

**代码与公式对应**：`torch.norm(..., 10:12)` 是 xy 平面速度范数，`torch.sqrt()` 对小速度更敏感（梯度大），`* contact` 仅在触地时非零。

---

### 4.4 `_reward_feet_rotation1` — 左脚旋转 (pitch)

**配置权重**：$w = 0.3$（正，鼓励）

**物理意义**：鼓励左脚（index 0）在**摆动相**保持水平（pitch 角接近 0）。脚掌在空中应平行于地面，以便触地时脚底平整着地。

**物理公式**：

$$r_{\text{rot1}} = \exp\left(-15 \cdot \theta_{\text{pitch, L}}^2\right) \cdot \mathbb{1}[F_{L,z} < F_{\text{thresh}}]$$

**源码**（[legged_robot.py:2104-2110](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L2104)）：
```python
def _reward_feet_rotation1(self):
    feet_euler_xyz = self.feet_euler_xyz
    nag_contacts = self.contact_forces[:, self.feet_indices[0], 2] < self.cfg.rewards.touch_thr
    rotation = (torch.square(feet_euler_xyz[:, 0, 1]))  # foot[0], pitch axis (index 1)
    r = torch.exp(-rotation * 15) * nag_contacts
    return r
```

**代码与公式对应**：`feet_euler_xyz[:, 0, 1]` = 左脚 pitch 角，`nag_contacts` = 左脚未触地（摆动态），仅在摆动相开启。

---

### 4.5 `_reward_feet_rotation2` — 右脚旋转 (pitch)

**配置权重**：$w = 0.3$（正，鼓励）

与 `feet_rotation1` 对称，作用于右脚（index 1）。

$$r_{\text{rot2}} = \exp\left(-15 \cdot \theta_{\text{pitch, R}}^2\right) \cdot \mathbb{1}[F_{R,z} < F_{\text{thresh}}]$$

**源码**（[legged_robot.py:2112-2118](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L2112)）：
```python
def _reward_feet_rotation2(self):
    feet_euler_xyz = self.feet_euler_xyz
    nag_contacts = self.contact_forces[:, self.feet_indices[1], 2] < self.cfg.rewards.touch_thr
    rotation = (torch.square(feet_euler_xyz[:, 1, 1]))  # foot[1], pitch axis
    r = torch.exp(-rotation * 15) * nag_contacts
    return r
```

---

### 4.6 `_reward_feet_contact_forces` — 足底力惩罚

**配置权重**：$w = -0.01$（负，惩罚）

**物理意义**：惩罚过高的足底接触力。超过 `max_contact_force = 120N` 的部分被 clip 后求和。防止策略学会"猛踩地面"来获得推进力。

**物理公式**：

$$r_{\text{contact\_force}} = \sum_{f \in \text{feet}} \max\left(0, \ \|\mathbf{F}_f\| - F_{\text{max}}\right)$$

**源码**（[legged_robot.py:2065-2067](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L2065)）——注意此函数在文件中定义了两次（2034 行和 2065 行），后者覆盖前者：
```python
def _reward_feet_contact_forces(self):
    return torch.sum((torch.norm(self.contact_forces[:, self.feet_indices, :], dim=-1)
                      - self.cfg.rewards.max_contact_force).clip(min=0.), dim=1)
```

**代码与公式对应**：`torch.norm(contact_forces, dim=-1)` → $\|\mathbf{F}_f\| = \sqrt{F_{fx}^2 + F_{fy}^2 + F_{fz}^2}$，`.clip(min=0.)` → $\max(0, \cdot)$。

---

### 4.7 `_reward_stumble` — 绊倒检测

**配置权重**：$w = -0.02$（负，惩罚）

**物理意义**：检测足部是否有异常的侧向力。当足部水平接触力 > 5倍的垂直接触力时，判定为"绊到东西"（如踢到台阶边缘）。

**物理公式**：

$$r_{\text{stumble}} = \mathbb{1}\left[\exists f: \sqrt{F_{fx}^2 + F_{fy}^2} > 5 \cdot |F_{fz}|\right]$$

**源码**（[legged_robot.py:2029-2032](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L2029)）：
```python
def _reward_stumble(self):
    return torch.any(torch.norm(self.contact_forces[:, self.feet_indices, :2], dim=2) >
                     5 * torch.abs(self.contact_forces[:, self.feet_indices, 2]), dim=1)
```

---

## 5. 接触/站立类 (Contact & Standing)

### 5.1 `_reward_stand_2leg` — 双腿站立奖励

**配置权重**：$w = 6.0$（正，鼓励）

**物理意义**：在**零速度命令**时鼓励双腿同时触地（稳定站立姿态），惩罚单腿支撑或双脚离地。仅在 $\|\mathbf{c}\| <$ `command_dead` 时激活。

**物理公式**：

$$r_{\text{stand\_2leg}} = \begin{cases}
1.0 & \text{双腿触地} \land \|\mathbf{c}\| < 0.005 \\
-0.5 & \text{双脚离地} \land \|\mathbf{c}\| < 0.005 \\
-1.0 & \text{单腿触地} \land \|\mathbf{c}\| < 0.005 \\
0 & \text{otherwise}
\end{cases}$$

**源码**（[legged_robot.py:2054-2063](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L2054)）：
```python
def _reward_stand_2leg(self):
    contacts = self.contact_forces[:, self.feet_indices, 2] > self.cfg.rewards.touch_thr
    single_contact = torch.sum(1.*contacts, dim=1) == 1
    no_contact = torch.sum(1.*contacts, dim=1) == 0
    double_contact = torch.sum(1.*contacts, dim=1) == 2
    reward_out1 = 1. * (double_contact) * (torch.norm(self.commands[:, :3], dim=1) < ...)
    reward_out2 = -0.5 * (no_contact) * (torch.norm(self.commands[:, :3], dim=1) < ...)
    reward_out3 = -1. * (single_contact) * (torch.norm(self.commands[:, :3], dim=1) < ...)
    return reward_out1 + reward_out2 + reward_out3
```

---

### 5.2 `_reward_stand_still_force` — 站立力对称

**配置权重**：$w = -0.1$（负，惩罚）

**物理意义**：零速度命令时鼓励两脚接触力均匀分布。如果一只脚受力远大于另一只，说明机器人重心偏移，站立姿态不对称。

**物理公式**：

$$r_{\text{stand\_force}} = \exp\left(-0.0001 \cdot (F_{L,z} - F_{R,z})^2\right) \cdot \mathbb{1}[\|\mathbf{c}\| < \text{dead}]$$

**源码**（[legged_robot.py:2045-2052](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L2045)）：
```python
def _reward_stand_still_force(self):
    left_foot_force = self.contact_forces[:, self.feet_indices[0], 2]
    right_foot_force = self.contact_forces[:, self.feet_indices[1], 2]
    rew = torch.exp(-torch.square(0.01 * (left_foot_force - right_foot_force)))
    return rew * (torch.norm(self.commands[:, :3], dim=1) < self.cfg.rewards.command_dead)
```

**代码与公式对应**：`0.01 * (F_L - F_R)` 即 $(F_{L,z} - F_{R,z}) / 100$，除以 100 起到缩放作用，使得 100N 的差异才触发明显的惩罚。

---

### 5.3 `_reward_no_jump` — 禁止跳跃

**配置权重**：$w = 0.7$（正，但含负子项——混合奖惩）

**物理意义**：在**有速度命令**时，鼓励单腿支撑（交替步态），惩罚双脚同时离地（跳跃），轻微惩罚双腿同时触地（非交替）。
- 单腿触地 +1.0（最大鼓励——这是期望的交替行走状态）
- 无触地 -2.0（跳起来了，严格惩罚）
- 双腿触地 -0.5（双支撑相，轻度惩罚——Trot 步态中要短暂经过但不应停留）

**物理公式**：

$$r_{\text{jump}} = \begin{cases}
1.0 & \text{单腿触地} \land \|\mathbf{c}\| > 0.005 \\
-2.0 & \text{无触地} \land \|\mathbf{c}\| > 0.005 \\
-0.5 & \text{双腿触地} \land \|\mathbf{c}\| > 0.005
\end{cases}$$

**源码**（[legged_robot.py:2120-2129](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L2120)）：
```python
def _reward_no_jump(self):
    contacts = self.contact_forces[:, self.feet_indices, 2] > self.cfg.rewards.touch_thr
    single_contact = torch.sum(1.*contacts, dim=1) == 1
    no_contact = torch.sum(1.*contacts, dim=1) == 0
    double_contact = torch.sum(1.*contacts, dim=1) == 2
    reward_out1 = 1. * (single_contact) * (torch.norm(self.commands[:, :3], dim=1) > ...)
    reward_out2 = -2. * (no_contact) * (torch.norm(self.commands[:, :3], dim=1) > ...)
    reward_out3 = -0.5 * (double_contact) * (torch.norm(self.commands[:, :3], dim=1) > ...)
    return reward_out1 + reward_out2 + reward_out3
```

---

### 5.4 `_reward_hip_pos` — 髋关节位置

**配置权重**：$w = -1.0$（负，惩罚）

**物理意义**：惩罚髋关节的 roll/yaw 偏离默认值。对左右髋的 roll（J_L0, J_R0）偏差加倍惩罚（×2），因为这直接影响基座侧向稳定性。这本质上是对"侧向摇摆"的抑制。

**物理公式**：

$$r_{\text{hip}} = 4 \cdot (q_{L0} - q_{L0}^{\text{default}})^2 + (q_{L1} - q_{L1}^{\text{default}})^2 + 4 \cdot (q_{R0} - q_{R0}^{\text{default}})^2 + (q_{R1} - q_{R1}^{\text{default}})^2$$

其中关节 0/5 为髋 roll（L/R），关节 1/6 为髋 yaw。

**源码**（[legged_robot.py:2079-2086](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L2079)）：
```python
def _reward_hip_pos(self):
    vel_y = self.cfg.commands.ranges.lin_vel_y[1]
    temp = self.dof_pos[:, [0, 1, 5, 6]] - self.default_dof_pos[:, [0, 1, 5, 6]]
    temp[:, 0] *= 2  # 髋 roll L 加倍
    temp[:, 2] *= 2  # 髋 roll R 加倍
    return torch.sum(torch.square(temp), dim=1)
```

---

## 6. 终止类 (Termination)

### 6.1 `_reward_termination` — 终止惩罚

**配置权重**：$w = -5.0$（负，惩罚）

**物理意义**：当 episode 因跌倒（非超时）终止时给予惩罚。每次跌倒触发 -5.0 的 reward 扣减，这是训练中**最强的惩罚信号**，直接推动策略避免跌倒行为。

**物理公式**：

$$r_{\text{term}} = \mathbb{1}[\text{reset}] \cdot \mathbb{1}[\neg\text{timeout}]$$

**源码**（[legged_robot.py:1940-1942](https://github.com/OmniBotSeries-Tinker/OmniBotCtrl/blob/main/OmniBotCtrl/envs/legged_robot.py#L1940)）：
```python
def _reward_termination(self):
    return self.reset_buf * ~self.time_out_buf
```

**特殊性**：termination reward 在 `compute_reward()` 中**最后单独加上**，不受 `only_positive_rewards` 裁剪影响。

---

## 7. 约束函数 (Cost Functions)

NP3O 的 cost 函数通过 Lagrangian 乘子法与 reward 协同优化。当前 Trot 配置激活 5 个 cost。

### 7.1 `_cost_pos_limit` — 关节位置限位

**k_value**：0.1 | **d_value**：0.0

$$c_{\text{pos\_limit}} = \mathbb{1}\left[\exists j: q_j > q_j^{\text{upper}} \lor q_j < q_j^{\text{lower}}\right]$$

```python
def _cost_pos_limit(self):
    upper_limit = 1.*(self.dof_pos > self.dof_pos_limits[:, 1])
    lower_limit = 1.*(self.dof_pos < self.dof_pos_limits[:, 0])
    out_limit = 1.*(torch.sum(upper_limit + lower_limit, dim=1) > 0.0)
    return out_limit
```

二进制约束：任何关节超出限位 → 1，否则 0。

---

### 7.2 `_cost_torque_limit` — 关节力矩限位

**k_value**：0.1 | **d_value**：0.0

$$c_{\text{torque\_limit}} = \mathbb{1}\left[\exists j: |\tau_j| > \tau_j^{\text{max}} \cdot 0.9\right]$$

```python
def _cost_torque_limit(self):
    return 1.*(torch.sum(1.*(torch.abs(self.torques) > self.torque_limits*self.cfg.rewards.soft_torque_limit), dim=1) > 0.0)
```

`soft_torque_limit = 0.9`（未在 config 中显式设，使用默认值）——即力矩超过最大值的 90% 时触发。

---

### 7.3 `_cost_dof_vel_limits` — 关节速度限位

**k_value**：0.1 | **d_value**：0.0

$$c_{\text{dof\_vel}} = \mathbb{1}\left[\exists j: |\dot{q}_j| > \dot{q}_j^{\text{max}} \cdot 0.9\right]$$

```python
def _cost_dof_vel_limits(self):
    return 1.*(torch.sum(1.*(torch.abs(self.dof_vel) > self.dof_vel_limits*self.cfg.rewards.soft_dof_vel_limit), dim=1) > 0.0)
```

---

### 7.4 `_cost_feet_air_time` — 支撑相时长约束

**k_value**：0.1 | **d_value**：0.1

$$c_{\text{air\_time}} = \max\left(0, \ T_{\text{cycle}} - t_{\text{air}}\right) \cdot \mathbb{1}[\text{first\_contact}]$$

```python
def _cost_feet_air_time(self):
    # ... (与 _reward_feet_air_time 前半段相同) ...
    rew_airTime = torch.sum((self.feet_air_time - self.cfg.rewards.cycle_time) * first_contact, dim=1)
    return torch.max(torch.zeros_like(rew_airTime), -1.*rew_airTime)
```

当 `feet_air_time < cycle_time`（支撑相过短），`rew_airTime < 0`，`max(0, -negative) > 0`，触发约束惩罚。`d_value = 0.1` 允许 0.1 秒的容忍。

---

### 7.5 `_cost_hip_pos` — 髋关节侧展约束

**k_value**：0.1 | **d_value**：0.0

$$c_{\text{hip\_pos}} = 4(q_{L0} - q_{L0}^{\text{def}})^2 + (q_{L1} - q_{L1}^{\text{def}})^2 + 4(q_{R0} - q_{R0}^{\text{def}})^2 + (q_{R1} - q_{R1}^{\text{def}})^2$$

```python
def _cost_hip_pos(self):
    temp = self.dof_pos[:, [0, 1, 5, 6]] - self.default_dof_pos[:, [0, 1, 5, 6]]
    temp[:, 0] *= 2; temp[:, 2] *= 2
    return torch.sum(torch.square(temp), dim=1)
```

与 reward 版本相同但作为软约束而非惩罚项，通过 Lagrangian 乘子影响策略。

---

## 8. 奖励权重汇总与贡献分析

以 iteration 21999 的日志数据为例：

| 奖励函数 | 权重 | 日志均值 | 有效贡献 | 类别 |
|---|---|---|---|---|
| `tracking_lin_vel` | 2.5 | 2.18459 | **主导正项** | 任务 |
| `tracking_ang_vel` | 2.0 | 1.71578 | **第二正项** | 任务 |
| `orientation_eular` | 1.5 | 0.83483 | **第三正项** | 姿态 |
| `no_jump` | 0.7 | 0.55564 | 步态鼓励 | 接触 |
| `feet_rotation1` | 0.3 | 0.11932 | 足部对齐 | 足部 |
| `feet_rotation2` | 0.3 | 0.11569 | 足部对齐 | 足部 |
| `base_height` | 0.2 | 0.10705 | 高度维持 | 姿态 |
| `low_speed` | 0.2 | 0.02612 | 速度匹配 | 任务 |
| `track_vel_hard` | 0.5 | 0.01262 | 硬速度跟踪 | 任务 |
| `base_acc` | 0.02 | 0.00137 | 加速度平滑 | 姿态 |
| `dof_acc` | -2e-7 | -0.07107 | 关节加速度 | 能量 |
| `dof_vel` | -5e-4 | -0.20951 | **最大负项** | 能量 |
| `foot_slip` | -0.05 | -0.07653 | 足底滑动 | 足部 |
| `action_smoothness` | -0.01 | -0.05596 | 动作平滑 | 能量 |
| `hip_pos` | -1.0 | -0.05134 | 髋关节 | 接触 |
| `ang_vel_xy` | -0.05 | -0.04276 | 摇晃抑制 | 能量 |
| `feet_contact_forces` | -0.01 | -0.03336 | 接触力 | 足部 |
| `lin_vel_z` | -2.0 | -0.02685 | 跳动抑制 | 能量 |
| `foot_clearance` | -3.0 | -0.00231 | 抬脚高度 | 足部 |
| `powers` | -2e-5 | -0.00305 | 机械功率 | 能量 |
| `torques` | -1e-5 | -0.00120 | 关节力矩 | 能量 |
| `stumble` | -0.02 | 0.00000 | 绊倒 | 足部 |
| `stand_still_force` | -0.1 | 0.00000 | 站立力对称 | 接触 |
| `stand_2leg` | 6.0 | 0.00000 | 双腿站立 | 接触 |
| `termination` | -5.0 | -0.00036 | 终止 | 终止 |

**关键发现**：
- Top 3 正向贡献（`tracking_lin_vel`, `tracking_ang_vel`, `orientation_eular`）占总 reward 的 ~95%
- 负向贡献中 `dof_vel`（-0.21）最大，关节速度是被惩罚最重的行为
- 站立类 reward（`stand_2leg`, `stand_still_force`）在非零命令时均为 0，仅在 `command_dead` 条件下激活
- `stumble = 0.00000` 说明当前策略完全没有绊倒行为

---

## 9. 未激活的奖励/约束函数

Tinker 代码中**已实现但未使用**（scale=0）的函数：

| 函数 | 原因 |
|---|---|
| `_reward_collision` | 自碰撞检测用 `self_collisions=1` 禁用 |
| `_reward_action_rate` | 被 `action_smoothness` 替代（功能更全） |
| `_reward_dof_pos_limits` | 移到 cost 系统作为硬约束 |
| `_reward_dof_vel_limits` | 移到 cost 系统 |
| `_reward_torque_limits` | 移到 cost 系统 |
| `_reward_orientation` | 被 `orientation_eular` 替代 |
| `_reward_joint_ref_pos` | 需要参考轨迹，未使用 |
| `_reward_vel_mismatch_exp` | 被 `track_vel_hard` 替代 |
| `_reward_feet_contact_number` | 被 `no_jump` + `stand_2leg` 替代 |
| `_reward_ankle_pos` | ankle 关节角度约束 |
| `_reward_feet_rotation` | 被 `feet_rotation1/2` 替代 |
| `_reward_foot_clearance1` | 世界系变体，与 `foot_clearance`（身体系）功能重叠 |
| `_reward_raibert_heuristic` | 四足 Raibert 启发式（Tinker 是双足） |
| `_reward_feet_contact_forces` (第一版) | 被第二版覆盖 |
