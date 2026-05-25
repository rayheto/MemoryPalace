# LIPM 作为参考生成器的 RL 落脚点规划

> 源项目：`ModelBasedFootstepPlanning-IROS2024`
> 核心思想：**LIPM 给出"理论上动态平衡"的落脚点参考，RL 学会用真实多刚体动力学跟踪这个参考**
>
> ⚠️ **机器人尺度敏感**：该范式在 MIT Humanoid（腿长 0.55m、高 0.62m）上验证有效，但**不适用于 Tinker 这类小型双足**（腿长 0.22m、高 0.30m）。详见 [`tinker/algorithm/LIPM_not_for_tinker.md`](../tinker/algorithm/LIPM_not_for_tinker.md)

---

## 1. 范式定位：模型生成参考 + RL 跟踪

这不是"LIPM 作为软约束惩罚"的范式，而是：

```
机器人状态 (CoM, 速度, 支撑脚)
        ↓
LIPM 解析推进 + XCoM 公式  →  下一步落脚点 step_commands (x, y, θ)
        ↓
RL Policy 观测 step_commands，输出关节动作
        ↓
摆动脚实际落点 vs step_commands  →  step_location_offset
        ↓
形如 e^(-error/σ) 的奖励项 (contact_schedule, 权重 3.0)
```

**对比纯端到端 RL**：稳定性的物理先验（capture point 理论）植入到目标场里，避免几十亿样本去隐式学"不摔倒"。

---

## 2. LIPM 三条简化假设

真实人形机器人是 30+ DoF 多刚体系统。LIPM 把它压缩成一个标量二阶 ODE：

| 假设 | 物理含义 |
|------|----------|
| 全身质量集中在 CoM | 忽略肢体惯量、上身角动量 |
| **CoM 高度 $z = z_c$ 恒定** | 消去 $z$ 方向动力学，3D → 2D |
| 摆动腿质量忽略 | 只有支撑脚一个支点 |

对支撑脚做力矩平衡：

$$
\ddot{x} = \omega^2 x, \qquad \omega = \sqrt{\frac{g}{z_c}}
$$

$y$ 方向同理。注意 $\omega^2 > 0$ —— 是**不稳定**二阶系统（不是钟摆的负号），这是为什么人不站着会摔。

---

## 3. 不稳定系统的闭式解（关键算力优势）

### 3.1 方程 $\ddot{x} = \omega^2 x$ 是怎么来的

牛顿第二定律：$F = ma$，即 $\ddot{x} = F/m$。

把机器人看成"杆长 $z_c$、质点在顶端、底端铰接在支撑脚"的倒立摆。重力 $mg$ 通过质点向下，**支撑脚对杆的反作用力沿杆方向**。把这个反作用力做水平分解（在 CoM 高度恒定假设下消去 $z$ 方向），得到：

$$
\ddot{x} = \frac{g}{z_c}\,x = \omega^2\,x, \qquad \omega = \sqrt{\frac{g}{z_c}}
$$

**直觉**：CoM 离支撑脚水平距离 $x$ 越远，重力的"倾翻力矩"越大，水平加速度越大。

### 3.2 为什么解是双曲函数 —— "不稳定"的真正含义

对比两个看起来很像的方程：

| 方程 | 物理 | 解 |
|------|------|------|
| $\ddot{x} = -\omega^2 x$ | 普通钟摆（吊在上面） | $x(t) = A\cos(\omega t) + B\sin(\omega t)$ |
| $\ddot{x} = +\omega^2 x$ | **倒立**摆（撑在下面） | $x(t) = A\cosh(\omega t) + B\sinh(\omega t)$ |

**关键差异在那个符号**：

- 钟摆：偏离平衡时，恢复力把它拉回来 → 振荡 → 三角函数
- 倒立摆：偏离平衡时，重力把它**推得更远** → 指数发散 → 双曲函数

回忆双曲函数定义：

$$
\cosh(\omega t) = \frac{e^{\omega t} + e^{-\omega t}}{2}, \qquad
\sinh(\omega t) = \frac{e^{\omega t} - e^{-\omega t}}{2}
$$

当 $t$ 增大时，$e^{\omega t}$ 项主导 → **状态指数发散** → 这就是"不稳定"。物理对应：人闭着眼睛站着稍微前倾，倾角会越来越大直到摔倒，不会自己荡回来。

代入当前位置 $x_0$ 和速度 $v_{x_0}$ 作初值：

$$
x(t) = x_0\cosh(\omega t) + \frac{v_{x_0}}{\omega}\sinh(\omega t)
$$

$$
v_x(t) = x_0\,\omega\sinh(\omega t) + v_{x_0}\cosh(\omega t)
$$

> 第二个公式直接对第一个求导即可验证：$\frac{d}{dt}\cosh(\omega t) = \omega\sinh(\omega t)$，$\frac{d}{dt}\sinh(\omega t) = \omega\cosh(\omega t)$。

### 3.3 "闭式解"为什么是 GPU 训练的关键

"闭式"= 只用基本函数（cosh, sinh, exp）就能直接写出答案，**不需要数值求解**。

| 方法 | 算法 | 每个环境每步代价 |
|------|------|------------------|
| 真实多刚体动力学 | RK4 数值积分 | 几十次矩阵求逆，毫秒级 |
| MPC 规划 | 凸优化求解器 | 迭代收敛，10–100 ms |
| **LIPM 闭式解** | **5–6 次浮点乘加** | **几微秒** |

LIPM 公式没有 if/else、没有迭代收敛 → **可以批量并行**。代码里 `T*w` 是 shape `[N, 1]` 的张量，$N$ 是并行环境数：

```python
# humanoid_controller.py:468-471 —— LIPM 推进
# 几千个环境同时算，一次 GPU kernel 完成
x_f  = x_0*torch.cosh(T*w) + vx_0*torch.sinh(T*w)/w
vx_f = x_0*w*torch.sinh(T*w) + vx_0*torch.cosh(T*w)
y_f  = y_0*torch.cosh(T*w) + vy_0*torch.sinh(T*w)/w
vy_f = y_0*w*torch.sinh(T*w) + vy_0*torch.cosh(T*w)
```

这是为什么项目能在 GPU 上同时跑 4096 个环境训练 RL —— **落脚点规划几乎不占算力**。

### 3.4 给定 $(x_0, v_0)$，直接代入 $t = T$ 得到 $T$ 秒后状态

对一般 ODE，"预测未来状态"需要积分。LIPM 因为有闭式解，只要把 $t = T$ 代进去：

$$
x_f = x_0\cosh(\omega T) + \frac{v_{x_0}}{\omega}\sinh(\omega T)
$$

$$
v_{x_f} = x_0\,\omega\sinh(\omega T) + v_{x_0}\cosh(\omega T)
$$

$T$ 是步周期（约 0.35 秒），所以这两行算的是"如果机器人保持当前 CoM 状态、支撑脚不变，0.35 秒后 CoM 会跑到哪里、速度多大"。

这个 $(x_f, v_{x_f})$ 接下来会代入第 4 章的 ICP 公式 $\xi = x_f + v_{x_f}/\omega$，得到"应该把下一只脚放在哪里"。

### 3.5 一句话回顾

> **倒立摆是不稳定系统，状态指数发散；但正因为方程线性，发散过程可以用 $\cosh / \sinh$ 写出闭式解；闭式解可以 GPU 批量算 → 落脚点规划免费。**

---

## 4. ICP / Capture Point —— 把不稳定动力学折叠成一个标量

**核心技巧**：对 $\ddot{x} = \omega^2 x$ 做线性变换

$$
\xi = x + \frac{v}{\omega}
$$

求导：

$$
\dot{\xi} = \dot{x} + \frac{\dot{v}}{\omega} = v + \frac{\omega^2 x}{\omega} = v + \omega x = \omega\,\xi
$$

也就是说

$$
\boxed{\dot{\xi} = \omega\,\xi}
$$

**$\xi$ 的演化是一阶纯发散**。物理意义：

- $\xi$ 是"如果支点不动，CoM 最终会跑到哪里"的稳态投影
- **把支撑脚放在 $\xi$ 位置** $\;\Rightarrow\;$ 等价于 $\dot{\xi} = 0$ $\;\Rightarrow\;$ CoM 不再发散 $\;\Rightarrow\;$ 不摔

把 2 维不稳定状态 $(x, v)$ 压缩成 1 维"危险标量" $\xi$，再把脚放上去抵消它。代码就一行：

```python
# humanoid_controller.py:475-476
eICP_x = x_f_world + vx_f/w     # T 秒后的预测 ICP
eICP_y = y_f_world + vy_f/w
```

注意算的是 **$T$ 秒后的 ICP**（用 $x_f, v_{x_f}$ 而非当前 $x_0, v_{x_0}$），因为落脚命令要给"步周期结束时"的新支撑脚。

---

## 5. XCoM 偏置 —— 从"稳住"到"按速度走"

只把脚放在 eICP 上 → 机器人**原地不动**。要按命令速度行进，叠加偏置 $b$：

$$
b_x = \frac{L_d}{e^{\omega T} - 1}, \qquad
b_y = \frac{W_d}{e^{\omega T} + 1}
$$

其中 $L_d$ 为期望步长、$W_d$ 为期望步宽。

**公式来源**：周期步态的不动点条件。若每步按 $(b_x, b_y)$ 偏置落脚，且 CoM 状态在每步开始时相同（周期解），代入 LIPM 解析解反解得到。

- $(e^{\omega T} - 1)$ 在分母 → 前进方向：每步开始 ICP 相对支撑脚有偏置 $-b_x$，$T$ 秒后发散 $e^{\omega T}$ 倍越过支撑脚，恰好前进 $L_d$
- $(e^{\omega T} + 1)$ 在分母 → 侧向：左右脚每步符号相反，形成稳定的横向极限环（步宽）

```python
# humanoid_controller.py:477-485
b_x = dstep_length / (torch.exp(T*w) - 1)
b_y = dstep_width  / (torch.exp(T*w) + 1)

original_offset_x = -b_x
original_offset_y = -b_y                          # 右脚摆动 → 落 ICP 右侧
original_offset_y[left_step_ids] = b_y[...]       # 左脚摆动 → 落 ICP 左侧

# 按命令朝向 θ 旋转，对齐用户速度方向
offset_x = torch.cos(theta)*original_offset_x - torch.sin(theta)*original_offset_y
offset_y = torch.sin(theta)*original_offset_x + torch.cos(theta)*original_offset_y
```

---

## 6. 最终落脚点公式 —— 一行总结物理直觉

$$
\boxed{\;u = \text{eICP} + \text{offset}\;}
$$

```python
# humanoid_controller.py:487-488
u_x = eICP_x + offset_x
u_y = eICP_y + offset_y
```

翻译：

> **下一步落脚点 = 让机器人不摔的位置 + 按期望速度走的小偏置**

- 第一项 eICP：**保证动态平衡**（capture point 理论）
- 第二项 offset：**实现速度跟踪**（XCoM 周期解）

全程无数值优化、全闭式公式，每个仿真步几千个并行环境几乎免费。

---

## 7. 与 RL 的耦合 —— 奖励函数

```python
# humanoid_controller.py:823-830
def _reward_contact_schedule(self):
    contact_rewards  = (左脚接触 - 右脚接触) * contact_schedule
    tracking_rewards = 3.0 * exp(-step_location_offset / 1.0 / σ)
    return contact_rewards * tracking_rewards
```

**乘积形式的耦合奖励**：只有"该接触的脚正在接触" **且** "落点准"才得满分。把时序（步态相位）和空间（落脚位置）耦合进单一奖励项。

权重 `contact_schedule = 3.0`（配置文件高权重项），$\sigma$ 为 `tracking_sigma = 0.25`。

奖励核形如：

$$
R = e^{-\|p_{\text{foot}} - p_{\text{LIPM}}\|^2 / \sigma^2}
$$

---

## 8. 调用链路 / 时间尺度

| 频率 | 函数 | 做什么 |
|------|------|--------|
| 每步周期 (~0.35s) | `_generate_step_command_by_3DLIPM_XCoM` | LIPM + XCoM 算下一步落脚点 |
| 每仿真步 (0.01s) | `_update_LIPM_CoM` | LIPM 把 CoM 推进 $dt$，得瞬时 CoM 参考 |
| 每仿真步 | `_reward_contact_schedule` | 比较实际脚 vs LIPM 落脚点，给奖励 |
| 每仿真步 | RL policy forward | 观测含 step_commands，输出关节动作 |

**反馈式 LIPM**：`_update_LIPM_CoM` 中 `self.LIPM_CoM[ids] = self.CoM[ids].clone()` —— 每次用真实 CoM 重置 LIPM 初始状态，模型误差不累积漂移。是"在线滚动 + 反馈"，不是开环规划。

---

## 9. 为什么叫"理论上"动态平衡

LIPM 三条假设都不严格成立：
- 真机器人有摆动腿质量、上身摇晃、躯干角动量
- CoM 高度其实在波动（非恒定 $z_c$）
- 关节力矩有限、脚底摩擦有极限

所以 LIPM 给的是**简化模型下的稳定解**，未必真机器人能完美执行。RL 学到的本质是 **LIPM 模型误差的补偿器** —— 怎么用真实关节动作和上身姿态去贴近 LIPM 这个理想参考。

---

## 10. 关键代码索引

| 路径 | 用途 |
|------|------|
| `LIPM/LIPM_3D.py` | LIPM 基础数学模型（numpy 单体版） |
| `gym/envs/humanoid/humanoid_controller.py:424-494` | XCoM 落脚点规划（torch 批量版） |
| `gym/envs/humanoid/humanoid_controller.py:569-599` | 在线滚动 LIPM CoM 推进 |
| `gym/envs/humanoid/humanoid_controller.py:823-830` | contact_schedule 奖励函数 |
| `gym/envs/humanoid/humanoid_controller_config.py:301-335` | 奖励权重配置 |

---

## 11. 设计哲学总结

> **复杂留给学习，物理结构留给模型。**

- LIPM：解析、稳定、可微的落脚点生成器（物理先验）
- RL：在真实多刚体动力学里"模仿" LIPM 参考（处理模型残差）
- capture point：把不稳定 2D 状态压缩成 1D 标量的精妙数学结构（让落脚点计算变得平凡）
