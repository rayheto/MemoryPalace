# 把 LIPM 换成 SLIP 数值积分的失败调试记录

> 源项目：`ModelBasedFootstepPlanning-IROS2024/SLIP/`
> 路线 B：用 SLIP（Spring-Loaded Inverted Pendulum）的非线性 z 动力学，替代 LIPM 的 `z=zc` 恒定 + cosh/sinh 解析解，给落脚点规划器提供"会上下起伏"的参考。
> 结论：**在固定步时 + Raibert 启发式 + 主动腿长 PD 的组合下没有走出稳定步态**。下面记录为什么。

---

## 1. 目标与范式

LIPM 版本能跑是因为它做了三件事：

1. CoM 高度恒定 → 解析 cosh/sinh，没有数值漂移；
2. XCoM 是 LIPM 解析 ICP 的精确闭式 → 落脚点公式 `b_x = s_d/(e^{ωT}-1)` 是收敛的不动点；
3. `switchSupportLeg` 只搬运 `x_t, y_t, vx_t, vy_t`，z 自动维持。

想换 SLIP：保留 LIPM scaffolding（5 个方法：`__init__ / initializeModel / step / calculateXfVf / switchSupportLeg`），把动力学换成弹簧腿。

```python
def _stance_dyn(self, s):
    x, y, z, vx, vy, vz = s
    L = sqrt(x² + y² + z²)
    L0_eff = L0 + kp_z*(zc - z) - kd_z*vz   # 主动腿长 PD
    F = k*(L0_eff - L)                       # 弹簧力
    a = F/(m·L) · [x, y, z] - [0, 0, g]
```

落脚点策略：放弃 XCoM 解析（LIPM 专属），改用 **Raibert 启发式 + SLIP 数值预测末态**：

```
u = CoM_TD + 0.5·T·v_TD + k_r·(v_TD - v_d) ± step_width/2
```

其中 `v_TD = SLIP.calculateXfVf()` —— 让 z 动力学通过 vx_f 间接进入落脚点决定。

---

## 2. 实际跑出来的失败模式

最终参数：

```python
SLIP3D(dt=0.02, T=0.30, mass=50, k_leg=20000, L0=0.74, g=9.81,
       n_substeps=20, kp_z=0.3, kd_z=0.05, vz_restitution=0.0, k_r=0.3)
```

10 秒 33 步，headless trace：

```
x range: [-5.92, 5.52]    y range: [-5.12, 5.73]
z range: [-5.42, 1.20]    ← z 直接钻到地下 5m
vx range: [-99, +102]     ← 速度发散到 100 m/s
```

单步追踪：

```
init:        x=0.200 y=-0.300 z=0.600 vz=0
end stance1: x=0.486 y=-0.471 z=0.470 vx=1.44 vy=-1.08 vz=-1.17
            ↑ z 掉了 13 cm，vz 已经是 -1.17 m/s
after switch (vz_restitution=0): vz_0=0  但 z_0=0.470 已经低于 zc=0.6
end stance2: z=0.352 vz=-1.17 vx=0.20
            ↑ z 又掉 12 cm，能量没补回来
```

**每一步 CoM 净掉高 ~12cm**。在第 4~5 步左右越过零点，之后弹簧把它向上"踹"，触发横向位置发散。

---

## 3. 为什么不成功 —— 三层原因

### 3.1 物理层：纯 SLIP 在固定步时下不存在 free 极限环

SLIP 走稳的两个先决条件：

| 条件 | LIPM | SLIP |
|------|------|------|
| 步态周期 | 任意 T，闭式都解 | **必须用事件触发**：腿伸直 (L = L0) → 切换支撑 |
| 能量管理 | 不需要（z 恒定 → 重力做零功） | 弹簧能量必须每步精确归还（无损 SLIP）或主动注入 |

我们用了**固定 T=0.30s**强制切换。这意味着：

- 弹簧没有走完整周期就强切，触底速度残留 → vz 在切换前是负的；
- 步与步之间没有能量平衡机制，每一步净损耗叠加 → z 单调向下。

文献里 SLIP 走稳定步态用的是 Raibert 1986 三回路：**腿长决定 hop 高度 + 腿摆速度决定前进速度 + 髋扭矩决定姿态**，并且每步是**触地-脱离-触地**事件触发，不是固定 T。

### 3.2 控制层：主动 PD 量级估算错了

`L0_eff = L0 + kp_z·(zc - z) - kd_z·vz` 当时按"让 z 偏 0.1m 产生 ~mg 反力"估算 kp_z ≈ 0.25~1.0。问题是这个估算假设的是**静态平衡**，没考虑：

1. 主动腿长会改变弹簧力符号（如果 L0_eff < L 弹簧变拉力，把 CoM **拉下**而不是托起）；
2. kp_z·(zc - z) 是补偿，但 kd_z·vz 会在 vz<0（下落）时给 L0_eff **正贡献**，导致弹簧在最该用力的时候反而泄掉劲；
3. kp_z=1.0 直接数值爆炸（弹簧刚度 k=20000 × ΔL₀ = 2000N 量级，远超合理）。

### 3.3 几何层：初始 L 和 L0 差 4cm 已经偏大

`foot=(−0.2, ±0.3, 0)`, `CoM=(0, 0, 0.6)` → 实际腿长 `L = sqrt(0.04 + 0.09 + 0.36) = 0.7m`。

试过 L0=0.6（弹簧已伸长 0.1m → 触地瞬间弹簧把 CoM **拉向地面**），改 L0=0.74（4cm 压缩，触地有 800N 向上力）。

但 4cm 压缩对 m=50kg 只产生 `F=20000·0.04=800N`，刚刚好抵消 mg=490N，剩 310N 向上 → vz 增益 6.2 m/s²·dt → 0.3s 内能给 ~0.5m 向上位移。问题是这个力**会随 L 变化**：CoM 经过支撑脚正上方时 L 最小 → 弹簧最强；后半步 L 增长 → 弹簧变弱甚至变拉力。

**净效应**：前半步 z 弹起 5~10 cm，后半步 z 跌 15~20 cm，**不对称**。

---

## 4. 三个旋钮的真实作用

代码留了三层 z 控制（从"老老实实 SLIP"到"完全 cheat"）：

| 旋钮 | 作用 | 当前值 | 完全 cheat 等价 |
|------|------|--------|----------------|
| `kp_z, kd_z` | stance 中持续主动腿长 PD | 0.3 / 0.05 | 太弱，治标不治本 |
| `vz_restitution` | 触地时 vz *= 此系数 | 0.0（完全清零） | 0 = 完全清零 |
| `z_t ← zc` | 触地时 z 也归位 | **没启用** | 启用就退化成 LIPM 路径 A |

把三个旋钮全开（vz=0, z=zc, PD=0），等价于 LIPM。把它们都关（vz_restitution=1, PD=0），是 free SLIP，**没有事件触发会发散**。中间区段没有走通。

---

## 5. 如果再做一次会怎么做

按优先级：

1. **切到事件触发** —— `switchSupportLeg` 的条件改成 "L >= L0 且 vz > 0"（腿伸直且在抬升），而不是 `i % swing_data_len == 0`。这是 SLIP 走通的关键改动，固定 T 是 LIPM 的奢侈品。
2. **回到经典 SLIP 控制** —— Raibert 1986 三回路而不是 1D Raibert + Z PD。`Vrest = L0` 不是常数而是按速度误差自适应：`L0_cmd = L0_nominal + kp_v·(v_d - v_TD)`。
3. **能量观测** —— 每步 log `E = 0.5·m·|v|² + m·g·z + 0.5·k·(L0 - L)²`，看是单调降还是震荡。
4. **几何要对齐** —— `L0 = sqrt(foot_offset² + zc²)` 严格相等，初始 stance 没有任何弹簧压缩，靠 swing 触地时压缩；这是 SLIP 标准启动条件。
5. **如果只是给 RL 当 reference**：根本不需要走稳的 SLIP，只需要每步给一个比 LIPM 更"现实"的 (foot_x, foot_y, z_ref(t)) 序列。这种情况下 vz_restitution=0 + 主动 PD 给一个**有 z 波动但 bounded** 的参考就够了 —— RL 跟踪的是参考，参考本身发不发散无所谓，只要 bounded。**这是这个 demo 真正应该做的事**，但我跑偏到追求 demo 本身走稳。

---

## 6. 一句话教训

> **SLIP 是 hybrid dynamical system，离散切换事件必须由物理状态触发，不能拍脑袋固定步时**。LIPM 因为 z 恒定 + 解析解，没有这个约束，所以可以用固定 T；把 LIPM scaffolding 直接套到 SLIP 上必然损失这个差异 —— 这不是参数调不准，是范式不匹配。

---

## 7. 未来研究方向：用 SLIP 引导 RL 学到 running gait（flight phase）

### 7.1 观察到的 LIPM 训练局限

用 LIPM 当 reference generator 训出来的 policy：

- **永远至少一只脚着地** —— contact_schedule 只有 {左单支撑, 右单支撑, 双支撑}，没有 0 接触段；
- 速度起来后摔倒 —— 因为 single-support 时长被 LIPM 的 T 卡死，腿摆速度跟不上 CoM 前移，强行单脚支撑必摔；
- 物理根因：LIPM `z=zc` 恒定 → 重力做功为 0 → **不可能有 flight phase**（flight 要求重力做功改变动能 / 势能）。

### 7.2 SLIP 为什么有 flight phase

SLIP 是真正的 hybrid system，三种 mode：

| Mode | 物理条件 | 动力学 |
|------|---------|--------|
| Stance | `L ≤ L0` 且有接触 | 弹簧力 `F = k(L0 - L)` + 重力 |
| Apex / Flight | `L > L0`（弹簧自然长度，无接触）| 纯重力，抛物线 |
| Touchdown | `L = L0` 且 vz < 0 | 切换 stance |

running gait 的标志就是 flight phase 占整个步态周期的非零比例（trot 0%，run 20~40%，sprint >50%）。

### 7.3 接入 RL 的设计草案

**关键设计原则**：不要把 z 当 reward 来跟踪（z 是被动量，policy 不知道怎么"动"出 z），而是让 contact_schedule 强制 policy 学起跳。

```
SLIP reference generator (event-triggered)
        ↓ 每一步输出
{ foot_xy_ref,        ← 落脚点（同 LIPM）
  foot_z_traj(t),     ← swing 脚的 z 轨迹，含起飞/着陆 timing
  contact_schedule,   ← 关键：{LL, LR, RR, FLIGHT(0,0)} 四态而非三态
  z_ref(t) }          ← CoM z 轨迹，供 soft 跟踪
```

**reward 三层（按权重从大到小）**：

1. **contact_schedule 匹配**（权重最大）—— 直接复用 LIPM 框架里那一项，但 schedule 含 FLIGHT 段。policy 想避免高 penalty 就必须在 schedule 要求 0 接触时把两脚都抬起来 → 自然 emerge 出 flight。
2. **foot_xy_offset**（中等权重）—— 落脚点跟踪，同 LIPM。
3. **z 轨迹 soft 跟踪**（小权重）—— 不当硬约束；给个 e^(-|z - z_ref|/σ) 的 bonus，让 policy 在多个可行 gait 中倾向于上下波动 ≈ SLIP 预测的那个。

**observation 增加**：把 `(z_ref, vz_ref, schedule_ahead_N_steps)` 拼进 policy input，让它能"看到"参考。

### 7.4 课程学习路径

```
速度 0~1.0 m/s   → SLIP 输出 ≈ LIPM（FLIGHT 占比 ~0）   → walk
速度 1.0~2.5 m/s → SLIP 输出含 ~10% FLIGHT             → bounce walk
速度 2.5~4.0 m/s → SLIP 输出含 ~30% FLIGHT             → run
速度 >4.0 m/s    → SLIP 输出含 ~50%+ FLIGHT            → sprint
```

curriculum 关键：**让 SLIP 自己根据 v_d 决定 FLIGHT 占比**，不是人工设定。SLIP 在合理 (k, L0, T) 参数下，速度越高弹簧能量越大，flight 越长 —— 这是物理而不是 heuristic。

### 7.5 预期收益 vs 风险

**收益**：

- 解锁 LIPM 框架的速度瓶颈，让 RL 训出真正的 running gait；
- 把"双脚腾空"的物理先验植入 reward 场，而不是靠数十亿样本试出来；
- 步态自然性提升（人跑步就是 SLIP 行为，不是 LIPM 行为）。

**主要风险（按严重度）**：

1. **sim2real gap 爆炸** —— flight phase 着陆冲击大，仿真线性化的接触模型 vs 真机橡胶脚底差异巨大。LIPM 训出来的"安全 gait"sim2real 已经勉强，run 会差一个数量级。
2. **点质量假设失效** —— SLIP 忽略上身角动量和踝力矩，但真机 flight phase 落地姿态完全靠这俩补偿。reward 必须额外叠加 ZMP / 角动量惩罚，否则 policy 学到的是 "frog hopping" 而不是 running。
3. **SLIP reference 本身要先能稳定生成** —— 见第 1~6 节，事件触发的 SLIP 还没调通，这是前置任务。
4. **schedule 强约束的副作用** —— FLIGHT 期间 policy 可能学到的是 "蹦跶到正确高度然后空中扭动"，不是真正向前推进。需要给 forward velocity reward 足够大的权重平衡。

### 7.6 下一步具体任务

按优先级排：

- [ ] **P0**：把 SLIP_3D.py 的 `switchSupportLeg` 改成事件触发（L=L0 且 vz 符号判定），先证明能输出含 flight 的稳定 limit cycle；
- [ ] **P1**：扩展 `calculateXfVf` 输出 `(contact_schedule, z_traj)` 时间序列而不仅是末态；
- [ ] **P2**：在 humanoid_controller.py 里加 FLIGHT 这个 phase enum，扩展 contact_schedule 奖励项；
- [ ] **P3**：observation 拼接 z_ref；
- [ ] **P4**：curriculum scheduler 用 v_d 驱动 SLIP 参数，自动产生 walk→run 过渡；
- [ ] **P5**：sim2real 缓解（ZMP 惩罚、域随机化加大着陆刚度变化范围）。

### 7.7 替代方案备忘

如果 SLIP 路线工程量太大，备选：

- **直接 demo 数据驱动**：用真人 motion capture 的 running 数据当 reference（AMP / DeepMimic 路线），跳过解析模型；
- **TSLIP / aSLIP**：带躯干角动量的 SLIP 变体，更贴近 humanoid，但解析性几乎丧失，纯数值积分，复杂度和上 7.3 差不多；
- **Spring-mass + flywheel**：Pratt 系列把 SLIP 加上 reaction wheel 模拟躯干，trajectory optimization 文献里有现成参考。
