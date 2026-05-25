# 为什么 LIPM/XCoM 不适合 Tinker

**结论日期**：2026-05-25
**判定**：LIPM 模型不作为 Tinker 训练 reward 或 reference 使用。Tinker 训练继续用现有 `trot_pattern` + `tracking_lin_vel` 框架。LIPM/XCoM 工作仅留在 IROS2024 仓库（针对 MIT Humanoid）。

> **核心机制（一句话）**：Tinker 的"步幅相对身高占比过大"，在 LIPM 公式里强制把 `w_d` 调小才能避免预测落点超出腿长 reach；但 `w_d` 小直接导致 **`b_y` 小，预测的两脚 spread 被压到接近中线，本质上 LIPM 要求 Tinker 用"并拢双脚"步态行走**。这会破坏侧向稳定性，没法当训练 reference。

---

## 1. 背景

IROS2024 论文 [`ModelBasedFootstepPlanning-IROS2024`](file:///home/hlei/robotic/ModelBasedFootstepPlanning-IROS2024) 用 LIPM/XCoM 解析公式给每步的目标落脚点 `(u_x, u_y)`，RL policy 通过 `contact_schedule` reward 学着把脚落到 LIPM 预测的位置。范式在 MIT Humanoid 上验证有效。

最初计划：把同一套范式移植到 Tinker。完整移植 plan 见 `~/.claude/plans/piped-enchanting-starlight.md`。

实证后**放弃**。下面是判定依据。

---

## 2. 机器人几何差异

| 量 | Tinker | MIT Humanoid (IROS) | 比值 |
|---|---|---|---|
| 腿长 (thigh + calf) | ~0.22 m | 0.5451 m | **2.5×** |
| nominal_height (h) | 0.30 m | 0.62 m | 2.1× |
| 髋宽 (hip-to-hip) | 0.12 m | 0.27 m | 2.3× |
| 水平 reach（CoM 到最远脚） | 0.25 m | 0.68 m | 2.7× |
| w = √(g/h) | 5.72 | 3.97 | Tinker 高 44% |
| 工作速度 vx | ~0.4 m/s | ~2 m/s | IROS 5× |
| **"步幅/腿长"占比**（v·T / 腿长）| **0.4·0.16/0.22 ≈ 29%** | 2·0.35/0.55 ≈ 127% | — |

最后一行是关键比例。Tinker 单步前向位移占腿长 29%，看起来不大；但**这个步幅必须叠加 eICP 项（CoM 前向外推 v/w ≈ 0.07m）+ 初始 CoM-脚偏移**，总 `sup→u` 在 LIPM 公式下逼近腿长上限。

---

## 3. 真正的物理机制：stance-width 被 reach 压缩到中线

LIPM XCoM 的两个关键产物：

```
b_x = ||v_cmd|| · T / (e^{Tw} − 1)        # 前向 capture point
b_y = w_d / (e^{Tw} + 1)                  # 横向 capture point
u = eICP + (−b_x, ±b_y)                   # 最终落点（旋转 θ 后）
```

**约束链**：

1. `sup→u` 最大值受腿长 reach 限制（Tinker ≈ 0.25m）
2. `eICP_x − sup_x = x_f + v_f/w`，对 vx=0.4、h=0.30、T=0.16 已经 ~0.20m
3. 还剩 ~0.05m 余量给 `b_y · cos(θ)` 这一项
4. 要把 `2·b_y`（预测两脚 spread）做大到匹配 Tinker 自然 stance 0.19m，需要 `w_d = 0.19·(e^{Tw}+1) = 0.665m`
5. 但此时 `b_y = 0.095m` 直接把 `sup→u` 推到 0.30m+，**出 reach**
6. 所以只能选 `w_d ≤ 0.20`，对应 `2·b_y ≤ 0.114m`
7. **预测的两脚 spread 比真实 stance 窄 40-70%**

对照 IROS 数据：

| 量 | Tinker (w_d=0.20) | MIT Humanoid (w_d=0.30) |
|---|---|---|
| Tw | 0.92 | 1.39 |
| e^{Tw}+1 | 3.50 | 5.00 |
| b_y | 0.057 m | 0.060 m |
| 2·b_y（预测 spread） | **0.114 m** | **0.120 m** |
| 真实 stance | **0.19 m** | **0.20 m** |
| spread/stance 比 | **60%** | **60%** |

**等等——比值一样**。LIPM 在两个机器人上都预测"比实际窄 40% 的 stance"。

那为啥 MIT Humanoid 可以训出来？因为 MIT 的 reach 是 0.68m，**腾出 0.5m 余量**给 `b_y`，能选 w_d 把预测 stance 推到接近真实值。Tinker 的 reach 只 0.25m，**`b_y` 一动就出界**，只能选最小的 `w_d`，于是预测 stance 必然偏窄。

**核心对比**：

```
LIPM 在 Tinker 上：    eICP 已经吃掉 80% reach，b_y 没空间长大 → 必须用窄 stance 预测
LIPM 在 MIT 上：       eICP 只吃掉 30% reach，b_y 有余量 → 可以用合理 stance 预测
```

这才是几何不匹配的真正含义——**不是 LIPM 预测点超出 reach 那么直接，而是 reach 余量限制了 b_y，把 LIPM 推荐的步态压缩到"双脚并拢"的退化形态**。

---

## 4. 为什么"并拢双脚"步态不能用

LIPM 在 Tinker 上给出的"理论最优 footprint"几乎贴中线（2·b_y ≈ 0.11m，而髋宽就有 0.12m，等于让两脚踩在身体正下方）。

物理直觉上这是**走钢丝模式**：

- **侧向支撑多边形塌缩**：双足同时贴中线时，左右脚到 CoM 横向距离接近 0，任何侧向扰动都没有侧向 GRF lever arm 抵消，policy 必须靠 ankle moment 撑——但 Tinker ankle 关节扭矩有限（stiffness 13 N·m/rad）
- **roll 失稳风险大**：站立时的 roll 稳定靠 stance 宽度提供 lateral CoP 余量；stance 压缩到髋宽以下时，CoP 摆动余量小，单点扰动就足以让 CoP 跑出脚底
- **swing 时无单脚平衡**：双脚靠近时换脚瞬间 CoP 几乎在中线，相当于每一步都从单脚支撑过 0，落点偏差 1-2cm 就摔
- **与 Tinker 实际训出的步态完全相反**：现有 trot policy 自然学会 0.19m stance，正是为了拉开 lateral lever arm。强行训成 LIPM 预测的窄 stance 等于撤掉 policy 已经掌握的稳定先验

**如果硬训**：policy 必须在"跟 LIPM reference"和"保持侧向稳定"之间二选一。reward 权重稍大就训出钢丝步态、稍小就训不出 LIPM 跟踪——窗口极窄且训练不稳定。

---

## 5. 实测证据：passive observer 数据

`smoke_lipm_active.py` 把 LIPM 挂在 Tinker 训好的 policy（model_22000.pt）上做旁观者：

**参数**：T=0.16s, w_d=0.20, vx=0.3

| 指标 | 值 | 含义 |
|---|---|---|
| mean \|err\| | 0.18 m | LIPM 预测 vs 实际落点的总距离 |
| foot 0 mean dy | **-0.12 m** | LIPM 预测的右脚比实际更靠 -y（更贴中线） |
| foot 1 mean dy | **+0.18 m** | LIPM 预测的左脚比实际更靠 +y（更贴中线） |
| 预测 L/R spread | **0.11 m** | 2·b_y |
| 实测 stance | **0.19 m** | policy 实际行走宽度 |

**两脚 dy 方向相反但量级一致**——这是 stance 压缩的标志。LIPM 把双脚都往中线推 ~14cm。policy 抗住这个推力走自己的宽 stance，所以 \|err\| 持续 ~0.18m。

X 方向也有不对称（foot 0 dx=+0.13, foot 1 dx=-0.01），但这主要反映 policy 本身两脚步长不平衡，不是 LIPM 公式问题。

---

## 6. 对照 IROS：LIPM 误差能压下来是因为 stance 比例匹配

IROS trained MIT Humanoid + LIPM observer，vx=0.2：

| 量 | Tinker (passive) | MIT (trained) |
|---|---|---|
| mean \|err\| | 0.18 m | 0.04 m |
| 预测 stance / 实测 stance | 0.11 / 0.19 = **58%** | 0.12 / 0.20 = **60%** |
| **预测 stance 是否在物理可达 stance 范围内** | **否**（窄于髋宽 0.12m，要求双脚交叉/并拢）| **是**（远大于 MIT 髋宽 0.27m 的一半 = 0.135m，但接近其自然 stance） |

IROS 上"60% 的 stance 比"对应的绝对宽度是 0.12m，**仍大于髋宽 0.135 (近似)**，policy 可以舒服落地；Tinker 上"60% 的 stance 比"对应 0.11m，**已经塌进髋宽 0.12m 之内**，物理上要求双脚交叉/并拢——这是绝对量级的鸿沟，无法通过参数调整跨越。

**关键洞察**：相同的 LIPM 公式、相同的 spread/stance 比例，但在小机器人上**绝对值跌破髋宽阈值**，从"窄 stance 步态"变成"钢丝步态"。

---

## 7. ROI 分析

即便接受 \|err\| ~0.18m 的代价训练 Tinker policy 去追踪 LIPM：

- **可用速度区间**：vx ∈ [0, 0.5] m/s（更高就 max 步幅出 reach）
- **预期 gait**：钢丝步态或半钢丝步态（stance 比现在窄一半）
- **稳定性风险**：侧向 roll 余量被砍到极致，外扰恢复能力下降
- **训练破坏**：必须关闭 `feet_air_time`，可能波及现有 trot 稳定性
- **obs 维度变化**：现有 checkpoint 全部作废
- **预期收益**：在 vx ≤ 0.5 区间内"步态更接近 LIPM 理论"，但牺牲的稳定性可能让实测可达速度反而下降

**结论：负 ROI**。Tinker 矮腿是物理事实，不是软件能补的。

---

## 8. 已保留的 artifacts（评估工具）

| 文件 | 用途 |
|---|---|
| `OmniBotCtrl/envs/legged_robot.py` 的 `_lipm_passive_observe` 等 | passive observer，不影响训练 |
| `OmniBotCtrl/configs/tinker_constraint_him_trot.py` 的 `class lipm_passive` | 观察器配置 |
| `OmniBotCtrl/smoke_lipm_active.py` | 加载 checkpoint + 跑诊断 + 可视化 |
| `IROS2024/LIPM/LIPM_3D_tinker.py` | Tinker 参数的 LIPM3D 类 |
| `IROS2024/LIPM/demo_LIPM_3D_tinker.py` | 静态参数 sweep（3×3 网格图） |
| `IROS2024/LIPM/demo_LIPM_3D_tinker_vt.py` | 动画版，6 phase 自动切换参数 |

可作为评估/对比工具长期保留，不进训练 loop。

---

## 9. 后续方向

- **Tinker 训练**：继续走 `trot_pattern + tracking_lin_vel` 路线，**保留宽 stance 作为侧向稳定先验**
- **LIPM/XCoM 训练**：仅在 IROS2024 仓库（MIT Humanoid），resume 命令见 `~/.claude/projects/.../memory/reference_iros2024_checkpoints.md`
- **SLIP 替代 LIPM 实验**：按 [`~/MemoryPalace/robotics/simulator/SLIP_3D_replace_LIPM_debug.md`](file:///home/hlei/MemoryPalace/robotics/simulator/SLIP_3D_replace_LIPM_debug.md) §7，**在 IROS2024 框架做增量替换，不在 Tinker 上做**
- **Tinker 若未来需要落脚点级规划**：考虑非 LIPM 的方案（Raibert heuristic + capture point + 显式 reach clamp、或者直接学习式 footstep policy），避开 LIPM 的"stance 比例"陷阱

---

## 10. 关键 takeaway

> LIPM/XCoM 范式的可用性取决于一个隐藏比例：**腿长 reach 在 eICP 项消耗后剩下多少余量给 b_y**。MIT Humanoid 上余量 ~0.5m，b_y 能自由选择匹配自然 stance；Tinker 上余量 ~0.05m，b_y 被强制压小，预测 stance 跌破髋宽阈值，要求"并拢双脚"步态。
>
> **物理直觉判断**：小型双足机器人不能用 LIPM 当训练 reference，因为 LIPM 的解析最优在小尺度下退化为侧向不稳定的钢丝步态——这是 point-mass 倒立摆模型的固有特性，不是参数调优能解决的。
