# Tinker 落脚点 reference：RL + MPC/WBC 混合方向

**立项日期**：2026-05-25
**前置结论**：[`LIPM_not_for_tinker.md`](LIPM_not_for_tinker.md) —— LIPM/XCoM 解析公式在 Tinker 尺度下被 reach 余量挤压到"钢丝步态"，不可作训练 reference。
**新方向**：用**带 reach / 可达性 / 接触约束的优化型 MPC（或 NMPC）作为 reference generator**，喂给 RL policy 训练。

---

## 1. 为什么是 RL+MPC 而不是纯 RL 或纯 MPC

业界已经收敛到这条路：

- **波士顿动力**：Marc Raibert 公开表态"传统控制技术在未来一段时间内仍是机器人控制最有效的方式"，同时积极尝试 RL。Atlas 后续工作明确采用 **RL 生成多样化行为 + NMPC 精确跟踪**，实现攀爬大跨度地形。
- **ETH ANYmal**：ETH legged-robotics 团队把 MPC 跟 RL 结合做 perceptive locomotion，在复杂越障场景效果显著强于纯 RL；并且开源了 OCS2 NMPC 框架。
- **LeggedGym 范式自身**：很多 reward 项（tracking、smoothness、energy）的权重设计本质等同于 MPC cost 项加权 —— 二者都是优化问题，只是求解时机不同（RL 离线、MPC 在线）。

**RL 与 MPC 互补性**：

| 维度 | RL | MPC/NMPC |
|---|---|---|
| 强项 | 闭环鲁棒性、高维感知、长期回报、对模型误差宽容 | 明确约束（reach、摩擦锥、关节限位）、可解释、采样效率高 |
| 弱项 | 训练慢、约束硬塞进 reward 不可靠、行为不可控 | 模型依赖强、对扰动外的未建模动力学脆、长 horizon NLP 慢 |

**对 Tinker 的具体意义**：LIPM 失败的本质是"约束（reach、髋宽）没有显式进规划"。换成优化型 MPC，reach 0.25m 和髋宽 0.12m 就是 QP/NLP 的硬约束，不会再退化到中线塌缩。MPC 给出的落脚点序列就能作为 RL policy 的 reference / observation 增强 / reward shaping 目标。

---

## 2. 推荐的整体架构

```
                       ┌─────────────────────────┐
                       │  Command (vx, vy, yaw)  │
                       └────────────┬────────────┘
                                    │
                                    ▼
        ┌───────────────────────────────────────────────────────┐
        │  Reference Generator (MPC / NMPC)                     │
        │   - 状态: CoM, joint q, contact phase                  │
        │   - 约束: reach 0.25m, 髋宽 0.12m, 摩擦锥, 关节限位     │
        │   - 输出: u_x, u_y(N步), F_contact, swing trajectory   │
        └────────────────────┬──────────────────────────────────┘
                             │ (asynchronously cached per env)
                             ▼
        ┌───────────────────────────────────────────────────────┐
        │  RL Policy (LeggedGym / Isaac Gym)                    │
        │   obs += [next footstep in base frame, phase clock]   │
        │   reward += contact_schedule + step_location          │
        │   action = joint torques                              │
        └───────────────────────────────────────────────────────┘
```

**关键设计选择**：

- MPC **不进 inner loop**（每个 sim step 跑一次太贵），而是按 step_period 异步更新 reference，policy 在两次 MPC update 之间用插值/缓存的落脚点
- MPC 输出的是**轨迹与落脚点**，**不是 torque**；torque 仍由 RL policy 直接输出，保留 RL 的鲁棒性
- 与 IROS2024 范式一脉相承：换掉 reference generator 后台、reward 与 obs 接口保持一致

---

## 3. 开源实现选型调研

| 项目 | 语言/依赖 | 形态 | MPC 类型 | WBC | Python 接入 | 实时性 | 活跃 |
|---|---|---|---|---|---|---|---|
| **Crocoddyl** (LAAS) | C++ + 官方 Python, Pinocchio | quad/biped/humanoid (Talos, Cassie 例子) | DDP / FDDP / Box-FDDP，全身动力学 | 隐式 (whole-body OC) | **极佳**，pip + notebook 教程多 | 50–200 Hz MPC | 非常活跃 |
| **OCS2** (ETH) | C++17, ROS1, Pinocchio, HPIPM | quad + humanoid (ANYmal, Cassie 类双足) | SLQ / iLQR / Multiple-shooting NMPC, 1–2s horizon | 无（自接 WBC） | PyBind11 wrapper (社区 fork) | 50–200 Hz | 活跃 2024-2025 |
| **bipedal-locomotion-framework** (IIT) | C++, YARP, Pinocchio, OSQP | humanoid (iCub, ergoCub) | DCM-MPC + WBC | 是 | 完整 Py bindings | 100 Hz MPC / 1 kHz WBC | 活跃 |
| **biped_lip_mpc / DCM-MPC** (Scianca/Smaldone IS-MPC) | C++/Python, Eigen, OSQP/qpOASES | biped | **滚动 LIP QP**（落脚点是决策变量，可加 ZMP/CMP/reach 约束，**≠ LIPM 解析公式**） | 配 stabilizer | 部分 Python | 100–500 Hz | 中等，学术原型 |
| **TSID + Pinocchio** | C++ + 官方 Python | humanoid + biped | 单步 QP（**纯 WBC，非预测**） | 是，核心就是 WBC | 极佳 pip 安装 | 1 kHz | 活跃 |
| **mit-biped-mpc / Cheetah-MPC** | C++, Eigen, qpOASES | 原生 quad，Humanoid-MPC 分支 biped | Convex QP, SRBD + force | 内置 force mapping | 无官方 Py | 200–500 Hz | MIT Humanoid 分支活跃 |
| **TOWR** (ETH Winkler) | C++, Ifopt + Ipopt | 任意 legged | **离线 NLP** trajectory optimization | 否 | 非官方 py wrapper | 离线 0.5–5s/轨迹 | 半停更 (2022) |
| **quad-SDK** | C++, ROS, NLopt | quadruped only | Convex MPC + NLP global planner | 是 | ROS topic, 无直接 Py | 100 Hz | 中等 |

**关键差异点**：
- `biped_lip_mpc` ≠ LIPM 解析：它把落脚点作为 QP 决策变量，加 reachability/ZMP 约束 → **不会**像 XCoM 那样塌缩到并脚，对 Tinker 短腿应该可行。
- `Crocoddyl` / `OCS2` 用全身或 centroidal 动力学，能感知真实质量与髋宽 → 不会有 LIPM 的尺度退化。
- `TSID` 是瞬时 WBC，需要上层规划器给 CoM/footstep 参考 → 适合做 OCS2/Crocoddyl 输出的下游 tracker，不适合单独当 reference generator。
- `TOWR` 是离线轨迹优化，不适合 RL rollout 在线使用。

---

## 4. 推荐路线（按落地优先级）

### 路线 A（首选）：Crocoddyl-based footstep reference

**理由**：Python 一等公民、Pinocchio 全身动力学、Cassie/Talos 例子直接可改 URDF、LAAS 持续更新 box-FDDP/constrained DDP，**最适合塞进 RL 训练 rollout**。

**步骤**：
1. 装 `pip install crocoddyl pinocchio` —— 5 分钟
2. 把 Tinker URDF 导入 Pinocchio，验证 mass / inertia / joint limits
3. 跑 Crocoddyl 的 Cassie walking 示例，改成 Tinker 的几何参数
4. 离线验证：给 vx=[0, 0.5, 1.0]，看输出落脚点 spread 是否合理（**目标：stance 接近 0.19m，不再塌中线**）
5. 接入 LeggedGym：每 step_period 调用一次 Crocoddyl，把下一步 (u_x, u_y) 写进 obs
6. 训练对照实验：LIPM-baseline（=不用 reference）vs Crocoddyl-ref，看 vx ≥ 0.5 的存活率

**预估工作量**：调研 + 离线验证 2–3 天；接训练 loop 1 周；首轮训练对照 2 周。

### 路线 B（备选轻量）：DCM-MPC / biped_lip_mpc baseline

**理由**：2–3 天能跑通，明确避开 LIPM 解析退化，可作为对 Crocoddyl 的下限对照。
**风险**：模型仍是 LIP 简化，比 Crocoddyl 表达能力差；学术原型代码工程化程度低。

### 路线 C（远期）：OCS2 NMPC

**理由**：工业级，ETH ANYmal 路线，能扩到 perceptive locomotion / 越障。
**门槛**：ROS1 依赖重、Python wrapper 是社区 fork、robot interface 要自己写。建议路线 A 跑通有 baseline 后再考虑。

**避坑**：
- 不选 TOWR（离线）
- 不选 mit-biped-mpc 原版（quadruped 迁移成本高）
- TSID 单独不够（要配合 Crocoddyl/OCS2 上层规划器才能完整覆盖 footstep 生成）

---

## 5. 与 LIPM 工作的关系

- LIPM passive observer 工具链（`smoke_lipm_active.py`、`LIPM_3D_tinker.py` 等）**保留**作为 baseline 对照
- LIPM 在 IROS2024 仓库（MIT Humanoid）继续训练，与 Tinker 这条线**互不干扰**
- Crocoddyl/MPC 这条线**只在 Tinker 上**做，不污染 IROS2024 仓库

---

## 6. 关键参考

- ETH ANYmal perceptive locomotion (OCS2 + RL) — Miki et al. 2022 Science Robotics
- Boston Dynamics Atlas NMPC + RL 行为生成（公开技术 talk）
- Crocoddyl: Mastalli et al., "Crocoddyl: An Efficient and Versatile Framework for Multi-Contact Optimal Control"
- IS-MPC / DCM-MPC: Scianca, De Simone, Lanari, Oriolo, "MPC for Humanoid Gait Generation"
- 论证背景：本仓库 [`LIPM_not_for_tinker.md`](LIPM_not_for_tinker.md) §10 takeaway

---

## 7. Takeaway

> **LIPM 失败的根因是"约束没进规划器"，不是"模型不够复杂"**。换成 Crocoddyl/OCS2 这类把 reach 和可达性作为优化硬约束的方法，配合 RL 处理鲁棒性与未建模动力学，就能在 Tinker 这种小尺度双足上获得既稳定又有动态性的步态 reference。这也是 BD/ETH 验证过的工业路线 —— 不要重复造轮子去硬训纯 RL，也不要单用 MPC 放弃 RL 的鲁棒性优势。
