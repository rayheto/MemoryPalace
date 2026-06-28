---
tags:
  - robotics
  - sim2real
  - subnode
---

# Sim2Real

Sim2Real 关注如何让仿真中训练或验证得到的模型、策略和控制器，在真实机器人系统中保持可用、稳定和可泛化。

## 笔记索引

| 模块 | 内容 |
| --- | --- |
| [基础问题与差距来源](fundamentals.md) | Sim2Real gap、感知差异、物理建模误差、动力学参数不一致 |
| [系统辨识](system_identification.md) | 通过真实数据反推仿真参数，让仿真器更接近真实系统 |
| [域随机化与域自适应](domain_randomization_and_adaptation.md) | 扩展仿真分布、对齐源域和目标域特征分布 |
| [策略迁移与真实反馈](policy_transfer_and_feedback.md) | 直接部署、人类在线纠错、残差策略、开环与闭环 Sim2Real |
| [Real2Sim2Real](real2sim2real.md) | 用真实场景和真实表现反向优化仿真器，再迁移回现实 |

## 学习顺序

1. 先读 [基础问题与差距来源](fundamentals.md)，明确 Sim2Real 要解决什么。
2. 再读 [系统辨识](system_identification.md) 和 [域随机化与域自适应](domain_randomization_and_adaptation.md)，理解两类主流缩小差距的方法。
3. 然后读 [策略迁移与真实反馈](policy_transfer_and_feedback.md)，关注真实系统表现如何进入训练闭环。
4. 最后读 [Real2Sim2Real](real2sim2real.md)，理解从真实到仿真再到真实的闭环范式。
