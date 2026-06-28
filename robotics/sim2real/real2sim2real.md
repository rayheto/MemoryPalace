---
tags:
  - robotics
  - sim2real
  - real2sim2real
  - digital-twin
---

# Real2Sim2Real

Real2Sim2Real 将 Sim2Real 从开环流程变成闭环流程。它不是只从仿真训练然后部署，而是先用真实世界生成数字资产或真实反馈，再回过头优化仿真器，最后重新训练或微调策略并迁移回现实。

![[assets/rsr-three-stage-flow.svg|697]]

## 三阶段流程

1. Real2Sim：从真实场景采集数据，进行三维重建、参数识别和接触模型估计。
2. Simulation Training：在更新后的仿真器或数字孪生中大规模训练策略。
3. Sim2Real：将优化后的策略部署到真实系统。

典型做法包括使用 NeRF、VR 扫描或三维重建构建高保真数字孪生环境，然后在虚拟世界中训练策略，最后迁移到真实场景。

## 真实场景生成的数字资产

真实场景生成的数字资产可以用于：

- 构建高保真数字孪生仿真环境。
- 反推仿真器中的建模结构，例如动力学参数和接触模型。
- 影响策略学习过程，例如改变奖励、损失或探索策略。
- 构建可重复迭代的建模框架。

## 仿真器参数反推

RSR 可以把物理参数当作可优化变量，用真实数据反向修正仿真器。

![[assets/rsr-modeling-pipeline.svg|691]]

一个典型管线包括：

- Simulation Dataset Generation：给定物理参数、动作和上一时刻状态，通过仿真器生成仿真数据。
- Surrogate Modelling：用神经网络拟合仿真器输入到输出的映射。
- Gradient-based Refinement：将物理参数作为可优化变量，通过真实数据误差进行梯度更新。

符号含义：

- $f, p, d$：friction, stiffness, damping coefficients，即摩擦、刚度和阻尼系数。
- $P_t$：仿真器数据。
- $S_t$：真机数据。

## 用真实表现优化策略学习

![[assets/rsr-policy-loop.svg|665]]

广义 RSR 不只优化仿真器参数，也可以让真实系统表现影响策略训练过程。比如在策略训练循环中设计自适应损失函数，动态平衡任务完成和数据探索。

可表示为：

$$
a_{k,t} = \arg\min \mathcal{L}(a_{k,t}) = \mathcal{L}_{task}(a_t) + \mathcal{L}_{sr}(a_{k,t})
$$

其中 $\mathcal{L}_{task}$ 关注任务完成，$\mathcal{L}_{sr}$ 关注真实到仿真的闭环校准或探索需求。

## 适用场景

- 真实数据昂贵但可以少量采集。
- 任务需要高保真场景几何或接触模型。
- 真实场景长期复用，值得构建数字孪生。
- 希望把真实系统反馈纳入仿真器和策略学习闭环。
