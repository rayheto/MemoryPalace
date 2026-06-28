---
tags:
  - robotics
  - motor_control_sensor_fusion
  - subnode
---

# 电机控制 (FOC) 与传感器融合

针对机械臂或人形机器人，关节模组的底层控制是核心。

---

## FOC 知识体系（序列化文档索引）

FOC 五大核心模块均有独立深度文档，按学习顺序阅读：

| 序号 | 主题 | 文档 | 核心要点 |
|------|------|------|---------|
| 1 | 坐标变换 | [coordinate_transform.md](coordinate_transform.md) | Clarke/Park 推导、等幅值形式、电角度、逆变换链、开环代码 |
| 2 | 三环级联结构 | [three_loop_cascade.md](three_loop_cascade.md) | 电流环/速度环/位置环带宽设计、PI 整定顺序、Anti-windup |
| 3 | SVPWM | [SVPWM.md](SVPWM.md) | 8 基本矢量、扇区判断、七段式占空比计算、过调制 |
| 4 | 电流采样 | [current_sampling.md](current_sampling.md) | 单/双/三电阻方案对比、采样时序、噪声滤波、零偏校正 |
| 5 | 弱磁控制 | [field_weakening.md](field_weakening.md) | 负 Id 注入原理、电压/电流限制圆、MTPA、代码实现 |

> **FOC 原始笔记**（含克拉克/帕克变换推导、PID 详解、开环/闭环代码）：[FOCnote.md](FOCnote.md)

---

## 面试核心考点速查

### FOC (磁场定向控制)

- **三环级联结构:**
  - **电流环（最内层）:** 转矩控制，10–50 kHz；PI 调节 Id/Iq；带宽 1–5 kHz
  - **速度环（中间层）:** 速度控制，1–10 kHz；输出 Iq_ref；需 LPF + Anti-windup
  - **位置环（最外层）:** 位置控制，100–1000 Hz；通常纯 P 控制；可加前馈
  - **带宽比原则：** 相邻环路带宽比 ≥ 5:1，从内到外整定
- **坐标变换:**
  - Clarke 变换: $(i_a, i_b, i_c)$ → $(I_\alpha, I_\beta)$，等幅值系数 2/3
  - Park 变换: $(I_\alpha, I_\beta)$ → $(I_d, I_q)$，旋转矩阵角为电角度 $\theta_e = p\cdot\theta_m$
  - 逆 Park: $(V_d, V_q)$ → $(V_\alpha, V_\beta)$ → SVPWM
- **SVPWM:** 6 个非零矢量 + 2 个零矢量，正六边形扇区；七段对称调制；母线利用率 57.7%
- **电流采样方案对比:**
  - 单电阻：成本最低，不支持过调制，时序复杂
  - 双电阻：兼顾成本和精度，大多数场景首选
  - 三电阻：精度最高，支持过调制，高性能伺服首选
- **弱磁控制:** 高速区注入负 Id 电流，削弱气隙磁通降低反电动势；基于电压限幅 PI 或前馈公式；代价是转矩能力下降（恒功率特性）

### 传感器融合

- **AHRS (航向姿态参考系统):**
  - Mahony 算法（计算轻量级，适合 MCU）
  - Madgwick 算法（梯度下降优化，适合 MEMS IMU）
  - EKF（扩展卡尔曼滤波）—— 精度最高但计算量大
- **IMU 数据处理:**
  - 零偏（Bias）估计与温度补偿
  - Allan 方差分析噪声特性（角度随机游走 / 零偏不稳定性）
  - 低通滤波 / 陷波滤波去除电机振动噪声
- **多传感器融合架构:**
  - 视觉惯性里程计（VIO）: 相机 + IMU 紧耦合
  - 编码器 + IMU 互补滤波: 差分运动学 + 惯性数据

### 如何在 MCU 上部署算法

- 定点化 vs 浮点化（Cortex-M4/M7 带 FPU 可直用浮点）
- CMSIS-DSP 库加速矩阵运算和 FFT
- 实时性分拆: 高频控制（FOC 电流环）放 MCU，低频规划（轨迹生成）放 Linux

---

## 推荐知识库

- **SimpleFOC:** [github.com/simplefoc/Arduino-FOC](https://github.com/simplefoc/Arduino-FOC) —— 开源 FOC 库，代码架构清晰
- **ODrive:** [github.com/odriverobotics/ODrive](https://github.com/odriverobotics/ODrive) —— 高性能伺服驱动，编码器校准参考
- **Madgwick AHRS:** [github.com/xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) —— 传感器融合参考实现

