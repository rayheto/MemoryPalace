# 四足/双足机器人腿部机械结构类型对比

> 配合 Tinker 代码：`vmc_all.param.leg_type` 字段（枚举 `Leg_type`）选择不同的运动学/工作空间分支。

## 一、三类拓扑对照

| 维度 | 串联腿 Serial | 并联腿 Parallel（5R / 五连杆） | 闭环连杆 / 混联 Closed-loop / Hybrid |
|------|---------------|------------------------------|-----------------------------------|
| **拓扑** | 开链：髋 → 大腿 → 小腿（电机沿腿堆叠） | 闭链：两条 R 链共享同一末端（足端） | 大关节并联 + 远端串联 / 四连杆传动 |
| **电机位置** | 沿腿分布（含膝关节） | **全部上置机身**（base） | 大关节上置，小关节走连杆传动 |
| **工作空间** | **大、灵活**（垂直行程不受连杆约束） | **小**（受闭链奇异点限制） | 介于两者之间，且能避开纯并联的奇异 |
| **摆动惯量** | 大（电机分布在腿上） | **极小**（腿是被动连杆） | 小（电机仍上置） |
| **刚度 / 承载** | 低 | **高**（两路力分担，自带预紧） | 高（保留并联载荷优势） |
| **动态响应** | 慢 | **快**（适合冲跳、Trot） | 快 |
| **控制复杂度** | 简单（解析 FK/IK） | 复杂（需解约束方程，可能多解） | 中等 |
| **典型代表** | ANYmal、Spot 早期 | MIT Mini Cheetah、Stanford Doggo、Q8bot | 宇树 A1/Go1、**Tinker**（小腿四连杆传动） |

### 选型决策提要

- **冲跳/快速 Trot**：并联或闭环（低惯量、上置电机、高刚度）；
- **复杂地形/越障**：串联或闭环（工作空间大、垂直行程足）；
- **量产/工程化最常见**：**闭环连杆**——兼顾两者，控制复杂度可接受。

---

## 二、与 Tinker 代码的对应

`vmc_inc/base_struct.h:416-419`：

```cpp
enum Leg_type {
  PARALE = 0,      // 并联
  LOOP,            // 闭环连杆（默认）
  PARALE_LOOP      // 并联 + 闭环混合
};
```

启动初始化 `vmc_src/locomotion_sfm.cpp:95` 硬编码为 `LOOP`，因此 Tinker 系列腿默认按"闭环连杆腿"建模。

### 影响的三处分支（见 `locomotion_sfm.cpp:110-155`）

| 分支 | 闭环（LOOP / PARALE_LOOP） | 并联（PARALE） |
|------|---------------------------|----------------|
| 足端工作空间 | `MIN/MAX_Z = -sind(·)·l1/l2`，X/Y 用 `sind(55/65)·(l1+l2)` | `MIN/MAX_Z = -(cosd/sind 复合)`，X/Y = `±l1·1.15` |
| 测试零位 `sita_test` | `{90, 90}`（小腿水平参考） | `{0, 180}`（伸直参考） |
| 最大速度 `MAX_SPD` | `MAX_X/(0.4)·1.5` | `l1·2/gait_time` |

零位定义差异源自闭环腿习惯把"竖直伸直"对应到 90°（关节链上多了一个传动构件，零位偏置 90°），而并联五连杆习惯用 0°/180° 表示伸直/全屈。

---

## 三、相关阅读（按优先级）

### 入门 / 中文

1. **[四足机器人——机械结构: 并联腿和串联腿 (CSDN)](https://blog.csdn.net/m0_58585940/article/details/121416830)** — 中文最直接的图文对比，含 IK 推导。
2. **[两万字梳理: 四足机器人的结构、控制及运动控制 (知乎)](https://zhuanlan.zhihu.com/p/15527181044)** — 体系化综述，含腿型选择章节。
3. **[机械结构篇之四足机器人腿部结构 (CSDN)](https://blog.csdn.net/qq_60513199/article/details/136921192)** — 把串/并/混三种分别附结构图。
4. **[四足机器人——并联腿运动学逆解 (CSDN)](https://blog.csdn.net/m0_58585940/article/details/124053445)** — 并联腿 IK 推导细节，可对照代码里 `kin_math.cpp::inv_KI` 的几何法。

### 进阶 / 学术

5. **[Optimal 5R parallel leg design for quadruped robot gait cycle (Extrica)](https://www.extrica.com/article/21806)** — 五连杆设计优化，含工作空间数值边界，**对照 Tinker 工作空间公式**最合适。
6. **[Optimal Design of a Five-Bar Leg Mechanism for a Quadruped Robot (Springer)](https://link.springer.com/chapter/10.1007/978-981-15-8458-9_19)** — 用于设计参数（杆长/转角范围）选型。
7. **[新型四足步行机器人串并混联腿的运动学分析 (光学精密工程)](https://www.opticsjournal.net/Articles/OJ7d00710fa8ab4f34/Abstract)** — 中文学术，对照代码里 `PARALE_LOOP` 的混联拓扑最贴近。
8. **[Type synthesis of 3-DOF single-loop parallel leg mechanisms (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S0094114X19323018)** — 单环 3-DOF 并联腿型综合。
9. **[Mechanism Design and Comparison for Quadruped Robot with Parallel-serial Leg](https://qikan.cmes.org/jxgcxb/EN/Y2013/V49/I6/81)** — 直接比较串/并/混三方案。
10. **[四足机器人运动及稳定控制关键技术综述](https://cjournal.hep.com.cn/1671-5497/CN/1190590488199385971)** — 控制+机构综述。
11. **[Five Bar Parallel Mechanism Planar Robot Arm (Bench Robotics)](https://benchrobotics.com/arduino/five-bar-parallel-mechanism-planar-robot-arm/)** — 5R 闭链原理可视化（机械臂示例，类比腿）。

### 推荐路径

- 时间紧只看 1 篇 → 知乎《两万字梳理》（中文、含决策矩阵、有图）；
- 想对照 Tinker 代码里 LOOP/PARALE 的几何公式 → Extrica 5R 论文 + CSDN 并联腿 IK 两篇配合阅读。

---

## 四、术语对照

| 中文 | 英文 | 别名 |
|------|------|------|
| 串联腿 | Serial leg / Open-chain leg | 开链腿 |
| 并联腿 | Parallel leg / 5R linkage | 五连杆腿、闭链腿（狭义） |
| 闭环连杆腿 | Closed-loop linkage leg / Four-bar drive | 平行四连杆腿、远端传动腿 |
| 混联腿 | Hybrid serial-parallel leg | 串并混联 |
| 工作空间 | Workspace / Reachability envelope | 可达域 |
| 奇异位形 | Singular configuration | 死点、奇点 |
