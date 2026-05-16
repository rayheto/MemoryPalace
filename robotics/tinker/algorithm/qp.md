# 二次规划（QP, Quadratic Programming）在足式机器人控制中的应用

> **关联阅读**：[vmc.md](vmc.md) §3.3 解释了 Tinker 为什么"链接 qpOASES 但 VMC 路径不调用它"——QP 真正的发挥舞台在未来的 [mpc_locomotion/](mpc_locomotion/)。本文集中讲清 QP 是什么、在足式控制里干什么、以及 Tinker 仓库里现成的 `qpOASES` 库怎么用。

---

## 〇、一句话直觉：从"上帝视角"到"足端执行"

VMC 假装机器人是一个**悬浮在空中的刚体**——可以直接给它施加任意虚拟力 / 力矩（"上帝之手"）。但现实里没有这只手，**唯一能产生外力的地方是脚 ↔ 地面的接触**。

所以 VMC 一旦算出"上帝视角"的期望 `F_d / T_d`，就必须**把它翻译成 N 个足端力**才能交给电机执行。这一步翻译就是本文的主题——而 **QP 是当前公认最优雅的翻译器**：

> 在**物理定律允许的范围内**（不打滑、不吸地、不超扭矩），**尽可能完美地执行控制意图**。

后续章节就是把这句话拆开来讲。

---

## 一、QP 是什么

### 1.1 标准形式

二次规划是带**线性约束**的**二次目标函数**最优化：

```
   min    ½ xᵀ H x  +  gᵀ x
    x
   s.t.   A_eq · x  =  b_eq        ← 等式约束
          A_in · x  ≤  b_in        ← 不等式约束
          lb  ≤  x  ≤  ub          ← 变量上下界
```

- `H` 是 **半正定** 的 Hessian（保证问题凸 → 全局最优）；
- `g` 是线性项；
- `x ∈ ℝⁿ` 是待求决策变量。

凸 QP 一定有**唯一的全局最优解**（或不可行 / 无界），不会陷入局部极小——这正是它在实时控制里被偏爱的原因。

### 1.2 LP / QP / NLP 谱系

| 类型 | 目标函数 | 约束 | 求解开销 | 控制中典型用途 |
|------|----------|------|----------|---------------|
| LP（线性规划） | 线性 | 线性 | 极低 | 资源分配 |
| **QP** | **二次** | **线性** | **低（毫秒级）** | **WBC / GRF / MPC** |
| QCQP | 二次 | 二次 | 中 | 摩擦锥精确建模 |
| SOCP | 线性 | 二阶锥 | 中 | 精确摩擦锥（圆锥而非金字塔） |
| NLP（非线性） | 任意 | 任意 | 高 | 离线轨迹优化 |

足式机器人控制几乎全部跑 QP——动力学是线性可化的（SRBD 单刚体假设），摩擦锥用**线性化金字塔**近似就够了。

---

## 二、足式机器人为什么离不开 QP

### 2.1 核心问题：地面反作用力（GRF）的分配

机器人有 **N 条接地腿**，每条腿足端可施加一个 3 维力 `F_i ∈ ℝ³`；上层 VMC / MPC 给出 **期望合外力 `F_d`** 和 **期望合外力矩 `T_d`**。问题：

> 怎么把这 6 维"机体期望"翻译成 3N 维"足端力指令"？

刚体力 + 力矩平衡列出来是这样：

```
┌  I₃    I₃    …    I₃   ┐ ┌ F₁ ┐   ┌ F_d − m·g ┐
│ ⌊p₁⌋× ⌊p₂⌋× … ⌊p_N⌋×  │ │ ⋮  │ = │    T_d    │
└                        ┘ └ F_N ┘   └           ┘
       A  (6 × 3N)             x         b  (6×1)
```

其中 `⌊p_i⌋×` 是足端相对质心位置 `p_i` 的叉乘斜对称矩阵（`⌊p⌋× · F ≡ p × F`）。

- 四足时 N=4 → A 是 **6 × 12** → 12 个未知 vs 6 个方程 → **欠定**，解不唯一；
- 双足 N=2 → A 是 6 × 6 但常常**秩亏**（两脚共线 → 力矩列线性相关）。

直接 `inv(A)·b` 不可行，必须额外引入**最优化准则**才能挑出唯一解。这就是 QP 入场的地方。

### 2.2 QP 形式的 GRF 分配（MIT Balance Controller）

```
   min    ‖A · F − b‖²_S   +   α · ‖F − F_prev‖²
    F
   s.t.   F_i,z  ≥  F_min                          ← 法向力非负（不能拽地）
          |F_i,x|  ≤  μ · F_i,z                    ← 摩擦锥（防滑，金字塔近似）
          |F_i,y|  ≤  μ · F_i,z
          F_i,z  ≤  F_max                          ← 单腿最大支撑力
```

每一项的物理含义：

| 项 | 作用 |
|----|------|
| `‖A·F − b‖²_S` | **跟踪误差**：让合力/合矩尽量接近期望 |
| `S = diag(w_x, w_y, w_z, w_pit, w_rol, w_yaw)` | **6 个方向独立加权**——你最在乎哪个轴就把权重调大 |
| `α · ‖F − F_prev‖²` | **正则化**：避免连续两帧力指令跳变（足端不撕扯） |
| `F_z ≥ F_min` | 法向力非负，物理上"脚不能吸地" |
| `‖F_xy‖ ≤ μ·F_z` | **摩擦锥**——超过这个比例脚就打滑 |

化成标准 QP（展开二次型）：

```
H  =  2 · Aᵀ · S · A   +   2α · I
g  =  −2 · Aᵀ · S · b  −   2α · F_prev
```

`A`, `b` 每个控制周期重新算（足端位置在变）；`H`, `g` 跟着每帧重算。

#### 摩擦锥的线性化（金字塔近似）

真实的摩擦锥是**圆锥**：

```
√(F_x² + F_y²)  ≤  μ · F_z          (二次约束 → QCQP / SOCP)
```

为了化成 QP（**线性**约束）必须近似，工程上用 **4 边金字塔**（也称 4-面体 / "正方形摩擦锥"）：

```
  −μ·F_z  ≤  F_x  ≤  μ·F_z
  −μ·F_z  ≤  F_y  ≤  μ·F_z
```

代价是**保守了约 30%**（金字塔内切于圆锥）——但换来 QP 可解，实时性巨幅提升。MIT Cheetah / ANYmal / OpenLoong **全部都用 4 边金字塔近似**。要更精确可以用 8 边、16 边金字塔，约束数翻倍但误差减半；嵌入式端 4 边足够。

写成 `A_in · x ≤ b_in` 形式：对每条腿、每个轴向各 2 行不等式，4 条腿 × 2 轴 × 2 边 = **16 行摩擦约束** + 4 行 `F_z ≥ F_min` + 4 行 `F_z ≤ F_max` = 共 24 行不等式约束（变量 12 维）。这是 qpOASES 单帧典型规模。

### 2.3 没 QP 会怎样——参考你给的仿真

| 方法 | 数学上 | 物理上 |
|------|--------|--------|
| `inv(A) · b`（欧拉/最小二乘） | 严格满足 `A·F = b` | **常出负 F_z**（要求脚吸地）→ 现实中脚直接离地 |
| **QP** | 允许 `A·F ≈ b` 有微小残差 | **始终满足 F_z ≥ F_min 和摩擦锥** → 物理上可执行 |

工程上的取舍很清楚：**宁可让上层期望"差一点"，也不能给执行器一个物理上做不到的指令**。QP 自动放弃做不到的目标力矩，把脚力卡在物理边界——这种"主动放弃"是数学解法做不到的。

---

## 三、QP 的求解：在线 Active-Set 策略（qpOASES 用的方法）

### 3.1 几种 QP 求解器流派

| 流派 | 代表 | 特点 |
|------|------|------|
| **Active-Set** | **qpOASES** | 跟踪激活约束集；**热启动**优势大；小规模问题（n<100）极快 |
| Interior-Point | OSQP（部分）、Mosek | 大规模（n>1000）有优势；冷启动也快 |
| ADMM | **OSQP**（主流派）、SCS | 分布式友好；嵌入式实现简单 |
| 投影梯度 | PGS / PGS-PD | 物理引擎里常用，精度较低 |

足式控制 GRF / WBC 维度通常 12~30，**qpOASES 和 OSQP 是两大主流**。Tinker 选了 qpOASES。

### 3.2 qpOASES 的核心思想

> 论文：Ferreau et al., *qpOASES: A parametric active-set algorithm for quadratic programming*, MPC 2014.

- **激活集**：约束分两类——"等号成立的"叫激活（active），"严格不等的"叫非激活（inactive）；
- **迭代过程**：每步只**修改一个**约束的激活状态（加进来或踢出去），并在该子空间内解一个小线性系统；
- **热启动（hot-start）**：上一帧的最优激活集**几乎一定**是下一帧最优激活集的近邻 → 跳过大部分迭代直接从那里开始——**对实时 MPC 极友好**。

### 3.3 复杂度感觉

| 问题规模 | 冷启动 | 热启动（典型 MPC 情形） |
|----------|--------|------------------------|
| n=12 GRF | ~50 μs | ~5 μs |
| n=30 WBC | ~200 μs | ~20 μs |
| n=300 MPC 时域 | ~10 ms | ~1 ms |

（数据来源：qpOASES 论文 + MIT Cheetah 实测；具体平台会差 2~3 倍。）

---

## 四、qpOASES 在 Tinker 仓库里的现状

### 4.1 链接配置

| 位置 | 配置 |
|------|------|
| [CMakeLists.txt:94](CMakeLists.txt#L94) | `add_subdirectory(qpOASES lib/qpOASES EXCLUDE_FROM_ALL)` |
| [CMakeLists.txt:95](CMakeLists.txt#L95) | `include_directories(.../qpOASES/include)` |
| [CMakeLists.txt:96](CMakeLists.txt#L96) | `target_link_libraries(${PROJECT_NAME} qpOASES)` |
| [qpOASES/](../../../../OmniBotHub/Linux/control_task2/qpOASES/) | qpOASES v3.2.0 完整源码（19 个 .cpp 静态库） |

### 4.2 实际调用情况——**没有**

| 主程序源码引用 | 状态 |
|----------------|------|
| [src/main.cpp:29](src/main.cpp#L29) `#include <qpOASES.hpp>` | 死引用 |
| [src/main.cpp:44](src/main.cpp#L44) `using namespace qpOASES;` | 死引用 |
| [src/memory_share.cpp:29](src/memory_share.cpp#L29) `#include <qpOASES.hpp>` | 死引用 |
| [vmc_src/sdk_api.cpp:7](vmc_src/sdk_api.cpp#L7) `#include <qpOASES.hpp>` | 死引用 |

全仓 grep `QProblem | SQProblem | qpOASES::… | hotstart | getPrimalSolution`：**0 处主程序命中**。即头、命名空间、库都备齐了，**但没有一个 `QProblem` 对象被实例化**——典型的"预留架子"状态。

### 4.3 等待激活的目标位置

[mpc_locomotion/](mpc_locomotion/) 目录在 [CMakeLists.txt:73](CMakeLists.txt#L73) 通过 `AUX_SOURCE_DIRECTORY` 接入，但目录当前**为空**。QP 将在此目录的 MPC 实现中真正激活。

---

## 五、qpOASES API 速成（接入 mpc_locomotion 时直接照抄）

### 5.1 单次求解（冷启动）

```cpp
#include <qpOASES.hpp>
using namespace qpOASES;

const int nV = 12;   // 4 条腿 × 3 维力
const int nC = 16;   // 4 × (1 法向 + 2 摩擦锥下界 + 2 摩擦锥上界 − 重复 = 4 各类) ...

real_t H[nV*nV], g[nV];
real_t A[nC*nV], lbA[nC], ubA[nC];
real_t lb[nV],   ub[nV];

// ... 填充 H, g, A, lb, ub, lbA, ubA ...

QProblem qp(nV, nC);
Options options;
options.printLevel = PL_NONE;     // 实时控制必须静默
qp.setOptions(options);

int_t nWSR = 100;                  // 最大工作集变更次数
qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

real_t F_opt[nV];
qp.getPrimalSolution(F_opt);
```

### 5.2 实时控制循环（热启动）

```cpp
// 全局 / 类成员
static QProblem qp(nV, nC);
static bool first_call = true;

// 每个控制周期
update_H_g_A_b(...);              // 重算系数

int_t nWSR = 20;                  // 热启动通常 5~10 次就收敛
if (first_call) {
    qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    first_call = false;
} else {
    qp.hotstart(H, g, A, lb, ub, lbA, ubA, nWSR);
}

if (qp.isSolved()) {
    qp.getPrimalSolution(F_opt);
} else {
    // 失败回退：上一帧解 / 简单分配 / 切到 RECOVER
}
```

### 5.3 关键设置

| 字段 | 推荐值 | 说明 |
|------|--------|------|
| `options.printLevel` | `PL_NONE` | 实时回路禁打印 |
| `options.enableRegularisation` | `BT_TRUE` | 数值稳定（H 弱奇异时） |
| `options.terminationTolerance` | `1e-6 ~ 1e-8` | 实时可放宽到 1e-5 |
| `nWSR` | 冷启动 100；热启动 10~20 | 失败时不重试，直接回退 |

### 5.4 失败回退（**生产代码必须**）

QP 可能 infeasible（摩擦锥太严 + 期望力矩太大）。生产代码必须有失败路径：

| 回退层级 | 策略 |
|----------|------|
| 1. 第一级 | 用上一帧的 `F_opt`（hold） |
| 2. 第二级 | 走教科书 VMC（按相位平均分配） |
| 3. 第三级 | 切到 `RECOVER` 模式（VMC + 关节 PD） |
| 4. 第四级 | 软着陆 `SOFT` 然后停机 |

Tinker 当前等价于"永久走第二级"——见 [force_imp_controller.cpp:42-105](vmc_src/force_imp_controller.cpp#L42-L105)。

### 5.5 嵌入式部署的求解器选型

不同算力平台的可行选择：

| 平台 | 算力（MFLOPS） | 推荐求解器 | 单帧典型耗时（12 维 GRF） |
|------|----------------|------------|-----------------------------|
| **NVIDIA Jetson Xavier / Orin**（Tinker 当前） | 数千 GFLOPS | **qpOASES**（已链接）/ OSQP | < 50 μs |
| **RK3568 / RK3588** | 数十 GFLOPS | **OSQP**（更轻）/ qpOASES | 100~500 μs |
| **STM32H7 / ESP32-S3** | 数百 MFLOPS | **自实现 Active-Set** / OSQP 嵌入式版（osqp_embedded） | 1~5 ms |
| **STM32F4** | < 100 MFLOPS | **解析法 + 简单不等式** / 不建议 QP | — |

工程要点：

- **热启动是嵌入式实时 QP 的命脉**——冷启动比热启动慢 5~10 倍，控制周期不允许这种波动；
- **避免动态内存**：qpOASES 默认 `new/delete`，嵌入式要预分配 + 关闭异常（`USE_LONG_FINTS=0`，`__NO_COPYRIGHT__`）；
- **失败必须有兜底**：见 §5.4 的 4 级回退；
- 控制频率优于求解最优性：**一次 1 kHz 的"次优 QP"** 远胜于**一次 200 Hz 的"精确 QP"**——所以宁可降阶（4 边金字塔、放宽 `terminationTolerance`），不要漏拍。

---

## 六、QP 在足式控制里的三个层级

### 6.1 层级 1：GRF 分配（本文 §2 主线）

- 维度：**12 (四足) / 6 (双足)**
- 频率：**500 Hz~1 kHz**
- 例：MIT Cheetah Balance Controller（无 MPC 时的力分配）

### 6.2 层级 2：WBC（Whole-Body Control，全身控制）

QP 同时优化**关节加速度 + 接触力 + 任务空间残差**：

```
   min    Σ w_i · ‖J_i·q̈ + J̇_i·q̇ − a_des_i‖²
   s.t.   M·q̈ + h = Sᵀτ + Jᵀ·F          ← 全身动力学
          F ∈ 摩擦锥
          τ ∈ 力矩极限
```

- 维度：**~50** （nDoF 关节 + 接触力 + 拉格朗日乘子）
- 频率：**1 kHz**
- 例：ANYmal、Cassie、Mini Cheetah

### 6.3 层级 3：MPC（Model Predictive Control）

QP 优化**未来 N 步的状态轨迹 + 控制序列**：

```
   min   Σ_{k=0..N} (‖x_k − x_ref‖²_Q + ‖u_k‖²_R)
   s.t.  x_{k+1} = A_k·x_k + B_k·u_k    ← SRBD 线性化
         u_k ∈ 摩擦锥
```

- 维度：**几百**（N=10~20 时域 × 状态/控制维度）
- 频率：**30~100 Hz**（高于此 QP 跑不完）
- 例：MIT Cheetah 3 跑跳、ANYmal 楼梯

> Tinker 的 [mpc_locomotion/](mpc_locomotion/) 留空，最有可能先实现**层级 1（GRF QP）**，再升级到**层级 3（SRBD MPC）**——这是 MIT 的演化路径。

---

## 七、调参直觉

调 QP 的 `S = diag(w_x, w_y, w_z, w_pit, w_rol, w_yaw)`：

| 现象 | 调整 |
|------|------|
| 高度不稳，机体上下颠 | ↑ `w_z` |
| 姿态摇晃（pitch/roll 大） | ↑ `w_pit`, `w_rol` |
| 横向滑行 | ↑ `w_x`, `w_y` 但**先**确认摩擦锥 μ 设的够大 |
| 足端力跳变剧烈，能听到响声 | ↑ 正则项 `α` |
| QP 经常 infeasible | ↓ 期望力矩、↑ μ、放宽 `nWSR` |

对照 Tinker 当前的 VMC PD 增益（[vmc.md §3.3](vmc.md)）：

| QP 权重 | Tinker 标量 PD 等价 | YAML 字段 |
|---------|---------------------|-----------|
| `w_z` (大) | `imp_z_kp = 0.03` (X/Y 的 3 倍)、`kp_pos_z = 2500` (X/Y 的 125 倍) | imp_param / stand_param |
| `w_pit / w_rol` | `kp_pit = 40 / kp_rol = 30` | stand_param |
| `w_yaw` | `kp_yaw = 35` | stand_param |
| `α` 正则化 | `imp_param.imp_x/y/z_kd ≈ 0.0001`（阻尼） | imp_param |
| 摩擦锥 μ | `ground_mu = 0.5`（但仅做缩放，**不当约束**） | stand_param |

Tinker 现在做的是**离线调参版的 S**；上 MPC 后就升级为**在线 QP 版的 S**。

---

## 八、参考资料

### 论文（按优先级）

1. **Di Carlo et al., *Dynamic Locomotion in the MIT Cheetah 3 through Convex MPC*, IROS 2018**
   — 凸 MPC + 摩擦锥金字塔，是当前足式 MPC 的事实标准。
2. **Focchi et al., *High-slope terrain locomotion for torque-controlled quadruped robots*, AURO 2017**
   — HyQ 上的 QP-based GRF 分配，与 VMC 对接最自然。
3. **Ferreau et al., *qpOASES: A parametric active-set algorithm*, Mathematical Programming Computation 2014**
   — qpOASES 算法本体。
4. **Stellato et al., *OSQP: An Operator Splitting Solver for QP*, MPC 2020**
   — OSQP（另一主流求解器），与 qpOASES 对比。
5. **Bledt et al., *MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot*, IROS 2018**
   — Balance Controller (QP-GRF) → MPC 的演化路径完整描述。

### 工程文档

- [qpOASES 官方文档](https://github.com/coin-or/qpOASES) — 含 example1~example5 入门代码。
- [OSQP 官方](https://osqp.org/) — 横向参考，嵌入式部署更轻量。
- 本仓库内例子：[qpOASES/examples/example1.cpp](../../../../OmniBotHub/Linux/control_task2/qpOASES/examples/example1.cpp) ——最小可运行 demo。

### 中文综述

- 知乎《两万字梳理: 四足机器人的结构、控制及运动控制》—— 含 MPC 与 QP 介绍。
- CSDN《MIT Cheetah MPC 源码分析》系列 —— 配合论文 [1] 阅读。

---

## 九、与其他模块的关联

- 当前 VMC 不解 QP 的来龙去脉 → [vmc.md §3.3](vmc.md)
- 力 / 阻抗控制律实现 → [module-vmc.md](../src_analysis/omnibothub/module/module-vmc.md)
- 未来 MPC 实现位置 → [module-reserved.md](../src_analysis/omnibothub/module/module-reserved.md)
- 调参对应 YAML 字段 → [module-param.md](../src_analysis/omnibothub/module/module-param.md)

---

## 十、一句话核心结论

> **基于 QP 的 VMC 力分配，本质是在「物理定律（约束）」允许的范围内，尽可能完美地执行「控制意图（VMC 目标）」的过程**。
>
> 它是现代足端力控机器人（MIT Cheetah / Unitree / ANYmal）稳定运行的基石；Tinker 当前用"教科书 VMC + 离线调参 PD"绕开了它，等 [mpc_locomotion/](mpc_locomotion/) 上线后才会真正激活——届时 `qpOASES` 这套预留库就会从死代码变成控制回路里跳得最快的那一拍。
