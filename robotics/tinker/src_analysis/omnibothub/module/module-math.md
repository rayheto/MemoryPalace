# control_task_tinker — `math_src/` 数学计算层（5b）

## 概览

`math_src/` 是机器人控制的**数学基础层**：通用数学函数、坐标变换、运动学（FK/IK）、雅可比、滤波器/观测器（卡尔曼 / TD / ESO）、Bezier 轨迹、阻抗力学。整层 ≈ 3516 行 C++/C，由 `vmc_src/`、`gait_src/` 大量调用，自身**不持有控制律**，只提供"算子"。

| 文件 | 行数 | 类别 |
|------|------|------|
| `common_math.cpp` | 1001 | 通用数学（三角、矩阵、四元数、快速算法） |
| `kin_math.cpp` | 803 | 腿部 FK/IK/Jacobian/力-扭矩映射 |
| `fliter_math.cpp` | 478 | 数字滤波器、卡尔曼、跟踪微分器 |
| `eso.cpp` | 363 | 扩展状态观测器（ADRC） |
| `RT_math.cpp` | 318 | 坐标系变换（Simulink Coder 生成代码） |
| `bezier_math.cpp` | 99 | 足端摆动 Bezier 轨迹 |
| `imp_math.cpp` | 91 | 阻抗前馈 / 力前馈计算 |
| `eso_RL.cpp` | 35 | RL 专用关节级 ESO |
| `rtGetInf.cpp` / `rtGetNaN.cpp` / `rt_nonfinite.cpp` | 141 / 98 / 89 | Simulink RT 助手（IEEE 754 NaN/Inf 处理） |

对应头文件位于 [math_inc/](math_inc/)：`cppTypes.h`、`common_types.h`、`eso.h`、`eso_RL.h`、`FootSwingTrajectory.h`、`Interpolation.h`、`rt_defines.h`、`rtwtypes.h` 以及 Simulink 内置头。

---

## 一、`common_math.cpp` — 通用数学库

提供基础数学函数，全部在编译时 O3 优化，采用 32-bit float 优先。

### 关键能力

| 类别 | 函数（举例） | 说明 |
|------|-------------|------|
| 三角函数 | `sin/cos/tan`（度与弧度版本） | 标准接口 |
| **快速 atan2** | 基于 **256 项查表 + 线性插值** | 速度优先，精度足够实时控制 |
| **快速 sqrt** | **Quake 算法**（魔数 0x5f3759df） | 避免标准库浮点除法 |
| 矩阵求逆 | 2×2、3×3 解析法 | 含行列式 |
| 四元数 ↔ DCM | quat↔Rotation Matrix | 用于 IMU 姿态融合 |
| 矩阵代数 | 乘法、转置、点积 | 实时控制中频繁使用 |

被 `RT_math.cpp`、`kin_math.cpp`、`eso.cpp`、`fliter_math.cpp` 普遍调用。

参考 [math_src/common_math.cpp](math_src/common_math.cpp)。

---

## 二、`kin_math.cpp` — 运动学（最关键的几何层）

实现机器人腿部（3 DOF MIT 风格腿，髋外摆 + 髋俯仰 + 膝）的完整**正/逆运动学 + Jacobian**。

### 2.1 正运动学（FK）

**入口**：`estimate_end_state_new()`，约 [kin_math.cpp:219](math_src/kin_math.cpp#L219)。

**输入**：3 个关节角 `sita1`（髋俯仰）、`sita2`（膝）、`sita3`（髋外摆）。
**输出**：
- `epos`（足端在髋坐标系下的位置）；
- `epos_b`（足端在 body 坐标系下的位置）；
- `epos_n`（足端在 world/n 坐标系下的位置）。

**几何**：
```
H = l1·cos(q1) + l2·cos(180° − q2 − q1)         # 腿长在矢量平面投影
xyz_hip = [H·sin(q1), 0, H·cos(q1)]              # 髋系下足端
xyz_body = R(sita3) · xyz_hip + hip_offset       # 加上侧摆
xyz_world = Rb_n · xyz_body                       # 转到世界系
```

### 2.2 逆运动学（IK）

**入口**：`inv_KI()`，约 [kin_math.cpp:189](math_src/kin_math.cpp#L189)。

**方法**：**几何闭式解**（无迭代）。利用 atan2 + acos 求出 3 个关节角。同时支持 2-DOF 退化（仅髋俯仰+膝）和 3-DOF 完整解（含外摆）。

### 2.3 Jacobian

**入口**：`cal_jacobi_new()`，约 [kin_math.cpp:420](math_src/kin_math.cpp#L420)。

3 个变体（kin1 / kin12 / kin2）对应不同的质量分布模型，主要用于：
- 力 → 关节扭矩映射：**τ = Jᵀ · F**（[kin_math.cpp:41-187](math_src/kin_math.cpp#L41-L187)）；
- 加速度前馈：求 J 的时间导数。

### 2.4 关节编号约定

3-DOF/腿，`sita1`（髋俯仰，索引 0）→ `sita2`（膝，索引 1）→ `sita3`（髋外摆/abduction，索引 2）。

---

## 三、`fliter_math.cpp` — 数字滤波器与跟踪微分器

| 滤波器 | 类型 | 用途 |
|--------|------|------|
| `DigitalLPF` | 1 阶 IIR 低通 | IMU/编码器噪声滤除，**9 档可选截止频率**（0.5 Hz ~ 100 Hz） |
| `kalman2_filter` | 2 维卡尔曼 | 从位置测量推估速度（[fliter_math.cpp:193](math_src/fliter_math.cpp#L193)） |
| `Moving_Average` | FIR 滑动均值 | 信号平滑 |
| 中值滤波 | 排序后取中 | 抗野值 |
| **TD4_track4** | 4 阶非线性跟踪微分器 | 平滑信号 + 求微分（[fliter_math.cpp:255](math_src/fliter_math.cpp#L255)） |
| `TD3` | 3 阶 TD | 同上更轻量 |
| `ESO_AngularRate` | 2 阶 ESO | 陀螺角速率估计（轻量版，区别于 `eso.cpp` 的多种 ESO） |

跟踪微分器（TD）解决"求导放大噪声"的经典问题，是 ADRC 框架的预处理环节。

---

## 四、`eso.cpp` — 扩展状态观测器（ADRC 核心）

实现 ADRC（Active Disturbance Rejection Control）框架的核心：把"未建模动力学 + 外扰"作为一个**额外状态**，由观测器实时估计后做前馈补偿。

### 4.1 2 阶 ESO

入口约 [eso.cpp:118](math_src/eso.cpp#L118)：

```
ż[0] = z[1] − β0 · e + b0 · u
ż[1] = − β1 · fal(e, 0.5, h0)
e    = z[0] − y         (实测 vs 估计)
disturbance = z[1] / b0
```

`fal(·)` 为非线性反馈（分段幂函数），让小误差用线性段、大误差用饱和段以加快收敛。

### 4.2 3 阶 ESO

扩展一个加速度状态，跟踪二阶系统更精准。`auto_b0` 自适应增益用于在不同负载下保持收敛。

### 4.3 多实例

代码中按观测对象分组实例化（全局数组）：
- `eso_pos[3]` — 三轴位置外扰；
- `eso_att_outter_c[4]` — 姿态外环（4 路）；
- `leg_td[4][3]` — 4 条腿 × 3 关节的跟踪微分器。

参考 `math_inc/eso.h`。

---

## 五、`eso_RL.cpp` — RL 专用关节级 ESO

仅 35 行，专门为 RL 步态的**关节扭矩补偿**设计。

参数（[eso_RL.cpp:16-29](math_src/eso_RL.cpp#L16-L29)）：

```cpp
wn       = 50    // 带宽
alpha_v  = 20    // 阻尼
ctrl_gain= 100   // 控制增益

l1 = 3·wn        // 三阶 ESO 增益（位置）
l2 = 3·wn²       // 速度
l3 = wn³         // 扰动

pos_error      = measured − estimated
dof_pos_obsrd  = vel + l1·pos_error
dof_vel_obsrd  = −α_v·vel + gain·τ + dist + l2·pos_error
dof_dist_obsrd = l3·pos_error
```

全局存储：`joint_eso[4][3]`（4 腿 × 3 关节）。参考 `math_inc/eso_RL.h`。

---

## 六、`RT_math.cpp` — 坐标系变换（Simulink Coder 生成）

**注意**：本文件由 MATLAB/Simulink Coder 自动生成，命名风格与代码风格与手写代码不同（`rt_` 前缀、`rtwtypes.h` 类型）。

### 6.1 支持的坐标系

| 缩写 | 含义 |
|------|------|
| `n` | World / inertial（地理北、上） |
| `b` | Body（机身固连） |
| `leg` | Leg local（髋关节局部） |
| `g` | Gravity-aligned（地心方向对齐） |
| `bw` | Body with roll compensation（带横滚补偿的中间系） |

### 6.2 关键变换函数

| 函数 | 方向 |
|------|------|
| `converV_n_to_bw()` | World → Body（含 roll 补偿） |
| `converV_b_to_nw()` | Body → World |
| `converV_b_to_legw()` | Body → 髋系 |

变换矩阵存于全局 `robotwb`：`Rn_b[3][3]`、`Rb_n[3][3]` 等。也包含 3×3、2×2 的矩阵-向量乘法、向量叉积（→ 反对称矩阵）等基础算子。

VMC 控制律严重依赖这些函数实现"力在 world 系规划，扭矩在 joint 系下发"的换坐标流程。

---

## 七、`bezier_math.cpp` — 足端 Bezier 摆动轨迹

99 行，实现腿部摆动相的足端笛卡尔轨迹规划：

- **X/Y 平面**：起点 → 终点的**单段三次 Bezier**；
- **Z 轴**：**两段 Bezier**（上升 + 下降）以实现指定的离地高度（clearance）；
- 输出 position / velocity / acceleration（由 Bezier 解析微分得到）。

被 `vmc_src/locomotion_sfm.cpp`、`gait_src/` 在摆动相足端跟踪时调用。配套头 `math_inc/FootSwingTrajectory.h`、`math_inc/Interpolation.h`。

---

## 八、`imp_math.cpp` — 阻抗力前馈

91 行，提供：
- 站立相竖向力前馈（基于 Bezier 软启动）；
- 俯仰力矩前馈；
- 地面接触系数估计；
- 摆动相高度补偿。

最终输出叠加到 VMC 的 `force_imp_controller`（在 `vmc_src/`）中。

---

## 九、Simulink RT 助手（rtGet*.cpp、rt_nonfinite.cpp）

负责 IEEE 754 NaN / +∞ / −∞ 的跨平台获取（不同字节序），仅供 `RT_math.cpp` 等 Simulink Coder 自动生成代码使用。手写代码一般直接用 `std::nan/std::isinf`。

---

## 十、调用与依赖关系

```
              ┌──────────────────────────────┐
              │  Eigen3 / std math.h         │
              └────────────┬─────────────────┘
                           │
        ┌──────────────────┴───────────────────┐
        │                                       │
        ▼                                       ▼
 ┌──────────────┐                       ┌──────────────┐
 │ common_math  │←─── 基础算子─────┐    │ rtGet*/rt_*  │
 └──────┬───────┘                  │    └──────┬───────┘
        │                          │           │
        ▼                          │           ▼
 ┌──────────────┐    ┌─────────────┴──┐  ┌──────────────┐
 │  kin_math    │    │ fliter_math    │  │  RT_math     │
 │  FK/IK/Jaco  │    │ LPF/Kalman/TD  │  │  坐标变换    │
 └──────┬───────┘    └────────┬───────┘  └──────┬───────┘
        │                     │                  │
        │                     ▼                  │
        │             ┌──────────────┐           │
        │             │  eso / eso_RL│           │
        │             │  ADRC 观测器 │           │
        │             └──────┬───────┘           │
        │                    │                   │
        └────────────────────┴───────────────────┘
                             │
                             ▼
              ┌─────────────────────────────┐
              │ vmc_src/, gait_src/ 调用方   │
              │ (locomotion_sfm, rl, stand,  │
              │  hardware_interface, ...)    │
              └─────────────────────────────┘

 ┌──────────────┐   ┌──────────────┐
 │ bezier_math  │←──│ imp_math     │   被 VMC 在足端轨迹与
 └──────────────┘   └──────────────┘   阻抗前馈中调用
```

### 主要消费者

| 消费方 | 使用什么 |
|--------|---------|
| `vmc_src/locomotion_sfm.cpp` | RT_math（坐标变换）、bezier（摆动）、eso（姿态外扰）、common_math（矩阵） |
| `vmc_src/hardware_interface.cpp` | fliter（IMU 平滑）、common_math（四元数↔DCM） |
| `vmc_src/force_imp_controller.cpp` | kin_math（Jacobian）、imp_math（前馈） |
| `gait_src/rl.cpp` | eso_RL（关节扰动观测）、common_math |
| `gait_src/self_right.cpp` | common_math（角度规划） |

---

## 十一、性能特征

- **编译**：O3 + C++11；
- **采样**：主控 500 Hz~2 kHz（取决于具体环节）；
- **浮点精度**：实时路径用 `float`；MATLAB / Simulink 接口段用 `double`；
- **快速路径**：atan2 查表、Quake sqrt、多项式 sin/cos 取代库函数。
