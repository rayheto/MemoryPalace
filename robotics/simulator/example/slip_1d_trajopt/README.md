# 1D SLIP 三段式轨迹优化（C++ port of MATLAB `optimTraj`）

把 MATLAB 那段 `optimTraj`（自由落体 → 支撑相直接配点 → 飞行段）翻成 C++，用 **NLopt SLSQP** 求解，把结果作为参考曲线叠加到 [../slip_1d](../slip_1d) 的 dashboard 上和实时仿真对比。

> 前置阅读：[../slip_1d/README.md](../slip_1d/README.md) 讲清楚了 SLIP 模型和坐标。

---

## 三段式公式

```
┌──── 阶段 0 ────┬──── 阶段 1 ────┬──── 阶段 2 ────┐
│  自由落体      │  支撑相 NLP    │  飞行段        │
│  (X0, 0)       │  弹簧+控制     │  (l, dx_LO)    │
│   → (l, dx_TD) │   → (l, dx_LO) │   → apex       │
│  解析积分      │  直接配点 SLSQP│  解析积分      │
└────────────────┴────────────────┴────────────────┘
```

**只有阶段 1 是优化问题**，0 和 2 都有闭式解。

### 阶段 1 — 决策变量
`y = [x_0..x_N, dx_0..dx_N, u_0..u_{N-1}, T] ∈ R^{3N+3}`（默认 N=40 → 123 维）

### 阶段 1 — 目标
**跟踪目标 apex + 最小化控制力**：

`J = h · Σ u_k²`，其中 `h = T/N`。

> 注：纯效力惩罚，没有 effort_w；目标 apex 通过下面的等式约束硬性指定。

### 阶段 1 — 等式约束（共 2N+4 个）
- 边界：`x_0 = l`、`dx_0 = dx_TD`、`x_N = l`、**`dx_N = sqrt(2g·(x_apex_target − l))`**
- 梯形配点缺陷（每 k=0..N-1 两个）：
  - `x_{k+1} - x_k = (h/2)(dx_k + dx_{k+1})`
  - `dx_{k+1} - dx_k = (h/2)(a_k + a_{k+1})`，`a = -g - (kp/m)(x-l) - kd·dx + u_k`

### 阶段 1 — 不等式（用边界框处理）
`x_k ∈ [l_min, l]`、`u_k ∈ [-u_max, u_max]`、`T ∈ [T_lo, T_hi]`

支撑相动力学是 **线性** 的（弹簧 + 阻尼 + 控制），所以雅可比 90% 是常数，全部手写解析；只有 `∂/∂T` 那几列依赖当前 `h, z`，每次求值现算。

---

## API（[trajopt.hpp](trajopt.hpp)）

```cpp
slip_trajopt::Options opt;          // 默认 N=40, u_max=50, x_apex_target=1.2
opt.x_apex_target = 1.2;            // 想跳多高
opt.verbose = true;
auto R = slip_trajopt::solve(P, /*X0=*/2.5, opt);
if (!R.ok) { /* nlopt_code 看错码 */ }

// 关心的字段：
R.T          // 支撑时长
R.dx_LO      // 离地速度
R.x_apex     // 最高点高度
R.x, R.dx, R.u   // 优化器原始输出（节点上）
R.traj_t,  R.traj_x,  R.traj_dx   // 三段拼接的状态序列（绘图用 float）
R.traj_tu, R.traj_u               // 三段拼接的控制序列
```

---

## UI：实时仿真叠参考轨迹

[slip_trajopt_ui.cpp](slip_trajopt_ui.cpp) 用 [`simulation_dashboard`](../simulation_dashboard) 外壳：

1. 启动时跑一次 `solve()`，把 `(x*, dx*, u*)` 通过 `dash.add_static_curve(..., group_id)` 注册为 **参考曲线**。
2. 每条「live」滚动曲线和它对应的「reference」静态曲线共用同一个 `group_id` → 在同一个子图里叠加显示。
3. 物理回调里用 `u = u*(t)`（线性插值）+ RK4 推进 → live 应该几乎贴住 reference。
4. Controls 面板提供 `kp / kd / l / m / X0 / N / u_max / effort_w` 滑块，按 **Re-optimize** 重新求解、刷新参考曲线。
5. 默认开启 **Loop trajectory**：仿真过完一遍（自由落体 → 弹跳 → 顶点）后自动 `request_reset()`，循环播放。

需要的 dashboard 扩展（已合入 `simulation_dashboard`）：
- `add_curve(label, color, group=-1)`：同 group 的曲线叠在同一 subplot
- `add_static_curve(label, color, ts, vs, group)`：一次性给出的离线曲线
- `update_static_curve(id, ts, vs)`：刷新数据
- `request_reset()`：外部触发复位

---

## 编译与运行

依赖：
- 已有的 MuJoCo / GLFW / OpenGL（父 CMake 已配好）
- **NLopt v2.7.1**（顶层 CMake 通过 `FetchContent` 拉取，编译 ~30 s）

```bash
cd /home/hlei/MemoryPalace/robotics/simulator/example
cmake -S . -B build
cmake --build build -j

cd build/slip_1d_trajopt
DISPLAY=:110 ./slip_trajopt_ui
```

---

## 调参套路

| 想看什么 | 怎么调 |
|---|---|
| 跳得更低（耗能） | `target apex` 调小（如 X0=2.5 → target=1.0）；u 会一直取负，必要时贴住 `-u_max` |
| 跳得更高（注能） | `target apex > X0`，u 取正，贴住 `+u_max` |
| 控制饱和的极限工况 | 把 `u_max` 调小，target 维持，让问题接近不可行（看 u 整段贴在 ±u_max） |
| 弹簧更软、支撑相更长 | `kp` 调小，`T*` 会变大 |
| 看不可行 → 求解失败 | target 设得太离谱（`(X0 → target)` 能量差超过 `u_max·T_hi · max\|x-l\|`），nlopt 会汇报无解 |

---

## 已知坑

- **SLSQP 对初值敏感**：暖启用了 sin-dip + 余弦速度，工作良好；但参数变得过于极端（如 `kp < 50` 或 `m > 5`）时可能不收敛，注意看终端 `nlopt_code`。
- **N 太大**：内存正比 `O(N²)`（雅可比稠密）。N=80 仍然秒级；N=200 就要等了。
- **阶段 0 要求 `X0 > l`**：否则 `solve()` 返回 `ok=false`。Controls 面板里 X0 滑块下界已设为 `l + 0.01`。
- **kd 单位**：和 MATLAB / `slip_1d_model.cpp` 保持一致——`kd` 直接乘 `dx`（量纲 s⁻¹），**不是** Ns/m。改这里得连改三处。

---

## 参考实现

- [trajopt.cpp](trajopt.cpp) — NLP 公式 + 解析雅可比 + 三段拼接
- [slip_trajopt_ui.cpp](slip_trajopt_ui.cpp) — Dashboard 业务层
- [../slip_1d/slip_1d_model.cpp](../slip_1d/slip_1d_model.cpp) — 复用的动力学和运动学
- [../simulation_dashboard/dashboard.hpp](../simulation_dashboard/dashboard.hpp) — UI 外壳 API
