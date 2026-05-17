# 1D SLIP 跳跃器：MATLAB → C++ + MuJoCo 可视化 + ImGui/ImPlot 调参面板

把 [cartPoleDynamics.m](cartPoleDynamics.m)（1D 垂直跳跃器 / SLIP 模型）翻译成 C++ + Eigen，由我们自己做 RK4 积分，**MuJoCo 仅作可视化**（`mj_forward` 只跑前向运动学，不参与物理求解），上层套 **ImGui + ImPlot**：左 MuJoCo 视口、右 滑块面板（kp/kd/m/l/u/初值）、底部 三条示波器（x、ẋ、u 滚动 5 s）。

> 命名说明：MATLAB 函数虽叫 `cartPoleDynamics`，内部其实是 1D 垂直跳跃器——只有 `z(0)=高度 / z(1)=速度` 两个状态。

---

## 一、文件清单

| 文件 | 角色 |
|------|------|
| [cartPoleDynamics.m](cartPoleDynamics.m) | 原版 MATLAB 动力学 + 运动学（分相位） |
| [slip_1d_model.hpp](slip_1d_model.hpp) / [.cpp](slip_1d_model.cpp) | C++ 翻译：`cartPoleDynamics` + `cartPoleKinematics` |
| [model.xml](model.xml) | MJCF：球 body + 脚 body，两个 slide 关节，tendon 当弹簧 |
| [hopper_sim_ui.cpp](hopper_sim_ui.cpp) | 业务侧：RK4 推进 + 写 qpos + 注册滑块/曲线（UI 外壳来自 `simulation_dashboard`） |
| [CMakeLists.txt](CMakeLists.txt) | 子项目构建：静态库 `slip_1d_model` + 可执行 `hopper_sim_ui` |
| [../simulation_dashboard/](../simulation_dashboard/) | 通用面板外壳（MuJoCo 视口 + Controls + 可拖动 Scope） |
| [../CMakeLists.txt](../CMakeLists.txt) | 顶层：MuJoCo / Eigen / ImGui+ImPlot（FetchContent）/ `simulation_dashboard` |

---

## 二、MATLAB → C++ 对照

物理（`cartPoleDynamics`）：三分支 if/else 一一直译

| 相位 | 条件 | 动力学 |
|------|------|--------|
| FLIGHT | `x > l` | `ẍ = -g` |
| STANCE | `l_min ≤ x ≤ l` | `ẍ = -g - kp/m·(x - l) - kd·ẋ + u` |
| BOTTOM | `x < l_min` | 速度强置 1e-5，`ẍ = -kp/m·(x - l) - kd·ẋ + u` |

运动学（`cartPoleKinematics`）：决定脚的 z 高度

| 相位 | 脚的位置 |
|------|----------|
| FLIGHT | `p_leg = x - l`（腿伸到自然长度，脚悬空） |
| STANCE | `p_leg = x - l + l_min`，过深时 clamp 到 `0` |

---

## 三、物理 vs 可视化的分工

```
┌──────────────────────┐    z_state = [x, ẋ]      ┌─────────────────┐
│  RK4 自写积分        │  ─────────────────────►  │  MuJoCo         │
│  cartPoleDynamics    │                          │  mj_forward     │
│  cartPoleKinematics  │  ◄── 仅运动学，不物理    │  渲染 + 鼠标交互│
└──────────────────────┘                          └─────────────────┘
```

- `d->qpos[0] = x - z_body_init`（hopper slide 关节）
- `d->qpos[1] = cartPoleKinematics(x, P)`（foot slide 关节）
- tendon `spring` 自动跟随两个 site 之间的距离 → 视觉弹簧

---

## 四、参数与初值

`hopper_sim_ui.cpp` 顶部默认值（运行时都可用滑块改）：

```cpp
static double X0  = 2.0;   // 初始高度 [m]
static double DX0 = 0.0;   // 初始速度 [m/s]

static Params P{
    0.5,   // l     自然长度
    0.05,  // l_min 最小压缩
    800.0, // kp    弹簧刚度
    5.0,   // kd    阻尼
    1.0    // m     质量
};
static double u_manual = 0.0;     // 控制输入 [m/s^2]
static double u_limit  = 50.0;    // |u| 饱和上限
```

理论平衡点：`x_eq = l - g·m/kp = 0.5 - 9.81/800 ≈ 0.488 m`（弹簧压缩 12 mm 支撑重力）。Controls 面板右下也会实时显示 `x_eq`，方便边改 `kp` 边看平衡点变化。

---

## 五、UI 布局

```
┌────────────────────────────────────┬──────────────┐
│                                    │  Controls    │
│        MuJoCo 视口                 │  (sliders)   │
│   左键旋转 / 右键平移 / 滚轮缩放    │  kp kd l m   │
│                                    │  u  X0 DX0   │
│                                    │  Pause/Reset │
├────────────────────────────────────┤              │
│  Scope (last 5 s)                  │              │
│   ── x(t)                          │              │
│   ── dx(t)                         │              │
│   ── u(t)                          │              │
└────────────────────────────────────┴──────────────┘
```

- 鼠标悬停在 ImGui 面板上时不会触发相机旋转（`io.WantCaptureMouse` 拦截）
- 曲线缓冲 6000 样本（@1 kHz ≈ 6 s），X 轴始终滚动到最新 5 s

---

## 六、编译与运行

首跑 `cmake` 会通过 `FetchContent` 从 GitHub 拉取 ImGui v1.91.5 + ImPlot v0.16（约 30 MB，缓存到 `build/_deps/`）。

```bash
cd /home/hlei/MemoryPalace/robotics/simulator/example
cmake -S . -B build
cmake --build build -j

# model.xml 在 build/slip_1d/ 下，必须从那里启动
cd build/slip_1d
DISPLAY=:110 ./hopper_sim_ui      # 无显示器时用虚拟 X
```

### 交互

| 操作 | 功能 |
|------|------|
| 鼠标悬停在 3D 视口 → 左键拖拽 | 旋转视角 |
| 同上 → 右键拖拽 | 平移视角 |
| 同上 → 滚轮 | 缩放 |
| Space | 暂停 / 继续 |
| Backspace | 重置状态（用当前面板 `X0/DX0`） |
| ESC | 退出 |
| Controls 面板上的滑块 | 实时改 `kp / kd / l / l_min / m / u / u_limit / X0 / DX0` |

调参套路：把球放高一点（`X0=2.5`）→ Reset → 看 Scope 里 x 触地后回弹幅度；改 `kp` → 弹得更高；改 `kd` → 几次反弹后稳定到 `x_eq`。

---

## 七、扩展

- **接闭环控制**：把 `apply_limit(u_manual)` 换成你自己的策略函数（PD 跟踪、轨迹规划输出、RL 动作等）
- **改参数**：运行时直接拖滑块；要改默认值就编辑 [hopper_sim_ui.cpp:34-43](hopper_sim_ui.cpp)
- **加新曲线**：在 `PlotData` 加字段，在物理循环 `hist.push` 处加上记录，主循环里多调一次 `draw_series`
- **新例子**：在 [../CMakeLists.txt](../CMakeLists.txt) 加 `add_subdirectory(<name>)`，父级已经准备好 `mujoco::*` / `Eigen3::Eigen` / `imgui_backend` 目标
