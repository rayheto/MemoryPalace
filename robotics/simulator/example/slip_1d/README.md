# 1D SLIP 跳跃器：MATLAB → C++ + MuJoCo 可视化

把 [cartPoleDynamics.m](cartPoleDynamics.m)（1D 垂直跳跃器 / SLIP 模型）翻译成 C++ + Eigen，由我们自己做 RK4 积分，**MuJoCo 仅作可视化**（`mj_forward` 只跑前向运动学，不参与物理求解）。

> 命名说明：MATLAB 函数虽叫 `cartPoleDynamics`，内部其实是 1D 垂直跳跃器——只有 `z(0)=高度 / z(1)=速度` 两个状态。

---

## 一、文件清单

| 文件 | 角色 |
|------|------|
| [cartPoleDynamics.m](cartPoleDynamics.m) | 原版 MATLAB 动力学 + 运动学（分相位） |
| [slip_1d_model.hpp](slip_1d_model.hpp) / [.cpp](slip_1d_model.cpp) | C++ 翻译：`cartPoleDynamics` + `cartPoleKinematics` |
| [model.xml](model.xml) | MJCF：球 body + 脚 body，两个 slide 关节，tendon 当弹簧 |
| [hopper_sim_ui.cpp](hopper_sim_ui.cpp) | GLFW 主程序：RK4 积分 + 写 qpos + 渲染 |
| [CMakeLists.txt](CMakeLists.txt) | 子项目构建：静态库 `slip_1d_model` + 可执行 `hopper_sim_ui` |

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

`hopper_sim_ui.cpp` 顶部常量：

```cpp
static constexpr double X0  = 2.0;   // 初始高度 [m]
static constexpr double DX0 = 0.0;   // 初始速度 [m/s]

static const Params P{
    0.5,   // l     自然长度
    0.05,  // l_min 最小压缩
    800.0, // kp    弹簧刚度
    5.0,   // kd    阻尼
    1.0    // m     质量
};
```

理论平衡点：`x_eq = l - g·m/kp = 0.5 - 9.81/800 ≈ 0.488 m`（弹簧压缩 12 mm 支撑重力）。

---

## 五、编译与运行

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
| 左键拖拽 | 旋转视角 |
| 右键拖拽 | 平移视角 |
| 滚轮 | 缩放 |
| Space | 暂停 / 继续 |
| Backspace | 重置状态（回到 `X0/DX0`） |
| ESC | 退出 |

---

## 六、扩展

- **加控制输入**：把 `rk4_step` 的 `u` 参数从 `0.0` 改成你的策略输出（如 PD、强化学习动作）
- **改参数**：直接改 [hopper_sim_ui.cpp:33-39](hopper_sim_ui.cpp) 的 `Params P`，或 [hopper_sim_ui.cpp:29-30](hopper_sim_ui.cpp) 的 `X0/DX0`
- **新例子**：在 [../CMakeLists.txt](../CMakeLists.txt) 加 `add_subdirectory(<name>)`，父级已经准备好 `mujoco::*` 和 `Eigen3::Eigen` 目标
