// slip_2d_model.hpp
//
// 2D-SLIP（单腿支撑）动力学 + 地形 + RK4 + 事件触发（zero-crossing terminating）
// 对应 MATLAB ode45 + odeset('Events',…) 那一套，C++ 自写最小实现，无新依赖。
//
// §1.3.2 重构后：
//   - State = [x, y, dx, dy] (4 维)，全部为「相对当前支撑点 p0」的局部坐标
//     腿角 α 不再属于积分变量（dα/dt=0 在 RK4 中纯属负担），改由 Sim2D 在
//     touchdown 事件时直接采样、由 swing_angle() 给 viewport 插值
//   - 控制器与动力学解耦：dynamics(...) 收 u_stance 实参，由调用方 ZOH 注入
//     Kp_z / fly_z 从 Params2D 迁到 Controller
//   - 压缩极限保护：rk4_step_with_events() 在 RK4 后 post-clamp，
//     若 q.y<0 且仍在下沉则把 dy 清零（对齐 MATLAB `if y<0; dy=ddy=0`）
//
// 两套动力学：
//   Flight :  ddx=0, ddy=-g
//   Stance :  l1 = sqrt(x²+y²)
//             a1 = k*(l0-l1)/m + u_stance      ← u 由外部注入
//             ddx = a1/l1 * x
//             ddy = a1/l1 * y - kd*dy - g
//
// 事件由调用方按相组装（见 slip_2d_sim.cpp 的 build_guards）。

#pragma once

#include <Eigen/Core>

#include <cmath>
#include <functional>
#include <vector>

namespace slip2d {

using State = Eigen::Matrix<double, 4, 1>;   // [x, y, dx, dy]

struct Params2D {
    double l0    = 1.0;        // 弹簧原长
    double k     = 1500.0;     // 弹簧刚度
    double m     = 1.0;        // 质量
    double kd    = 1.0;        // 支撑相竖直阻尼
    double g     = 9.81;
};

struct Terrain {
    std::vector<Eigen::Vector2d> profile;   // (x, h)，按 x 升序，至少 2 点
    double height(double x_world) const;
};

enum class Phase { Flight, Stance };

// u_stance：支撑相注入力（飞行相忽略），由 Controller 在 RK4 步前算好（ZOH）
State dynamics(Phase ph, const State& q, const Params2D& P, double u_stance);

// 局部坐标系下足端位置：(x + l0*cosα,  y - l0*sinα)
Eigen::Vector2d foot_rel(const State& q, double alpha, const Params2D& P);

// 事件守卫：g(q) 在 [t0, t0+dt] 内沿 direction 方向过零 ⇒ 终止该段积分
struct Guard {
    std::function<double(const State&)> g;
    int  direction = -1;       // -1: 从正到负；+1: 反向；0: 双向
    bool terminal  = true;
};

struct StepOut {
    State  q_end;
    double t_end;              // 没触发：t0+dt；触发：事件时刻
    int    event_idx = -1;     // -1 = 未触发
};

// 一步 RK4 + 事件检测 + 压缩极限 post-clamp。
// u_stance 在该步整段视为常量（ZOH，对齐 MATLAB ode45 stage 行为）。
StepOut rk4_step_with_events(Phase ph,
                             const State& q0,
                             double t0,
                             double dt,
                             const Params2D& P,
                             double u_stance,
                             const std::vector<Guard>& guards,
                             double tol = 1e-6);

} // namespace slip2d
