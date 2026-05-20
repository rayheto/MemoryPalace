// slip_2d_model.hpp
//
// 2D-SLIP（单腿支撑）动力学 + 地形 + RK4 + 事件触发（zero-crossing terminating）
// 对应 MATLAB ode45 + odeset('Events',…) 那一套，C++ 自写最小实现，无新依赖。
//
// 状态 q = [x, y, dx, dy, alpha]      x,y,dx,dy 都是「相对当前支撑点 p0」的局部坐标
// 全局位姿 (x0+x, y0+y)，alpha = 腿与地面夹角（rad，π/2 即腿垂直机体正下方）
//
// 两套动力学：
//   Flight :  ddx=0, ddy=-g, dα/dt=0
//   Stance :  l1 = sqrt(x²+y²)
//             u  = Kp_z*(l0+fly_z - y)          // 可选 Raibert 高度反馈
//             a1 = k*(l0-l1)/m + u
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

using State = Eigen::Matrix<double, 5, 1>;   // [x, y, dx, dy, alpha(rad)]

struct Params2D {
    double l0    = 1.0;        // 弹簧原长
    double k     = 1500.0;     // 弹簧刚度
    double m     = 1.0;        // 质量
    double kd    = 1.0;        // 支撑相竖直阻尼
    double g     = 9.81;
    double Kp_z  = 0.0;        // Raibert 高度控制增益（=0 即被动）
    double fly_z = 0.0;        // 期望弹跳高度偏移
};

struct Terrain {
    std::vector<Eigen::Vector2d> profile;   // (x, h)，按 x 升序，至少 2 点
    double height(double x_world) const;
};

enum class Phase { Flight, Stance };

State dynamics(Phase ph, const State& q, const Params2D& P);

// 局部坐标系下足端位置：(x + l0*cosα,  y - l0*sinα)
Eigen::Vector2d foot_rel(const State& q, const Params2D& P);

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

// 一步 RK4 + 事件检测；若 [q0, q1] 之间有 guard 沿匹配方向过零 ⇒ 二分细化定位事件时刻
StepOut rk4_step_with_events(Phase ph,
                             const State& q0,
                             double t0,
                             double dt,
                             const Params2D& P,
                             const std::vector<Guard>& guards,
                             double tol = 1e-6);

} // namespace slip2d
