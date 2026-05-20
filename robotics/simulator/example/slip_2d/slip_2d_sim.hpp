// slip_2d_sim.hpp
//
// 2D-SLIP 相切换驱动：把 MATLAB 那段 for 循环（飞行↔支撑反复切换）封装成
//   - run(q0_world)    ：batch 跑到摔倒 / t_max ⇒ 切成 Segment 数组（对应 sim_data(i)）
//   - reset / advance ：stepwise 单步，给 dashboard 在线仿真用

#pragma once

#include "slip_2d_model.hpp"

#include <Eigen/Core>
#include <vector>

namespace slip2d {

struct Segment {
    Phase                phase;
    Eigen::Vector2d      support_xy;     // 该段支撑点（全局）
    std::vector<double>  t;              // 全局时刻
    std::vector<State>   q;              // 支撑相对坐标（与 MATLAB 一致）
    enum class End { TimeOut, Liftoff, Touchdown, Fell } end = End::TimeOut;
};

struct Sim2D {
    Params2D        P;
    Terrain         terrain;
    double          dt         = 1e-3;
    double          t_max      = 10.0;
    double          alpha_TD   = 1.57079632679;   // 触地腿角 (rad)，π/2 = 腿垂直机体正下方

    // ── batch ──
    std::vector<Segment> run(const State& q0_world);

    // ── stepwise（dashboard 在线） ──
    Phase           cur_phase  = Phase::Flight;
    State           q          = State::Zero();   // 支撑相对状态
    Eigen::Vector2d p0         = {0.0, 0.0};      // 当前支撑点（全局）
    double          t          = 0.0;
    bool            fell       = false;

    void reset(const State& q0_world);
    // 推进 dt_step（≤ this->dt 时用单步；否则等价多次 dt 推进）。
    // 内部自动处理事件触发与相切换；fell=true 后空转。
    void advance(double dt_step);

    // 便捷查询
    double          x_world() const { return p0.x() + q(0); }
    double          y_world() const { return p0.y() + q(1); }
    Eigen::Vector2d foot_world() const;       // 飞行：跟随机体；支撑：== p0
    double          spring_len() const { return std::hypot(q(0), q(1)); }
};

} // namespace slip2d
