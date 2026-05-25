// slip_2d_sim.hpp
//
// 2D-SLIP 相切换驱动：把 MATLAB 那段 for 循环（飞行↔支撑反复切换）封装成
//   - run(q0_world)    ：batch 跑到摔倒 / t_max ⇒ 切成 Segment 数组（对应 sim_data(i)）
//   - reset / advance ：stepwise 单步，给 dashboard 在线仿真用
//
// §1.3.2 扩展：
//   - Controller 解耦：Sim2D 持有 Controller，每次 RK4 前算 u_stance（ZOH）
//   - CtrlCtx：跨步保留的控制器状态（en_out 锁存、上一步 dy、fly_reach_z、d_support_y）
//   - 录制侧信道：Segment::{u_out, q_swing} 与 t / q 同长，dashboard 画曲线 / 渲染腿用
//   - 地形 builder：make_flat / make_staircase / make_random_discrete

#pragma once

#include "slip_2d_model.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <vector>

namespace slip2d {

// ── 控制器上下文（跨 RK4 子步保留） ───────────────────────────────
struct CtrlCtx {
    double en_out      = 0.0;   // 0/1 锁存：本支撑相是否已穿过压缩底部
    double last_dy     = 0.0;   // 上一步 q(3)，用于检测 dy 零交叉
    double fly_reach_z = 0.0;   // 最近飞行段的顶点 y_world（TD 瞬间冻结，能量补偿用）
    double support_y   = 0.0;   // 当前支撑点的世界 y（TD 瞬间刷新）；EnergyInject 用它构造世界系 err 缩放虚拟弹簧
    double d_support_y = 0.0;   // p0.y - p0_prev.y（TD 瞬间刷新）；HUD/调试显示用，controller 不依赖
    bool   armed       = false; // 首次完整 liftoff 之后才置真；EnergyInject 的 en_out 仅在置真后允许翻 1
};

struct Controller {
    enum class Mode { Passive, SimpleKp, EnergyInject };
    Mode   mode  = Mode::Passive;
    double Kp_z  = 0.0;
    double fly_z = 0.0;

    // 由 Sim2D::advance() 每次 RK4 子步前调用
    double compute(Phase ph, const State& q,
                   const CtrlCtx& ctx, const Params2D& P) const;
};

struct Segment {
    Phase                phase;
    Eigen::Vector2d      support_xy;     // 该段支撑点（全局）
    std::vector<double>  t;              // 全局时刻
    std::vector<State>   q;              // 支撑相对坐标 [x,y,dx,dy]
    std::vector<double>  u_out;          // 该采样点的控制器输出（ZOH 输入下一拍）
    std::vector<double>  q_swing;        // 该采样点的腿角 α (rad)
    std::vector<double>  fly_reach_z;    // 该采样点对应的 ctx.fly_reach_z（飞行段顶点高度）
    enum class End { TimeOut, Liftoff, Touchdown, Fell } end = End::TimeOut;
};

struct Sim2D {
    Params2D    P;
    Terrain     terrain;
    Controller  ctrl;
    double      dt              = 1e-3;
    double      t_max           = 10.0;
    double      alpha_TD        = 1.57079632679;   // 触地腿角 (rad)
    double      alpha_prev_TD   = 1.57079632679;   // 上一次触地腿角（飞行插值用）
    double      swing_set_time  = 0.3;             // 飞行段腿摆到 alpha_TD 的时长 (s)

    // ── batch ──
    std::vector<Segment> run(const State& q0_world);

    // ── stepwise（dashboard 在线） ──
    Phase           cur_phase  = Phase::Flight;
    State           q          = State::Zero();   // [x,y,dx,dy] 局部
    Eigen::Vector2d p0         = {0.0, 0.0};      // 当前支撑点（全局）
    Eigen::Vector2d p0_prev    = {0.0, 0.0};      // 上一支撑点（全局）
    double          t          = 0.0;
    bool            fell       = false;
    CtrlCtx         ctx;
    double          t_flight_start = 0.0;

    void reset(const State& q0_world);
    void advance(double dt_step);

    // 便捷查询
    double          x_world()   const { return p0.x() + q(0); }
    double          y_world()   const { return p0.y() + q(1); }
    double          spring_len() const { return std::hypot(q(0), q(1)); }
    double          swing_angle() const;       // Flight 内 lerp(alpha_prev_TD → alpha_TD)
    Eigen::Vector2d foot_world() const;        // 支撑：== p0；飞行：随机体 + l0·(cosα,-sinα)

    double          last_u_out = 0.0;          // 最近一次控制器输出（HUD/曲线用）
};

// ── 地形 builder ──────────────────────────────────────────────────
Terrain make_flat(double x_lo = -2.0, double x_hi = 20.0, double h = 0.0);

// 崎岖阶梯：x_lo..x_start 段平地（h=0，机器人起步在此），之后每 step_dx 抬
//   step_dh + uniform(-jitter, +jitter)
// 共 n_steps 级，jitter=0 即均匀阶梯；竖直边用 edge_len 近似。末端再延伸 5m 平台。
Terrain make_staircase(uint32_t seed    = 1,
                       double   x_lo    = -2.0,
                       double   x_start =  1.0,
                       double   step_dx = 0.5,
                       double   step_dh = 0.05,
                       double   jitter  = 0.03,
                       int      n_steps = 20,
                       double   edge_len= 0.03);

// 随机离散：在 [x_lo, x_hi] 上以 plat_len 为周期排平台，每段 Δh∈uniform(-max_dh,+max_dh)
Terrain make_random_discrete(uint32_t seed,
                             double x_lo     = -2.0,
                             double x_hi     = 20.0,
                             double plat_len = 0.20,
                             double max_dh   = 0.10,
                             double edge_len = 0.03);

} // namespace slip2d
