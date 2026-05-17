// slip_1d_trajopt/trajopt.hpp
//
// 1D SLIP 跳跃器三段式轨迹优化（对应 MATLAB optimTraj 那一套）：
//   阶段 0 — 自由落体（高 → 触地）：解析积分
//   阶段 1 — 支撑相（弹簧压缩 → 回弹）：直接配点 NLP（NLopt SLSQP）
//   阶段 2 — 飞行段（离地 → 顶点）：解析积分
//
// 阶段 1 决策变量：[x_0..x_N, dx_0..dx_N, u_0..u_{N-1}, T]   总维度 = 3N+3
// 目标：最小化 h · Σ u_k²       （纯控制力消耗 → 自然按需饱和到 u_max）
// 等式约束：
//   边界 4 个     x_0=l, dx_0=dx_TD, x_N=l, dx_N=dx_LO_target
//   配点缺陷 2N 个
// 其中 dx_LO_target = sqrt(2g·(x_apex_target − l))，由用户指定目标顶点高度。
// 不等式：x ∈ [l_min, l]、u ∈ [-u_max, u_max]、T ∈ [T_lo, T_hi]
//
// 由于支撑相动力学线性，雅可比 90% 是常数，全部手写解析。

#pragma once

#include "slip_1d_model.hpp"

#include <vector>

namespace slip_trajopt {

struct Options {
    int    N             = 40;     // 配点段数
    double x_apex_target = 1.2;    // 期望顶点高度 [m]（必须 > p.l）
    double u_max         = 50.0;   // |u| 上界 [m/s^2]
    double T_lo          = 0.05;   // 支撑时长下界 [s]
    double T_hi          = 1.0;    // 支撑时长上界 [s]
    double T_init        = 0.20;   // 支撑时长初值
    int    max_iter      = 400;
    double xtol_rel      = 1e-7;
    double ftol_rel      = 1e-8;
    bool   verbose       = false;  // 求解完毕打印总结
};

struct Result {
    bool   ok          = false;
    int    nlopt_code  = 0;
    int    n_eval      = 0;
    double obj_value   = 0.0;

    // 阶段时间
    double t_TD        = 0.0;     // 触地时刻
    double T           = 0.0;     // 支撑时长
    double t_LO        = 0.0;     // 离地时刻
    double t_apex      = 0.0;     // 最高点时刻
    double dx_TD       = 0.0;
    double dx_LO       = 0.0;
    double x_apex      = 0.0;

    // 优化器原始输出（支撑相）
    std::vector<double> x;        // [N+1]
    std::vector<double> dx;       // [N+1]
    std::vector<double> u;        // [N]    分段常值

    // 绘图用：三段拼接，时间从 t=0
    std::vector<float> traj_t;    // 状态时间网格
    std::vector<float> traj_x;    // 高度
    std::vector<float> traj_dx;   // 速度
    std::vector<float> traj_tu;   // 控制时间网格（u 离散点位）
    std::vector<float> traj_u;    // 控制
};

// X0 = 触地前的初始高度（必须 > p.l）。
// 失败时 result.ok == false，nlopt_code 给出 NLopt 错误码。
Result solve(const Params& p, double X0, const Options& opt = Options{});

} // namespace slip_trajopt
