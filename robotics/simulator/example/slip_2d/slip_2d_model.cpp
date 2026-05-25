#include "slip_2d_model.hpp"

namespace slip2d {

double Terrain::height(double x_world) const {
    if (profile.empty()) return 0.0;
    if (profile.size() == 1) return profile[0].y();
    if (x_world <= profile.front().x()) return profile.front().y();
    if (x_world >= profile.back().x())  return profile.back().y();
    // 二分定位区间
    int lo = 0, hi = (int)profile.size() - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (profile[mid].x() <= x_world) lo = mid; else hi = mid;
    }
    double x0 = profile[lo].x(), x1 = profile[hi].x();
    double y0 = profile[lo].y(), y1 = profile[hi].y();
    double dx = x1 - x0;
    if (dx < 1e-12) return y1;          // 竖直边沿（edge_len 极小）
    double w = (x_world - x0) / dx;
    return (1.0 - w) * y0 + w * y1;
}

State dynamics(Phase ph, const State& q, const Params2D& P, double u_stance) {
    State dq = State::Zero();
    dq(0) = q(2);
    dq(1) = q(3);
    if (ph == Phase::Flight) {
        dq(2) = 0.0;
        dq(3) = -P.g;
    } else {
        // 局部坐标下「弹簧向量」= (x, y)，长度 l1，方向沿 q
        double l1 = std::hypot(q(0), q(1));
        if (l1 < 1e-9) l1 = 1e-9;       // 兜底，避免 0 除
        double a1 = P.k * (P.l0 - l1) / P.m + u_stance;
        dq(2) = a1 / l1 * q(0);
        dq(3) = a1 / l1 * q(1) - P.kd * q(3) - P.g;
    }
    return dq;
}

Eigen::Vector2d foot_rel(const State& q, double alpha, const Params2D& P) {
    return { q(0) + P.l0 * std::cos(alpha),
             q(1) - P.l0 * std::sin(alpha) };
}

// 单步 RK4（u_stance ZOH）
static State rk4(Phase ph, const State& q0, double dt,
                 const Params2D& P, double u) {
    State k1 = dynamics(ph, q0,                 P, u);
    State k2 = dynamics(ph, q0 + 0.5 * dt * k1, P, u);
    State k3 = dynamics(ph, q0 + 0.5 * dt * k2, P, u);
    State k4 = dynamics(ph, q0 +       dt * k3, P, u);
    return q0 + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

// 方向匹配（对齐 MATLAB ode45 events：g0 == 0 视为「正侧」，允许从 0 跨到负触发）。
// 不这样做会漏掉「初始足端正好贴在地面 (g_TD=0)」这种合法场景。
static bool crossed(double g0, double g1, int dir) {
    if (dir < 0) return (g0 >= 0.0 && g1 <  0.0);
    if (dir > 0) return (g0 <= 0.0 && g1 >  0.0);
    return (g0 > 0.0) != (g1 > 0.0);
}

// MATLAB §1.3.2 的「if y<0 && ddy<0 then dy=ddy=0」效果：
// RK4 出步后若质心被压到支撑点之下且仍在下沉，把竖直分量回弹至上一拍。
static void compression_clamp(const State& q0, State& q1) {
    if (q1(1) < 0.0 && q1(1) < q0(1)) {
        q1(1) = q0(1);
        q1(3) = 0.0;
    }
}

StepOut rk4_step_with_events(Phase ph,
                             const State& q0,
                             double t0,
                             double dt,
                             const Params2D& P,
                             double u_stance,
                             const std::vector<Guard>& guards,
                             double tol)
{
    State q1 = rk4(ph, q0, dt, P, u_stance);
    if (ph == Phase::Stance) compression_clamp(q0, q1);

    // 收集候选事件
    std::vector<int> cand;
    cand.reserve(guards.size());
    for (size_t k = 0; k < guards.size(); ++k) {
        double g0 = guards[k].g(q0);
        double g1 = guards[k].g(q1);
        if (crossed(g0, g1, guards[k].direction)) cand.push_back((int)k);
    }
    if (cand.empty()) {
        return { q1, t0 + dt, -1 };
    }

    // 二分细化：寻找最早事件
    double lo = 0.0, hi = dt;
    int winner = cand.front();
    while (hi - lo > tol) {
        double mid = 0.5 * (lo + hi);
        State qm = rk4(ph, q0, mid, P, u_stance);
        if (ph == Phase::Stance) compression_clamp(q0, qm);
        int hit = -1;
        for (int k : cand) {
            double g0 = guards[k].g(q0);
            double gm = guards[k].g(qm);
            if (crossed(g0, gm, guards[k].direction)) { hit = k; break; }
        }
        if (hit >= 0) { hi = mid; winner = hit; }
        else          { lo = mid; }
    }
    State qe = rk4(ph, q0, hi, P, u_stance);
    if (ph == Phase::Stance) compression_clamp(q0, qe);
    return { qe, t0 + hi, winner };
}

} // namespace slip2d
