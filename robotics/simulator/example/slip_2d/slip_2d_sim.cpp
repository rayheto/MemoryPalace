#include "slip_2d_sim.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace slip2d {

// ── Controller ───────────────────────────────────────────────────
double Controller::compute(Phase ph, const State& q,
                           const CtrlCtx& ctx, const Params2D& P) const {
    if (ph != Phase::Stance) return 0.0;
    switch (mode) {
        case Mode::Passive:
            return 0.0;
        case Mode::SimpleKp:
            return Kp_z * (P.l0 + fly_z - q(1));
        case Mode::EnergyInject: {
            // 首拍 stance 全程 passive（自由弹簧），等首次完整 liftoff 之后才介入。
            if (!ctx.armed) return 0.0;
            // 结构上沿用 Simple-Kp 的"虚拟弹簧"形式（整段 stance 持续作用，按当前 spring 偏移成比例），
            // 但用上一跳世界系顶点 fly_reach_z 与目标 (support_y + l0 + fly_z) 的偏差作为强度缩放。
            //
            // 旧版只在 dy 翻号后施加一次常值力，做的净功 ≈ u·Δl_ext，远不足以补偿阻尼耗散，
            // 也帮不到压缩段的能量保存 → 宏观表现就是"弹不起来"。
            //
            // 现在的语义：
            //   - 上一跳正好到目标   → scale=1，退化为 Simple-Kp 的等效刚度（k+Kp_z）；
            //   - 上一跳跳不够       → scale>1，本拍把"虚拟弹簧"调更硬，下一拍追上目标；
            //   - 上一跳过冲         → scale<1（可负），本拍变软甚至反向减能；
            //   - 爬台阶 / 下台阶    → support_y 上升/下降自动把 Δh 算进 err_world，无需 d_support_y 前馈。
            const double target    = P.l0 + fly_z;
            const double err_world = ctx.support_y + target - ctx.fly_reach_z;
            const double scale     = 1.0 + err_world / target;
            return scale * Kp_z * (target - q(1));
        }
    }
    return 0.0;
}

// ── 事件守卫构造 ─────────────────────────────────────────────────
// Flight: g_TD (足端世界 z - 地形)、g_fall (机身世界 z - 地形)
// Stance: g_LO (l0² - (x²+y²))、       g_fall (同上)
// 注意 g_fall 在 Stance 期间通常 > 0（机身比支撑点高）；为避免触地瞬间 g_fall=0 的伪触发，
// 我们在 Stance 仅监听 g_LO，把摔倒检测留给 Flight 相后接触地时再判断。
static std::vector<Guard> build_guards(Phase ph,
                                       const Eigen::Vector2d& p0,
                                       const Params2D& P,
                                       const Terrain& terr,
                                       double alpha_TD)
{
    std::vector<Guard> guards;
    if (ph == Phase::Flight) {
        // event 0 : 足端触地（dir = -1）
        guards.push_back(Guard{
            [&terr, &P, p0, alpha_TD](const State& q) -> double {
                Eigen::Vector2d f = foot_rel(q, alpha_TD, P);
                double fx = p0.x() + f.x();
                double fz = p0.y() + f.y();
                return fz - terr.height(fx);
            },
            -1, true
        });
        // event 1 : 机身触地（摔倒）
        guards.push_back(Guard{
            [&terr, p0](const State& q) -> double {
                double bx = p0.x() + q(0);
                double bz = p0.y() + q(1);
                return bz - terr.height(bx);
            },
            -1, true
        });
    } else {
        // event 0 : 弹簧伸到自然长度（离地）
        guards.push_back(Guard{
            [&P](const State& q) -> double {
                return P.l0 * P.l0 - (q(0)*q(0) + q(1)*q(1));
            },
            -1, true
        });
        // event 1 : 机身触地（摔倒）—— 防止支撑相被压扁
        guards.push_back(Guard{
            [&terr, p0](const State& q) -> double {
                double bx = p0.x() + q(0);
                double bz = p0.y() + q(1);
                return bz - terr.height(bx);
            },
            -1, true
        });
    }
    return guards;
}

// ── 查询 ─────────────────────────────────────────────────────────
double Sim2D::swing_angle() const {
    if (cur_phase == Phase::Stance) return alpha_TD;
    double T = std::max(swing_set_time, 1e-6);
    double s = std::min(std::max((t - t_flight_start) / T, 0.0), 1.0);
    return (1.0 - s) * alpha_prev_TD + s * alpha_TD;
}

Eigen::Vector2d Sim2D::foot_world() const {
    if (cur_phase == Phase::Stance) return p0;
    Eigen::Vector2d f = foot_rel(q, swing_angle(), P);
    return { p0.x() + f.x(), p0.y() + f.y() };
}

// ── reset ────────────────────────────────────────────────────────
void Sim2D::reset(const State& q0_world) {
    cur_phase = Phase::Flight;
    p0      = Eigen::Vector2d(q0_world(0), terrain.height(q0_world(0)));
    p0_prev = p0;
    q       = State::Zero();
    q(0) = q0_world(0) - p0.x();
    q(1) = q0_world(1) - p0.y();
    q(2) = q0_world(2);
    q(3) = q0_world(3);
    t       = 0.0;
    fell    = false;
    ctx     = CtrlCtx{};
    ctx.last_dy     = q(3);
    ctx.fly_reach_z = p0.y() + q(1);
    ctx.support_y   = p0.y();
    ctx.d_support_y = 0.0;
    t_flight_start  = 0.0;
    alpha_prev_TD   = alpha_TD;
    last_u_out      = 0.0;
}

// ── advance：把 dt_step 切成 ≤ this->dt 的子步 ───────────────────
void Sim2D::advance(double dt_step) {
    if (fell || t >= t_max || dt_step <= 0.0) return;

    double remain = dt_step;
    while (remain > 1e-12 && !fell && t < t_max) {
        double h = std::min(remain, dt);
        double t_before = t;

        // 1) 控制器（ZOH，在 RK4 步前算）
        double u   = ctrl.compute(cur_phase, q, ctx, P);
        last_u_out = u;

        // 2) RK4 + 事件
        auto guards = build_guards(cur_phase, p0, P, terrain, alpha_TD);
        StepOut out = rk4_step_with_events(cur_phase, q, t, h, P, u, guards);
        q = out.q_end;
        t = out.t_end;
        remain -= (t - t_before);          // 真正推进了多少

        // 3) 控制器上下文更新（基于当前相）
        if (cur_phase == Phase::Stance) {
            // ctx.armed = false 时表示首拍 stance，控制器全程被动（像自由弹簧），
            // 等首次 liftoff 把 fly_reach_z 写成实测顶点之后，下一拍 stance 才允许注入能量。
            if (ctx.armed && ctx.last_dy < 0.0 && q(3) > 0.0) ctx.en_out = 1.0;
            ctx.last_dy = q(3);
        } else {
            ctx.fly_reach_z = std::max(ctx.fly_reach_z, p0.y() + q(1));
        }

        // 4) 事件处理
        if (out.event_idx >= 0) {
            if (cur_phase == Phase::Flight) {
                if (out.event_idx == 0) {
                    // 触地：rebase
                    Eigen::Vector2d shift = foot_rel(q, alpha_TD, P);
                    p0_prev          = p0;
                    p0              += shift;
                    q(0)            -= shift.x();
                    q(1)            -= shift.y();
                    alpha_prev_TD    = alpha_TD;
                    ctx.en_out       = 0.0;
                    ctx.last_dy      = q(3);
                    ctx.support_y    = p0.y();
                    ctx.d_support_y  = p0.y() - p0_prev.y();
                    // ctx.fly_reach_z 保留（由刚结束的飞行段写入）
                    cur_phase        = Phase::Stance;
                } else {
                    fell = true;
                }
            } else {
                if (out.event_idx == 0) {
                    cur_phase       = Phase::Flight;
                    t_flight_start  = t;
                    ctx.fly_reach_z = p0.y() + q(1);
                    ctx.armed       = true;     // 首次完整 liftoff 完成，下一拍 stance 起允许 EnergyInject
                } else {
                    fell = true;
                }
            }
        }
    }
}

// ── batch run ────────────────────────────────────────────────────
std::vector<Segment> Sim2D::run(const State& q0_world) {
    std::vector<Segment> out;

    Sim2D s = *this;             // copy params/terrain/ctrl/dt/alpha_TD
    s.reset(q0_world);

    auto push_sample = [&](Segment& seg) {
        seg.t.push_back(s.t);
        seg.q.push_back(s.q);
        seg.u_out.push_back(s.last_u_out);
        seg.q_swing.push_back(s.swing_angle());
        seg.fly_reach_z.push_back(s.ctx.fly_reach_z);
    };

    Segment cur;
    cur.phase      = s.cur_phase;
    cur.support_xy = s.p0;
    push_sample(cur);

    while (!s.fell && s.t < s.t_max) {
        Phase           prev_phase = s.cur_phase;
        Eigen::Vector2d prev_p0    = s.p0;

        s.advance(s.dt);

        if (s.cur_phase != prev_phase || s.p0 != prev_p0) {
            cur.end = (prev_phase == Phase::Flight) ? Segment::End::Touchdown
                                                    : Segment::End::Liftoff;
            out.push_back(std::move(cur));

            cur = Segment{};
            cur.phase      = s.cur_phase;
            cur.support_xy = s.p0;
        }
        push_sample(cur);
    }
    cur.end = s.fell ? Segment::End::Fell : Segment::End::TimeOut;
    out.push_back(std::move(cur));
    return out;
}

// ── 地形 builder ─────────────────────────────────────────────────
Terrain make_flat(double x_lo, double x_hi, double h) {
    Terrain t;
    t.profile = { { x_lo, h }, { x_hi, h } };
    return t;
}

Terrain make_staircase(uint32_t seed, double x_lo, double x_start,
                       double step_dx, double step_dh, double jitter,
                       int n_steps, double edge_len) {
    Terrain t;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> dist(-jitter, jitter);

    // 平地缓冲段：(x_lo, 0) → (x_start, 0)，机器人在这上面起跳
    t.profile.push_back({ x_lo,    0.0 });
    t.profile.push_back({ x_start, 0.0 });

    double x = x_start, h = 0.0;
    for (int i = 0; i < n_steps; ++i) {
        double x_right_lo = x + step_dx - edge_len;
        if (x_right_lo > x) t.profile.push_back({ x_right_lo, h });
        x += step_dx;
        h += step_dh + dist(gen);          // 加 jitter ⇒ 崎岖
        t.profile.push_back({ x, h });
    }
    t.profile.push_back({ x + 5.0, h });    // 终点平台延伸 5m
    return t;
}

Terrain make_random_discrete(uint32_t seed, double x_lo, double x_hi,
                             double plat_len, double max_dh, double edge_len) {
    Terrain t;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> dist(-max_dh, max_dh);

    double h = 0.0;
    double x = x_lo;
    t.profile.push_back({ x, h });
    while (x + plat_len < x_hi) {
        double x_right_lo = x + plat_len - edge_len;
        if (x_right_lo > x) t.profile.push_back({ x_right_lo, h });
        x += plat_len;
        h += dist(gen);
        t.profile.push_back({ x, h });
    }
    t.profile.push_back({ x_hi, h });
    return t;
}

} // namespace slip2d
