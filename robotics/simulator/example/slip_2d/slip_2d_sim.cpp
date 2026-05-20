#include "slip_2d_sim.hpp"

#include <cmath>

namespace slip2d {

// 构造当前相对应的事件守卫列表
//   Flight: g_TD (足端世界 z - 地形)、g_fall (机身世界 z - 地形)
//   Stance: g_LO (l0² - (x²+y²))、       g_fall (同上)
// 注意 g_fall 在 Stance 期间通常 > 0（机身比支撑点高）；为避免触地瞬间 g_fall=0 的伪触发，
// 我们在 Stance 仅监听 g_LO，把摔倒检测留给 Flight 相后接触地时再判断。
static std::vector<Guard> build_guards(Phase ph,
                                       const Eigen::Vector2d& p0,
                                       const Params2D& P,
                                       const Terrain& terr,
                                       double /*alpha_TD*/)
{
    std::vector<Guard> guards;
    if (ph == Phase::Flight) {
        // event 0 : 足端触地（dir = -1）
        guards.push_back(Guard{
            [&, p0](const State& q) -> double {
                Eigen::Vector2d f = foot_rel(q, P);
                double fx = p0.x() + f.x();
                double fz = p0.y() + f.y();
                return fz - terr.height(fx);
            },
            -1, true
        });
        // event 1 : 机身触地（摔倒）
        guards.push_back(Guard{
            [&, p0](const State& q) -> double {
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
            [&, p0](const State& q) -> double {
                double bx = p0.x() + q(0);
                double bz = p0.y() + q(1);
                return bz - terr.height(bx);
            },
            -1, true
        });
    }
    return guards;
}

Eigen::Vector2d Sim2D::foot_world() const {
    if (cur_phase == Phase::Stance) return p0;
    Eigen::Vector2d f = foot_rel(q, P);
    return { p0.x() + f.x(), p0.y() + f.y() };
}

void Sim2D::reset(const State& q0_world) {
    // 假设初始处于飞行相；支撑点取在机体正下方地面上作占位（仅用于守卫坐标，触地时会被替换）
    cur_phase = Phase::Flight;
    p0 = Eigen::Vector2d(q0_world(0), terrain.height(q0_world(0)));
    q  = State::Zero();
    q(0) = q0_world(0) - p0.x();     // = 0
    q(1) = q0_world(1) - p0.y();     // 机身高度差
    q(2) = q0_world(2);
    q(3) = q0_world(3);
    q(4) = q0_world(4);
    t    = 0.0;
    fell = false;
}

void Sim2D::advance(double dt_step) {
    if (fell || t >= t_max || dt_step <= 0.0) return;

    // 把大 dt 切成 ≤ this->dt 的子步，确保单步 RK4 精度
    double remain = dt_step;
    while (remain > 1e-12 && !fell && t < t_max) {
        double h = std::min(remain, dt);
        auto guards = build_guards(cur_phase, p0, P, terrain, alpha_TD);
        StepOut out = rk4_step_with_events(cur_phase, q, t, h, P, guards);
        q = out.q_end;
        t = out.t_end;
        remain -= h;

        if (out.event_idx < 0) continue;

        if (cur_phase == Phase::Flight) {
            if (out.event_idx == 0) {
                // 触地：更新支撑点 + rebase 局部状态
                Eigen::Vector2d shift = foot_rel(q, P);
                p0 += shift;
                q(0) -= shift.x();
                q(1) -= shift.y();
                q(4)  = alpha_TD;
                cur_phase = Phase::Stance;
            } else {
                fell = true;
            }
        } else {
            if (out.event_idx == 0) {
                cur_phase = Phase::Flight;   // 离地：p0 不变
            } else {
                fell = true;
            }
        }
        // 触发后通常 remain 还剩一点，下一轮 while 继续推进
    }
}

std::vector<Segment> Sim2D::run(const State& q0_world) {
    std::vector<Segment> out;

    // 用独立的 Sim 状态跑（不污染 stepwise 的 cur_phase）
    Sim2D s = *this;             // copy params/terrain/dt/alpha_TD
    s.reset(q0_world);

    auto push_sample = [&](Segment& seg) {
        seg.t.push_back(s.t);
        seg.q.push_back(s.q);
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
            // 给上一段补一个事件时刻的点（在切换之前的相坐标系下）
            // 这里 s 已经切到新相，但 s.t 是事件时刻，rebase 后 s.q 已变；
            // 为简单起见，仅在 segment 末尾标记 end 类型；连续画图时 segments 拼接即可。
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

} // namespace slip2d
