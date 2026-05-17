// slip_trajopt_ui.cpp —— 把离线优化轨迹叠到在线仿真上。
//
// 操作：
//   - 启动时跑一次 slip_trajopt::solve()，把 (x*, dx*, u*) 作为静态曲线注册到 dashboard。
//   - 实时仿真采用 RK4，控制 u 从离散 u_k 按时间线性插值取得 → live 应紧贴 reference。
//   - 调 X0/kp/... 等滑块 → 按「Re-optimize」重新求解、刷新 reference。
//   - 仿真时间超过轨迹末端 → 自动 Reset 形成循环演示。

#include "dashboard.hpp"
#include "slip_1d_model.hpp"
#include "trajopt.hpp"

#include <mujoco.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

// ── 状态 ────────────────────────────────────────────────────────────
static double X0  = 2.5;   // 默认从 2.5 m 落（势能比目标 apex 高 → 控制器需要耗能）
static double DX0 = 0.0;
static Eigen::Vector2d z_state(X0, DX0);

static Params P{ 0.5, 0.05, 800.0, 5.0, 1.0 };   // l, l_min, kp, kd, m
static double z_body_init = 0.5;

static slip_trajopt::Options g_opt;
static slip_trajopt::Result  g_traj;
static double g_loop_horizon = 0.0;
static bool   g_loop = true;
// 一圈跑完 → 暂停在最终状态，等用户按 Space 后再 Reset 开新一圈
static bool   g_awaiting_resume = false;

// ── 工具 ────────────────────────────────────────────────────────────
static double u_from_traj(double t) {
    // 在 g_traj.traj_tu / traj_u 上线性插值（注意 traj_u 在阶段 0/2 已置 0）
    const auto& tt = g_traj.traj_tu;
    const auto& uu = g_traj.traj_u;
    if (tt.empty()) return 0.0;
    if (t <= tt.front()) return uu.front();
    if (t >= tt.back())  return uu.back();
    // 二分
    int lo = 0, hi = (int)tt.size() - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (tt[mid] <= t) lo = mid; else hi = mid;
    }
    double w = (t - tt[lo]) / (tt[hi] - tt[lo]);
    return (1.0 - w) * uu[lo] + w * uu[hi];
}

static void rk4_step(Eigen::Vector2d& z, double u, const Params& p, double dt) {
    Eigen::Vector2d k1 = cartPoleDynamics(z,             u, p);
    Eigen::Vector2d k2 = cartPoleDynamics(z + 0.5*dt*k1, u, p);
    Eigen::Vector2d k3 = cartPoleDynamics(z + 0.5*dt*k2, u, p);
    Eigen::Vector2d k4 = cartPoleDynamics(z +     dt*k3, u, p);
    z += (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

static bool sld(const char* label, double* v, double mn, double mx,
                const char* fmt = "%.3f") {
    return ImGui::SliderScalar(label, ImGuiDataType_Double, v, &mn, &mx, fmt);
}

static const char* phase_of(double x, const Params& p) {
    if (x > p.l)     return "FLIGHT";
    if (x < p.l_min) return "BOTTOM";
    return "STANCE";
}

// ── 主程序 ──────────────────────────────────────────────────────────
int main() {
    sim_dashboard::Config cfg;
    cfg.title      = "Hopper 1D — trajopt vs sim";
    cfg.model_path = "model.xml";
    cfg.scope_time_span = 2.5;

    sim_dashboard::Dashboard dash(cfg);
    if (!dash.model()) return 1;

    if (mjModel* m = dash.model()) {
        int id = mj_name2id(m, mjOBJ_BODY, "hopper");
        if (id >= 0) z_body_init = m->body_pos[3*id + 2];
    }

    // ── 首跑一次优化 ──
    g_opt.verbose = true;
    g_traj = slip_trajopt::solve(P, X0, g_opt);
    // traj_t.back() = t_apex（在 trajopt.cpp 里我们把阶段 2 截断到了 apex）。
    // 不要再加 buffer：buffer 越大，live sim 越会越过 apex 往回掉，
    // 暂停时 dx 就不再是 0 而是负的（自由下落）。
    g_loop_horizon = g_traj.ok ? (double)g_traj.traj_t.back() : 3.0;

    // ── 注册曲线（每组同一个 group 让 live + ref 叠在同一个 subplot） ──
    auto cv_x_live = dash.add_curve("x [m]",      IM_COL32(255, 200,  80, 255), /*group=*/0);
    auto cv_x_ref  = dash.add_static_curve("x* [m]",
                                           IM_COL32(255, 200,  80, 160),
                                           g_traj.traj_t, g_traj.traj_x, 0);

    auto cv_dx_live = dash.add_curve("dx [m/s]",  IM_COL32( 80, 200, 255, 255), /*group=*/1);
    auto cv_dx_ref  = dash.add_static_curve("dx* [m/s]",
                                            IM_COL32( 80, 200, 255, 160),
                                            g_traj.traj_t, g_traj.traj_dx, 1);

    auto cv_u_live  = dash.add_curve("u [m/s^2]", IM_COL32(255, 120, 200, 255), /*group=*/2);
    auto cv_u_ref   = dash.add_static_curve("u* [m/s^2]",
                                            IM_COL32(255, 120, 200, 160),
                                            g_traj.traj_tu, g_traj.traj_u, 2);

    // ── 回调 ──
    dash.on_reset([&]{
        z_state << X0, DX0;
        g_awaiting_resume = false;  // 任何复位路径都清掉等待标志
    });

    dash.on_step([&](double dt){
        double t = dash.sim_time();
        double u_cmd = g_traj.ok ? u_from_traj(t) : 0.0;
        // 在 |u| 上限内夹一下（动力学其他时间应当 u=0）
        u_cmd = std::max(-g_opt.u_max, std::min(g_opt.u_max, u_cmd));
        rk4_step(z_state, u_cmd, P, dt);
        dash.push(cv_x_live,  (float)z_state(0));
        dash.push(cv_dx_live, (float)z_state(1));
        dash.push(cv_u_live,  (float)u_cmd);
    });

    dash.on_sync([&](mjModel* m, mjData* d){
        // on_sync 仅在「未暂停」时被回调。若 awaiting_resume 仍为 true，
        // 说明上次暂停（cycle 末尾）后用户刚按 Space 解开 → 触发 Reset。
        if (g_awaiting_resume) {
            dash.request_reset();      // 下一帧主循环里会清 sim_time、调 on_reset
            g_awaiting_resume = false; // on_reset 也会清，这里冗余清一次以防万一
            return;
        }

        d->qpos[0] = z_state(0) - z_body_init;
        d->qpos[1] = cartPoleKinematics(z_state(0), P);
        d->qvel[0] = z_state(1);
        d->qvel[1] = 0.0;
        mj_forward(m, d);

        // 跑完一圈 → 暂停在末态，等用户按 Space 继续
        if (g_loop && dash.sim_time() > g_loop_horizon) {
            g_awaiting_resume = true;
            dash.set_paused(true);
        }
    });

    auto re_optimize = [&]{
        g_traj = slip_trajopt::solve(P, X0, g_opt);
        if (g_traj.ok) {
            dash.update_static_curve(cv_x_ref,  g_traj.traj_t,  g_traj.traj_x);
            dash.update_static_curve(cv_dx_ref, g_traj.traj_t,  g_traj.traj_dx);
            dash.update_static_curve(cv_u_ref,  g_traj.traj_tu, g_traj.traj_u);
            g_loop_horizon = (double)g_traj.traj_t.back();
        }
    };

    dash.on_controls([&]{
        ImGui::Text("phase = %s", phase_of(z_state(0), P));
        ImGui::Text("x = %.4f m   dx = %+.4f m/s", z_state(0), z_state(1));

        ImGui::Separator();
        ImGui::TextUnformatted("Spring & body");
        bool changed = false;
        changed |= sld("kp [N/m]",  &P.kp,    10.0, 5000.0, "%.1f");
        changed |= sld("kd [Ns/m]", &P.kd,    0.0,  50.0,   "%.2f");
        changed |= sld("l (rest)",  &P.l,     0.2,  1.0);
        changed |= sld("l_min",     &P.l_min, 0.01, 0.2);
        changed |= sld("m [kg]",    &P.m,     0.1,  10.0,   "%.2f");

        ImGui::Separator();
        ImGui::TextUnformatted("Initial state (apply on Reset)");
        changed |= sld("X0 [m]",    &X0,  P.l + 0.01,  3.0,  "%.2f");
        sld("DX0 [m/s]", &DX0, -5.0, 5.0,  "%.2f");

        ImGui::Separator();
        ImGui::TextUnformatted("TrajOpt knobs");
        ImGui::SliderInt("N (nodes)", &g_opt.N,        10, 80);
        sld("u_max",                  &g_opt.u_max,    0.0, 200.0, "%.1f");
        sld("target apex [m]",        &g_opt.x_apex_target,
            P.l + 0.01, std::max(X0 * 1.2, P.l + 1.0), "%.3f");
        ImGui::Checkbox("Auto-pause at end of cycle", &g_loop);
        if (g_awaiting_resume) {
            ImGui::TextColored(ImVec4(1, 0.85f, 0.2f, 1),
                               ">> Press Space to start next cycle <<");
        }

        ImGui::Separator();
        if (ImGui::Button("Re-optimize (or use sliders then click)")) {
            re_optimize();
        }
        if (changed) {
            // 参数变更时给出明显提示
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "params changed");
        }

        ImGui::Separator();
        if (g_traj.ok) {
            ImGui::Text("T*    = %.4f s", g_traj.T);
            ImGui::Text("dx_TD = %+.3f m/s   dx_LO = %+.3f m/s",
                        g_traj.dx_TD, g_traj.dx_LO);
            ImGui::Text("x_apex= %.3f m  (target %.3f m)",
                        g_traj.x_apex, g_opt.x_apex_target);
            ImGui::Text("nlopt = %d  evals=%d  J=%.4f (effort)",
                        g_traj.nlopt_code, g_traj.n_eval, g_traj.obj_value);
        } else {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1),
                              "TrajOpt FAILED (code=%d)", g_traj.nlopt_code);
        }
    });

    dash.on_status([&]{
        char buf[256];
        double u_now = g_traj.ok ? u_from_traj(dash.sim_time()) : 0.0;
        std::snprintf(buf, sizeof(buf),
                      "t=%.2fs  x=%.3fm  dx=%+.3fm/s  u*=%.2f  phase=%s",
                      dash.sim_time(), z_state(0), z_state(1), u_now,
                      phase_of(z_state(0), P));
        return std::string(buf);
    });

    return dash.run();
}
