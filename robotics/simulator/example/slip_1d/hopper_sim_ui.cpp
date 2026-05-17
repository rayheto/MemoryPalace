// hopper_sim_ui.cpp —— 1D SLIP 跳跃器（基于 simulation_dashboard 外壳）
//   动力学：cartPoleDynamics (slip_1d_model.hpp) + RK4 自写积分
//   UI：simulation_dashboard 提供 MuJoCo 视口 + Controls 面板 + Scope 三联曲线 + 可拖动分隔条

#include "dashboard.hpp"
#include "slip_1d_model.hpp"

#include <mujoco.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

// ── 仿真状态 ────────────────────────────────────────────────────────
static double X0  = 2.0;
static double DX0 = 0.0;
static Eigen::Vector2d z_state(X0, DX0);

static Params P{ 0.5, 0.05, 800.0, 5.0, 1.0 };   // l, l_min, kp, kd, m
static double u_manual = 0.0;
static double u_limit  = 50.0;
static double z_body_init = 0.5;

static double apply_limit(double u) {
    return std::max(-u_limit, std::min(u_limit, u));
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

int main() {
    sim_dashboard::Config cfg;
    cfg.title      = "Hopper 1D — SLIP tuner";
    cfg.model_path = "model.xml";

    sim_dashboard::Dashboard dash(cfg);
    if (!dash.model()) return 1;

    // 读 MJCF 里 hopper body 的初始世界 z（XML 写 0.5）
    if (mjModel* m = dash.model()) {
        int id = mj_name2id(m, mjOBJ_BODY, "hopper");
        if (id >= 0) z_body_init = m->body_pos[3*id + 2];
    }

    // ── 注册曲线（X 共享 t） ──
    auto cv_x  = dash.add_curve("x [m]",      IM_COL32(255, 200,  80, 255));
    auto cv_dx = dash.add_curve("dx [m/s]",   IM_COL32( 80, 200, 255, 255));
    auto cv_u  = dash.add_curve("u [m/s^2]",  IM_COL32(255, 120, 200, 255));

    // ── 回调 ──
    dash.on_reset([&]{
        z_state << X0, DX0;
    });

    dash.on_step([&](double dt){
        double u_cmd = apply_limit(u_manual);
        rk4_step(z_state, u_cmd, P, dt);
        dash.push(cv_x,  (float)z_state(0));
        dash.push(cv_dx, (float)z_state(1));
        dash.push(cv_u,  (float)u_cmd);
    });

    dash.on_sync([&](mjModel* m, mjData* d){
        d->qpos[0] = z_state(0) - z_body_init;            // hopper slide
        d->qpos[1] = cartPoleKinematics(z_state(0), P);   // foot slide
        d->qvel[0] = z_state(1);
        d->qvel[1] = 0.0;
        mj_forward(m, d);
    });

    dash.on_controls([&]{
        ImGui::Text("phase = %s", phase_of(z_state(0), P));
        ImGui::Text("x = %.4f m   dx = %+.4f m/s", z_state(0), z_state(1));

        ImGui::Separator();
        ImGui::TextUnformatted("Spring & body");
        sld("kp [N/m]",  &P.kp,    10.0, 5000.0, "%.1f");
        sld("kd [Ns/m]", &P.kd,    0.0,  50.0,   "%.2f");
        sld("l (rest)",  &P.l,     0.2,  1.0);
        sld("l_min",     &P.l_min, 0.01, 0.2);
        sld("m [kg]",    &P.m,     0.1,  10.0,   "%.2f");

        ImGui::Separator();
        ImGui::TextUnformatted("Control input");
        sld("u [m/s^2]", &u_manual, -100.0, 100.0, "%.2f");
        sld("|u| limit", &u_limit,  0.0,    200.0, "%.1f");
        if (ImGui::Button("u = 0")) u_manual = 0.0;

        ImGui::Separator();
        ImGui::TextUnformatted("Initial state (apply on Reset)");
        sld("X0 [m]",    &X0,  0.1,  5.0,  "%.2f");
        sld("DX0 [m/s]", &DX0, -5.0, 5.0,  "%.2f");

        ImGui::Separator();
        double x_eq = P.l - 9.81 * P.m / P.kp;
        ImGui::Text("x_eq = l - g*m/kp = %.4f m", x_eq);
        ImGui::Text("(static equilibrium)");
    });

    dash.on_status([&]{
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "t=%.2fs  x=%.3fm  dx=%+.3fm/s  phase=%s",
                      dash.sim_time(), z_state(0), z_state(1),
                      phase_of(z_state(0), P));
        return std::string(buf);
    });

    return dash.run();
}
