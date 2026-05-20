// slip_2d_sim_ui.cpp —— 2D-SLIP 事件触发仿真 + Dashboard 集成
//
// 操作：
//   - 启动跑一遍开环 batch（默认 Kp_z=0，对应 MATLAB 截图：弹几下后摔倒）
//     注册为静态参考曲线（x*, y*）
//   - 在线仿真：每个 phys_dt 调 Sim2D::advance(dt) 推进 → 推 live 曲线
//   - 调参数 / IC → 「Reset」复位状态，「Re-run open-loop」刷新参考曲线
//   - 勾选 "Enable Raibert height control" 启用 u = Kp_z*(l0+fly_z - y)

#include "dashboard.hpp"
#include "slip_2d_sim.hpp"

#include <mujoco.h>

#include <cmath>
#include <cstdio>
#include <string>

using slip2d::Params2D;
using slip2d::Phase;
using slip2d::Sim2D;
using slip2d::State;

namespace {

// ── 仿真状态（全局，被各 callback 共享） ─────────────────────────
Sim2D       g_sim;
Params2D    g_P_ui;                          // UI 滑块上的参数（可能 Kp_z=滑块值或 0）
double      g_Kp_z_slider = 300.0;           // Raibert 增益滑块（独立保存）
bool        g_raibert_on  = false;           // 复选框状态
double      g_alpha_TD_deg = 90.0;           // 触地腿角（度，UI）

// 初始条件（世界坐标）：x, y, dx, dy, leg_angle(deg)
// y0 必须 > l0，否则起始时足端已经在地面下，没有飞行段
double g_x0_w   = 0.0;
double g_y0_w   = 1.5;
double g_dx0_w  = 0.3;
double g_dy0_w  = 0.0;

// MuJoCo body 的初始位置（model.xml 里 hopper pos="0 0 1"），用来把世界 z 转 qpos
double g_bx_init = 0.0;
double g_bz_init = 1.0;

// 静态参考曲线 id
sim_dashboard::StaticCurveId cv_x_ref, cv_y_ref;
// live 曲线 id
sim_dashboard::CurveId       cv_x_live, cv_y_live;
sim_dashboard::CurveId       cv_dx_live, cv_dy_live;
sim_dashboard::CurveId       cv_alpha_live, cv_L_live;

// ── 工具 ────────────────────────────────────────────────────────
State make_q0_world() {
    State q0;
    q0 << g_x0_w, g_y0_w, g_dx0_w, g_dy0_w,
          g_alpha_TD_deg * M_PI / 180.0;
    return q0;
}

void apply_params_to_sim() {
    g_sim.P        = g_P_ui;
    g_sim.P.Kp_z   = g_raibert_on ? g_Kp_z_slider : 0.0;
    g_sim.alpha_TD = g_alpha_TD_deg * M_PI / 180.0;
}

// 把 segments 摊平为 (ts, x_world, y_world)
void flatten_segments(const std::vector<slip2d::Segment>& segs,
                      std::vector<float>& ts,
                      std::vector<float>& xs,
                      std::vector<float>& ys)
{
    ts.clear(); xs.clear(); ys.clear();
    for (const auto& seg : segs) {
        for (size_t i = 0; i < seg.t.size(); ++i) {
            ts.push_back((float)seg.t[i]);
            xs.push_back((float)(seg.support_xy.x() + seg.q[i](0)));
            ys.push_back((float)(seg.support_xy.y() + seg.q[i](1)));
        }
    }
}

bool sld(const char* label, double* v, double mn, double mx,
         const char* fmt = "%.3f") {
    return ImGui::SliderScalar(label, ImGuiDataType_Double, v, &mn, &mx, fmt);
}

const char* phase_name(Phase ph) {
    return ph == Phase::Flight ? "FLIGHT" : "STANCE";
}

} // namespace

int main() {
    // ── Dashboard 配置 ──
    sim_dashboard::Config cfg;
    cfg.title           = "SLIP 2D — event-triggered";
    cfg.model_path      = "model.xml";
    cfg.phys_dt         = 1e-3;
    cfg.scope_time_span = 3.0;

    sim_dashboard::Dashboard dash(cfg);
    if (!dash.model()) return 1;

    // 读 MJCF 默认 body 高度，便于 qpos 转世界
    if (mjModel* m = dash.model()) {
        int id = mj_name2id(m, mjOBJ_BODY, "hopper");
        if (id >= 0) {
            g_bx_init = m->body_pos[3*id + 0];
            g_bz_init = m->body_pos[3*id + 2];
        }
    }

    // ── 默认参数：与 MATLAB 截图对齐 ──
    g_P_ui.l0    = 1.0;
    g_P_ui.k     = 1500.0;
    g_P_ui.m     = 1.0;
    g_P_ui.kd    = 1.0;
    g_P_ui.g     = 9.81;
    g_P_ui.Kp_z  = 0.0;
    g_P_ui.fly_z = 0.0;

    // 地形：平坦，x ∈ [-2, 10]，h=0
    g_sim.terrain.profile = { { -2.0, 0.0 }, { 10.0, 0.0 } };
    g_sim.dt       = cfg.phys_dt;
    g_sim.t_max    = cfg.scope_time_span * 1.2;   // 比 scope 略长，便于看到摔倒

    apply_params_to_sim();
    g_sim.reset(make_q0_world());

    // ── 一次开环 batch：作为静态参考曲线（默认 Kp_z=0） ──
    {
        Sim2D ref = g_sim;            // 拷贝当前 params/terrain
        ref.P.Kp_z = 0.0;             // 强制开环
        auto segs = ref.run(make_q0_world());
        std::vector<float> ts, xs, ys;
        flatten_segments(segs, ts, xs, ys);

        cv_x_live = dash.add_curve(      "x [m]",  IM_COL32(255, 200,  80, 255), /*group=*/0);
        cv_x_ref  = dash.add_static_curve("x* [m]", IM_COL32(255, 200,  80, 160), ts, xs, 0);

        cv_y_live = dash.add_curve(      "y [m]",  IM_COL32( 80, 200, 255, 255), /*group=*/1);
        cv_y_ref  = dash.add_static_curve("y* [m]", IM_COL32( 80, 200, 255, 160), ts, ys, 1);

        cv_dx_live    = dash.add_curve("dx [m/s]",      IM_COL32(255, 120, 200, 255), /*group=*/2);
        cv_dy_live    = dash.add_curve("dy [m/s]",      IM_COL32(120, 255, 200, 255), /*group=*/3);
        cv_alpha_live = dash.add_curve("alpha [deg]",   IM_COL32(200, 200, 255, 255), /*group=*/4);
        cv_L_live     = dash.add_curve("spring L [m]",  IM_COL32(255, 255, 120, 255), /*group=*/5);
    }

    // ── 回调 ──
    dash.on_reset([&]{
        apply_params_to_sim();
        g_sim.reset(make_q0_world());
    });

    dash.on_step([&](double dt){
        g_sim.advance(dt);
        dash.push(cv_x_live,     (float)g_sim.x_world());
        dash.push(cv_y_live,     (float)g_sim.y_world());
        dash.push(cv_dx_live,    (float)g_sim.q(2));
        dash.push(cv_dy_live,    (float)g_sim.q(3));
        dash.push(cv_alpha_live, (float)(g_sim.q(4) * 180.0 / M_PI));
        dash.push(cv_L_live,     (float)g_sim.spring_len());
    });

    dash.on_sync([&](mjModel* m, mjData* d){
        double xb = g_sim.x_world();
        double zb = g_sim.y_world();
        Eigen::Vector2d f = g_sim.foot_world();

        // hopper：bx 沿 X，bz 沿 Z（model.xml 里 hopper 初始 (0,0,bz_init)）
        d->qpos[0] = xb - g_bx_init;     // bx
        d->qpos[1] = zb - g_bz_init;     // bz
        // foot：fx, fz 直接放到世界位
        d->qpos[2] = f.x();              // fx
        d->qpos[3] = f.y();              // fz
        d->qvel[0] = g_sim.q(2);
        d->qvel[1] = g_sim.q(3);
        d->qvel[2] = 0.0;
        d->qvel[3] = 0.0;
        mj_forward(m, d);
    });

    auto rerun_openloop = [&]{
        Sim2D ref = g_sim;
        ref.P.Kp_z = 0.0;
        auto segs = ref.run(make_q0_world());
        std::vector<float> ts, xs, ys;
        flatten_segments(segs, ts, xs, ys);
        dash.update_static_curve(cv_x_ref, ts, xs);
        dash.update_static_curve(cv_y_ref, ts, ys);
    };

    dash.on_controls([&]{
        ImGui::Text("phase = %s   fell = %s",
                    phase_name(g_sim.cur_phase),
                    g_sim.fell ? "true" : "false");
        ImGui::Text("x = %+.3f  y = %+.3f", g_sim.x_world(), g_sim.y_world());
        ImGui::Text("dx = %+.3f  dy = %+.3f", g_sim.q(2), g_sim.q(3));
        ImGui::Text("p0 = (%+.3f, %+.3f)   L = %.3f",
                    g_sim.p0.x(), g_sim.p0.y(), g_sim.spring_len());

        ImGui::Separator();
        ImGui::TextUnformatted("Spring & body");
        bool changed = false;
        changed |= sld("k [N/m]",   &g_P_ui.k,    50.0,  5000.0, "%.1f");
        changed |= sld("kd [Ns/m]", &g_P_ui.kd,   0.0,   20.0,   "%.2f");
        changed |= sld("l0 [m]",    &g_P_ui.l0,   0.3,   1.5,    "%.2f");
        changed |= sld("m [kg]",    &g_P_ui.m,    0.1,   5.0,    "%.2f");
        changed |= sld("alpha_TD [deg]", &g_alpha_TD_deg, 30.0, 150.0, "%.1f");

        ImGui::Separator();
        ImGui::TextUnformatted("Raibert height control");
        ImGui::Checkbox("Enable u = Kp_z*(l0+fly_z - y)", &g_raibert_on);
        changed |= sld("Kp_z",      &g_Kp_z_slider, 0.0, 2000.0, "%.1f");
        changed |= sld("fly_z [m]", &g_P_ui.fly_z, -0.3, 0.5,    "%.3f");

        ImGui::Separator();
        ImGui::TextUnformatted("Initial state (apply on Reset)");
        sld("x0 [m]",    &g_x0_w,  -2.0, 5.0,  "%.2f");
        sld("y0 [m]",    &g_y0_w,   0.3, 3.0,  "%.2f");
        sld("dx0 [m/s]", &g_dx0_w, -3.0, 3.0,  "%.2f");
        sld("dy0 [m/s]", &g_dy0_w, -3.0, 3.0,  "%.2f");

        ImGui::Separator();
        if (ImGui::Button("Reset")) {
            apply_params_to_sim();
            dash.request_reset();
        }
        ImGui::SameLine();
        if (ImGui::Button("Re-run open-loop reference")) {
            apply_params_to_sim();
            rerun_openloop();
        }
        if (changed) {
            apply_params_to_sim();    // 让在线仿真立刻吃到新参数
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "params updated");
        }
    });

    dash.on_status([&]{
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "t=%.2fs  x=%+.3f  y=%+.3f  dx=%+.2f  dy=%+.2f  phase=%s  L=%.3f  p0=(%+.2f,%+.2f)",
                      dash.sim_time(),
                      g_sim.x_world(), g_sim.y_world(),
                      g_sim.q(2),      g_sim.q(3),
                      phase_name(g_sim.cur_phase),
                      g_sim.spring_len(),
                      g_sim.p0.x(),    g_sim.p0.y());
        return std::string(buf);
    });

    return dash.run();
}
