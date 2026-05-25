// slip_2d_sim_ui.cpp —— 2D-SLIP 事件触发仿真 + Dashboard 集成（§1.3.2 崎岖地形扩展）
//
// 改动相对 §1.3.1：
//   - State 降为 4 维 [x,y,dx,dy]：IC 不再有 alpha
//   - 控制器与动力学解耦：g_sim.ctrl.{mode, Kp_z, fly_z}
//        Off          : Passive（u=0）
//        Simple-Kp    : u = Kp_z*(l0+fly_z - q.y)
//        Energy-Inject: u = en_out * Kp_z*(l0+fly_z - fly_reach_z + d_support_y)
//   - 地形可选 flat / staircase / random_discrete（+ seed），下拉框切换
//   - MJCF 预 bake 64 个 box tile，Reset 时 apply_terrain_to_model() 重画
//   - Scope 新增曲线：u, alpha_swing(deg), fly_reach_z，均带同色静态参考

#include "dashboard.hpp"
#include "slip_2d_sim.hpp"

#include <mujoco.h>

#include <cmath>
#include <cstdio>
#include <string>

using slip2d::Controller;
using slip2d::Params2D;
using slip2d::Phase;
using slip2d::Sim2D;
using slip2d::State;
using slip2d::Terrain;

namespace {

// ── 仿真状态（全局，被各 callback 共享） ─────────────────────────
Sim2D    g_sim;
Params2D g_P_ui;

// 控制器 UI 状态（独立于 sim.ctrl，方便切换 mode 时保留滑块值）
int      g_ctrl_mode_ui   = 0;             // 0=Off, 1=SimpleKp, 2=EnergyInject
double   g_Kp_z_slider    = 800.0;
double   g_fly_z_slider   = 0.0;
double   g_alpha_TD_deg   = 90.0;

// 地形 UI
int      g_terrain_mode   = 0;             // 0=flat, 1=staircase, 2=random
int      g_seed_slider    = 1;
double   g_stair_x_start  = 1.0;
double   g_stair_dh       = 0.05;
double   g_stair_dx       = 0.5;
double   g_stair_jitter   = 0.03;
int      g_stair_n        = 20;
double   g_rand_max_dh    = 0.05;
double   g_rand_plat_len  = 0.20;

// 初始条件
double g_x0_w   = 0.0;
double g_y0_w   = 1.5;
double g_dx0_w  = 0.5;
double g_dy0_w  = 0.0;

// MuJoCo body 初始位置（model.xml 里 hopper pos="0 0 1"）
double g_bx_init = 0.0;
double g_bz_init = 1.0;

// MJCF tile 名字 → id 缓存
constexpr int  kTileCount = 64;
int            g_tile_ids[kTileCount];

// 曲线 id
sim_dashboard::StaticCurveId cv_x_ref, cv_y_ref, cv_u_ref, cv_alpha_ref, cv_fz_ref;
sim_dashboard::CurveId       cv_x_live, cv_y_live;
sim_dashboard::CurveId       cv_dx_live, cv_dy_live;
sim_dashboard::CurveId       cv_alpha_live, cv_L_live;
sim_dashboard::CurveId       cv_u_live, cv_fz_live;

// ── 工具 ────────────────────────────────────────────────────────
State make_q0_world() {
    State q0;
    q0 << g_x0_w, g_y0_w, g_dx0_w, g_dy0_w;
    return q0;
}

Terrain build_terrain() {
    switch (g_terrain_mode) {
        case 1: return slip2d::make_staircase(
                    (uint32_t)g_seed_slider, -2.0, g_stair_x_start,
                    g_stair_dx, g_stair_dh, g_stair_jitter, g_stair_n);
        case 2: return slip2d::make_random_discrete(
                    (uint32_t)g_seed_slider, -2.0, 20.0,
                    g_rand_plat_len, g_rand_max_dh);
        default: return slip2d::make_flat();
    }
}

Controller::Mode mode_from_ui(int m) {
    switch (m) {
        case 1:  return Controller::Mode::SimpleKp;
        case 2:  return Controller::Mode::EnergyInject;
        default: return Controller::Mode::Passive;
    }
}

void apply_params_to_sim() {
    g_sim.P                 = g_P_ui;
    g_sim.ctrl.mode         = mode_from_ui(g_ctrl_mode_ui);
    g_sim.ctrl.Kp_z         = g_Kp_z_slider;
    g_sim.ctrl.fly_z        = g_fly_z_slider;
    g_sim.alpha_TD          = g_alpha_TD_deg * M_PI / 180.0;
    g_sim.alpha_prev_TD     = g_sim.alpha_TD;
}

// 把 Terrain 的 polyline 段贴到 64 个预 bake 的 MJCF box tile 上。
// 段切分：相邻两点 (x0,h0) - (x1,h1)，dx>edge_threshold 视为水平段，
// 否则 (竖直边)忽略——只画水平台面。
// 平地（所有水平段高度都为 0）由 MJCF 自带的 floor plane 直接显示，tile 全藏。
void apply_terrain_to_model(mjModel* m, const Terrain& terr) {
    if (!m) return;

    // 1) 全部藏起来：把 size 缩到接近 0 ⇒ 不会被相机看到，与位置无关
    //    （之前用 z=-10 的方案：默认相机角度仍能瞥到一小条灰片在地面下方）
    for (int i = 0; i < kTileCount; ++i) {
        int gid = g_tile_ids[i];
        if (gid < 0) continue;
        m->geom_pos [3*gid + 0] = 0.0;
        m->geom_pos [3*gid + 1] = 0.0;
        m->geom_pos [3*gid + 2] = -1000.0;
        m->geom_size[3*gid + 0] = 1e-5;
        m->geom_size[3*gid + 1] = 1e-5;
        m->geom_size[3*gid + 2] = 1e-5;
    }

    // 2) 遍历 polyline，把每个水平段填到下一个 tile：固定 2cm 厚薄板，顶面贴 h
    //    h≈0 的段交给 MJCF 自带 floor plane 渲染，避免 Z-fight。
    constexpr double kEdgeThresh    = 0.05;
    constexpr double kTileThickness = 0.02;
    constexpr double kHalfDz        = 0.5 * kTileThickness;
    int slot = 0;
    for (size_t i = 0; i + 1 < terr.profile.size() && slot < kTileCount; ++i) {
        double x0 = terr.profile[i].x(),   x1 = terr.profile[i+1].x();
        double y0 = terr.profile[i].y(),   y1 = terr.profile[i+1].y();
        double dx = x1 - x0;
        if (dx < kEdgeThresh) continue;
        double h = 0.5 * (y0 + y1);
        if (std::abs(h) < 1e-6) continue;     // 0 高度的段 ⇒ floor 已经显示
        double half_dx  = 0.5 * dx;
        double center_z = h - kHalfDz;        // 顶面正好贴 h
        int gid = g_tile_ids[slot++];
        if (gid < 0) continue;
        m->geom_pos [3*gid + 0] = 0.5 * (x0 + x1);
        m->geom_pos [3*gid + 1] = 0.0;
        m->geom_pos [3*gid + 2] = center_z;
        m->geom_size[3*gid + 0] = half_dx;
        m->geom_size[3*gid + 1] = 0.5;
        m->geom_size[3*gid + 2] = kHalfDz;
    }
}

const char* phase_name(Phase ph) {
    return ph == Phase::Flight ? "FLIGHT" : "STANCE";
}

bool sld(const char* label, double* v, double mn, double mx,
         const char* fmt = "%.3f") {
    return ImGui::SliderScalar(label, ImGuiDataType_Double, v, &mn, &mx, fmt);
}

// 把 segments 摊平为多条曲线（世界坐标）
struct FlatCurves {
    std::vector<float> ts;
    std::vector<float> xs, ys;
    std::vector<float> us;
    std::vector<float> alpha_deg;
    std::vector<float> fly_reach;
};

FlatCurves flatten_segments(const std::vector<slip2d::Segment>& segs) {
    FlatCurves c;
    for (const auto& seg : segs) {
        for (size_t i = 0; i < seg.t.size(); ++i) {
            c.ts.push_back((float)seg.t[i]);
            c.xs.push_back((float)(seg.support_xy.x() + seg.q[i](0)));
            c.ys.push_back((float)(seg.support_xy.y() + seg.q[i](1)));
            c.us.push_back((float)seg.u_out[i]);
            c.alpha_deg.push_back((float)(seg.q_swing[i] * 180.0 / M_PI));
            c.fly_reach.push_back((float)seg.fly_reach_z[i]);
        }
    }
    return c;
}

} // namespace

int main() {
    // ── Dashboard 配置 ──
    sim_dashboard::Config cfg;
    cfg.title           = "SLIP 2D — uneven terrain";
    cfg.model_path      = "model.xml";
    cfg.phys_dt         = 1e-3;
    cfg.scope_time_span = 4.0;

    sim_dashboard::Dashboard dash(cfg);
    if (!dash.model()) return 1;

    if (mjModel* m = dash.model()) {
        int id = mj_name2id(m, mjOBJ_BODY, "hopper");
        if (id >= 0) {
            g_bx_init = m->body_pos[3*id + 0];
            g_bz_init = m->body_pos[3*id + 2];
        }
        // 缓存 64 个 tile geom id
        for (int i = 0; i < kTileCount; ++i) {
            char buf[8]; std::snprintf(buf, sizeof(buf), "t%02d", i);
            g_tile_ids[i] = mj_name2id(m, mjOBJ_GEOM, buf);
        }
    }

    // ── 默认参数 ──
    g_P_ui.l0 = 1.0;
    g_P_ui.k  = 1500.0;
    g_P_ui.m  = 1.0;
    g_P_ui.kd = 1.0;
    g_P_ui.g  = 9.81;

    g_sim.terrain = build_terrain();
    g_sim.dt      = cfg.phys_dt;
    g_sim.t_max   = cfg.scope_time_span * 1.5;

    apply_params_to_sim();
    g_sim.reset(make_q0_world());
    apply_terrain_to_model(dash.model(), g_sim.terrain);

    // ── 一次开环 batch：作为静态参考曲线 ──
    auto register_curves_and_refs = [&]{
        Sim2D ref = g_sim;
        ref.ctrl.mode = Controller::Mode::Passive;
        auto segs = ref.run(make_q0_world());
        FlatCurves fc = flatten_segments(segs);

        cv_x_live = dash.add_curve(      "x [m]",  IM_COL32(255, 200,  80, 255), /*group=*/0);
        cv_x_ref  = dash.add_static_curve("x* [m]", IM_COL32(255, 200,  80, 160), fc.ts, fc.xs, 0);

        cv_y_live = dash.add_curve(      "y [m]",  IM_COL32( 80, 200, 255, 255), /*group=*/1);
        cv_y_ref  = dash.add_static_curve("y* [m]", IM_COL32( 80, 200, 255, 160), fc.ts, fc.ys, 1);

        cv_dx_live    = dash.add_curve("dx [m/s]",      IM_COL32(255, 120, 200, 255), /*group=*/2);
        cv_dy_live    = dash.add_curve("dy [m/s]",      IM_COL32(120, 255, 200, 255), /*group=*/3);
        cv_alpha_live = dash.add_curve("alpha_sw [deg]",IM_COL32(180, 220, 255, 255), /*group=*/4);
        cv_alpha_ref  = dash.add_static_curve("alpha_sw* [deg]", IM_COL32(180, 220, 255, 160),
                                              fc.ts, fc.alpha_deg, 4);
        cv_L_live     = dash.add_curve("spring L [m]",  IM_COL32(255, 255, 120, 255), /*group=*/5);

        cv_u_live = dash.add_curve("u [N]", IM_COL32(255, 200, 120, 255), /*group=*/6);
        cv_u_ref  = dash.add_static_curve("u* [N]", IM_COL32(255, 200, 120, 160),
                                           fc.ts, fc.us, 6);

        cv_fz_live = dash.add_curve("fly_reach_z [m]", IM_COL32(220, 160, 255, 255), /*group=*/7);
        cv_fz_ref  = dash.add_static_curve("fly_reach_z* [m]", IM_COL32(220, 160, 255, 160),
                                            fc.ts, fc.fly_reach, 7);
    };
    register_curves_and_refs();

    auto rerun_openloop = [&]{
        Sim2D ref = g_sim;
        ref.ctrl.mode = Controller::Mode::Passive;
        auto segs = ref.run(make_q0_world());
        FlatCurves fc = flatten_segments(segs);
        dash.update_static_curve(cv_x_ref,     fc.ts, fc.xs);
        dash.update_static_curve(cv_y_ref,     fc.ts, fc.ys);
        dash.update_static_curve(cv_u_ref,     fc.ts, fc.us);
        dash.update_static_curve(cv_alpha_ref, fc.ts, fc.alpha_deg);
        dash.update_static_curve(cv_fz_ref,    fc.ts, fc.fly_reach);
    };

    // ── 回调 ──
    dash.on_reset([&]{
        g_sim.terrain = build_terrain();
        apply_params_to_sim();
        g_sim.reset(make_q0_world());
        apply_terrain_to_model(dash.model(), g_sim.terrain);
    });

    dash.on_step([&](double dt){
        g_sim.advance(dt);
        dash.push(cv_x_live,     (float)g_sim.x_world());
        dash.push(cv_y_live,     (float)g_sim.y_world());
        dash.push(cv_dx_live,    (float)g_sim.q(2));
        dash.push(cv_dy_live,    (float)g_sim.q(3));
        dash.push(cv_alpha_live, (float)(g_sim.swing_angle() * 180.0 / M_PI));
        dash.push(cv_L_live,     (float)g_sim.spring_len());
        dash.push(cv_u_live,     (float)g_sim.last_u_out);
        dash.push(cv_fz_live,    (float)g_sim.ctx.fly_reach_z);
    });

    dash.on_sync([&](mjModel* m, mjData* d){
        double xb = g_sim.x_world();
        double zb = g_sim.y_world();
        Eigen::Vector2d f = g_sim.foot_world();

        d->qpos[0] = xb - g_bx_init;
        d->qpos[1] = zb - g_bz_init;
        d->qpos[2] = f.x();
        d->qpos[3] = f.y();
        d->qvel[0] = g_sim.q(2);
        d->qvel[1] = g_sim.q(3);
        d->qvel[2] = 0.0;
        d->qvel[3] = 0.0;
        mj_forward(m, d);
    });

    dash.on_controls([&]{
        ImGui::Text("phase = %s   fell = %s",
                    phase_name(g_sim.cur_phase),
                    g_sim.fell ? "true" : "false");
        ImGui::Text("x = %+.3f  y = %+.3f", g_sim.x_world(), g_sim.y_world());
        ImGui::Text("dx = %+.3f  dy = %+.3f", g_sim.q(2), g_sim.q(3));
        ImGui::Text("p0 = (%+.3f, %+.3f)   L = %.3f",
                    g_sim.p0.x(), g_sim.p0.y(), g_sim.spring_len());
        ImGui::Text("en_out = %.0f   fly_reach_z = %+.3f   d_sup_y = %+.3f",
                    g_sim.ctx.en_out, g_sim.ctx.fly_reach_z, g_sim.ctx.d_support_y);

        ImGui::Separator();
        ImGui::TextUnformatted("Terrain");
        const char* terrain_items[] = { "flat", "staircase", "random discrete" };
        bool terrain_changed = ImGui::Combo("type", &g_terrain_mode,
                                            terrain_items, IM_ARRAYSIZE(terrain_items));
        if (g_terrain_mode == 1) {
            int s = g_seed_slider;
            if (ImGui::SliderInt("seed", &s, 1, 200)) { g_seed_slider = s; terrain_changed = true; }
            terrain_changed |= sld("x_start [m]",  &g_stair_x_start, -1.0, 5.0, "%.2f");
            terrain_changed |= sld("step dh [m]",  &g_stair_dh,    -0.10, 0.10, "%.3f");
            terrain_changed |= sld("jitter [m]",   &g_stair_jitter,  0.0, 0.10, "%.3f");
            terrain_changed |= sld("step dx [m]",  &g_stair_dx,     0.20, 1.50, "%.2f");
            int n = g_stair_n;
            if (ImGui::SliderInt("n_steps", &n, 1, 30)) { g_stair_n = n; terrain_changed = true; }
        } else if (g_terrain_mode == 2) {
            int s = g_seed_slider;
            if (ImGui::SliderInt("seed", &s, 1, 200)) { g_seed_slider = s; terrain_changed = true; }
            terrain_changed |= sld("max dh [m]",   &g_rand_max_dh,   0.0, 0.20, "%.3f");
            terrain_changed |= sld("plat len [m]", &g_rand_plat_len, 0.10, 0.80, "%.2f");
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Spring & body");
        bool changed = false;
        changed |= sld("k [N/m]",   &g_P_ui.k,    50.0,  5000.0, "%.1f");
        changed |= sld("kd [Ns/m]", &g_P_ui.kd,   0.0,   20.0,   "%.2f");
        changed |= sld("l0 [m]",    &g_P_ui.l0,   0.3,   1.5,    "%.2f");
        changed |= sld("m [kg]",    &g_P_ui.m,    0.1,   5.0,    "%.2f");
        changed |= sld("alpha_TD [deg]", &g_alpha_TD_deg, 30.0, 150.0, "%.1f");

        ImGui::Separator();
        ImGui::TextUnformatted("Height controller");
        ImGui::RadioButton("Off",            &g_ctrl_mode_ui, 0); ImGui::SameLine();
        ImGui::RadioButton("Simple-Kp",      &g_ctrl_mode_ui, 1); ImGui::SameLine();
        ImGui::RadioButton("Energy-Inject",  &g_ctrl_mode_ui, 2);
        changed |= sld("Kp_z",     &g_Kp_z_slider,  0.0, 3000.0, "%.1f");
        changed |= sld("fly_z [m]",&g_fly_z_slider,-0.3, 0.5,    "%.3f");

        ImGui::Separator();
        ImGui::TextUnformatted("Initial state (apply on Reset)");
        sld("x0 [m]",    &g_x0_w,  -2.0, 5.0,  "%.2f");
        sld("y0 [m]",    &g_y0_w,   0.3, 3.0,  "%.2f");
        sld("dx0 [m/s]", &g_dx0_w, -3.0, 3.0,  "%.2f");
        sld("dy0 [m/s]", &g_dy0_w, -3.0, 3.0,  "%.2f");

        ImGui::Separator();
        if (ImGui::Button("Reset")) {
            dash.request_reset();
        }
        ImGui::SameLine();
        if (ImGui::Button("Re-run open-loop ref")) {
            apply_params_to_sim();
            rerun_openloop();
        }
        if (terrain_changed) {
            g_sim.terrain = build_terrain();
            apply_terrain_to_model(dash.model(), g_sim.terrain);
            rerun_openloop();
        }
        if (changed) {
            apply_params_to_sim();
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "params updated");
        }
    });

    dash.on_status([&]{
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "t=%.2fs  x=%+.3f y=%+.3f  dx=%+.2f dy=%+.2f  phase=%s  L=%.3f  p0=(%+.2f,%+.2f)  u=%+.1f  fr_z=%+.2f",
                      dash.sim_time(),
                      g_sim.x_world(), g_sim.y_world(),
                      g_sim.q(2),      g_sim.q(3),
                      phase_name(g_sim.cur_phase),
                      g_sim.spring_len(),
                      g_sim.p0.x(),    g_sim.p0.y(),
                      g_sim.last_u_out, g_sim.ctx.fly_reach_z);
        return std::string(buf);
    });

    return dash.run();
}
