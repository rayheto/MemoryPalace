// simulation_dashboard/dashboard.cpp
//
// MuJoCo + ImGui + ImPlot 仿真面板外壳，详见 dashboard.hpp。
//
// 典型用法：
//   sim_dashboard::Config cfg;
//   cfg.title = "My sim"; cfg.model_path = "model.xml";
//   sim_dashboard::Dashboard dash(cfg);
//   auto cv = dash.add_curve("x", IM_COL32(255,200,80,255));
//   dash.on_reset   ([&]{ /* 复位自己的状态 */ });
//   dash.on_step    ([&](double dt){ /* 推进 + dash.push(cv, ...) */ });
//   dash.on_sync    ([&](mjModel* m, mjData* d){ /* 写 qpos + mj_forward */ });
//   dash.on_controls([&]{ /* ImGui::Slider... */ });
//   dash.on_status  ([&]{ return std::string("..."); });
//   return dash.run();

#include "dashboard.hpp"

#include <mujoco.h>
#include <glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace sim_dashboard {

// ── 内部结构 ────────────────────────────────────────────────────────
struct Curve {
    std::string label;
    ImU32       color;
    int         group = -1;     // -1 = 独占子图；>=0 = 同 group 合并叠加
    std::vector<float> y;
};

struct StaticCurve {
    std::string label;
    ImU32       color;
    int         group = -1;
    std::vector<float> ts;
    std::vector<float> vs;
};

struct DashboardImpl {
    Config cfg;

    // MuJoCo
    mjModel*   m = nullptr;
    mjData*    d = nullptr;
    mjvCamera  cam{};
    mjvOption  opt{};
    mjvScene   scn{};
    mjrContext con{};

    // GLFW
    GLFWwindow* win = nullptr;

    // 输入与运行状态
    bool   button_left = false, button_middle = false, button_right = false;
    double lastx = 0.0, lasty = 0.0;
    bool   paused      = false;
    bool   want_reset  = false;
    int    plot_h      = 320;

    // 时间 / 环形缓冲
    double             sim_time = 0.0;
    int                next   = 0;
    int                filled = 0;
    std::vector<float> t_buf;
    std::vector<Curve>       curves;
    std::vector<StaticCurve> static_curves;

    // 回调
    std::function<void(double)>            cb_step;
    std::function<void()>                  cb_reset;
    std::function<void(mjModel*, mjData*)> cb_sync;
    std::function<void()>                  cb_controls;
    std::function<std::string()>           cb_status;
};

// ── GLFW 回调（通过 WindowUserPointer 拿回 Impl） ──────────────────
static void on_key(GLFWwindow* w, int key, int /*sc*/, int act, int /*mods*/) {
    auto* I = static_cast<DashboardImpl*>(glfwGetWindowUserPointer(w));
    if (!I || ImGui::GetIO().WantCaptureKeyboard) return;
    if (act != GLFW_PRESS) return;
    if      (key == GLFW_KEY_BACKSPACE) I->want_reset = true;
    else if (key == GLFW_KEY_SPACE)     I->paused     = !I->paused;
    else if (key == GLFW_KEY_ESCAPE)    glfwSetWindowShouldClose(w, GLFW_TRUE);
}

static void on_mouse_button(GLFWwindow* w, int /*b*/, int /*act*/, int /*mods*/) {
    auto* I = static_cast<DashboardImpl*>(glfwGetWindowUserPointer(w));
    if (!I) return;
    if (ImGui::GetIO().WantCaptureMouse) {
        I->button_left = I->button_middle = I->button_right = false;
        return;
    }
    I->button_left   = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    I->button_middle = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    I->button_right  = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    glfwGetCursorPos(w, &I->lastx, &I->lasty);
}

static void on_mouse_move(GLFWwindow* w, double xpos, double ypos) {
    auto* I = static_cast<DashboardImpl*>(glfwGetWindowUserPointer(w));
    if (!I) return;
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (!I->button_left && !I->button_middle && !I->button_right) return;
    double dx = xpos - I->lastx, dy = ypos - I->lasty;
    I->lastx = xpos; I->lasty = ypos;
    int width, height; glfwGetWindowSize(w, &width, &height);
    bool shift = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
                  glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if      (I->button_right)  action = shift ? mjMOUSE_MOVE_H   : mjMOUSE_MOVE_V;
    else if (I->button_left)   action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else                       action = mjMOUSE_ZOOM;
    mjv_moveCamera(I->m, action, dx/height, dy/height, &I->scn, &I->cam);
}

static void on_scroll(GLFWwindow* w, double /*xoff*/, double yoff) {
    auto* I = static_cast<DashboardImpl*>(glfwGetWindowUserPointer(w));
    if (!I || ImGui::GetIO().WantCaptureMouse) return;
    mjv_moveCamera(I->m, mjMOUSE_ZOOM, 0, -0.05 * yoff, &I->scn, &I->cam);
}

// ── 复位 ────────────────────────────────────────────────────────────
static void do_reset(DashboardImpl& I) {
    I.sim_time = 0.0;
    I.next   = 0;
    I.filled = 0;
    if (I.cb_reset) I.cb_reset();
    if (I.d) I.d->time = 0.0;
    if (I.cb_sync && I.m && I.d) I.cb_sync(I.m, I.d);
}

// ── Dashboard 接口 ─────────────────────────────────────────────────
Dashboard::Dashboard(const Config& cfg) : impl_(new DashboardImpl) {
    impl_->cfg    = cfg;
    impl_->plot_h = cfg.default_plot_h;
    impl_->t_buf.assign(cfg.history_capacity, 0.0f);

    char err[1024] = {0};
    impl_->m = mj_loadXML(cfg.model_path.c_str(), nullptr, err, sizeof(err));
    if (!impl_->m) {
        fprintf(stderr, "[dashboard] mj_loadXML failed: %s\n", err);
        return;
    }
    impl_->d = mj_makeData(impl_->m);
}

Dashboard::~Dashboard() {
    if (impl_->d) { mj_deleteData(impl_->d);  impl_->d = nullptr; }
    if (impl_->m) { mj_deleteModel(impl_->m); impl_->m = nullptr; }
}

mjModel*   Dashboard::model()      { return impl_->m; }
mjData*    Dashboard::data()       { return impl_->d; }
double     Dashboard::sim_time() const { return impl_->sim_time; }
bool       Dashboard::paused()   const { return impl_->paused; }
mjvCamera* Dashboard::camera()     { return &impl_->cam; }
void Dashboard::request_reset()    { impl_->want_reset = true; }
void Dashboard::set_paused(bool p) { impl_->paused = p; }

void Dashboard::on_step    (std::function<void(double)> fn)              { impl_->cb_step     = std::move(fn); }
void Dashboard::on_reset   (std::function<void()> fn)                    { impl_->cb_reset    = std::move(fn); }
void Dashboard::on_sync    (std::function<void(mjModel*, mjData*)> fn)   { impl_->cb_sync     = std::move(fn); }
void Dashboard::on_controls(std::function<void()> fn)                    { impl_->cb_controls = std::move(fn); }
void Dashboard::on_status  (std::function<std::string()> fn)             { impl_->cb_status   = std::move(fn); }

CurveId Dashboard::add_curve(const std::string& label, ImU32 color, int group) {
    Curve c;
    c.label = label;
    c.color = color;
    c.group = group;
    c.y.assign(impl_->cfg.history_capacity, 0.0f);
    impl_->curves.push_back(std::move(c));
    CurveId id; id.idx = (int)impl_->curves.size() - 1;
    return id;
}

void Dashboard::push(CurveId id, float value) {
    if (id.idx < 0 || id.idx >= (int)impl_->curves.size()) return;
    impl_->curves[id.idx].y[impl_->next] = value;
}

StaticCurveId Dashboard::add_static_curve(const std::string& label, ImU32 color,
                                          std::vector<float> ts, std::vector<float> vs,
                                          int group) {
    StaticCurve c;
    c.label = label;
    c.color = color;
    c.group = group;
    c.ts    = std::move(ts);
    c.vs    = std::move(vs);
    impl_->static_curves.push_back(std::move(c));
    StaticCurveId id; id.idx = (int)impl_->static_curves.size() - 1;
    return id;
}

void Dashboard::update_static_curve(StaticCurveId id,
                                     std::vector<float> ts, std::vector<float> vs) {
    if (id.idx < 0 || id.idx >= (int)impl_->static_curves.size()) return;
    impl_->static_curves[id.idx].ts = std::move(ts);
    impl_->static_curves[id.idx].vs = std::move(vs);
}

// ── 主循环 ──────────────────────────────────────────────────────────
int Dashboard::run() {
    auto* I = impl_.get();
    if (!I->m || !I->d) return 1;

    if (!glfwInit()) {
        fprintf(stderr, "[dashboard] glfwInit failed\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    I->win = glfwCreateWindow(I->cfg.window_w, I->cfg.window_h,
                              I->cfg.title.c_str(), nullptr, nullptr);
    if (!I->win) {
        fprintf(stderr, "[dashboard] glfwCreateWindow failed (DISPLAY OK?)\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(I->win);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(I->win, I);

    mjv_defaultCamera (&I->cam);
    mjv_defaultOption (&I->opt);
    mjv_defaultScene  (&I->scn);
    mjr_defaultContext(&I->con);
    mjv_makeScene  (I->m, &I->scn, 2000);
    mjr_makeContext(I->m, &I->con, mjFONTSCALE_150);

    // 默认相机（调用方可在 add_curve / on_* 之间通过 dash.camera() 改）
    if (I->cam.distance == 0.0) {
        I->cam.distance  = 1.8;
        I->cam.azimuth   = 90;
        I->cam.elevation = -15;
        I->cam.lookat[0] = 0; I->cam.lookat[1] = 0; I->cam.lookat[2] = 0.4;
    }

    // 我们的回调必须在 ImGui_ImplGlfw_InitForOpenGL 之前注册，ImGui 会链式调用
    glfwSetKeyCallback        (I->win, on_key);
    glfwSetCursorPosCallback  (I->win, on_mouse_move);
    glfwSetMouseButtonCallback(I->win, on_mouse_button);
    glfwSetScrollCallback     (I->win, on_scroll);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(I->win, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    do_reset(*I);

    double wall_prev = glfwGetTime();
    while (!glfwWindowShouldClose(I->win)) {
        double wall_now = glfwGetTime();
        double dt_wall  = wall_now - wall_prev;
        wall_prev = wall_now;
        if (dt_wall > 0.1) dt_wall = 0.1;

        // ── 物理子步 ──
        if (!I->paused) {
            int n_sub = std::max(1, (int)std::round(dt_wall / I->cfg.phys_dt));
            for (int i = 0; i < n_sub; ++i) {
                if (I->cb_step) I->cb_step(I->cfg.phys_dt);
                I->sim_time += I->cfg.phys_dt;
                I->t_buf[I->next] = (float)I->sim_time;
                I->next = (I->next + 1) % I->cfg.history_capacity;
                if (I->filled < I->cfg.history_capacity) I->filled++;
            }
            if (I->cb_sync) I->cb_sync(I->m, I->d);
            if (I->d) I->d->time = I->sim_time;
        }

        if (I->want_reset) {
            I->want_reset = false;
            do_reset(*I);
        }

        // ── 布局 ──
        int fb_w, fb_h;
        glfwGetFramebufferSize(I->win, &fb_w, &fb_h);
        const int panel_w = I->cfg.side_panel_w;
        const int split_h = I->cfg.splitter_h;
        I->plot_h = std::max(80, std::min(I->plot_h, fb_h - 120));

        glViewport(0, 0, fb_w, fb_h);
        glClearColor(0.10f, 0.10f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // MuJoCo 视口（左上）
        const int view_bottom = I->plot_h + split_h;
        mjrRect viewport = {0, view_bottom, fb_w - panel_w, fb_h - view_bottom};
        mjv_updateScene(I->m, I->d, &I->opt, nullptr, &I->cam, mjCAT_ALL, &I->scn);
        mjr_render(viewport, &I->scn, &I->con);

        if (I->cb_status) {
            std::string s = I->cb_status();
            if (!s.empty())
                mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, s.c_str(), nullptr, &I->con);
        }

        // ── ImGui 帧 ──
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 右侧 Controls 面板
        ImGui::SetNextWindowPos (ImVec2((float)(fb_w - panel_w), 0),         ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2((float)panel_w,          (float)fb_h), ImGuiCond_Always);
        ImGui::Begin("Controls", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

        ImGui::Text("t = %.3f s", I->sim_time);
        if (ImGui::Button(I->paused ? "Resume (Space)" : "Pause (Space)")) I->paused = !I->paused;
        ImGui::SameLine();
        if (ImGui::Button("Reset (BS)")) I->want_reset = true;
        ImGui::Separator();

        if (I->cb_controls) I->cb_controls();

        ImGui::Separator();
        ImGui::Text("FPS: %.0f", ImGui::GetIO().Framerate);
        ImGui::End();

        // 分隔条
        ImGui::SetNextWindowPos (ImVec2(0, (float)(fb_h - I->plot_h - split_h)),     ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2((float)(fb_w - panel_w), (float)split_h),    ImGuiCond_Always);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,    ImVec2(0, 0));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin("##splitter", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBringToFrontOnFocus);
        ImGui::InvisibleButton("##split_btn",
                               ImVec2((float)(fb_w - panel_w), (float)split_h));
        if (ImGui::IsItemHovered() || ImGui::IsItemActive())
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        if (ImGui::IsItemActive())
            I->plot_h -= (int)ImGui::GetIO().MouseDelta.y;
        {
            ImU32 col = (ImGui::IsItemActive())  ? IM_COL32(120, 180, 255, 220) :
                        (ImGui::IsItemHovered()) ? IM_COL32( 90, 130, 200, 180) :
                                                   IM_COL32( 60,  60,  72, 255);
            ImVec2 p0 = ImGui::GetItemRectMin();
            ImVec2 p1 = ImGui::GetItemRectMax();
            ImGui::GetWindowDrawList()->AddRectFilled(p0, p1, col);
        }
        ImGui::End();
        ImGui::PopStyleVar(2);

        // 底部 Scope
        ImGui::SetNextWindowPos (ImVec2(0, (float)(fb_h - I->plot_h)),                   ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2((float)(fb_w - panel_w), (float)I->plot_h),      ImGuiCond_Always);
        ImGui::Begin("Scope", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

        // 把曲线按 group 划成若干 subplot：
        //   group <  0 : 该条曲线独占一个 subplot（保持旧行为）
        //   group >= 0 : 所有相同 group 的 live + static 曲线合并到一个 subplot
        struct SubplotSlot {
            std::vector<int> live;   // 索引进入 I->curves
            std::vector<int> stat;   // 索引进入 I->static_curves
            std::string ylabel;
        };
        std::vector<SubplotSlot> slots;
        // group_id -> slot index（仅记录 group >= 0 的位置）
        // 用 vector<pair> 替代 map，避免引头文件
        std::vector<std::pair<int,int>> group_to_slot;
        auto find_slot = [&](int g) -> int {
            for (auto& kv : group_to_slot) if (kv.first == g) return kv.second;
            return -1;
        };
        for (int i = 0; i < (int)I->curves.size(); ++i) {
            const Curve& c = I->curves[i];
            int slot_idx;
            if (c.group < 0) {
                slots.emplace_back();
                slot_idx = (int)slots.size() - 1;
                slots[slot_idx].ylabel = c.label;
            } else {
                slot_idx = find_slot(c.group);
                if (slot_idx < 0) {
                    slots.emplace_back();
                    slot_idx = (int)slots.size() - 1;
                    group_to_slot.push_back({c.group, slot_idx});
                    slots[slot_idx].ylabel = c.label;
                }
            }
            slots[slot_idx].live.push_back(i);
        }
        for (int j = 0; j < (int)I->static_curves.size(); ++j) {
            const StaticCurve& c = I->static_curves[j];
            int slot_idx;
            if (c.group < 0) {
                slots.emplace_back();
                slot_idx = (int)slots.size() - 1;
                slots[slot_idx].ylabel = c.label;
            } else {
                slot_idx = find_slot(c.group);
                if (slot_idx < 0) {
                    slots.emplace_back();
                    slot_idx = (int)slots.size() - 1;
                    group_to_slot.push_back({c.group, slot_idx});
                    slots[slot_idx].ylabel = c.label;
                }
            }
            slots[slot_idx].stat.push_back(j);
        }

        const int n_slots = (int)slots.size();
        if (n_slots > 0) {
            const float avail_h = ImGui::GetContentRegionAvail().y;
            const ImVec2 plot_size(-1, (avail_h - 4.0f * n_slots) / (float)n_slots);
            const double t_lo = std::max(0.0, I->sim_time - I->cfg.scope_time_span);
            const int    cnt  = I->filled;
            const int    off  = I->filled < I->cfg.history_capacity ? 0 : I->next;

            for (int s = 0; s < n_slots; ++s) {
                SubplotSlot& sl = slots[s];
                char id[32]; std::snprintf(id, sizeof(id), "##scope%d", s);
                if (ImPlot::BeginPlot(id, plot_size,
                                      ImPlotFlags_NoTitle | ImPlotFlags_NoMenus |
                                      ImPlotFlags_NoMouseText)) {
                    ImPlot::SetupAxes("t [s]", sl.ylabel.c_str(),
                                      ImPlotAxisFlags_None,
                                      ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupAxisLimits(ImAxis_X1, t_lo, I->sim_time, ImGuiCond_Always);
                    // live curves
                    if (cnt > 0) {
                        for (int li : sl.live) {
                            Curve& c = I->curves[li];
                            ImPlot::PushStyleColor(ImPlotCol_Line, c.color);
                            ImPlot::PlotLine(c.label.c_str(),
                                             I->t_buf.data(), c.y.data(),
                                             cnt, 0, off, sizeof(float));
                            ImPlot::PopStyleColor();
                        }
                    }
                    // static reference curves（虚线感：偏淡颜色由调用方传入）
                    for (int si : sl.stat) {
                        StaticCurve& c = I->static_curves[si];
                        if (c.ts.size() != c.vs.size() || c.ts.empty()) continue;
                        ImPlot::PushStyleColor(ImPlotCol_Line, c.color);
                        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
                        ImPlot::PlotLine(c.label.c_str(),
                                         c.ts.data(), c.vs.data(),
                                         (int)c.ts.size());
                        ImPlot::PopStyleVar();
                        ImPlot::PopStyleColor();
                    }
                    ImPlot::EndPlot();
                }
            }
        }
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(I->win);
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    mjv_freeScene(&I->scn);
    mjr_freeContext(&I->con);
    glfwTerminate();
    return 0;
}

} // namespace sim_dashboard
