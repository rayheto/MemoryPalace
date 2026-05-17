// simulation_dashboard/dashboard.hpp
//
// 通用的 MuJoCo + ImGui + ImPlot 仿真面板外壳：
//   - 左上：MuJoCo 视口（鼠标控制相机）
//   - 右侧：Controls 面板（自带 Pause/Reset/FPS，业务侧填自定义控件）
//   - 底部：Scope 面板（用户注册曲线，自动滚动 X 轴）
//   - 中间分隔条：可拖动调节 Scope 高度
//
// 用法见同目录下 dashboard.cpp 顶部注释，或 slip_1d/hopper_sim_ui.cpp。
//
// 当前实现支持单 Dashboard 实例 + 一个 GLFW 窗口（多实例需要再扩展）。

#pragma once

#include <mujoco.h>
#include <imgui.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace sim_dashboard {

struct Config {
    std::string title           = "Simulation";
    std::string model_path      = "model.xml";   // 相对 CWD 或绝对路径
    int    window_w             = 1400;
    int    window_h             = 900;
    int    side_panel_w         = 360;           // 右侧 Controls 面板宽度
    int    default_plot_h       = 320;           // 底部 Scope 初始高度
    int    splitter_h           = 6;             // 分隔条高度
    double phys_dt              = 0.001;         // 物理子步 dt [s]
    int    history_capacity     = 6000;          // 曲线环形缓冲样本数
    double scope_time_span      = 5.0;           // Scope X 轴滚动窗口 [s]
};

struct CurveId       { int idx = -1; };
struct StaticCurveId { int idx = -1; };

struct DashboardImpl;

class Dashboard {
public:
    explicit Dashboard(const Config& cfg);
    ~Dashboard();

    Dashboard(const Dashboard&) = delete;
    Dashboard& operator=(const Dashboard&) = delete;

    // ── 曲线 ────────────────────────────────────────────────────────
    // 注册一条滚动曲线，返回 id；在 on_step 回调里用 push(id, value) 推数据。
    // 颜色用 IM_COL32(r,g,b,a) 构造。
    // group < 0（默认）：该曲线独占一个子图（旧行为）。
    // group >= 0：所有相同 group 的曲线（含 static_curve）合并到同一个子图叠加显示。
    CurveId add_curve(const std::string& label, ImU32 color, int group = -1);
    void    push(CurveId id, float value);

    // 注册一条「静态」参考曲线：一次性给出 (t, v) 数组，整体绘制。
    // 用于把离线计算（轨迹优化结果）叠到在线 scope 上。
    // ts / vs 必须等长。group 含义同 add_curve。
    StaticCurveId add_static_curve(const std::string& label, ImU32 color,
                                   std::vector<float> ts, std::vector<float> vs,
                                   int group = -1);

    // 重新设置某条 static_curve 的数据（用于「Re-optimize」按钮等）。
    void update_static_curve(StaticCurveId id,
                             std::vector<float> ts, std::vector<float> vs);

    // ── 回调（默认全部为空，按需注册） ───────────────────────────────
    //
    // step：物理子步推进，每个 cfg.phys_dt 调一次。
    //       在这里更新自己的状态，并 push 曲线数据。
    void on_step    (std::function<void(double dt)> fn);

    // reset：复位仿真状态时调用（启动一次 + 用户按 Reset / Backspace 时）。
    //        sim_time 已被外部清零。
    void on_reset   (std::function<void()> fn);

    // sync：每帧物理推进结束后调用一次，业务侧把状态写到 d->qpos/qvel
    //       并调用 mj_forward 把运动学算出来给 MuJoCo 渲染。
    void on_sync    (std::function<void(mjModel*, mjData*)> fn);

    // controls：在右侧 Controls 面板里画自定义 ImGui 控件（在自带的
    //           Pause/Reset 按钮和 FPS 之间）。
    void on_controls(std::function<void()> fn);

    // status：返回叠在 MuJoCo 视口左上角的状态字符串（一般是 t / x / phase）。
    void on_status  (std::function<std::string()> fn);

    // ── 访问内部状态 ────────────────────────────────────────────────
    mjModel* model();
    mjData*  data();
    double   sim_time() const;
    bool     paused()   const;
    mjvCamera* camera();   // 想初始化相机方位/距离时用

    // 请求复位（等同于按 Backspace 或 Reset 按钮）。
    // 下一帧 dash 会清零 sim_time、清环形缓冲，再回调 on_reset。
    void request_reset();

    // 外部设置暂停态（业务侧可用来「跑完一圈后停住等用户按空格」）。
    // 用户随时按 Space / 点 Resume 仍可切回。
    void set_paused(bool p);

    // 主循环；返回 0 = 正常退出
    int run();

private:
    std::unique_ptr<DashboardImpl> impl_;
};

} // namespace sim_dashboard
