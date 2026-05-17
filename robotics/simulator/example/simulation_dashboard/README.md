# simulation_dashboard

通用的 MuJoCo + ImGui + ImPlot 仿真面板外壳，把「窗口 / 相机 / Controls 面板 / 可拖动分隔条 / Scope 多曲线」这套样板抽出来，给所有 example 复用。业务侧只写动力学 + 几个 lambda 就能跑完整的调参 UI。

## 布局

```
┌────────────────────────────────────┬──────────────┐
│                                    │  Controls    │
│        MuJoCo 视口                 │  t / Pause   │
│   左键旋转 / 右键平移 / 滚轮缩放   │  Reset       │
│   (mjr_overlay 显示 status 字符串) │  ── 用户控件 │
│                                    │     ──       │
├──── ↕ 拖动分隔条改 Scope 高度 ─────┤  FPS         │
│                                    │              │
│  Scope（每条曲线一张子图）         │              │
│   ── curve 0                       │              │
│   ── curve 1                       │              │
│   ── ...                           │              │
└────────────────────────────────────┴──────────────┘
```

- 右侧 `side_panel_w`（默认 360 px）、底部 `default_plot_h`（默认 320 px，可拖动）、分隔条 `splitter_h`（6 px）
- Scope X 轴始终滚动到最新 `scope_time_span`（默认 5 s）
- 曲线环形缓冲 `history_capacity`（默认 6000 = 6 s @ 1 kHz）

## API（[dashboard.hpp](dashboard.hpp)）

```cpp
sim_dashboard::Config cfg;
cfg.title      = "My sim";
cfg.model_path = "model.xml";        // 相对 CWD 或绝对路径
sim_dashboard::Dashboard dash(cfg);

// 注册曲线（颜色用 IM_COL32）
auto cv = dash.add_curve("x [m]", IM_COL32(255, 200, 80, 255));

// 业务回调
dash.on_reset   ([&]{ /* 把自己的状态复位 */ });
dash.on_step    ([&](double dt){
    /* 推进一个物理子步，然后 dash.push(cv, ...) */
});
dash.on_sync    ([&](mjModel* m, mjData* d){
    /* 把状态写到 d->qpos / qvel，调 mj_forward 让 MuJoCo 渲染 */
});
dash.on_controls([&]{
    /* ImGui::Slider... 等自定义控件，画在 Controls 面板里 */
});
dash.on_status  ([&]{ return std::string("t=... x=..."); });

return dash.run();   // 阻塞主循环；ESC / 关窗退出
```

| 钩子 | 何时调用 | 典型用途 |
|------|----------|---------|
| `on_reset`    | 启动一次 + 用户按 Reset / Backspace | 把内部状态向量回到初始值 |
| `on_step`     | 每个物理子步（`cfg.phys_dt` 一次） | 调你自己的积分器 + `dash.push` |
| `on_sync`     | 每帧物理结束后一次 | 写 `qpos/qvel` + `mj_forward` |
| `on_controls` | 每帧 ImGui 阶段 | 在 Controls 面板里加滑块/按钮 |
| `on_status`   | 每帧 MuJoCo 渲染后 | 视口左上角的状态文字 |

## 内置交互

| 输入 | 行为 |
|------|------|
| 鼠标悬停 3D 视口 → 左键拖拽 | 旋转相机 |
| 右键拖拽 | 平移相机 |
| 滚轮 | 缩放 |
| 鼠标移到 Controls / Scope | 不会触发相机操作（`io.WantCaptureMouse` 拦截） |
| `Space` | 暂停 / 继续 |
| `Backspace` | 复位（触发 `on_reset`） |
| `ESC` | 退出 |
| 拖动 MuJoCo 与 Scope 之间的分隔条 | 调整 Scope 高度 |

## 在 CMake 里接入

父级（`example/CMakeLists.txt`）已经 `add_subdirectory(simulation_dashboard)`，子项目里直接：

```cmake
add_executable(my_example my_example.cpp)
target_link_libraries(my_example PRIVATE simulation_dashboard)
```

`simulation_dashboard` 的 PUBLIC 链接已经把 `imgui_backend / mujoco::* / OpenGL::GL` 全带上了，不用重复声明。

## 限制 / TODO

- 单实例：当前用 `glfwSetWindowUserPointer` 把 Impl 绑到唯一 GLFW 窗口；多 Dashboard 实例需要再扩展
- 控件区域大小固定：右侧宽度 / 分隔条高度走 `Config`，运行时不可改
- 曲线 X 共享 `sim_time`，目前不支持每条曲线自己的时间轴
- 没有内置「保存曲线到 CSV」按钮；想要的话在 `on_controls` 里加个按钮，自己读 `dash` 内部 buffer 即可

## 参考实现

[../slip_1d/hopper_sim_ui.cpp](../slip_1d/hopper_sim_ui.cpp) —— 1D SLIP 跳跃器，约 100 行业务代码（动力学 + 滑块 + 三条曲线）。
