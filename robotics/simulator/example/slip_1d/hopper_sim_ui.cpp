// hopper_sim_ui.cpp —— 1D SLIP 跳跃器查看器
//   动力学由 sli_1d_model.hpp 中的 cartPoleDynamics 提供（自写积分）
//   MuJoCo 仅作可视化：mj_forward 只跑前向运动学，不跑物理
//
// 鼠标 / 键盘：
//   左键拖拽 = 旋转视角     右键拖拽 = 平移视角     滚轮 = 缩放
//   Backspace = 重置仿真     Space = 暂停/继续     ESC = 退出

#include <mujoco.h>
#include <glfw3.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "slip_1d_model.hpp"

mjModel* m = nullptr;
mjData*  d = nullptr;
mjvCamera  cam;
mjvOption  opt;
mjvScene   scn;
mjrContext con;

bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;
bool paused = false;

// ── 初始状态 ────────────────────────────────────────────────────────
static constexpr double X0  = 2.0;   // 初始高度 [m]
static constexpr double DX0 = 0.0;   // 初始速度 [m/s]

// ── 自定义动力学状态 ────────────────────────────────────────────────
// z(0) = 球心高度 x [m]   z(1) = 垂直速度 dx [m/s]
static Eigen::Vector2d z_state(X0, DX0);
static double sim_time = 0.0;

// 与 MATLAB 默认参数对齐：m=1kg, l=0.5m (rest), l_min≈球半径
static const Params P{
    /* l     */ 0.5,
    /* l_min */ 0.05,
    /* kp    */ 800.0,
    /* kd    */ 5.0,
    /* m     */ 1.0
};

// 物理步长（与原 MJCF 的 timestep 一致）
static constexpr double DT_PHYS = 0.001;

// body "hopper" 的初始世界 z（XML 中 <body pos="0 0 0.5">）
static double z_body_init = 0.5;

static void reset_state() {
    z_state << X0, DX0;
    sim_time = 0.0;
    if (m && d) {
        d->qpos[0] = z_state(0) - z_body_init;            // hopper
        d->qpos[1] = cartPoleKinematics(z_state(0), P);   // foot
        d->qvel[0] = z_state(1);
        d->qvel[1] = 0.0;
        d->time    = 0.0;
        mj_forward(m, d);
    }
}

// 4 阶 Runge-Kutta，u 在子步内保持常数
static void rk4_step(Eigen::Vector2d& z, double u, const Params& p, double dt) {
    Eigen::Vector2d k1 = cartPoleDynamics(z,                u, p);
    Eigen::Vector2d k2 = cartPoleDynamics(z + 0.5*dt*k1,    u, p);
    Eigen::Vector2d k3 = cartPoleDynamics(z + 0.5*dt*k2,    u, p);
    Eigen::Vector2d k4 = cartPoleDynamics(z +     dt*k3,    u, p);
    z += (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

static void keyboard(GLFWwindow* w, int key, int sc, int act, int mods) {
    if (act != GLFW_PRESS) return;
    if      (key == GLFW_KEY_BACKSPACE) reset_state();
    else if (key == GLFW_KEY_SPACE)     paused = !paused;
    else if (key == GLFW_KEY_ESCAPE)    glfwSetWindowShouldClose(w, GLFW_TRUE);
}

static void mouse_button(GLFWwindow* w, int b, int act, int mods) {
    button_left   = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    button_middle = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    button_right  = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    glfwGetCursorPos(w, &lastx, &lasty);
}

static void mouse_move(GLFWwindow* w, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;
    double dx = xpos - lastx, dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
    int width, height; glfwGetWindowSize(w, &width, &height);
    bool shift = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
                  glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if      (button_right)  action = shift ? mjMOUSE_MOVE_H   : mjMOUSE_MOVE_V;
    else if (button_left)   action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else                    action = mjMOUSE_ZOOM;
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

static void scroll(GLFWwindow* w, double xoff, double yoff) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoff, &scn, &cam);
}

int main() {
    char err[1024];
    m = mj_loadXML("model.xml", nullptr, err, sizeof(err));
    if (!m) { fprintf(stderr, "load: %s\n", err); return 1; }
    d = mj_makeData(m);

    int body_id = mj_name2id(m, mjOBJ_BODY, "hopper");
    if (body_id >= 0) z_body_init = m->body_pos[3*body_id + 2];

    if (!glfwInit()) { fprintf(stderr, "GLFW init failed\n"); return 1; }
    GLFWwindow* win = glfwCreateWindow(1200, 900, "Hopper 1D (cartPoleDynamics)", nullptr, nullptr);
    if (!win) { fprintf(stderr, "GLFW window failed (DISPLAY OK?)\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.distance  = 1.8;
    cam.azimuth   = 90;
    cam.elevation = -15;
    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 0.4;

    glfwSetKeyCallback        (win, keyboard);
    glfwSetCursorPosCallback  (win, mouse_move);
    glfwSetMouseButtonCallback(win, mouse_button);
    glfwSetScrollCallback     (win, scroll);

    reset_state();   // 让 MuJoCo 显示与 z_state 一致的初始位姿

    char status[256];
    double wall_prev = glfwGetTime();
    while (!glfwWindowShouldClose(win)) {
        // wall-clock 推进：避免虚拟显示下 vsync 失效仿真飙速
        double wall_now = glfwGetTime();
        double dt_wall  = wall_now - wall_prev;
        wall_prev = wall_now;
        if (dt_wall > 0.1) dt_wall = 0.1;

        if (!paused) {
            int n_sub = std::max(1, (int)std::round(dt_wall / DT_PHYS));
            for (int i = 0; i < n_sub; ++i) {
                rk4_step(z_state, 0.0 /* u */, P, DT_PHYS);
                sim_time += DT_PHYS;
            }
            // 把 cartPoleDynamics / cartPoleKinematics 的输出写回 MuJoCo
            d->qpos[0] = z_state(0) - z_body_init;            // hopper
            d->qpos[1] = cartPoleKinematics(z_state(0), P);   // foot
            d->qvel[0] = z_state(1);
            d->qvel[1] = 0.0;
            d->time    = sim_time;
            mj_forward(m, d);          // 只跑运动学，不跑物理
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(win, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // —— 状态文字 ——
        const char* phase =
            z_state(0) > P.l        ? "FLIGHT"   :
            z_state(0) < P.l_min    ? "BOTTOM"   :
                                      "STANCE";
        snprintf(status, sizeof(status),
                 "t = %.2f s   x = %.3f m   dx = %+.3f m/s   phase = %s   %s",
                 sim_time, z_state(0), z_state(1), phase,
                 paused ? "[PAUSED]" : "");
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, status, nullptr, &con);
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                    "[SPACE] pause  [BS] reset  [ESC] quit\n"
                    "L-drag rotate  R-drag pan  scroll zoom",
                    nullptr, &con);

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}
