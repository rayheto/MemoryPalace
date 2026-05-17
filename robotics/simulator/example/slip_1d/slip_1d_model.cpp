#include "slip_1d_model.hpp"

// 状态：z = [x; dx]     x 为质心高度，dx 为垂直速度
// 输入：u            额外加速度指令（推力/m）
// 返回：dz = [dx; ddx]
Eigen::Vector2d cartPoleDynamics(const Eigen::Vector2d& z,
                                double u,
                                const Params& p)
{
    constexpr double g = 9.81;

    // 系统状态
    const double x = z(0);
    const double dx = z(1);

    //微分动力学方程输出
    double dx_out, ddx;

    if(x > p.l) {
        dx_out = dx;
        ddx = -g;
    }else if (x < p.l_min) {
        dx_out = 1e-5;
        dx_out = 1e-5;
        // 准备弹出
        ddx    = -p.kp / p.m * (x - p.l) - p.kd * dx;
        ddx   += u;
    }else {
        // —— 触地相：重力 + 弹簧 + 阻尼 + 控制输入 ——
        dx_out = dx;
        ddx    = -g - p.kp / p.m * (x - p.l) - p.kd * dx;
        ddx   += u;
    }

    Eigen::Vector2d dz;
    dz << dx_out, ddx;
    return dz;
}

// MATLAB cartPoleKinematics 的直译：根据 x 推出脚的世界 z 高度
double cartPoleKinematics(double x, const Params& p)
{
    double p_leg;
    if (x > p.l) {
        // 飞行相：腿伸到自然长度，脚悬空跟随身体
        p_leg = x - p.l;
    } else {
        // 支撑相：脚贴近地面（l_min 是"脚厚"），过深则贴地
        p_leg = x - p.l + p.l_min;
        if (p_leg < p.l_min) p_leg = 0.0;
    }
    return p_leg;
}