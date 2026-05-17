#pragma once

#include "Eigen/Eigen"

struct Params {
    double l;       // 自然长度（弹簧静止长度）
    double l_min;   // 最小压缩长度（硬限位）
    double kp;      // 弹簧刚度
    double kd;      // 阻尼系数
    double m;       // 质量
};

Eigen::Vector2d cartPoleDynamics(const Eigen::Vector2d& z,
    double u,
    const Params& p);

// 对应 MATLAB cartPoleKinematics —— 返回脚的世界 z 高度
double cartPoleKinematics(double x, const Params& p);
