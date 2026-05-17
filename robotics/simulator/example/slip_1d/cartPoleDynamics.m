function dz = cartPoleDynamics(z, u, p)
% 一维垂直跳跃器（"倒立摆 + 落足点"教学模型）
% z(1,:) : 垂直位置（足端高度）
% z(2,:) : 垂直速度
% u      : 控制输入（额外力）
% p.l       : 静止腿长
% p.l_min   : 最小压缩量
% p.kp, p.kd: 接触弹簧/阻尼
% p.m       : 质量
%
% 分段：
%   z(1) > l         → 自由落体（脚离地）
%   l_min < z(1) ≤ l → 接触：重力 + 弹簧阻尼
%   z(1) < l_min     → 过压缩（限位）

    g = 9.81;
    if z(1,:) > p.l                          % 控制动力学（脚离地）
        dx  = z(2,:);                         % 速度积分模型
        ddx = -g;
    else                                      % 支撑
        if z(1,:) < p.l_min                   % 小于最小长度（过压缩）
            dx  = 1e-5;
            ddx = -p.kp/p.m*(z(1,:) - p.l) - p.kd*z(2,:);
        else
            dx  = z(2,:);                     % 速度积分模型
            ddx = -g - p.kp/p.m*(z(1,:) - p.l) - p.kd*z(2,:);
        end
        ddx = ddx + u;
    end

    dz = [dx; ddx];                           % 一阶微分状态
end
