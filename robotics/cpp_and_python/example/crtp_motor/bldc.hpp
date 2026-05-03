#pragma once
#include "motor.hpp"

/// BLDC 电机: FOC 控制
class BLDC : public Motor<BLDC> {
public:
    BLDC(uint16_t max_rpm = 3000) {
        max_speed = max_rpm;
    }

private:
    friend class Motor<BLDC>;  // 允许基类访问 private 实现

    void init_impl() {
        // 模拟 FOC 初始化: 对齐转子, 配置 ADC/PWM
        aligned = true;
    }

    void run_impl(uint16_t speed) {
        uint16_t target = (speed < max_speed) ? speed : max_speed;
        // 模拟 FOC 电流环
        duty_a = target;
        duty_b = target;
        duty_c = target;
        cur_speed = target;
    }

    void stop_impl() {
        duty_a = duty_b = duty_c = 0;
        cur_speed = 0;
    }

    uint16_t get_speed_impl() const {
        return cur_speed;
    }

    bool     aligned = false;
    uint16_t duty_a  = 0;
    uint16_t duty_b  = 0;
    uint16_t duty_c  = 0;
};
