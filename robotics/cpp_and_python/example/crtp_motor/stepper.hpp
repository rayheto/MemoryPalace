#pragma once
#include "motor.hpp"

/// 步进电机: 脉冲驱动
class Stepper : public Motor<Stepper> {
public:
    Stepper(uint16_t max_steps_per_sec = 2000) {
        max_speed = max_steps_per_sec;
    }

private:
    friend class Motor<Stepper>;

    void init_impl() {
        phase = 0;
    }

    void run_impl(uint16_t speed) {
        uint16_t target = (speed < max_speed) ? speed : max_speed;
        // 模拟步进脉冲: 每步切换一次相位
        phase = (phase + 1) & 0x3;
        cur_speed = target;
    }

    void stop_impl() {
        cur_speed = 0;
    }

    uint16_t get_speed_impl() const {
        return cur_speed;
    }

    uint8_t phase = 0;
};
