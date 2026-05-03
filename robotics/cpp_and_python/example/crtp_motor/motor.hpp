#pragma once
#include <cstdint>

/// CRTP 基类模板: Motor<Derived>
///
/// 所有接口函数(motor_init, motor_run, motor_stop)都在编译期通过
/// static_cast<Derived*>(this) 分发到子类实现, 零虚函数开销。
template <typename Derived>
class Motor {
public:
    // ---- 公开接口 (非虚, 编译期多态) ----

    void motor_init() {
        static_cast<Derived*>(this)->init_impl();
    }

    void motor_run(uint16_t speed) {
        static_cast<Derived*>(this)->run_impl(speed);
    }

    void motor_stop() {
        static_cast<Derived*>(this)->stop_impl();
    }

    uint16_t motor_get_speed() const {
        return static_cast<const Derived*>(this)->get_speed_impl();
    }

protected:
    // 禁止通过基类指针 delete (基类析构非虚)
    ~Motor() = default;

    // 成员变量
    uint16_t max_speed  = 0;
    uint16_t cur_speed  = 0;
};
