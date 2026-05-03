#include <cstdio>
#include "bldc.hpp"
#include "stepper.hpp"

// 编译期多态: 同一个函数模板, 对不同类型生成不同的实例
template <typename MotorType>
void exercise_motor(const char* label, uint16_t speed) {
    MotorType m;

    // 这些调用在编译期被解析为具体类的实现, 无 vtable 访问
    m.motor_init();
    m.motor_run(speed);
    std::printf("%-10s: target=%4u  actual=%4u\n",
                label, speed, m.motor_get_speed());
    m.motor_stop();
}

int main() {
    // BLDC 和 Stepper 没有公共基类 (Motor<BLDC> ≠ Motor<Stepper>)
    // 多态通过模板在编译期完成, 零运行时开销
    exercise_motor<BLDC>("BLDC", 1500);
    exercise_motor<Stepper>("Stepper", 800);

    // 验证: 用 sizeof 对比 CRTP vs 虚函数的开销
    std::printf("\n--- 内存开销对比 ---\n");
    std::printf("sizeof(BLDC)    = %zu  (成员变量 + 零 vptr)\n", sizeof(BLDC));
    std::printf("sizeof(Stepper) = %zu  (成员变量 + 零 vptr)\n", sizeof(Stepper));
}
