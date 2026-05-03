#include <stdio.h>
#include "bldc.h"
#include "stepper.h"

/* 多态调用: 同一段代码, 根据 vtable 分派到不同实现 */
void exercise_motor(const char* label, Motor* m, uint16_t speed) {
    MOTOR_INIT(m);
    MOTOR_RUN(m, speed);
    printf("%-10s: target=%4u  actual=%4u\n",
           label, speed, MOTOR_GET_SPEED(m));
    MOTOR_STOP(m);
}

int main(void) {
    BLDC*    bldc    = bldc_new(3000);
    Stepper* stepper = stepper_new(2000);

    /* 向上转型: BLDC* / Stepper* → Motor* (base 是第一个成员, 地址相同) */
    Motor* motors[] = { &bldc->base, &stepper->base };

    /* 运行时多态: 循环内调用的是不同的 init/run/stop */
    const char* labels[] = { "BLDC", "Stepper" };
    uint16_t    speeds[] = { 1500, 800 };
    for (int i = 0; i < 2; i++)
        exercise_motor(labels[i], motors[i], speeds[i]);

    /* 内存开销对比 */
    printf("\n--- 内存开销 ---\n");
    printf("sizeof(Motor)   = %zu  (vtable 指针 + 成员)\n", sizeof(Motor));
    printf("sizeof(BLDC)    = %zu  (Motor + 3 路 PWM + 对齐标志)\n", sizeof(BLDC));
    printf("sizeof(Stepper) = %zu  (Motor + phase)\n", sizeof(Stepper));
    printf("vtable 共享: 1000 个 BLDC 对象共用同一张表 (%zu B)\n",
           sizeof(struct MotorVTable));

    bldc_free(bldc);
    stepper_free(stepper);
    return 0;
}
