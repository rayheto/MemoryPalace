#ifndef BLDC_H
#define BLDC_H
#include "motor.h"

/* BLDC 继承 Motor: base 必须是第一个成员 */
typedef struct {
    Motor base;         /* 父类, 必须排第一 */
    uint16_t duty_a;    /* 三相 PWM 占空比 */
    uint16_t duty_b;
    uint16_t duty_c;
    uint8_t  aligned;   /* 是否已完成转子对齐 */
} BLDC;

BLDC* bldc_new(uint16_t max_rpm);
void  bldc_free(BLDC* b);

#endif
