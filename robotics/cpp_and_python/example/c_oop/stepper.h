#ifndef STEPPER_H
#define STEPPER_H
#include "motor.h"

typedef struct {
    Motor base;        /* 父类, 必须排第一 */
    uint8_t phase;     /* 当前相 (0-3) */
} Stepper;

Stepper* stepper_new(uint16_t max_steps_per_sec);
void     stepper_free(Stepper* s);

#endif
