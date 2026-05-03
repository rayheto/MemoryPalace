#include "stepper.h"
#include <stdlib.h>

static void stepper_init(Motor* self) {
    Stepper* s = (Stepper*)self;
    s->phase = 0;
}

static void stepper_run(Motor* self, uint16_t speed) {
    Stepper* s = (Stepper*)self;
    uint16_t target = (speed < self->max_speed) ? speed : self->max_speed;
    /* 模拟步进脉冲: 每步切换一相 */
    s->phase = (s->phase + 1) & 0x3;
    self->cur_speed = target;
}

static void stepper_stop(Motor* self) {
    self->cur_speed = 0;
}

static uint16_t stepper_get_speed(const Motor* self) {
    return self->cur_speed;
}

static const struct MotorVTable stepper_vtable = {
    .init      = stepper_init,
    .run       = stepper_run,
    .stop      = stepper_stop,
    .get_speed = stepper_get_speed,
};

Stepper* stepper_new(uint16_t max_steps_per_sec) {
    Stepper* s = (Stepper*)malloc(sizeof(Stepper));
    s->base.vtable    = &stepper_vtable;
    s->base.max_speed = max_steps_per_sec;
    s->base.cur_speed = 0;
    return s;
}

void stepper_free(Stepper* s) {
    free(s);
}
