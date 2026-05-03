#include "bldc.h"
#include <stdlib.h>

/* ---- 虚函数实现 (private) ---- */

static void bldc_init(Motor* self) {
    BLDC* b = (BLDC*)self;
    b->duty_a  = 0;
    b->duty_b  = 0;
    b->duty_c  = 0;
    b->aligned = 1;  /* 模拟转子对齐 */
}

static void bldc_run(Motor* self, uint16_t speed) {
    BLDC* b = (BLDC*)self;
    uint16_t target = (speed < self->max_speed) ? speed : self->max_speed;
    /* 模拟 FOC 电流环: 更新三相 PWM 占空比 */
    b->duty_a = b->duty_b = b->duty_c = target;
    self->cur_speed = target;
}

static void bldc_stop(Motor* self) {
    BLDC* b = (BLDC*)self;
    b->duty_a = b->duty_b = b->duty_c = 0;
    self->cur_speed = 0;
}

static uint16_t bldc_get_speed(const Motor* self) {
    return self->cur_speed;
}

/* BLDC 全局共享的 vtable (const → 放 .rodata) */
static const struct MotorVTable bldc_vtable = {
    .init      = bldc_init,
    .run       = bldc_run,
    .stop      = bldc_stop,
    .get_speed = bldc_get_speed,
};

/* ---- 构造/析构 ---- */

BLDC* bldc_new(uint16_t max_rpm) {
    BLDC* b = (BLDC*)malloc(sizeof(BLDC));
    b->base.vtable    = &bldc_vtable;
    b->base.max_speed = max_rpm;
    b->base.cur_speed = 0;
    return b;
}

void bldc_free(BLDC* b) {
    free(b);
}
