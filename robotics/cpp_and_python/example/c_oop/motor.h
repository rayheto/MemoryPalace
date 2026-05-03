#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

/* 虚函数表: 所有同类型 Motor 共享同一张表 (与 C++ vtable 设计一致) */
typedef struct Motor Motor;
struct MotorVTable {
    void (*init)(Motor* self);
    void (*run)(Motor* self, uint16_t speed);
    void (*stop)(Motor* self);
    uint16_t (*get_speed)(const Motor* self);
};

/* 基类: vptr 必须是第一个成员 (保证向上转型时指针地址不变) */
struct Motor {
    const struct MotorVTable* vtable;
    uint16_t max_speed;
    uint16_t cur_speed;
};

/* 便捷调用宏 */
#define MOTOR_INIT(m)       ((m)->vtable->init(m))
#define MOTOR_RUN(m, spd)   ((m)->vtable->run(m, spd))
#define MOTOR_STOP(m)       ((m)->vtable->stop(m))
#define MOTOR_GET_SPEED(m)  ((m)->vtable->get_speed(m))

#endif
