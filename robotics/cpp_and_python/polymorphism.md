# 多态 —— C++ 静态/动态多态 与 C 语言 OOP 实现

## 一、多态的分类

```
           多态
           / \
          /   \
    静态多态    动态多态
   (编译期)    (运行时)
    / \           / \
   /   \         /   \
模板   重载    虚函数  C的OOP
CRTP  operator 继承  (函数指针表)
```

---

## 二、C++ 静态多态（编译期）

静态多态的核心：**编译时就确定调用哪个函数，零运行时开销**。

### 2.1 函数重载

```cpp
void log(int value)    { printf("int: %d\n", value); }
void log(float value)  { printf("float: %f\n", value); }
void log(const char* s) { printf("str: %s\n", s); }

log(42);       // 编译器根据参数类型匹配 void log(int)
log(3.14f);    // 匹配 void log(float)
log("hello");  // 匹配 void log(const char*)
```

底层原理：C++ 使用 **name mangling** 为每个重载函数生成唯一符号名。在 GCC 中：

```
void log(int)      → _Z3logi      (i = int)
void log(float)    → _Z3logf      (f = float)
void log(const char*)  → _Z3logPKc   (PKc = pointer to const char)
```

调用时编译器根据参数类型直接跳转到对应符号——零运行时分支。

### 2.2 模板多态

```cpp
template<typename T>
T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

int    a = clamp(100, 0, 255);     // 实例化 clamp<int>
float  b = clamp(1.5f, 0.0f, 1.0f); // 实例化 clamp<float>
```

编译器为每种类型组合生成独立的函数实例。每个实例都被内联优化的候选。

### 2.3 CRTP（奇异递归模板模式）—— 静态多态替代虚函数

```cpp
// 基类模板（没有虚函数，没有 vtable）
template<typename Derived>
class Motor {
public:
    void run() {
        // 编译期将 this 转为具体子类指针
        static_cast<Derived*>(this)->run_impl();
    }
    // 注意: Motor 本身不定义 run_impl()
};

// BLDC 实现
class BLDC : public Motor<BLDC> {
public:
    void run_impl() {
        write_reg(PHASE_U, duty_cycle);  // FOC 控制
        write_reg(PHASE_V, duty_cycle);
        write_reg(PHASE_W, duty_cycle);
    }
};

// Stepper 实现
class Stepper : public Motor<Stepper> {
public:
    void run_impl() {
        step_pulse();  // 步进脉冲
    }
};
```

**调用端：**

```cpp
template<typename T>
void execute_motor(Motor<T>& m) {
    m.run();  // 编译期解析：直接调用 BLDC::run_impl() 或 Stepper::run_impl()
}
```

**CRTP vs 虚函数对比：**

| | 虚函数 (dynamic) | CRTP (static) |
|---|---------|-------|
| 调用方式 | `obj->vptr->vtable[i]()` 间接跳转 | `BLDC::run_impl()` 直接调用 |
| 开销 | 两次间接访存 + 分支预测压力 | 零开销，可 inline |
| 能否容器存储不同类型 | ✅ `vector<Motor*>` | ❌ 必须知道具体类型或用 variant |
| 适用场景 | 运行时多态，类型动态变化 | 性能热路径，类型编译期已知 |

---

## 三、C++ 动态多态（运行时）

### 3.1 虚函数与 vtable 的完整机制

```cpp
class Motor {
public:
    virtual void init() {}
    virtual void run()  {}
    virtual void stop() {}
    virtual ~Motor() {}
};
```

**对象内存布局：**

```
┌────────────────────────────┐
│  对象实例 (64-bit)          │
├──────┬─────────────────────┤
│ vptr │ 成员变量...          │
│ (8B) │                     │
└──┬───┴─────────────────────┘
   │
   │ 指向
   ▼
┌───────────────────────────────────────────┐
│  Motor 的 vtable (在 .data.rel.ro 段)      │
├───────────────────────────────────────────┤
│  [0] type_info*        → RTTI 信息        │
│  [1] &Motor::init      → init 的函数指针   │
│  [2] &Motor::run       → run 的函数指针    │
│  [3] &Motor::stop      → stop 的函数指针   │
│  [4] &Motor::~Motor    → 析构函数指针       │
└───────────────────────────────────────────┘
```

**虚函数调用的完整汇编路径：**

```cpp
Motor* m = get_next_motor();
m->run();
```

```asm
; 1) 取对象地址
    ldr  r0, [fp, #-8]       ; r0 = m (指向对象的指针)

; 2) 解引用 vptr
    ldr  r1, [r0]            ; r1 = *m = vptr 的值 = vtable 基地址

; 3) 从 vtable 取函数指针 (偏移量由编译期确定)
    ; run() 是 vtable 的第 2 个虚函数 (索引 1, init 占索引 0)
    ; 每个函数指针 8 字节，所以偏移量 = 1 * 8 = 8
    ldr  r2, [r1, #8]        ; r2 = vtable[1] = &Motor::run (或 &BLDC::run)
                              ;    #8 是 8 字节偏移 (64-bit 指针宽度)

; 4) 间接跳转
    blr  r2                  ; 调用 r2 指向的函数 (branch with link to register)
```

### 3.2 vtable 偏移量在编译期计算

编译器在编译 `Motor` 类时，为每个虚函数分配固定的 vtable 索引。调用点不关心对象到底是 `Motor`、`BLDC` 还是 `Stepper`——只需知道"run 函数在 vtable 的偏移 +8 位置"。

### 3.3 容器中的动态多态

```cpp
std::vector<std::unique_ptr<Motor>> motors;
motors.push_back(std::make_unique<BLDC>());
motors.push_back(std::make_unique<Stepper>());

for (auto& m : motors) {
    m->run();  // 每次循环调用不同类的 run()
    // 第 1 次: vptr → BLDC vtable   → &BLDC::run
    // 第 2 次: vptr → Stepper vtable → &Stepper::run
}
```

这是虚函数最关键的价值：**同一段调用代码，在运行时根据对象类型自动分派到不同的实现**。静态多态做不到这点——`std::vector<Motor*>` 不能存不同的模板具现化类型。

---

## 四、C 语言实现 OOP 多态

C 没有 class、virtual 关键字，但可以手动模拟 vtable。

### 4.1 基础版本：函数指针直接嵌入

```c
// motor.h
typedef struct Motor {
    // 函数指针作为"虚函数"
    void (*init)(struct Motor* self);
    void (*run)(struct Motor* self);
    void (*stop)(struct Motor* self);

    // 成员变量
    uint16_t max_speed;
    uint16_t current_speed;
} Motor;

// 便捷调用宏
#define MOTOR_INIT(m)  ((m)->init(m))
#define MOTOR_RUN(m)   ((m)->run(m))
#define MOTOR_STOP(m)  ((m)->stop(m))
```

```c
// bldc.c — BLDC 实现
#include "motor.h"

static void bldc_init(struct Motor* self) {
    // FOC 初始化: 对齐转子、配置 ADC/PWM
    *(volatile uint32_t*)TIM1_CCR = 0;
    *(volatile uint32_t*)ADC_CR1  |= ADC_EN;
    self->current_speed = 0;
}

static void bldc_run(struct Motor* self) {
    // FOC 电流环
    self->current_speed = calculate_speed_from_encoder();
    set_svpwm_duty(self->current_speed);
}

static void bldc_stop(struct Motor* self) {
    // 刹车
    set_svpwm_duty(0);
    self->current_speed = 0;
}

// 构造函数
Motor* bldc_new(void) {
    Motor* m = (Motor*)malloc(sizeof(Motor));
    m->init = bldc_init;
    m->run  = bldc_run;
    m->stop = bldc_stop;
    m->max_speed = 3000;
    return m;
}
```

```c
// main.c — 多态调用
Motor* m1 = bldc_new();      // 创建 BLDC
Motor* m2 = stepper_new();   // 创建步进电机

Motor* motors[] = { m1, m2 };
for (int i = 0; i < 2; i++) {
    MOTOR_INIT(motors[i]);   // m1 → bldc_init,  m2 → stepper_init
    MOTOR_RUN(motors[i]);    // m1 → bldc_run,   m2 → stepper_run
    MOTOR_STOP(motors[i]);   // m1 → bldc_stop,  m2 → stepper_stop
}
```

**缺点：** 每个对象都存储函数指针（3 × 8 = 24 字节），大量对象时浪费内存。C++ 用 vtable 解决了这个问题——所有同类型对象共享一张表。

### 4.2 进阶版本：共享 vtable（更接近 C++ 的实现）

```c
// motor.h
typedef struct MotorVTable {
    void (*init)(struct Motor* self);
    void (*run)(struct Motor* self);
    void (*stop)(struct Motor* self);
} MotorVTable;

typedef struct Motor {
    const MotorVTable* vtable;  // 共享的函数表指针
    uint16_t max_speed;
    uint16_t current_speed;
} Motor;

// 调用宏
#define MOTOR_INIT(m)  ((m)->vtable->init(m))
#define MOTOR_RUN(m)   ((m)->vtable->run(m))
#define MOTOR_STOP(m)  ((m)->vtable->stop(m))
```

```c
// bldc.c
static void bldc_init(Motor* self) { /* FOC 初始化 */ }
static void bldc_run(Motor* self)  { /* FOC 电流环 */ }
static void bldc_stop(Motor* self) { /* 刹车 */ }

// 全局共享的 vtable — 与 C++ 的设计一致
static const MotorVTable bldc_vtable = {
    .init = bldc_init,
    .run  = bldc_run,
    .stop = bldc_stop,
};

Motor* bldc_new(void) {
    Motor* m = (Motor*)malloc(sizeof(Motor));
    m->vtable = &bldc_vtable;   // 所有 BLDC 对象共享同一个 vtable
    m->max_speed = 3000;
    return m;
}
```

**内存对比（1000 个 Motor 对象）：**

| 方案 | 函数指针开销 | 总内存 |
|------|------------|--------|
| 指针嵌入 struct | 1000 × 3 × 8 = 24KB | 大 |
| 共享 vtable | 1000 × 1 × 8 = 8KB | 省 16KB |
| C++ 虚函数 | ≈ 同上 | 编译器自动管理 |

### 4.3 继承模拟

```c
// 通过将基类作为第一个成员模拟"is-a"关系,
// 使父类指针可以直接指向子类对象
typedef struct BLDC {
    Motor base;        // 必须是第一个成员！
    uint16_t foc_mode;
    float    current_kp;
    float    current_ki;
} BLDC;

BLDC* bldc_new(void) {
    BLDC* b = (BLDC*)malloc(sizeof(BLDC));
    b->base.vtable = &bldc_vtable;
    b->foc_mode = 0;
    b->current_kp = 0.1f;
    return b;
}

// 安全的向上转型
Motor* as_motor(BLDC* b) {
    return (Motor*)b;  // base 是第一个成员，指针地址相同
}

// 安全的向下转型
BLDC* to_bldc(Motor* m) {
    if (m->vtable == &bldc_vtable)  // 手动 RTTI
        return (BLDC*)m;
    return NULL;
}
```

这是 Linux 内核中无处不在的模式。例如 `struct file_operations` 就是 vtable，每个文件系统（ext4、proc、sysfs）提供自己的实现。

### 4.4 实际案例：STM32 HAL 的 UART 抽象

STM32 HAL 库本身就是用 C 实现多态的典型：

```c
// HAL 的"抽象基类": UART_HandleTypeDef
typedef struct {
    USART_TypeDef* Instance;     // 硬件寄存器基址
    uint8_t*       pTxBuffPtr;   // 发送缓冲区
    uint16_t       TxXferSize;   // 发送长度
    // ... 没有函数指针, 但通过 Instance 区分不同 USART 外设
} UART_HandleTypeDef;

// 同一个函数对 USART1 和 USART2 都有效
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef* huart,
                                     uint8_t* pData, uint16_t Size,
                                     uint32_t Timeout) {
    // huart->Instance 指向不同的硬件寄存器基址
    // 通过这个指针实现"多态"
    while (huart->TxXferCount < huart->TxXferSize) {
        huart->Instance->DR = *pData++;   // 写到具体 USART 的数据寄存器
        huart->TxXferCount++;
    }
}
```

---

## 五、应用场景总结

| 场景 | 推荐方案 | 理由 |
|------|---------|------|
| 机器人关节控制循环 (10kHz) | CRTP 静态多态 | 无 vtable 开销，可 inline FOC 算法 |
| 插件式传感器驱动（运行时加载） | C++ 虚函数 | 需运行时多态，类型不确定 |
| STM32 HAL 外设抽象 | C 的数据指针多态 | 无堆分配 + 极小代码体积 |
| 模板化的数学库（sin/cos 查表） | C++ 模板 | 不同类型特化编译期展开 |
| 引导程序或裸机环境 | C 共享 vtable | 无 C++ 运行时依赖，完全手动控制 |
