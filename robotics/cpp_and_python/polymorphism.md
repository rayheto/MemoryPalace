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

> **可运行的完整示例:** [example/crtp_motor/](example/crtp_motor/) — 包含 `Motor<Derived>` 模板、BLDC(FOC)/Stepper(脉冲) 子类实现、编译期多态调用和 `sizeof` 内存对比。
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

**为什么 base 必须是第一个成员？**

C 标准保证：**struct 第一个成员的地址等于整个 struct 的地址**（不允许在最前面插入 padding）。

```
BLDC 对象的内存布局:

  地址: 0x1000 ←── BLDC* b 指向这里
  ┌──────────────────────┐
  │  Motor base          │  ← 0x1000  (第一个成员, 地址与 b 相同)
  │    ├ vtable   (8B)   │
  │    ├ max_speed(2B)   │
  │    └ cur_speed(2B)   │
  ├──────────────────────┤
  │  uint16_t foc_mode   │  ← 0x100C
  ├──────────────────────┤
  │  float current_kp    │  ← 0x1010
  ├──────────────────────┤
  │  float current_ki    │  ← 0x1014
  └──────────────────────┘

  (BLDC*)0x1000  ──强制转换──►  (Motor*)0x1000   ← 地址完全相同！
```

反过来，如果 base 不是第一个成员：

```
错误布局: base 排在第二个位置

  地址: 0x1000 ←── BLDC* b 指向这里
  ┌──────────────────────┐
  │  uint16_t foc_mode   │  ← 0x1000  (第一个成员)
  ├──────────────────────┤
  │  Motor base          │  ← 0x1004  (偏移了 4 字节！)
  │    ├ vtable  (8B)    │
  │    └ ...
  └──────────────────────┘

  (BLDC*)0x1000  ──强制转换──►  (Motor*)0x1000
                                      ↑
                    Motor* 指向 0x1000, 但 base 实际在 0x1004

  m->vtable → *(0x1000) 读到的是 foc_mode, 不是 vtable → 函数指针跳转到垃圾地址 → 死机
```

这就是为什么 Linux 内核中所有"继承"关系（如 `struct usb_device` 内含 `struct device` 作为首成员）都把父类放在第一位——保证 `container_of` 宏和向上转型的正确性。

### 4.5 Linux 内核中的 C OOP 实例

当前系统内核版本：`6.8.0-107-generic`。以下摘录均来自实际源码。

**实例一：`struct file_operations` —— 纯虚函数表**

```c
// include/linux/fs.h:1992-2035 (Linux 6.8)
struct file_operations {
    struct module *owner;
    loff_t (*llseek) (struct file *, loff_t, int);
    ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
    ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
    ssize_t (*read_iter) (struct kiocb *, struct iov_iter *);
    ssize_t (*write_iter) (struct kiocb *, struct iov_iter *);
    int (*iopoll)(struct kiocb *, struct io_comp_batch *, unsigned int flags);
    int (*iterate_shared) (struct file *, struct dir_context *);
    __poll_t (*poll) (struct file *, struct poll_table_struct *);
    long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);
    long (*compat_ioctl) (struct file *, unsigned int, unsigned long);
    int (*mmap) (struct file *, struct vm_area_struct *);
    int (*open) (struct inode *, struct file *);
    int (*flush) (struct file *, fl_owner_t id);
    int (*release) (struct inode *, struct file *);
    int (*fsync) (struct file *, loff_t, loff_t, int datasync);
    int (*fasync) (int, struct file *, int);
    int (*lock) (struct file *, int, struct file_lock *);
    ssize_t (*splice_write)(struct pipe_inode_info *, struct file *, loff_t *, size_t, unsigned int);
    ssize_t (*splice_read)(struct file *, loff_t *, struct pipe_inode_info *, size_t, unsigned int);
    long (*fallocate)(struct file *, int mode, loff_t offset, loff_t len);
    ssize_t (*copy_file_range)(struct file *, loff_t, struct file *, loff_t, size_t, unsigned int);
    int (*fadvise)(struct file *, loff_t, loff_t, int);
    // ... 共 29 个函数指针
} __randomize_layout;
```

这张表就是 Linux VFS 的"虚函数表"。每个文件系统提供自己的实现——ext4 有 `ext4_file_operations`，proc 有 `proc_reg_file_ops`，sysfs 有 `sysfs_file_operations`。当用户进程调用 `read(fd)` 时，内核通过 `file->f_op->read()` 走 vtable 间接跳转到对应文件系统的实现。

**实例二：`container_of` —— 手工向下转型 (RTTI 的替代)**

```c
// include/linux/container_of.h:18-23 (Linux 6.8)
#define container_of(ptr, type, member) ({              \
    void *__mptr = (void *)(ptr);                       \
    static_assert(__same_type(*(ptr), ((type *)0)->member) ||  \
                  __same_type(*(ptr), void),            \
                  "pointer type mismatch in container_of()"); \
    ((type *)(__mptr - offsetof(type, member))); })
```

**`offsetof` 实现：**

这是 `container_of` 的数学基础——从子成员的地址反推出父结构体的地址：

```c
#define offsetof(type, member) __builtin_offsetof(type, member)
// 等价于: ((size_t)&(((type *)0)->member))
// 把 NULL 指针当成 type* 来取 member 的地址 → 得到 member 在 type 中的偏移量
```

**`container_of` 工作原理图解：**

```
   已知: Motor* m (指向 BLDC 的 base 成员), base 偏移量 = 12

   通过 container_of(m, BLDC, base):
        BLDC* b = (BLDC*)((uint8_t*)m - 12);
                                ↑
                    地址回退 12 字节就能拿到 BLDC* 指针
```

**实例三：`struct usb_device` —— 继承 + container_of**

```c
// include/linux/usb.h:651-661, 733
struct usb_device {
    struct device dev;               // ← 父类 (不是第一个成员!)
    int           devnum;
    char          devpath[16];
    enum usb_device_state state;
    enum usb_device_speed  speed;
    // ...
};

// include/linux/usb.h:733
#define to_usb_device(__dev)  container_of_const(__dev, struct usb_device, dev)

// 使用示例 (驱动代码中):
void usb_probe(struct device *dev) {           // 回调参数是父类指针
    struct usb_device *udev = to_usb_device(dev);  // 向下转型!
    // 访问子类特有字段
    udev->devnum = ...;
}
```

`struct device` 是 Linux 驱动模型的核心。`usb_device`、`platform_device`、`pci_dev` 都是以它为基础向下拓展。回调函数收到 `struct device*`，通过 `container_of` 找回原始的子类类型。

**实例四：`struct platform_device` —— 另一个继承链**

```c
// include/linux/platform_device.h:23-29 (Linux 6.8)
struct platform_device {
    struct device dev;               // ← 父类 (非首成员)
    const char   *name;
    int           id;
    bool          id_auto;
    u64           platform_dma_mask;
    // ...
};
```

**实例五：proc 文件系统的 vtable 实例化**

```c
// 内核中某个 proc 驱动的 file_operations 实例 (简化):
static const struct file_operations my_proc_fops = {
    .owner   = THIS_MODULE,
    .read    = my_proc_read,
    .write   = my_proc_write,
    .open    = my_proc_open,
    .release = my_proc_release,
    .llseek  = default_llseek,
};
// 未赋值的函数指针 (如 .mmap) 自然为 NULL → VFS 层会返回 -ENODEV
```

### 4.6 内核模式与我们 `c_oop` 示例的对应关系

| 内核中的角色 | 内核结构体/宏 | 我们的 c_oop 实现 |
|------------|-------------|---------------|
| 虚函数表 | `struct file_operations` | `struct MotorVTable` |
| 基类 | `struct device` | `struct Motor` |
| 子类 | `struct usb_device`, `struct platform_device` | `BLDC`, `Stepper` |
| 向上转型 | `(struct device*)udev` | `(Motor*)&bldc->base` |
| 向下转型 | `to_usb_device(dev)` → `container_of` | `to_bldc(m)` → vtable 对比 |
| vtable 实例 | `ext4_file_operations` | `bldc_vtable` |
| 多态调用 | `file->f_op->read(file, buf, len, off)` | `m->vtable->run(m, speed)` |

> 关键区别：内核的 `struct device` **不是** `usb_device` 的第一个成员，因此向下转型必须用 `container_of`/`offsetof` 做指针回退。我们 c_oop 示例把 `base` 放在第一位，使向上转型更简单（无需 offsetof 调整地址）。

> **可运行的完整示例:** [example/c_oop/](example/c_oop/) — 包含共享 vtable、BLDC(FOC)/Stepper(脉冲) 实现、`base` 首成员继承、运行时多态调用和内存对比。

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
