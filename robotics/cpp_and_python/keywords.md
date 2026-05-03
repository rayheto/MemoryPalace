# 关键字的具体应用 —— 从编译器原理理解 const / static / volatile 等

## 1. `volatile` —— 禁止编译器优化的"透明窗口"

### 1.1 底层原理

编译器在优化代码时会假设：**只有当前执行的代码会修改内存**。它会把变量缓存在寄存器中，合并重复的读写，甚至删除"无用"的写入。

`volatile` 打破了这一假设。它告诉编译器：

> "这个变量的值可能在你不知道的时候改变。每次读必须从内存重新加载，每次写必须立刻写回内存。不要优化、不要重排、不要消除。"

从汇编角度看：

```c
// 无 volatile
int x = STATUS_REG;
int y = STATUS_REG;  // 编译器优化: 复用寄存器中的值, 不重新读内存

// 有 volatile
volatile int* status = (volatile int*)STATUS_REG;
int a = *status;     // ldr r0, [STATUS_REG]    —— 必须读内存
int b = *status;     // ldr r1, [STATUS_REG]    —— 再次读内存, 值可能已变
```

### 1.2 三个必须使用的场景

**场景一：MMIO 寄存器（Memory-Mapped I/O）**

```c
// STM32 GPIOA 输出寄存器 (地址 0x40020014)
volatile uint32_t* const GPIOA_ODR = (volatile uint32_t*)0x40020014;

void led_toggle() {
    *GPIOA_ODR ^= (1 << 5);  // 翻转 PA5

    // 若不加 volatile:
    // 编译器看到 "写 *GPIOA_ODR; 写 *GPIOA_ODR" 可能删除第一次写入
    // 因为编译器认为"写入后无人读取、马上又被覆盖, 第一次写入无意义"
    // 但硬件恰恰需要这两次写入产生两个边沿信号
}
```

不加 `volatile`，编译器会把连续的寄存器写操作优化掉，硬件看到的就是错误的信号时序。更隐蔽的是——编译器可能将 `*GPIOA_ODR = 0x20` 优化为"反正没人读，干脆不写"，外设根本不动作。

**场景二：中断服务程序与主循环共享的变量**

```c
volatile bool data_ready = false;

// ISR
void UART_RX_ISR(void) {
    rx_buffer[tail++] = UART->DR;
    if (rx_buffer[tail-1] == '\n')
        data_ready = true;  // 通知主循环
}

// 主循环
void main_loop() {
    while (!data_ready) {}  // 无 volatile → 被优化为 if(!data_ready) while(1){}

    process_line(rx_buffer);
    data_ready = false;
}
```

编译器的视角：`main_loop` 中 `data_ready` 从来没被写入过（编译器看不到 ISR），因此 `!data_ready` 的求值结果永远不会变。优化器会将 `while (!data_ready) {}` 转换为：

```asm
    ldr r0, [data_ready]   ; 只读一次
    cmp r0, #0
    bne .L_skip
.L_spin:
    b   .L_spin            ; 死循环——data_ready 被 ISR 改成 true 也感知不到
```

加了 `volatile` 后，每次循环都从内存重新加载，ISR 的修改立即可见。

**场景三：`setjmp` / `longjmp` 跨栈帧变量**

```c
#include <setjmp.h>
jmp_buf env;

void second_stage() {
    volatile int retry_count = 0;  // 必须 volatile
    if (setjmp(env) != 0) {
        retry_count++;
        if (retry_count > 3) return;
    }
    risky_operation(env);  // 失败时 longjmp 回 setjmp 点
}
```

`longjmp` 会跳转回 `setjmp` 点，在此过程中可能修改的自动变量（如 `retry_count`）如果不加 `volatile`，编译器可能将其缓存在寄存器中——`longjmp` 后寄存器被恢复为 `setjmp` 时的快照，计数丢失。

> **面试重点**: `volatile` ≠ 原子操作。`volatile uint32_t` 在多核/多线程系统中不保证原子性。对于多线程共享变量，必须使用 `std::atomic`。`volatile` 的唯一用途是指示"硬件可见"。

---

## 2. `static` —— 从存储期与链接性两个维度理解

`static` 在 C/C++ 中有**三个完全不同的含义**，取决于它修饰的位置。

### 2.1 修饰局部变量：静态存储期 + 首次初始化

```c
void motor_encoder_tick() {
    static uint32_t pulse_count = 0;  // 只在第一次调用时初始化
    pulse_count++;
    if (pulse_count >= 1000) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        pulse_count = 0;
    }
}
```

**底层原理：**

- **存储位置**: 普通局部变量在**栈**上，函数返回后消失。`static` 局部变量在 **`.data` 段**（已初始化）或 **`.bss` 段**（零初始化），与全局变量相同。
- **初始化时机**: 编译期生成一个隐藏的 guard 标志。首次执行到该行时检查 guard，若未初始化则执行初始化并置 guard。这就是线程安全的"懒初始化"（C++11 起标准保证）。
- **生命周期**: 从程序启动到程序结束。
- **可见性**: 仅函数内部可见，函数外不可访问。

**汇编体现：**

```asm
; static uint32_t pulse_count = 0;
pulse_count:        ; 标签在 .bss 段
    .zero 4         ; 4 字节零

; pulse_count++ 的伪汇编:
    ldr r0, .L_pulse_count_addr   ; 取 pulse_count 地址 (在 .bss)
    ldr r1, [r0]                  ; 从内存加载
    add r1, r1, #1                ; +1
    str r1, [r0]                  ; 写回内存
```

### 2.2 修饰全局变量/函数：内部链接

```c
// file_a.c
static int motor_id = 0x42;       // 仅 file_a.c 可见
static void calibrate_sensor() {}  // 仅 file_a.c 可见

// file_b.c
extern int motor_id;  // 链接错误: motor_id 在 file_a 中是 static, 不可见
```

**底层原理：**

- **链接属性**: `static` 将符号从 external linkage（全局可见）改为 internal linkage（仅当前编译单元可见）。
- **ELF 符号表**: `static` 符号在 `.o` 文件中是 `LOCAL` 而非 `GLOBAL`，链接器不会为它们做跨文件符号解析。
- **实用价值**: 避免大型项目中的符号命名冲突。未加 `static` 的全局变量在嵌入式工程中是最常见的命名污染源头。

### 2.3 修饰类成员：类级别共享

```cpp
struct Motor {
    static int active_count;      // 所有 Motor 实例共享, 不占实例内存
    static void emergency_stop(); // 无需实例即可调用
};
int Motor::active_count = 0;      // 类外定义 (分配存储)

// sizeof(Motor) 不包含 active_count
// 等价于全局变量，但命名空间限定在 Motor 作用域内
```

---

## 3. `const` —— 从右向左读，理解"const 修饰谁"

### 3.1 核心规则：const 修饰它左边的内容（左边无物则修饰右边）

```c
const int  a = 10;   // a 是常量: const 修饰 int (右边)
int const  b = 10;   // 与上一行完全等价, b 是常量

// 关键区分:
const int *p;         // p 指向 const int —— 不能改 *p，可以改 p
int const *p;         // 与上一行完全等价
int* const p = &x;    // p 是 const 指针 —— 可以改 *p，不能改 p
const int* const p;   // 两者都不能改
```

### 3.2 从右向左读技巧

```c
          const int *p;    // p is a pointer to an int that is const
// 读法:    <──────    从右往左读: p → * → int → const

          int* const p;    // p is a const pointer to an int
// 读法:    <────       从右往左读: p → const → * → int
```

### 3.3 四个典型组合

| 声明 | 可改 `*p`？ | 可改 `p`？ | 典型场景 |
|------|-----------|-----------|---------|
| `const int *p` | ❌ | ✅ | 函数参数：保证不修改调用方数据 |
| `int* const p` | ✅ | ❌ | 硬件寄存器地址：地址固定但寄存器值可变 |
| `const int* const p` | ❌ | ❌ | 只读 ROM 表：地址和内容都不可变 |
| `int* p` | ✅ | ✅ | 普通可读写指针 |

### 3.4 嵌入式中的三种 `* const` 场景

```c
// 1. MMIO 寄存器 (地址固定, 值 volatile)
volatile uint32_t* const TIM_CR1 = (volatile uint32_t*)0x40010000;
// 读: TIM_CR1 is a const pointer to a volatile uint32_t
// TIM_CR1 本身(地址)不变, 但 *TIM_CR1 随时可能被硬件改变

// 2. 只读参数, 指针可变
void process(const SensorData* p) {  // 不修改 p 指向的传感器数据
    p = p->next;  // 可以移动指针
}

// 3. 只读 ROM 表
const uint16_t* const sin_lut = (const uint16_t*)0x0800F000;
// 地址固定(FLASH 映射), 内容固定(查表数据)
```

### 3.5 `const` 在 GCC 中的优化

GCC 将 `const` 全局变量放入 `.rodata` 段（只读数据段），在嵌入式系统中 `.rodata` 通常位于 Flash 而非 RAM：

```c
const char firmware_version[] = "v3.2.1";  // 占用 Flash, 不占 RAM
       char log_buffer[256];                // 占用 RAM
```

`const` 成员函数（`void get() const`）保证不修改对象成员，允许在 `const` 对象上调用，也为编译器提供了别名分析依据。

---

## 4. `constexpr` 编译期计算

### 4.1 `constexpr` 的演进

```cpp
// C++11: 仅允许单 return 语句
constexpr int square(int x) { return x * x; }

// C++14: 允许多语句
constexpr int factorial(int n) {
    int result = 1;
    for (int i = 2; i <= n; ++i) result *= i;
    return result;
}

// C++17: constexpr lambda
auto sq = [](int x) constexpr { return x * x; };

// C++20: constexpr 动态分配
constexpr auto make_vector() {
    std::vector<int> v;
    v.push_back(42);
    return v;
}

// C++23: constexpr unique_ptr / shared_ptr
constexpr auto p = std::make_unique<int>(10);
```

### 4.2 `const` vs `constexpr`

```c
const int x = rand();     // 运行时常量, 编译期不知道值
constexpr int y = 42;     // 编译期常量, 可用于模板参数、数组大小

int arr1[x];  // VLA (C99), C++ 中可能不合法
int arr2[y];  // 合法, y 是编译期常量
```

### 4.3 GCC 14 中条件式 `constexpr`

```cpp
// gcc-14.1.0/bits/new_allocator.h:87-97
__attribute__((__always_inline__))
_GLIBCXX20_CONSTEXPR   // >= C++20 时展开为 constexpr, 否则为空
__new_allocator() _GLIBCXX_USE_NOEXCEPT { }
```

`_GLIBCXX20_CONSTEXPR` 根据 `__cplusplus` 版本条件展开，保证老标准下不出现语法错误。

---

## 5. `noexcept` 优化

### 5.1 `noexcept` 对代码生成的影响

当函数标记 `noexcept` 时，编译器省略所有栈展开所需的 LSDA (Language-Specific Data Area) 元数据，减小 `.eh_frame` 段体积并提高指令缓存效率。对于热路径中的小型函数（如移动构造、swap），这个优化非常显著。

### 5.2 条件式 `noexcept`

```cpp
// gcc-14.1.0/bits/move.h:214-215
swap(_Tp& __a, _Tp& __b)
_GLIBCXX_NOEXCEPT_IF(__and_<is_nothrow_move_constructible<_Tp>,
                            is_nothrow_move_assignable<_Tp>>::value)
```

（详见 [move_semantics.md](move_semantics.md) 第 4 节和 [raii_exceptions.md](raii_exceptions.md) 第 2 节）

---

## 6. `decltype` 与 `decltype(auto)` 类型推导

### 6.1 `decltype` 的两种模式

```cpp
int x = 42;
int& rx = x;

// 模式 1: decltype(实体) → 保留引用性和 cv 限定符
decltype(rx) y = x;         // y 是 int&
decltype(x)  z = x;         // z 是 int (不是 int&, x 不是引用实体)

// 模式 2: decltype(表达式) → 根据值类别推导
decltype((x))  w = x;       // (x) 是左值表达式 → int& (括号使其成为表达式)
decltype(x + 0) v = x;      // x + 0 是纯右值 → int
```

### 6.2 `decltype(auto)` —— C++14 的完美返回类型推导

```cpp
auto          f1() { int x; return (x); }  // 返回 int   (剥离引用)
decltype(auto) f2() { int x; return (x); } // 返回 int&  (保留引用) → 悬垂引用！
```

（完整内容见原文件对应章节）

---

## 7. `explicit` / `override` / `final` 的防御性编程

```cpp
struct Base {
    virtual void process(const Data& d) = 0;
    virtual ~Base() = default;
};

struct Derived : Base {
    explicit Derived(int id) : _id(id) {}     // 阻止隐式转换
    void process(const Data& d) override;       // 编译器验证重写
    // void process(const Data& d) override final; // 禁止进一步重写
};
```

---

## 8. 应用场景

### 场景 1：`volatile` MMIO 寄存器映射

```c
// STM32 定时器控制寄存器
volatile uint32_t* const TIM1_CR1 = (volatile uint32_t*)0x40010000;
*TIM1_CR1 |= (1 << 0);  // CEN 使能位, 编译器保证立即写入
```

### 场景 2：`static` 实现函数调用计数 (无全局命名污染)

```c
uint32_t get_error_count() {
    static uint32_t count = 0;
    return ++count;
}  // count 在函数外不可见, 但生命周期贯穿整个程序
```

### 场景 3：`constexpr` 编译期生成查找表

```cpp
// 编译期计算 256 个 sin 值, 存入 .rodata (Flash)
constexpr auto sin_table = [] {
    std::array<float, 256> table{};
    for (int i = 0; i < 256; ++i)
        table[i] = std::sin(2 * M_PI * i / 256.0f);
    return table;
}();
float result = sin_table[(uint8_t)angle];  // 运行时仅查表
```

### 场景 4：`const int*` 保证函数不修改数据

```c
void send_packet(const uint8_t* data, size_t len) {
    // data[0] = 0;  // 编译错误: assignment of read-only location
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 100);
}
// 调用方可以放心传入只读缓冲区而不被修改
```

### 场景 5：`volatile` + ISR 实现无锁标志

```c
volatile bool transfer_complete = false;

void DMA_TransferComplete_ISR() {
    transfer_complete = true;  // 硬件保证: 写入内存
}

void wait_for_dma() {
    while (!transfer_complete) {}  // 每次循环重新从内存读取
    transfer_complete = false;
}
```

### 场景 6：`override` 防范静默的非重写

```cpp
class Motor {
    virtual void set_target(float pos) { /* ... */ }
};
class BLDC : public Motor {
    void set_target(float pos) override; // 若基类改为 set_target(double)
    // 此处立即编译错误, 而非静默创建新的重载函数
};
```
