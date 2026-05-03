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

**无 volatile —— 生成汇编 (ARM Cortex-M):**

```asm
    ldr  r0, =data_ready   ; ① 将 data_ready 的地址（一个 32 位数）加载到寄存器 r0
                           ;    =data_ready 是伪指令，汇编器会自动翻译成 PC 相对寻址
    ldrb r1, [r0]          ; ② 以 r0 的值作为内存地址，读 1 个字节到 r1
                           ;    ldrb = LoaD Register Byte，方括号表示"以地址访问内存"
    cmp  r1, #0            ; ③ 比较 r1 和 #0（# 表示立即数, #0 就是数字 0）
                           ;    cmp 的结果会写入 CPSR（状态寄存器）的 N/Z/C/V 标志位
    bne  .L_after_loop     ; ④ 如果标志位表示"不相等"（r1 != 0），跳转到 .L_after_loop
                           ;    bne = Branch if Not Equal
.L_spin:                   ;    ← 标签（Label），标记一个代码位置，仅供跳转指令使用
    b    .L_spin           ; ⑤ 无条件跳转回 .L_spin → 死循环
                           ;    不会再次从内存读 data_ready，r1 的值没有改变
```

**执行流程：** 读 data_ready 一次 → 如果是 true 就跳到后面 → 如果是 false 就死锁在 `.L_spin`。关键在第⑤条：**永远不会再回到第②条重新读内存**。ISR 此刻修改 `data_ready` 为 true 也没用——CPU 在死循环里不读内存了。

**有 volatile —— 生成汇编:**

```asm
.L_loop:                   ;    ← 标签，循环入口
    ldrb r1, [r0]          ; ① 每次循环都从内存重新读 data_ready 到 r1
    cmp  r1, #0            ; ② 比较 r1 和 0
    beq  .L_loop           ; ③ 如果相等（r1 == 0），跳回 .L_loop 重新读
                           ;    beq = Branch if EQual
                           ;    如果不等（r1 != 0，即 data_ready 为 true），顺序走到下一步
```

**执行流程：** 读 data_ready → 比较 → 为 false 则跳回重新读 → 为 true 则离开循环。关键在第①条：**每次循环都有一条 `ldrb` 指令**，实实在在从内存地址读取。ISR 在某次 `ldrb` 之后把 `data_ready` 改成 true，下一次循环的 `ldrb` 就能读到新值。

> **新手记忆口诀:** `ldr` = LoaD Register（从内存加载到寄存器），`str` = STore Register（从寄存器写回内存）。带 `b` 后缀 (`ldrb`, `strb`) = 只操作 1 字节。`b` 单独出现 = Branch（跳转）。`[]` = "把这个寄存器的值当作内存地址去访问"。

**场景三：`setjmp` / `longjmp` 跨栈帧变量**

### 3.1 `setjmp` / `longjmp` 是什么？

这是 C 语言提供的"非局部跳转"机制，可以理解为**跨函数的超级 goto**：

- `setjmp(env)`：把当前 CPU 的关键状态（程序计数器 PC、栈指针 SP、部分寄存器）保存到 `env` 结构体中。首次调用返回 0。
- `longjmp(env, val)`：把 CPU 状态恢复到 `env` 保存的那一瞬间。程序**不返回 longjmp 调用点**，而是"穿越"回 `setjmp` 当初返回的位置——但这次 `setjmp` 返回 `val` 而不是 0。

就像游戏存档/读档：`setjmp` 是存档，`longjmp` 是回退存档。

### 3.2 具体示例：硬件通信故障重试

```c
#include <setjmp.h>
jmp_buf checkpoint;

void send_motor_command(uint8_t cmd_id) {
    volatile int attempts = 0;   // 必须在内存中，不能只放在寄存器里

    if (setjmp(checkpoint) != 0) {
        // 走到这里 = 上一次发送失败了（longjmp 跳回来的）
        attempts++;
        if (attempts > 3) {        // 重试 3 次仍失败则放弃
            log_error("motor command failed");
            return;
        }
        HAL_Delay(10);            // 等 10ms 再重试
    }

    // 发送命令（中途可能失败）
    HAL_UART_Transmit(&huart_motor, &cmd_id, 1, 50);
    wait_for_ack(checkpoint);     // 若超时未收到 ACK，内部调用 longjmp(checkpoint, 1)
}
```

### 3.3 无 `volatile` 时的逐步骤分析

当 `attempts` 没有 `volatile` 修饰时，编译器会把它放在寄存器中（如 r5）。以下是完整的时间线：

```
步骤 1: attempts = 0
        → 汇编: mov r5, #0         （r5 是编译器分配给 attempts 的寄存器）

步骤 2: setjmp(checkpoint)
        → 汇编: 把当前所有寄存器的值写入 checkpoint 结构体
                r5 = 0 也被写入   ← "存档"
        → 返回 0
        → 代码进入 else 分支

步骤 3: HAL_UART_Transmit(...)
        → 发送电机命令

步骤 4: wait_for_ack(checkpoint)
        → 内部等 ACK 超时
        → 调用 longjmp(checkpoint, 1)

步骤 5: longjmp 执行"读档"操作
        → PC 跳回步骤 2 setjmp 的返回位置
        → 所有寄存器恢复为步骤 2 存档时的值
        → r5 被恢复为 0    ← 致命！attempts 从 1 回退为 0

步骤 6: setjmp 这次返回 1（非 0，表示"是 longjmp 跳回来的"）

步骤 7: attempts++
        → 汇编: add r5, r5, #1    （r5 现在是步骤 5 恢复的 0, 加 1 得 1）
        → attempts 本应有累加效果，但每次都从 0 开始

第二次 longjmp 后:
步骤 8: longjmp 再次触发 → r5 又双叒被恢复为快照中的 0
步骤 9: attempts++ → r5 = 1      ← 永远跳不过 3！
```

**根因：** `longjmp` 恢复的是 `setjmp` 时刻的**寄存器快照**。`attempts` 的值存储在寄存器中，所以每次 `longjmp` 都把它重置回存档时的初始值（0）。计数永远累加不到 3，重试变成无限循环。

### 3.4 有 `volatile` 时为什么就正确了？

`volatile` 强制 `attempts` 的每次读写都走栈上内存，不用寄存器缓存：

| 操作 | 无 volatile | 有 volatile |
|------|-----------|-----------|
| `attempts = 0` | `mov r5, #0` | `mov r0, #0; str r0, [sp, #8]` |
| `attempts++` | `add r5, r5, #1` | `ldr r0, [sp, #8]; add r0, #1; str r0, [sp, #8]` |
| `longjmp` 后 | r5 被快照覆盖为 0 | 栈上内存不受 longjmp 影响，值为之前累加的结果 |

`longjmp` 只恢复**寄存器**（CPU 上下文），不回溯**栈上内存**。`volatile` 把变量赶到栈上，它的值在 longjmp 前后保持不变。

### 3.5 直观类比

- **无 volatile**：你只在脑子里记着一个数（寄存器）。磕到头晕了一下（longjmp），脑子里的记忆回到上次清醒时，中间想的东西全部丢失。
- **有 volatile**：你每改一次数字就写在纸上（栈内存）。晕过去再醒来，纸上的内容还在，看一眼纸就能继续。

> **面试重点**: `volatile` ≠ 原子操作。`volatile uint32_t` 在多核/多线程系统中不保证原子性。对于多线程共享变量，必须使用 `std::atomic`。`volatile` 的唯一用途是指示"硬件可见"。

> **额外说明：** `setjmp`/`longjmp` 在嵌入式开发中很少使用（通常用状态机或异常处理代替）。这里讲解它，目的是帮你理解 **"变量在寄存器 vs 在内存"这个底层差异**——这个差异在 ISR 场景中同样存在，但 setjmp/longjmp 让它更直观可见。

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

## 7. `override` —— 虚函数表与继承规则

### 7.1 虚函数表（vtable）是什么？

当一个类包含虚函数时，编译器会为它生成一张**虚函数表（vtable）**——本质上是一个函数指针数组。每个对象的前 8 字节（64 位系统）存储一个指向这张表的**隐藏指针 vptr**。

```cpp
class Motor {
public:
    virtual void init()    { /* 默认初始化 */ }
    virtual void run()     { /* 默认运行 */ }
    virtual void stop()    { /* 默认停止 */ }
    virtual ~Motor() {}
};
```

**内存布局（64 位系统）:**

```
Motor 对象内存:
┌─────────────┬──────────────┐
│ vptr (8字节) │ 成员变量...   │
└──────┬──────┴──────────────┘
       │
       └────────────────────────────────────────┐
                                                ▼
Motor 的 vtable (存放在 .rodata 或 .data.rel.ro):
┌──────────────────┬───────────────────────────────┐
│  type_info*      │ 指向 std::type_info (RTTI)     │
├──────────────────┼───────────────────────────────┤
│ offset 0: &Motor::init   │ 第一个虚函数的函数指针    │
│ offset 1: &Motor::run    │ 第二个虚函数的函数指针    │
│ offset 2: &Motor::stop   │ 第三个虚函数的函数指针    │
│ offset 3: &Motor::~Motor │ 析构函数的函数指针        │
└──────────────────┴───────────────────────────────┘
```

**调用虚函数时的实际汇编指令：**

```cpp
Motor* m = get_motor();
m->run();  // 虚函数调用
```

```asm
; m->run() 的本质——通过 vptr 查 vtable，跳转到对应的函数
    ldr  r0, [sp, #8]        ; r0 = m (对象的地址)
    ldr  r1, [r0]            ; r1 = *(m) = vptr，即 vtable 的首地址
    ldr  r2, [r1, #8]        ; r2 = *(vtable + 8) = vtable[1] = &Motor::run
                              ;     +8 是因为 vtable[0] 是 init, run 在第二个槽位
    blx  r2                  ; 调用 r2 指向的函数
```

三步走：读对象 → 读 vptr → 读 vtable 中对应槽位的函数指针 → 跳转。这就是"运行时多态"的底层实现。

### 7.2 继承时 vtable 如何变化？

```cpp
class BLDC : public Motor {
public:
    void init() override { /* BLDC 专用初始化 */ }
    void run()  override { /* BLDC 专用运行 - FOC 算法 */ }
    void stop() override { /* BLDC 专用停止 */ }
    virtual void calibrate() { /* 新增虚函数, 父类没有 */ }
};
```

**继承后的 vtable 构造过程：**

```
★ 第一步: 从父类 Motor 复制 vtable 模板:
   Motor vtable:   [&Motor::init, &Motor::run, &Motor::stop, &Motor::~Motor]

★ 第二步: 子类覆盖了哪些函数就替换对应槽位:
   BLDC vtable:    [&BLDC::init, &BLDC::run, &BLDC::stop, &Motor::~Motor,
                    新槽位 →     &BLDC::calibrate]
                     ↑           ↑           ↑
                   覆盖了      覆盖了      覆盖了
                   init        run         stop

★ 结果: BLDC vtable 包含 5 个槽位（父类 4 个 + 新增 1 个）
```

**关键规则：**

- 子类的 vtable **前 N 个槽位与父类布局完全一致**（保证了多态的正确性）
- 只要函数签名与父类严格匹配，编译器就替换对应槽位
- **签名不匹配 = 新增槽位，而非覆盖**——这就是没 `override` 时的隐患

### 7.3 无 `override` 时的"静默新建"问题

假设基类作者在重构时改了函数签名：

```cpp
// 旧版 Motor:
class Motor {
    virtual void set_target(float pos);  // 参数是 float
};

// BLDC 作者写的代码（旧的 set_target）:
class BLDC : public Motor {
    void set_target(float pos) { /* FOC 位置控制 */ }  // 意图是覆盖
};

// ====== 后来基类更新了 ======
// 新版 Motor:
class Motor {
    virtual void set_target(double pos);  // 参数改成了 double  ← 签名变了！
};

// BLDC 的代码静默失效:
class BLDC : public Motor {
    void set_target(float pos) { ... }  // 不再覆盖！这是一个全新的虚函数
    // 原因: float ≠ double，签名不同，编译器认为这是两个不同的函数
};
```

**没有 `override` 时的 vtable 对比：**

```
BLDC vtable (基类签名变化前):            BLDC vtable (基类签名变化后):
[&Motor::SetUp,                          [&Motor::SetUp,
 &BLDC::set_target(float),  ← 覆盖        &Motor::set_target(double),  ← 没覆盖！用父类默认
 ...]                                     &BLDC::set_target(float),   ← 这是新函数, 新槽位
                                           ...]

调用方:                                  调用方:
m->set_target(0.5f);                    m->set_target(0.5f);
→ 走 BLDC::set_target(float)            → 走 Motor::set_target(double)
                                           FOC 控制逻辑不执行！电机不动！
```

**结果：** 代码编译无警告，但运行时调用的是父类的 `Motor::set_target(double)`，子类的 FOC 控制逻辑完全被绕过——电机不转、关节不动，而且没有任何编译期或运行时报错。

### 7.4 加上 `override` 后

```cpp
class BLDC : public Motor {
    void set_target(float pos) override;  // 编译错误！
    // error: 'void BLDC::set_target(float)' marked 'override', but does not override
};
```

编译器做的事情：
1. 检查 `BLDC` 的所有父类（`Motor` 以及 `Motor` 的父类）中是否有**签名完全一致**的虚函数
2. 如果找不到签名匹配的虚函数 → **编译错误**
3. 如果找到了 → 在 vtable 中替换对应槽位

**`override` 是一个零运行时开销的关键字。** 它只在编译期起作用，生成的汇编代码与不加 `override` 时完全相同。它的唯一目的就是让编译器帮你验证"你以为你在覆盖，实际上你真的在覆盖"。

### 7.5 `final` —— 阻断虚函数链

```cpp
class BLDC : public Motor {
    void run() override final;  // 覆盖, 且禁止子类再覆盖
};

class UltraBLDC : public BLDC {
    void run() override;  // 编译错误: run 在 BLDC 中是 final
};
```

`final` 的另一重作用：帮助编译器做**去虚拟化优化（devirtualization）**。如果编译器确定某个对象就是 `BLDC` 类型（而非 `BLDC*` 指向的未知子类），它可以跳过 vtable 查表，直接调用 `BLDC::run()`：

```asm
; 无 final: 必须走 vtable 间接调用
    ldr  r0, [obj_ptr]
    ldr  r1, [r0]           ; 读 vptr
    ldr  r2, [r1, #8]       ; 读 vtable 槽位
    blx  r2                 ; 间接跳转 (分支预测困难, 指令缓存 miss)

; 有 final 且类型明确: 直接调用
    bl   BLDC::run          ; 直接跳转 (分支预测准确, 可能 inline)
```

---

## 8. `explicit` 的防御性编程

```cpp
struct CAN_ID {
    explicit CAN_ID(uint16_t raw) : _raw(raw) {}
    uint16_t _raw;
};
void send_frame(CAN_ID id, const uint8_t* data, size_t len);

send_frame(0x123, buf, 8);       // 编译错误: 不能隐式转换
send_frame(CAN_ID(0x123), buf, 8); // 正确
```

`explicit` 阻止构造函数参与隐式类型转换，避免将纯整数误用作协议地址。

---

## 9. 应用场景

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
