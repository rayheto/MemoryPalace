# 关键字的具体应用 —— 从 GCC 14 源码理解 constexpr/noexcept/decltype

## 1. `constexpr` 编译期计算

### 1.1 `constexpr` 的演进

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

### 1.2 GCC 14 中 `constexpr` 在标准库的体现

从 `new_allocator.h` 中可以观察到，GCC 14 的 allocator 构造和析构函数都标记了 `_GLIBCXX20_CONSTEXPR`：

```cpp
// gcc-14.1.0/bits/new_allocator.h:87-97
__attribute__((__always_inline__))
_GLIBCXX20_CONSTEXPR   // >= C++20 时展开为 constexpr
__new_allocator() _GLIBCXX_USE_NOEXCEPT { }

__attribute__((__always_inline__))
_GLIBCXX20_CONSTEXPR
__new_allocator(const __new_allocator&) _GLIBCXX_USE_NOEXCEPT { }
```

`_GLIBCXX20_CONSTEXPR` 在 `bits/c++config.h` 中定义为：当 C++ 标准 >= 20 时是 `constexpr`，否则为空。这种条件式 `constexpr` 确保向后兼容。

### 1.3 `constexpr` 与 `__is_constant_evaluated()`

```cpp
// gcc-14.1.0/bits/stl_construct.h:111-118
if (std::__is_constant_evaluated())
{
    std::construct_at(__p, std::forward<_Args>(__args)...);
    return;
}
::new((void*)__p) _Tp(std::forward<_Args>(__args)...);
```

`__is_constant_evaluated()`（等价于 C++20 `std::is_constant_evaluated()`）在编译期求值时返回 true，允许选择不同的实现路径：
- 编译期路径：使用 `construct_at`（已标记 constexpr）
- 运行时路径：直接使用 placement new（更高效的机器码）

---

## 2. `noexcept` 优化

### 2.1 `noexcept` 对代码生成的影响

当函数标记 `noexcept` 时，编译器省略所有栈展开 (unwind) 所需的元数据，减小二进制大小并提高指令缓存效率。对于热路径中的小型函数（如移动构造、swap），这个优化非常显著。

### 2.2 条件式 `noexcept`

```cpp
// gcc-14.1.0/bits/move.h:214-215
swap(_Tp& __a, _Tp& __b)
_GLIBCXX_NOEXCEPT_IF(__and_<is_nothrow_move_constructible<_Tp>,
                            is_nothrow_move_assignable<_Tp>>::value)
```

`_GLIBCXX_NOEXCEPT_IF(condition)` 展开为 `noexcept(condition)`：只有当 `_Tp` 的移动构造和赋值都不抛异常时，`swap` 才标记为 `noexcept`。这使得上层模板（如 `vector::swap`）可以据此决定异常安全策略。

### 2.3 `noexcept` 运算符

```cpp
// 编译期检测函数是否会抛异常
static_assert(noexcept(std::declval<int>() + std::declval<int>()), "");
static_assert(!noexcept(std::string("hello").at(100)), ""); // at() 可能抛异常
```

`noexcept(expr)` 是编译期运算符，不实际求值表达式，仅检查表达式的异常规格。

---

## 3. `decltype` 与 `decltype(auto)` 类型推导

### 3.1 `decltype` 的两种模式

```cpp
int x = 42;
int& rx = x;

// 模式 1: decltype(实体) → 保留引用性和 cv 限定符
decltype(rx) y = x;         // y 是 int&
decltype(x)  z = x;         // z 是 int (不是 int&, x 不是引用)

// 模式 2: decltype(表达式) → 根据值类别推导
decltype((x))  w = x;       // (x) 是左值表达式 → int&
decltype(x + 0) v = x;      // x + 0 是纯右值 → int
```

### 3.2 `decltype(auto)` —— C++14 的完美返回类型推导

```cpp
template<typename T>
decltype(auto) forward_like(T&& arg) {  // 保留引用性和 cv 限定
    return std::forward<T>(arg);
}
// 等价于:
// 若 T=int&  → 返回 int&
// 若 T=int&& → 返回 int&&
// 若 T=int   → 返回 int
```

这在 GCC 14 的 `bits/invoke.h:53-56` 中有实际应用：

```cpp
template<typename _Tp, typename _Up = typename __inv_unwrap<_Tp>::type>
    constexpr _Up&&
    __invfwd(typename remove_reference<_Tp>::type& __t) noexcept
    { return static_cast<_Up&&>(__t); }
```

### 3.3 `auto` 与 `decltype(auto)` 的区别

```cpp
auto f1() { int x; return (x); }           // 返回 int   (剥离引用)
decltype(auto) f2() { int x; return (x); }  // 返回 int&  (保留引用) → 悬垂引用！
```

---

## 4. `explicit` / `override` / `final` 的防御性编程

```cpp
struct Base {
    virtual void process(const Data& d) = 0;
    virtual ~Base() = default;
};

struct Derived : Base {
    explicit Derived(int id) : _id(id) {}     // 阻止隐式转换
    void process(const Data& d) override;       // 编译器检查是否真的重写
    // void process(const Data& d) override final; // 禁止进一步重写
};
```

- `explicit`：阻止构造函数的隐式转换，避免意外的类型转换导致的微妙 bug。
- `override`：编译器验证函数签名确实重写了一个 virtual 函数。不加的话，如果基类签名变化，派生类静默创建了一个新的非虚函数。
- `final`：阻止类被继承或虚函数被进一步重写，允许编译器进行去虚拟化优化（devirtualization）。

---

## 5. 应用场景

### 场景 1：`constexpr` 编译期生成查找表

```cpp
// 在编译期计算 256 个 sin 值，存入 .rodata
constexpr auto sin_table = [] {
    std::array<float, 256> table{};
    for (int i = 0; i < 256; ++i)
        table[i] = std::sin(2 * M_PI * i / 256.0f);
    return table;
}();
// 运行时直接查表，零计算开销
float result = sin_table[(uint8_t)angle];
```

### 场景 2：`noexcept` 让 `std::vector` 选择移动而非拷贝

```cpp
struct PIDParams {
    float kp, ki, kd;
    PIDParams(PIDParams&&) noexcept = default; // 必须加 noexcept
};
std::vector<PIDParams> params;
params.emplace_back(1.0f, 0.1f, 0.01f);
params.emplace_back(2.0f, 0.2f, 0.02f);
// 扩容时调用移动构造而非拷贝构造，速度翻倍
```

### 场景 3：`decltype` 推导 HAL 返回值类型

```cpp
template<typename Func, typename... Args>
auto measure_execution_time(Func&& f, Args&&... args)
    -> decltype(std::forward<Func>(f)(std::forward<Args>(args)...))
{
    auto t0 = HAL_GetTick();
    auto result = std::forward<Func>(f)(std::forward<Args>(args)...);
    auto elapsed = HAL_GetTick() - t0;
    log_timing(elapsed);
    return result;
}
```

### 场景 4：`override` 防范编译期错误

```cpp
class Motor {
    virtual void set_target(float pos) { /* ... */ }
};
class BLDC : public Motor {
    void set_target(float pos) override; // 编译器验证签名匹配
    // 若基类改为 set_target(double)，此处立即编译错误而非静默创建新函数
};
```

### 场景 5：`explicit` 防止意外的参数类型转换

```cpp
struct CAN_ID {
    explicit CAN_ID(uint16_t raw) : _raw(raw) {}
    uint16_t _raw;
};
void send_frame(CAN_ID id, const uint8_t* data, size_t len);

send_frame(0x123, buf, 8);  // 编译错误: 0x123 不能隐式转 CAN_ID
send_frame(CAN_ID(0x123), buf, 8); // 正确: 显式构造
```

避免将纯整数误用作协议地址，减少运行时错误。
