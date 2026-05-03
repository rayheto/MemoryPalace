# 移动语义 —— 从 GCC 14 源码理解 move/forward/swap

## 1. `std::move` 源码分析

```cpp
// gcc-14.1.0/bits/move.h:123-127
template<typename _Tp>
    _GLIBCXX_NODISCARD
    constexpr typename std::remove_reference<_Tp>::type&&
    move(_Tp&& __t) noexcept
    { return static_cast<typename std::remove_reference<_Tp>::type&&>(__t); }
```

**原理分析 —— `move` 只是一个 cast:**

`std::move` 不产生任何机器码。它只是将参数无条件转换为右值引用。真正的"移动"发生在接收方（移动构造函数或移动赋值运算符）对右值引用做出反应时。

关键设计点：
- `_Tp&&` 是转发引用（当 `_Tp` 被推导时）：接受左值、右值、const 任何组合。
- `remove_reference<_Tp>::type&&` 总能得到一个非引用的右值引用类型。
- `[[nodiscard]]`：防止 `std::move(x);` 这样的死代码（只转换不使用）。
- `noexcept`：cast 操作永不抛异常。
- `constexpr`：C++11 起即可在编译期使用。

**类型推导示例：**
- `move(42)` → `_Tp = int`, 返回 `int&&`
- `move(x)` 当 x 是 `int&` → `_Tp = int&`, `remove_reference<int&> = int`, 返回 `int&&`
- `move(x)` 当 x 是 `const int&` → `_Tp = const int&`, 返回 `const int&&`（注意：const 右值引用通常无法被移动构造匹配，会退化到拷贝构造）

---

## 2. `std::forward` 源码分析

```cpp
// gcc-14.1.0/bits/move.h:67-87
template<typename _Tp>
    _GLIBCXX_NODISCARD
    constexpr _Tp&&
    forward(typename std::remove_reference<_Tp>::type& __t) noexcept
    { return static_cast<_Tp&&>(__t); }

template<typename _Tp>
    _GLIBCXX_NODISCARD
    constexpr _Tp&&
    forward(typename std::remove_reference<_Tp>::type&& __t) noexcept
    {
        static_assert(!std::is_lvalue_reference<_Tp>::value,
            "std::forward must not be used to convert an rvalue to an lvalue");
        return static_cast<_Tp&&>(__t);
    }
```

**原理分析 —— 有条件地转换:**

`std::forward` 和 `std::move` 同样只是条件 cast，但核心区别在于 **forward 是有条件的**：

- 当 `_Tp` 是 `int&` 时：`forward<int&>(x)` → `static_cast<int&>(x)` → 返回左值引用（无转换）
- 当 `_Tp` 是 `int` 时：`forward<int>(x)` → `static_cast<int&&>(x)` → 返回右值引用（触发移动）
- 当 `_Tp` 是 `int&&` 时：`forward<int&&>(x)` → `static_cast<int&&>(x)` → 返回右值引用

**完美转发的原理：** 转发引用 `_Tp&&` 在模板推导中保留参数的原始值类别：
```cpp
template<typename T> void wrapper(T&& arg) {
    // arg 作为有名字的变量，它自己是左值
    callee(std::forward<T>(arg));  // forward 恢复 arg 原始的值类别
}
```

第二个重载的 `static_assert`：阻止 `std::forward<int&>(42)` 这种将右值"假扮"为左值的调用，防止产生悬垂引用。

---

## 3. `std::swap` 源码分析

```cpp
// gcc-14.1.0/bits/move.h:203-224
template<typename _Tp>
    _GLIBCXX20_CONSTEXPR
    inline
    typename enable_if<__and_<__not_<__is_tuple_like<_Tp>>,
                              is_move_constructible<_Tp>,
                              is_move_assignable<_Tp>>::value>::type
    swap(_Tp& __a, _Tp& __b)
    _GLIBCXX_NOEXCEPT_IF(__and_<is_nothrow_move_constructible<_Tp>,
                                is_nothrow_move_assignable<_Tp>>::value)
    {
        _Tp __tmp = _GLIBCXX_MOVE(__a);   // 移动到 tmp
        __a = _GLIBCXX_MOVE(__b);          // b 移动到 a
        __b = _GLIBCXX_MOVE(__tmp);        // tmp 移动到 b
    }
```

**原理分析 —— 三移动操作:**

- SFINAE 约束：要求 `is_move_constructible<_Tp>` 和 `is_move_assignable<_Tp>` 都为 true，否则此重载被排除（针对 `std::array` 等有特化 swap 的类型）。
- `__is_tuple_like<_Tp>` 排除项：`std::tuple`/`std::pair` 等有特化 swap 的类型不走这个通用版本。
- `noexcept` 条件：仅当移动构造和移动赋值都不抛异常时，swap 才是 noexcept。这直接影响 `std::vector::swap` 的性能优化路径。
- 在 C++98 模式下，`_GLIBCXX_MOVE(__val)` 展开为 `(__val)`（无操作），退化为三次拷贝。

---

## 4. `std::move_if_noexcept` —— 异常安全的移动

```cpp
// gcc-14.1.0/bits/move.h:130-148
template<typename _Tp>
    struct __move_if_noexcept_cond
    : public __and_<__not_<is_nothrow_move_constructible<_Tp>>,
                    is_copy_constructible<_Tp>>::type { };

template<typename _Tp>
    _GLIBCXX_NODISCARD constexpr
    __conditional_t<__move_if_noexcept_cond<_Tp>::value, const _Tp&, _Tp&&>
    move_if_noexcept(_Tp& __x) noexcept
    { return std::move(__x); }
```

**原理分析:**

`__move_if_noexcept_cond` 的逻辑：如果类型的移动构造函数**可能**抛异常，并且类型**可**拷贝构造，则返回 `const _Tp&`（触发拷贝保证异常安全）；否则返回 `_Tp&&`（触发移动）。这就是 `std::vector` 在 `push_back` 扩容时强异常安全保证的核心机制。

---

## 5. Rule of Five

```cpp
struct MotorConfig {
    float* calibration_data;
    size_t size;

    // 1. 析构函数
    ~MotorConfig() { delete[] calibration_data; }

    // 2. 拷贝构造
    MotorConfig(const MotorConfig& other)
        : calibration_data(new float[other.size]), size(other.size) {
        std::memcpy(calibration_data, other.calibration_data, size * sizeof(float));
    }

    // 3. 拷贝赋值
    MotorConfig& operator=(const MotorConfig& other) { /* copy-swap idiom */ }

    // 4. 移动构造 (noexcept 对 vector 扩容至关重要)
    MotorConfig(MotorConfig&& other) noexcept
        : calibration_data(std::exchange(other.calibration_data, nullptr))
        , size(std::exchange(other.size, 0)) {}

    // 5. 移动赋值
    MotorConfig& operator=(MotorConfig&& other) noexcept {
        delete[] calibration_data;
        calibration_data = std::exchange(other.calibration_data, nullptr);
        size = std::exchange(other.size, 0);
        return *this;
    }
};
```

---

## 6. 应用场景

### 场景 1：`std::vector` 扩容的移动优化

`vector::push_back` 扩容时，如果元素类型的移动构造函数是 `noexcept`，则新容器的元素通过移动构造填充；否则退化到拷贝构造，保证强异常安全。这就是为什么自定义移动构造函数必须加 `noexcept`：

```cpp
static_assert(std::is_nothrow_move_constructible_v<MotorConfig>,
    "MotorConfig must have noexcept move ctor for vector optimization");
```

### 场景 2：工厂函数返回大对象 —— RVO + 移动回退

```cpp
std::vector<float> compute_trajectory(const Path& path) {
    std::vector<float> result;
    result.reserve(path.waypoints * 100);
    // ... 填充
    return result;  // NRVO 优化，零拷贝；若无法 NRVO，自动移动
}
```

C++17 起返回值优化 (RVO) 是强制的；退一步即使 NRVO 不可用，编译器也会优先调用移动构造函数。

### 场景 3：将 unique_ptr 所有权传入容器

```cpp
std::vector<std::unique_ptr<MotorControl>> controllers;
auto bldc = std::make_unique<BLDCControl>();
controllers.push_back(std::move(bldc));  // 必须显式 move
// bldc == nullptr
```

`unique_ptr` 不可拷贝，只能通过 `std::move` 转移所有权入容器。

### 场景 4：`std::exchange` 实现安全的状态转移

```cpp
// ISR 中原子地取出队列并置空
CommandQueue* take_commands() {
    return std::exchange(g_pending_commands, nullptr);
    // 等价于: old = g_pending_commands; g_pending_commands = nullptr; return old;
}
```

`std::exchange` 底层就是 `std::move(old) + assign(new)`，一行完成"提取旧值 + 写入新值"。

### 场景 5：完美转发包装器 —— 为 HAL 添加日志

```cpp
template<typename Func, typename... Args>
auto hal_call(const char* name, Func&& f, Args&&... args)
    -> decltype(std::forward<Func>(f)(std::forward<Args>(args)...))
{
    auto t0 = HAL_GetTick();
    auto result = std::forward<Func>(f)(std::forward<Args>(args)...);
    auto elapsed = HAL_GetTick() - t0;
    printf("[HAL] %s took %lu ms\n", name, elapsed);
    return result;
}
// 调用: hal_call("I2C_Write", HAL_I2C_Master_Transmit, &hi2c1, addr, buf, len, 1000);
```

`forward<Func>` + `forward<Args>` 组合保证了即使 HAL 函数的参数中有右值，也能正确传递，不因包装而引入额外的拷贝开销。
