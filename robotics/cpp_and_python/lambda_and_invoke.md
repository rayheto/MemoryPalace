# Lambda 表达式与捕获列表 —— 从 GCC 14 源码理解 INVOKE 协议

## 1. Lambda 底层的编译变换

Lambda 表达式是编译器自动生成的匿名函数对象（functor）。捕获列表决定了生成类的成员变量和构造函数：

```cpp
int threshold = 10;
auto pred = [threshold](const SensorData& d) { return d.value > threshold; };
// 编译器生成:
// struct __lambda {
//     int __threshold;                          // 按值捕获
//     auto operator()(const SensorData& d) const { return d.value > __threshold; }
// };
```

捕获方式对照：
- `[=]`：按值捕获所有，成员变量为值类型，`operator()` 是 const
- `[&]`：按引用捕获所有，成员变量为引用类型
- `[x, &y]`：x 按值，y 按引用
- `[this]`：捕获 `this` 指针
- `[*this]` (C++17)：捕获 `*this` 的副本
- `[x = std::move(x)]` (C++14 init-capture)：将外部变量移动进 lambda

---

## 2. `__invoke_impl` —— 统一调用协议的核心

```cpp
// gcc-14.1.0/bits/invoke.h:58-98
template<typename _Res, typename _Fn, typename... _Args>
    constexpr _Res
    __invoke_impl(__invoke_other, _Fn&& __f, _Args&&... __args)
    { return std::forward<_Fn>(__f)(std::forward<_Args>(__args)...); }

template<typename _Res, typename _MemFun, typename _Tp, typename... _Args>
    constexpr _Res
    __invoke_impl(__invoke_memfun_ref, _MemFun&& __f, _Tp&& __t,
                  _Args&&... __args)
    { return (__invfwd<_Tp>(__t).*__f)(std::forward<_Args>(__args)...); }

template<typename _Res, typename _MemFun, typename _Tp, typename... _Args>
    constexpr _Res
    __invoke_impl(__invoke_memfun_deref, _MemFun&& __f, _Tp&& __t,
                  _Args&&... __args)
    { return ((*std::forward<_Tp>(__t)).*__f)(std::forward<_Args>(__args)...); }

template<typename _Res, typename _MemPtr, typename _Tp>
    constexpr _Res
    __invoke_impl(__invoke_memobj_ref, _MemPtr&& __f, _Tp&& __t)
    { return __invfwd<_Tp>(__t).*__f; }

template<typename _Res, typename _MemPtr, typename _Tp>
    constexpr _Res
    __invoke_impl(__invoke_memobj_deref, _MemPtr&& __f, _Tp&& __t)
    { return (*std::forward<_Tp>(__t)).*__f; }
```

**原理分析 —— INVOKE 分派机制:**

`std::invoke`（底层是 `__invoke`）通过 tag dispatch 处理五种调用场景：

1. **`__invoke_other`**：普通函数调用 `f(args...)`
2. **`__invoke_memfun_ref`**：成员函数 + 引用 → `(t.*f)(args...)`，利用 `__invfwd` 保留 `reference_wrapper` 的隐式解引用
3. **`__invoke_memfun_deref`**：成员函数 + 指针/智能指针 → `((*t).*f)(args...)`
4. **`__invoke_memobj_ref`**：成员数据指针 + 引用 → `t.*f`（访问成员变量）
5. **`__invoke_memobj_deref`**：成员数据指针 + 指针 → `(*t).*f`

`__invfwd<_Tp>` 的作用：当 `_Tp` 是 `reference_wrapper<U>` 时，提取出 `U&`，使 `std::ref()` 包装的对象也能传递给需要普通引用的回调。这是 `std::bind` 和 `std::thread` 能正确处理 `std::ref` 的底层原因。

### 2.1 顶层 `std::invoke` 入口

```cpp
// gcc-14.1.0/bits/invoke.h:88-98
template<typename _Callable, typename... _Args>
    constexpr typename __invoke_result<_Callable, _Args...>::type
    __invoke(_Callable&& __fn, _Args&&... __args)
    noexcept(__is_nothrow_invocable<_Callable, _Args...>::value)
    {
        using __result = __invoke_result<_Callable, _Args...>;
        using __type = typename __result::type;
        using __tag = typename __result::__invoke_type;
        return std::__invoke_impl<__type>(__tag{},
            std::forward<_Callable>(__fn),
            std::forward<_Args>(__args)...);
    }
```

`__invoke_result` 编译期推导出两个关键信息：
- `::type`：返回类型
- `::__invoke_type`：tag 类型（`__invoke_other` / `__invoke_memfun_ref` / ...），用于 dispatch

---

## 3. `std::function` 的类型擦除与 SBO

### 3.1 小对象优化 (SBO) 的判断

```cpp
// gcc-14.1.0/bits/std_function.h:120-129
template<typename _Functor>
    class _Base_manager
    {
    protected:
        static const bool __stored_locally =
        (__is_location_invariant<_Functor>::value
         && sizeof(_Functor) <= _M_max_size
         && __alignof__(_Functor) <= _M_max_align
         && (_M_max_align % __alignof__(_Functor) == 0));

        using _Local_storage = integral_constant<bool, __stored_locally>;
    };
```

**原理分析:**

- `__is_location_invariant`：等同于 `is_trivially_copyable`。trivially copyable 的类型可以安全地用 `memcpy` 移动 —— 这对于 SBO buffer 中的对象重定位至关重要。
- `sizeof(_Functor) <= _M_max_size`：`_M_max_size == sizeof(_Nocopy_types)`，即 `sizeof(void*)` × 2（通常 16 字节）。
- `__alignof__(_Functor) <= _M_max_align` 和整除检查：确保目标 buffer 满足类型的对齐要求。

### 3.2 SBO 创建与堆回退

```cpp
// gcc-14.1.0/bits/std_function.h:148-162
template<typename _Fn>
    static void _M_create(_Any_data& __dest, _Fn&& __f, true_type)
    {
        ::new (__dest._M_access()) _Functor(std::forward<_Fn>(__f)); // SBO：就地构造
    }

template<typename _Fn>
    static void _M_create(_Any_data& __dest, _Fn&& __f, false_type)
    {
        __dest._M_access<_Functor*>()
            = new _Functor(std::forward<_Fn>(__f));  // 堆分配：存指针
    }
```

当 functor 足够小且 trivially copyable → 直接存储在 `std::function` 内部的 `_Any_data` union 中（零堆分配）。否则 → 堆分配并将指针存储在 `_Any_data` 中。

---

## 4. 应用场景

### 场景 1：回调注册 —— `std::function` 在中断处理中的边界

```cpp
class MotorDriver {
    std::function<void(uint16_t)> _position_callback;
public:
    void on_position_change(std::function<void(uint16_t)> cb) {
        _position_callback = std::move(cb);
    }
};

// 注册 lambda：捕获 this，用 init-capture 避免悬垂
driver.on_position_change([this](uint16_t pos) {
    this->_last_pos = pos;
    notify_controller(pos);
});
```

### 场景 2：泛型算法与投影 —— C++20 `std::ranges::sort` + lambda

```cpp
std::vector<JointState> joints = get_joint_states();
std::ranges::sort(joints, std::less{}, [](const auto& j) {
    return j.torque;  // 按力矩排序，零拷贝投影
});
```

Lambda 作为投影函数避免了 `std::sort` 需要额外比较器包装的模板膨胀。

### 场景 3：捕获外部变量的生命周期管理 —— init-capture 移动

```cpp
auto buffer = std::make_unique<uint8_t[]>(65536);
// C++14: 将 unique_ptr 移动进 lambda
auto processor = [buf = std::move(buffer)](size_t len) mutable {
    process(buf.get(), len);
};
// buffer 现在是 nullptr，所有权转入 lambda
```

init-capture `[buf = std::move(buffer)]` 解决了 C++11 中无法通过值捕获 move-only 类型的问题。

### 场景 4：`std::bind` 适配参数顺序

```cpp
// HAL_UART_Transmit(&huart1, pData, Size, Timeout)
// 但回调框架要求 void(uint8_t*, uint16_t)，缺少 handle 和 timeout
auto uart_send = std::bind(HAL_UART_Transmit, &huart1,
                           std::placeholders::_1,
                           std::placeholders::_2,
                           100); // 固定 timeout = 100ms
uart_send(buf, len); // 正常调用
```

`std::bind` 底层使用与 lambda 相同的 INVOKE 协议（`__invoke_impl`），`std::placeholders::_1` 等占位符是标记类型，在 `tuple` 展开时被替换为实际参数。

### 场景 5：无捕获 lambda 转为函数指针

```cpp
// 无捕获 lambda 可以隐式转换为 C 函数指针
using TimerCallback = void (*)(void);
TimerCallback cb = [] { HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); };
HAL_TIM_RegisterCallback(&htim2, HAL_TIM_PERIOD_ELAPSED_CB_ID, cb);
```

无捕获 lambda 没有数据成员，编译器直接将其 `operator()` 生成为静态函数，因此可以转换为 C 函数指针——这是 lambda 在嵌入式 HAL 回调中最常用的特性。
