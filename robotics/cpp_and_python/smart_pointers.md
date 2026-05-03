# 指针与引用 —— 从 GCC 14 源码理解智能指针

## 1. `unique_ptr` 源码分析

### 1.1 核心数据结构：`__uniq_ptr_impl`

```cpp
// gcc-14.1.0/bits/unique_ptr.h:142-227
template <typename _Tp, typename _Dp>
    class __uniq_ptr_impl
    {
        ...
    private:
        tuple<pointer, _Dp> _M_t;  // 用 tuple 存储指针和 deleter
    };
```

**原理分析:**

GCC 用 `std::tuple<pointer, _Dp>` 而非两个独立成员来存储指针和 deleter。这利用了 **Empty Base Optimization (EBO)** 和 `tuple` 的压缩优化：当 `_Dp` 是空类（如 `default_delete`、无捕获 lambda）时，`sizeof(__uniq_ptr_impl<T, D>) == sizeof(pointer)`，即零开销。`std::tuple` 内部通过递归继承实现空基类压缩，保证 `unique_ptr<T>` 的大小恰好等于一个原始指针。

### 1.2 `default_delete` —— "默认析构策略"

```cpp
// gcc-14.1.0/bits/unique_ptr.h:68-95
template<typename _Tp>
    struct default_delete
    {
        constexpr default_delete() noexcept = default;

        template<typename _Up,
                 typename = _Require<is_convertible<_Up*, _Tp*>>>
        _GLIBCXX23_CONSTEXPR
        default_delete(const default_delete<_Up>&) noexcept { }

        _GLIBCXX23_CONSTEXPR
        void operator()(_Tp* __ptr) const
        {
            static_assert(!is_void<_Tp>::value,
                          "can't delete pointer to incomplete type");
            static_assert(sizeof(_Tp)>0,
                          "can't delete pointer to incomplete type");
            delete __ptr;
        }
    };
```

**原理分析:**

- `static_assert(sizeof(_Tp)>0)` 阻止对不完整类型调用 `delete`，否则在 release 构建下 `delete` 不完整类型是未定义行为（析构函数不执行、内存可能泄漏）。
- 模板构造函数允许 `default_delete<Derived>` 转换为 `default_delete<Base>`（当 `Derived*` 可转为 `Base*` 时），这保证了多态场景下 `unique_ptr<Base> = unique_ptr<Derived>` 的正确性。但析构时调用的是 `Base` 的析构函数 — 因此 **基类必须有虚析构函数**。

### 1.3 `unique_ptr` 的析构 —— "有条件地 constexpr"

```cpp
// gcc-14.1.0/bits/unique_ptr.h:389-400
#if __cplusplus > 202002L && __cpp_constexpr_dynamic_alloc
    constexpr
#endif
    ~unique_ptr() noexcept
    {
        static_assert(__is_invocable<deleter_type&, pointer>::value,
                      "unique_ptr's deleter must be invocable with a pointer");
        auto& __ptr = _M_t._M_ptr();
        if (__ptr != nullptr)
            get_deleter()(std::move(__ptr));  // 用 move 语义调用 deleter
        __ptr = pointer();
    }
```

**原理分析:**

- 析构函数是 `noexcept`：标准要求 `unique_ptr` 析构不抛异常。如果自定义 deleter 抛异常，直接 `std::terminate`。
- 仅在 C++23 + `__cpp_constexpr_dynamic_alloc` 时标记 `constexpr`：允许 `unique_ptr` 在编译期上下文中使用（如 `constexpr` 容器）。
- `std::move(__ptr)` 传给 deleter：某些定制 deleter 需要知道所有权转移的最后时刻（例如将指针归还给对象池）。

### 1.4 `make_unique` —— 异常安全的工厂函数

```cpp
// gcc-14.1.0/bits/unique_ptr.h:1072-1076
template<typename _Tp, typename... _Args>
    _GLIBCXX23_CONSTEXPR
    inline __detail::__unique_ptr_t<_Tp>
    make_unique(_Args&&... __args)
    { return unique_ptr<_Tp>(new _Tp(std::forward<_Args>(__args)...)); }
```

**原理分析:**

`make_unique` 解决的核心问题是 C++ 表达式求值的未指定顺序：

```cpp
// 危险: 若 new T 先求值，随后 do_something_else() 抛异常，T 泄漏
foo(unique_ptr<T>(new T(args)), do_something_else());

// 安全: make_unique 将分配和构造绑定在同一语句中
foo(make_unique<T>(args), do_something_else());
```

`make_unique_for_overwrite`（C++20）：对于 trivial 类型，调用 `new _Tp` 而非 `new _Tp()`，即默认初始化而非值初始化，避免不必要的零填充开销。

---

## 2. `shared_ptr` 源码分析

### 2.1 引用计数控制块：`_Sp_counted_base`

```cpp
// gcc-14.1.0/bits/shared_ptr_base.h:124-193
template<_Lock_policy _Lp = __default_lock_policy>
    class _Sp_counted_base : public _Mutex_base<_Lp>
    {
    public:
        _Sp_counted_base() noexcept
        : _M_use_count(1), _M_weak_count(1) { }

        void _M_add_ref_copy()
        { __gnu_cxx::__atomic_add_dispatch(&_M_use_count, 1); }

        void _M_release() noexcept;

        void _M_release_last_use() noexcept
        {
            _GLIBCXX_SYNCHRONIZATION_HAPPENS_AFTER(&_M_use_count);
            _M_dispose();    // 析构被管理对象
            if (_Mutex_base<_Lp>::_S_need_barriers)
                __atomic_thread_fence(__ATOMIC_ACQ_REL);
            _GLIBCXX_SYNCHRONIZATION_HAPPENS_BEFORE(&_M_weak_count);
            if (__gnu_cxx::__exchange_and_add_dispatch(&_M_weak_count, -1) == 1)
            {
                _GLIBCXX_SYNCHRONIZATION_HAPPENS_AFTER(&_M_weak_count);
                _M_destroy();    // 销毁控制块自身
            }
        }

    private:
        _Atomic_word _M_use_count;   // 强引用计数
        _Atomic_word _M_weak_count;  // 弱引用计数
    };
```

**原理分析 —— 引用计数的生命周期分离:**

- `_M_use_count` 用于 `shared_ptr`：当降为 0 时调用 `_M_dispose()` 析构被管理对象，然后 `_M_weak_count` 减 1。
- `_M_weak_count` 用于 `shared_ptr` + `weak_ptr`：当降为 0 时调用 `_M_destroy()` 销毁控制块自身（`delete this`）。
- **两个计数分离的意义**：`weak_ptr::lock()` 需要检查被管理对象是否还存在（通过 `_M_use_count`），但即使对象已析构，控制块也必须存活直到所有 `weak_ptr` 也都销毁。
- `_M_add_ref_lock()`（`weak_ptr::lock()` 的实现）用 CAS 循环检查 `_M_use_count == 0`：如果为 0 则返回 false（已过期），否则原子 +1 并返回 true。
- `_GLIBCXX_SYNCHRONIZATION_HAPPENS_*`：给 ThreadSanitizer 的标注，标记 happens-before 关系，避免误报 data race。

### 2.2 `_Lock_policy` —— 多线程安全与单线程优化

```cpp
// gcc-14.1.0/bits/shared_ptr_base.h:105-122
template<>
    class _Mutex_base<_S_mutex>
    : public __gnu_cxx::__mutex
    {
    protected:
        enum { _S_need_barriers = 1 };
    };
```

当编译时指定 `-fno-threadsafe-statics` 或目标平台不支持原子操作时，GCC 退化为 mutex 保护的引用计数。对于单核裸机 MCU 场景，可通过 `_Lock_policy::_S_single` 完全消除同步开销。

### 2.3 `enable_shared_from_this` —— `this` 指针安全共享

```cpp
template<typename _Tp>
    class enable_shared_from_this
    {
        friend class __shared_ptr<_Tp>;
        mutable weak_ptr<_Tp> _M_weak_this;  // 存储 weak_ptr

        shared_ptr<_Tp> shared_from_this()
        { return shared_ptr<_Tp>(_M_weak_this); }
    };
```

当 `shared_ptr` 的构造函数检测到 `_Tp` 继承自 `enable_shared_from_this` 时，会调用 `_M_weak_this._M_assign(this, __n)` 设置内部的 `weak_ptr`。这使得 `shared_from_this()` 返回的是与已有 `shared_ptr` 共享同一控制块的新 `shared_ptr`，而非创建一个独立控制块导致 double free。

---

## 3. 应用场景

### 场景 1：硬件资源的独占所有权 —— `unique_ptr` + 自定义 deleter

```cpp
struct GPIODeleter {
    void operator()(GPIO_TypeDef* gpio) const {
        HAL_GPIO_DeInit(gpio);
    }
};
using GPIOPtr = std::unique_ptr<GPIO_TypeDef, GPIODeleter>;
GPIOPtr motor_enable(GPIOA, GPIODeleter{}); // 离开作用域自动释放
```

无需手动调用 `HAL_GPIO_DeInit`，RAII 保证了即使是异常路径也能正确释放。

### 场景 2：观察者模式 —— `weak_ptr` 打破循环引用

```cpp
class Motor { std::shared_ptr<Encoder> _encoder; }; // owns encoder
class Encoder { std::weak_ptr<Motor>   _motor;   }; // 不增加引用计数
```

Motor 和 Encoder 相互引用时，使用 `weak_ptr` 打破循环，确保两者在外部无人使用时能够正确析构。`weak_ptr::lock()` 在每次访问前验证对象是否存活。

### 场景 3：共享固件镜像 —— `shared_ptr` 避免重复加载

```cpp
std::map<std::string, std::weak_ptr<FirmwareImage>> image_cache;

std::shared_ptr<FirmwareImage> load_firmware(const std::string& path) {
    auto it = image_cache.find(path);
    if (it != image_cache.end()) {
        if (auto img = it->second.lock())
            return img;  // 缓存命中，共享已有镜像
    }
    auto img = std::make_shared<FirmwareImage>(path);
    image_cache[path] = img;  // 存储 weak_ptr，不影响生命周期
    return img;
}
```

`weak_ptr` 作为缓存的键值：当所有外部使用者释放后，`weak_ptr` 自动过期，释放内存。

### 场景 4：中断安全的数据结构 —— 无锁 `atomic<shared_ptr>` (C++20)

```cpp
std::atomic<std::shared_ptr<SensorData>> latest_data;

// ISR 中写入
void ISR_new_data() {
    latest_data.store(std::make_shared<SensorData>(read_sensor()),
                      std::memory_order_release);
}

// 主循环中读取
void main_loop() {
    auto data = latest_data.load(std::memory_order_acquire);
    process(data->values);
}
```

避免了 ISR 和主循环之间的数据竞争，`atomic<shared_ptr>` 内部通过全局 hash table 加锁或 wait-free 算法实现对控制块指针的原子更新。

### 场景 5：工厂函数返回多态对象

```cpp
std::unique_ptr<MotorController> create_motor(MotorType type) {
    switch (type) {
        case MotorType::BLDC:  return std::make_unique<BLDCController>();
        case MotorType::Stepper: return std::make_unique<StepperController>();
        default: return nullptr;
    }
}
```

返回 `unique_ptr<Base>` 而非裸指针，调用方无需关心何时 delete，且类型转换在 `unique_ptr` 内部安全处理（底层利用 `default_delete` 的协变模板构造）。
