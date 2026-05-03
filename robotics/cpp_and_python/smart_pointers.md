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

## 3. `weak_ptr` 源码分析 —— 不增引用计数的观察者

### 3.1 `__weak_ptr` 类的数据成员

```cpp
// bits/shared_ptr_base.h:1978-1993 (GCC 14.1.0)
class __weak_ptr
{
public:
    using element_type = typename remove_extent<_Tp>::type;

    constexpr __weak_ptr() noexcept
    : _M_ptr(nullptr), _M_refcount()      // ① _M_refcount 默认构造为空
    { }

    // 从 shared_ptr 构造
    template<typename _Yp, typename = _Compatible<_Yp>>
        __weak_ptr(const __shared_ptr<_Yp, _Lp>& __r) noexcept
        : _M_ptr(__r._M_ptr), _M_refcount(__r._M_refcount)  // ② 共享控制块, 但不增加 _M_use_count
        { }

private:
    element_type*  _M_ptr;          // 指向被管理对象的裸指针
    __weak_count<_Lp> _M_refcount;  // 弱引用计数管理
};
```

**关键：** 从 `shared_ptr` 构造 `weak_ptr` 时，只拷贝了 `_M_refcount`（控制块指针），但《没有调用 `_M_add_ref_copy()`》。这意味着 `_M_use_count` 不增加——被管理对象可能在 `weak_ptr` 存在期间被销毁。

### 3.2 `_M_weak_count` 的独立生命周期

```c
控制块的生命周期 = 最长的 { 最后一个 shared_ptr 的销毁, 最后一个 weak_ptr 的销毁 }

在 _Sp_counted_base 初始化时:
  _M_use_count = 1   (对应第一个 shared_ptr)
  _M_weak_count = 1  (对应"shared_ptr 存在"这个事实, 也即 #weak_ptr + 1)

weak_ptr 拷贝: _M_weak_count++     (line 204)
weak_ptr 析构: _M_weak_count--     (line 211-216)
  → 当 _M_weak_count == 0 时 → _M_destroy() → delete this

shared_ptr 的 _M_release_last_use():
  _M_dispose()             ← 先销毁被管理对象
  _M_weak_count--          ← 再通知"没有 shared_ptr 了"
    → 此时若 _M_weak_count == 0 → 说明也没有弱引用者 → _M_destroy()
```

**内存时序图：**

```
时间 →  创建 share1   创建 weak1    share1 销毁      weak1 销毁
         │            │             │                │
_M_use: 1            1             0 (dispose obj)   0
_M_weak:1            2             1                0 (delete this)

关键: share1 销毁后对象已析构, 但控制块还活着(weak_count=1, weak1 还在引用它)
      当 weak1 也析构后(weak_count→0), 控制块才被 delete
```

### 3.3 `lock()` —— 尝试晋升为 shared_ptr

```cpp
// bits/shared_ptr_base.h:2070-2072 (GCC 14.1.0)
__shared_ptr<_Tp, _Lp>
lock() const noexcept
{ return __shared_ptr<element_type, _Lp>(*this, std::nothrow); }
```

`lock()` 委托给 `__shared_ptr` 的一个特殊构造函数，该构造函数调用 `_M_add_ref_lock_nothrow()`：

```cpp
// bits/shared_ptr_base.h:1245-1256 (GCC 14.1.0)
// shared_ptr 的弱锁构造 (由 weak_ptr::lock() 调用)
template<typename _Yp, typename _Lp>
    __shared_ptr(const __weak_ptr<_Yp, _Lp>& __r, std::nothrow_t)
    : _M_ptr(), _M_refcount(__r._M_refcount)  // 先拷贝控制块
{
    if (_M_pi == nullptr || !_M_pi->_M_add_ref_lock_nothrow())
    {
        _M_ptr = nullptr;    // 若对象已死, shared_ptr 为空
        _M_refcount = __shared_count<_Lp>();  // 释放控制块引用
    }
    else
        _M_ptr = __r._M_ptr; // 成功提升: 对象还活着, _M_use_count 已 +1
}
```

### 3.4 `_M_add_ref_lock_nothrow()` 的三种实现

```cpp
// 实现一: _S_single (单线程, bits/shared_ptr_base.h:244-249)
bool _M_add_ref_lock_nothrow() noexcept
{
    if (_M_use_count == 0)
        return false;       // 对象已死
    ++_M_use_count;          // 原子 +1（单线程不需要原子指令）
    return true;
}

// 实现二: _S_atomic (多线程, bits/shared_ptr_base.h:269-283)
// ★ 这是最重要的版本 —— CAS 无锁循环
bool _M_add_ref_lock_nothrow() noexcept
{
    _Atomic_word __count = _M_get_use_count();
    do
    {
        if (__count == 0)
            return false;    // ① 发现 use_count 为 0 → 对象已死, 立即返回 false
    }
    while (!__atomic_compare_exchange_n(
        &_M_use_count,        // ② 尝试 CAS: 如果 _M_use_count 还是 __count
        &__count,              //    就替换为 __count + 1
        __count + 1,           // ③ CAS 失败 → __count 被更新为新值, 循环重试
        true, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED));
    return true;              // ④ CAS 成功 → use_count 已原子+1 → shared_ptr 构造有效
}

// 实现三: _S_mutex (锁保护, bits/shared_ptr_base.h:255-263)
bool _M_add_ref_lock_nothrow() noexcept
{
    __gnu_cxx::__scoped_lock sentry(*this);
    if (__gnu_cxx::__exchange_and_add_dispatch(&_M_use_count, 1) == 0)
    {
        _M_use_count = 0;  // 发现已为 0, 回退
        return false;
    }
    return true;
}
```

**CAS 版本的原理解析：**

```
假设两个线程同时调用 weak_ptr::lock():

线程 A                           线程 B
────────────────────────────────────────────────────
read count=1                     read count=1
CAS(1→2) 成功                     CAS(1→2) 失败 (已被 A 改成 2)
use_count=2                      重新 read count=2
return true                      CAS(2→3) 成功
                                 use_count=3
                                 return true

→ 两个 shared_ptr 都获取成功, use_count 正确递增到 3

过一段时间, 对象被销毁:

线程 C                          线程 D
────────────────────────────────────────────────────
read count=0                     read count=0
return false (对象已死)           return false (对象已死)

→ 两者都得到空 shared_ptr, 安全
```

### 3.5 `expired()` —— 检查对象是否存活

```cpp
bool expired() const noexcept
{ return _M_refcount._M_get_use_count() == 0; }
// 等价于 use_count() == 0
```

`expired()` 只是读取 `_M_use_count`。但它有竞态窗口：检查 `expired()` 为 false 后，另一个线程可能同时销毁最后一个 `shared_ptr`，导致 `lock()` 失败。**正确做法是直接调用 `lock()` 而非先 `expired()` 再 `lock()`。**

---

## 4. 应用场景

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
