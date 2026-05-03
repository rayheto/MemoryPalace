# 并发与多线程 —— 从 GCC 14 源码理解线程与同步

## 1. `std::thread` 源码分析

### 1.1 线程构造 —— 类型擦除 + 状态机

```cpp
// gcc-14.1.0/bits/std_thread.h:151-167
template<typename _Callable, typename... _Args,
         typename = _Require<__not_same<_Callable>>>
    explicit
    thread(_Callable&& __f, _Args&&... __args)
    {
        static_assert(__is_invocable<typename decay<_Callable>::type,
                                     typename decay<_Args>::type...>::value,
          "std::thread arguments must be invocable after conversion to rvalues");

        using _Wrapper = _Call_wrapper<_Callable, _Args...>;
        _M_start_thread(_State_ptr(new _State_impl<_Wrapper>(
            std::forward<_Callable>(__f), std::forward<_Args>(__args)...)),
            _M_thread_deps_never_run);
    }
```

**原理分析:**

- `__not_same<_Callable>` 约束阻止 `thread(thread&&)` 误匹配到构造函数模板（当参数是 `thread` 类型时，模板构造被 SFINAE 排除，转向移动构造）。
- `decay<_Callable>::type` 剥离引用和 cv 限定符后检查可调用性，确保 `is_invocable` 在按值捕获的上下文中正确工作（`std::thread` 按值捕获所有参数到新线程的栈中）。
- `_State_impl<_Wrapper>` 是类型擦除的关键：将任意 callable 包装为统一接口 `_State`（含虚函数 `_M_run()`），允许 `thread` 类本身不依赖模板参数，减小头文件膨胀。
- `unique_ptr<_State>` (`_State_ptr`) 管理类型擦除的堆对象，保证 `thread` 析构时若尚未 join/detach，至少释放 `_State` 对象。

### 1.2 `join()` / `detach()` 与析构语义

```cpp
// gcc-14.1.0/bits/std_thread.h:170-174
~thread()
{
    if (joinable())
        std::__terminate();  // 析构 joinable thread = std::terminate
}
```

**原理分析:**

这是一个严防泄漏的设计：`thread` 对象析构时必须明确线程的归宿。`joinable() == true` 意味着底层 OS 线程仍在运行且未被 detach。此时直接析构会导致：
- 无法知道线程何时结束
- 线程可能正在访问即将回收的栈内存
- 因此标准强制 `std::terminate()`，迫使开发者显式 `join()` 或 `detach()`。

### 1.3 `_M_start_thread` 和 pthread 依赖

```cpp
// gcc-14.1.0/bits/std_thread.h:142-148
static void
_M_thread_deps_never_run() {
#ifdef GTHR_ACTIVE_PROXY
    reinterpret_cast<void (*)(void)>(&pthread_create)();
    reinterpret_cast<void (*)(void)>(&pthread_join)();
#endif
}
```

这个永远不会被调用的函数使用 `reinterpret_cast` 强制生成对 `pthread_create` 和 `pthread_join` 的符号引用。在静态链接时，这确保链接器不会将这两个 pthread 函数当作"未使用"来裁剪——这是针对 `-Wl,--gc-sections` 的防御性措施。

---

## 2. `std::mutex` 源码分析

### 2.1 基于 pthread 的互斥锁

```cpp
// gcc-14.1.0/bits/std_mutex.h:59-138
class __mutex_base
{
protected:
    typedef __gthread_mutex_t __native_type;

#ifdef __GTHREAD_MUTEX_INIT
    __native_type  _M_mutex = __GTHREAD_MUTEX_INIT;
    constexpr __mutex_base() noexcept = default;
#else
    __native_type  _M_mutex;
    __mutex_base() noexcept { __GTHREAD_MUTEX_INIT_FUNCTION(&_M_mutex); }
    ~__mutex_base() noexcept { __gthread_mutex_destroy(&_M_mutex); }
#endif
    __mutex_base(const __mutex_base&) = delete;
    __mutex_base& operator=(const __mutex_base&) = delete;
};

class mutex : private __mutex_base
{
    void lock() {
        int __e = __gthread_mutex_lock(&_M_mutex);
        if (__e) __throw_system_error(__e);
    }
    bool try_lock() noexcept {
        return !__gthread_mutex_trylock(&_M_mutex);
    }
    void unlock() {
        __gthread_mutex_unlock(&_M_mutex);
    }
};
```

**原理分析:**

- `__GTHREAD_MUTEX_INIT` 宏：在支持此宏的平台（如 Linux + glibc），使用 `PTHREAD_MUTEX_INITIALIZER` 静态初始化，此时 `mutex` 的默认构造函数可以是 `constexpr`，无需运行时初始化。这对全局/静态 mutex 的初始化顺序至关重要 —— 避免了 Static Initialization Order Fiasco。
- `__gthread_mutex_t` 在 Linux 上是 `pthread_mutex_t` 的包装，`__gthread_mutex_lock` 调用 `pthread_mutex_lock`。
- `mutex` 是不可复制不可移动的——这与内核 mutex 对象一一对应：每个 `mutex` 对象代表一个内核同步资源。

### 2.2 `scoped_lock` 的原子加锁

C++17 引入的 `std::scoped_lock` 解决了 multi-lock 的顺序死锁问题，其实现使用 `std::lock` 算法（内部使用 try-lock 重试或排序策略保证同时对多个 mutex 加锁且不发生死锁）。

---

## 3. `std::atomic` 内存序

```cpp
// GCC 14 中 atomic 的核心使用模式
// 从 atomic_base.h 提取的关键设计

// store: 原子写入
// __atomic_store_n(&_M_val, __i, __ATOMIC_RELEASE)

// load: 原子读取
// __atomic_load_n(&_M_val, __ATOMIC_ACQUIRE)

// compare_exchange: CAS 循环
// __atomic_compare_exchange_n(&_M_val, &__expect, __desired, false,
//                             __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)
```

GCC 使用 `__atomic_*` builtins 而非 `std::atomic` 实现 `std::atomic` —— 因为 `std::atomic` 自身就由这些 builtins 实现。这些 builtins 直接映射到目标平台的原子指令（x86 的 `lock cmpxchg`、ARM 的 `ldrex/strex`）。

---

## 4. 应用场景

### 场景 1：`call_once` + 懒初始化 —— 无锁的单例

```cpp
std::once_flag motor_profile_flag;
MotorProfile g_profile;

void load_profile() {
    std::call_once(motor_profile_flag, [] {
        g_profile = load_from_eeprom();  // 只执行一次，线程安全
    });
}
```

`call_once` 内部使用原子 flag + futex 或 pthread_once，比 `mutex + double-checked locking` 更高效且不易出错。

### 场景 2：生产者-消费者 —— `condition_variable` + `unique_lock`

```cpp
std::queue<IMUData> data_queue;
std::mutex mtx;
std::condition_variable cv;

// ISR 生产者
void imu_callback(IMUData d) {
    {
        std::lock_guard<std::mutex> lk(mtx);
        data_queue.push(d);
    }
    cv.notify_one();  // 在锁外通知以减少唤醒争用
}

// 消费者线程
void imu_processor() {
    while (running) {
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait(lk, [] { return !data_queue.empty(); });
        auto d = data_queue.front(); data_queue.pop();
        lk.unlock();
        process(d);
    }
}
```

`unique_lock` 而非 `lock_guard`：因为 `cv.wait()` 需要临时 unlock → sleep → 被唤醒后重新 lock。

### 场景 3：无锁数据结构 —— CAS 循环

```cpp
std::atomic<uint32_t> encoder_count = 0;
std::atomic<uint32_t> encoder_overflow = 0;

// ISR 中原子更新
void encoder_timer_callback() {
    uint32_t expected = encoder_count.load(std::memory_order_relaxed);
    while (!encoder_count.compare_exchange_weak(expected, expected + 1,
            std::memory_order_release, std::memory_order_relaxed)) {
        if (expected == UINT32_MAX) {
            encoder_overflow.fetch_add(1, std::memory_order_relaxed);
            expected = 0;
        }
    }
}
```

`compare_exchange_weak` 适用于循环重试场景，比 `strong` 版本少一条分支指令，在 ARM 上有更优的指令生成。

### 场景 4：并行参数扫描 —— `std::async` + `std::future`

```cpp
std::vector<std::future<FOCResult>> futures;
for (float kp = 0.1f; kp <= 2.0f; kp += 0.1f)
    futures.push_back(std::async(std::launch::async,
        [kp] { return tune_foc_pi(kp); }));

for (auto& f : futures) {
    auto result = f.get();  // 阻塞直到该任务完成
    evaluate(result);
}
```

`std::async` 将任务提交到线程池（或按实现创建新线程），`future::get()` 在取结果时如任务未完成则阻塞等待。

### 场景 5：周期任务调度 —— `sleep_until` 精确周期

```cpp
void control_loop(std::stop_token st) {
    auto next = std::chrono::steady_clock::now();
    while (!st.stop_requested()) {
        next += std::chrono::microseconds(100);  // 10kHz 控制周期
        run_foc_loop();
        std::this_thread::sleep_until(next);  // 补偿执行时间
    }
}
```

`sleep_until` 而非 `sleep_for`：补偿每次循环计算的执行时间，避免漂移（drift）。结合 `std::stop_token`（C++20）优雅地停止线程。
