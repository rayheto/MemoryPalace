# RAII 与异常安全 —— 从 GCC 14 源码理解资源管理

## 1. 异常机制实现：`exception_ptr` 源码

### 1.1 异常的跨线程传递

```cpp
// gcc-14.1.0/bits/exception_ptr.h:97-100
class exception_ptr
{
    void* _M_exception_object;
    // ...
};

exception_ptr current_exception() _GLIBCXX_USE_NOEXCEPT;
void rethrow_exception(exception_ptr) __attribute__ ((__noreturn__));
```

**原理分析:**

- `exception_ptr` 是不透明指针，指向堆上分配的异常对象。它的拷贝是浅拷贝（引用计数或共享所有权），保证多个 `exception_ptr` 指向同一异常对象。
- `current_exception()` 仅在 catch 块内有效：返回当前活跃异常的 `exception_ptr`。如果当前没有异常或异常是"外来"的（foreign exception，如从非 C++ 代码抛出的），返回 null。
- `rethrow_exception()` 被标记为 `__attribute__((__noreturn__))`，告诉编译器此函数不会返回，从而消除调用后的死代码警告并优化寄存器分配。

### 1.2 `make_exception_ptr` —— 从任意异常生成 shared exception

```cpp
// gcc-14.1.0/bits/exception_ptr.h:78
template<typename _Ex>
    exception_ptr make_exception_ptr(_Ex) _GLIBCXX_USE_NOEXCEPT;
```

即使不在 catch 块中，也可以通过 `make_exception_ptr(ex)` 捕获任意异常对象。底层分配新异常对象并返回其 `exception_ptr`。

### 1.3 栈展开 (Stack Unwinding) 与析构顺序

当异常抛出时，运行时从 throw 点沿调用栈向上搜索匹配的 catch 块。在每层栈帧退出时，依次调用所有局部对象的析构函数——这是 RAII 依赖的核心机制。

---

## 2. `noexcept` 优化原理

```cpp
// gcc-14.1.0/bits/move.h:130-148
template<typename _Tp>
    struct __move_if_noexcept_cond
    : public __and_<__not_<is_nothrow_move_constructible<_Tp>>,
                    is_copy_constructible<_Tp>>::type { };
```

**原理分析 —— `move_if_noexcept` 与 vector 扩容:**

当 `vector` 扩容时，需要将旧元素转移到新存储。如果使用移动构造函数，且中途某元素抛出异常，部分元素已移动、部分未移动——无法回滚，因为源对象已被修改。

`move_if_noexcept` 的策略：
- 若移动构造是 `noexcept` → 放心移动，速度最快
- 若移动构造可能抛异常但类型可拷贝 → 回退到拷贝构造，保证强异常安全
- 若移动构造可能抛异常且类型不可拷贝 → 只能移动，接受弱异常安全

这就是为什么自定义移动构造必须标记 `noexcept`：直接影响 vector/deque 等容器的性能。

---

## 3. `scoped_lock` / `lock_guard` —— RAII 的典范

```cpp
// GCC 14 中 lock_guard 的核心设计
template<typename _Mutex>
    class lock_guard
    {
    public:
        explicit lock_guard(_Mutex& __m) : _M_device(__m) { _M_device.lock(); }
        ~lock_guard() { _M_device.unlock(); }
        lock_guard(const lock_guard&) = delete;
        lock_guard& operator=(const lock_guard&) = delete;
    private:
        _Mutex& _M_device;
    };
```

即使临界区内抛出异常，析构函数 `~lock_guard()` 保证调用 `unlock()`，永不造成死锁。这是 RAII 最经典的体现：**资源获取即初始化，资源释放即析构**。

---

## 4. `_Construct` 与异常安全

```cpp
// gcc-14.1.0/bits/stl_construct.h:105-130
template<typename _Tp, typename... _Args>
    inline void
    _Construct(_Tp* __p, _Args&&... __args)
    {
        ::new((void*)__p) _Tp(std::forward<_Args>(__args)...);
    }
```

当 `_Construct` 在 `vector::emplace_back` 内部调用时，如果构造函数抛出异常：
1. 已构造的对象不会被析构（因为构造未完成）
2. `vector` 的异常安全保证取决于"此前的元素是否已经成功转移"
3. 如果此前使用了 `move_if_noexcept` + 拷贝回退，可以保证强异常安全（vector 状态不变）
4. 如果只能移动且某次构造失败，则提供基本异常安全（vector 仍可析构，但内容可能已变化）

---

## 5. 异常安全的三个级别

| 级别 | 保证内容 | 典型实现 |
|------|---------|---------|
| 基本保证 | 对象仍可析构，不泄漏资源 | RAII + 智能指针 |
| 强保证 | 操作要么成功，要么对象状态不变（commit-or-rollback） | copy-and-swap 惯用法 |
| 不抛出保证 | 操作永不抛异常 | `noexcept` 标记 + 内部只用 C 函数 |

---

## 6. 应用场景

### 场景 1：RAII 管理文件/Socket/外设句柄

```cpp
struct FileGuard {
    FILE* _f;
    FileGuard(const char* path) : _f(fopen(path, "r")) {}
    ~FileGuard() { if (_f) fclose(_f); }
    operator FILE*() { return _f; }
    FileGuard(const FileGuard&) = delete;
    FileGuard& operator=(const FileGuard&) = delete;
};
```

无论读取成功与否、是否中途抛异常，`fclose` 一定被调用。无需在 `goto fail` 或 `return -1` 前手动清理。

### 场景 2：`scope_exit` (C++23) 实现异常安全的状态恢复

```cpp
motor.enable();
auto guard = std::experimental::scope_exit([&motor] {
    motor.disable();  // 无论正常返回还是异常，一定执行
});
calibrate_motor(motor);
// guard 析构 → motor.disable()
```

C++23 `std::experimental::scope_exit` 底层就是用 RAII：构造函数接受 callable，析构函数调用它。

### 场景 3：copy-and-swap 实现强异常安全

```cpp
class MotorConfig {
public:
    MotorConfig& operator=(const MotorConfig& other) {
        MotorConfig tmp(other);       // 1. 先拷到临时对象 (可能抛异常)
        tmp.swap(*this);              // 2. swap 不抛异常
        return *this;                 // 3. tmp 析构 → 释放旧资源
    } // 如果步骤 1 抛异常，*this 未修改 → 强保证
};
```

`swap` 被设计为 no-throw（仅交换指针和基本类型），异常只能在拷贝阶段抛出——此时 `*this` 尚未被修改，实现了 commit-or-rollback。

### 场景 4：`shared_ptr` + 自定义 deleter 实现 RAII 外设

```cpp
auto dma_stream = std::shared_ptr<DMA_Handle>(
    DMA_Init(DMA1_Stream0),
    [](DMA_Handle* h) { DMA_DeInit(h); }
);
// 多个 ISR 可以共享 dma_stream，最后一个释放者自动 DMA_DeInit
```

`shared_ptr` 的析构是零异常开销的（deleter 通常 noexcept），而自定义 deleter 提供了 RAII 之外的灵活性。

### 场景 5：`noexcept` 影响 vector 扩容性能的实际测量

```cpp
struct MoveNoexcept { MoveNoexcept(MoveNoexcept&&) noexcept {} };
struct MoveThrowing  { MoveThrowing(MoveThrowing&&) {} };

std::vector<MoveNoexcept> v1;  // 扩容时使用移动 → O(n)
std::vector<MoveThrowing>  v2; // 扩容时使用拷贝 → O(n) 但常数更大
```

编译器在 `vector` 的 `_M_realloc_insert` 中使用 `__move_if_noexcept_cond` 进行编译期分支，`noexcept` 移动构造直接走快速路径。
