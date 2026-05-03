# 内存管理 —— 从 GCC 14 标准库源码理解分配与释放

## 1. `std::allocator` 底层实现

GCC 14 中 `std::allocator` 继承自 `__gnu_cxx::new_allocator`，其核心实现在 `bits/new_allocator.h`。

### 1.1 `allocate()` —— 从 `operator new` 到对齐分配

```cpp
// gcc-14.1.0/bits/new_allocator.h:126-152
_GLIBCXX_NODISCARD _Tp*
allocate(size_type __n, const void* = static_cast<const void*>(0))
{
#if __cplusplus >= 201103L
    static_assert(sizeof(_Tp) != 0, "cannot allocate incomplete types");
#endif
    if (__builtin_expect(__n > this->_M_max_size(), false))
    {
        if (__n > (std::size_t(-1) / sizeof(_Tp)))
            std::__throw_bad_array_new_length();
        std::__throw_bad_alloc();
    }

#if __cpp_aligned_new
    if (alignof(_Tp) > __STDCPP_DEFAULT_NEW_ALIGNMENT__)
    {
        std::align_val_t __al = std::align_val_t(alignof(_Tp));
        return static_cast<_Tp*>(_GLIBCXX_OPERATOR_NEW(__n * sizeof(_Tp), __al));
    }
#endif
    return static_cast<_Tp*>(_GLIBCXX_OPERATOR_NEW(__n * sizeof(_Tp)));
}
```

**原理分析:**

- `_M_max_size()` 返回 `size_t(-1) / sizeof(_Tp)`，即理论最大可分配对象数。当 `__n` 超过该值时，触发 `bad_array_new_length`（C++17 起）或 `bad_alloc` 异常。
- `__builtin_expect(__n > this->_M_max_size(), false)` 是分支预测提示：告诉编译器溢出路径是冷路径，应优化热路径的指令布局。
- `__cpp_aligned_new`：如果类型对齐要求超过 `__STDCPP_DEFAULT_NEW_ALIGNMENT__`（通常为 16），则调用带 `align_val_t` 重载的 `operator new`。这对应 C++17 的对齐 new 特性。
- `_GLIBCXX_OPERATOR_NEW`：在支持 `__builtin_operator_new` 的编译器上直接用 builtin，否则是 `::operator new`。builtin 版本允许编译器进行更多优化（如省略不必要的 new 调用）。

### 1.2 `deallocate()` —— 对齐感知的释放

```cpp
// gcc-14.1.0/bits/new_allocator.h:155-173
void
deallocate(_Tp* __p, size_type __n __attribute__ ((__unused__)))
{
#if __cpp_sized_deallocation
# define _GLIBCXX_SIZED_DEALLOC(p, n) (p), (n) * sizeof(_Tp)
#else
# define _GLIBCXX_SIZED_DEALLOC(p, n) (p)
#endif

#if __cpp_aligned_new
    if (alignof(_Tp) > __STDCPP_DEFAULT_NEW_ALIGNMENT__)
    {
        _GLIBCXX_OPERATOR_DELETE(_GLIBCXX_SIZED_DEALLOC(__p, __n),
                                 std::align_val_t(alignof(_Tp)));
        return;
    }
#endif
    _GLIBCXX_OPERATOR_DELETE(_GLIBCXX_SIZED_DEALLOC(__p, __n));
}
```

**原理分析:**

- `__cpp_sized_deallocation`（C++14）：如果支持，调用带 size 参数的 `operator delete(void*, size_t)`，使内存分配器可以基于对象大小进行更高效的回收（如大小类缓存 Small Size Cache）。
- 对齐路径与分配时对称：若 `alignof(_Tp) > __STDCPP_DEFAULT_NEW_ALIGNMENT__`，必须调用带 `align_val_t` 的 delete，否则属于未定义行为。

### 1.3 `construct()` 与 `destroy()` —— Placement New

```cpp
// gcc-14.1.0/bits/new_allocator.h:186-198 (C++11 版本)
template<typename _Up, typename... _Args>
    void
    construct(_Up* __p, _Args&&... __args)
    noexcept(std::is_nothrow_constructible<_Up, _Args...>::value)
    { ::new((void *)__p) _Up(std::forward<_Args>(__args)...); }

template<typename _Up>
    void
    destroy(_Up* __p)
    noexcept(std::is_nothrow_destructible<_Up>::value)
    { __p->~_Up(); }
```

**原理分析:**

- `construct` 本质是 placement new：在已分配的内存地址上调用构造函数。`std::forward<_Args>(__args)...` 保证参数完美转发。
- `noexcept` 条件式声明：根据构造/析构函数是否会抛异常来推导 `noexcept` 规格，这影响 `vector::push_back` 等容器的强异常安全保证策略。
- C++20 起标准不再要求 allocator 提供 `construct`/`destroy`，改用 `std::construct_at` / `std::destroy_at`。

---

## 2. `std::construct_at` 与 `std::destroy_at` (C++17/20)

### 2.1 源码实现

```cpp
// gcc-14.1.0/bits/stl_construct.h:78-98
template <typename _Tp>
    _GLIBCXX20_CONSTEXPR inline void
    destroy_at(_Tp* __location)
    {
        if constexpr (__cplusplus > 201703L && is_array_v<_Tp>)
        {
            for (auto& __x : *__location)
                std::destroy_at(std::__addressof(__x));
        }
        else
            __location->~_Tp();
    }

template<typename _Tp, typename... _Args>
    constexpr auto
    construct_at(_Tp* __location, _Args&&... __args)
    noexcept(noexcept(::new((void*)0) _Tp(std::declval<_Args>()...)))
    -> decltype(::new((void*)0) _Tp(std::declval<_Args>()...))
    { return ::new((void*)__location) _Tp(std::forward<_Args>(__args)...); }
```

**原理分析:**

- `destroy_at` 的 `if constexpr (is_array_v<_Tp>)`：C++20 起支持对数组类型的析构，编译器自动展开循环逐个销毁数组元素。这解决了 `unique_ptr<T[]>` 的正确析构问题。
- `construct_at` 的返回类型推导：使用 `decltype(::new((void*)0) _Tp(...))` 而非显式写 `_Tp*`，以正确支持重载了 `operator new` 返回非 `void*` 的奇葩类型。
- `noexcept` 条件：构造一个 dummy 对象检查是否会抛异常，确保 `noexcept` 规格与实际的构造函数行为一致。

---

## 3. `operator new` / `operator delete` 可替换机制

标准库的 `allocate` 最终调用 `::operator new(size_t)`，这是一个可替换的全局函数：

```cpp
// 用户可替换 operator new 的四种重载
void* operator new(std::size_t count);                          // 普通 new
void* operator new(std::size_t count, std::align_val_t al);     // 对齐 new (C++17)
void* operator new(std::size_t count, const std::nothrow_t&);   // 不抛异常 new
void operator delete(void* ptr) noexcept;                       // 普通 delete
void operator delete(void* ptr, std::size_t sz) noexcept;       // 带大小 delete (C++14)
void operator delete(void* ptr, std::align_val_t al) noexcept;  // 对齐 delete (C++17)
```

GCC 14 内部用 `__builtin_operator_new` / `__builtin_operator_delete`（当 `__has_builtin(__builtin_operator_new) >= 201802L` 时），让编译器在编译期识别 `new`/`delete` 调用，从而实现：
- 消除不必要的堆分配（heap elision）
- 将多个小分配合并
- 常量表达式中的动态内存（C++20 `constexpr` 动态分配）

---

## 4. 应用场景

### 场景 1：嵌入式自定义 Allocator —— 内存池

在机器人 MCU 上，频繁的 `new`/`delete` 会导致碎片化。通过自定义 allocator 从静态池分配：

```cpp
template<typename T>
struct StaticPoolAllocator {
    static uint8_t pool[4096];
    static size_t  offset;

    T* allocate(size_t n) {
        if (offset + n * sizeof(T) > sizeof(pool))
            throw std::bad_alloc();
        T* p = reinterpret_cast<T*>(&pool[offset]);
        offset += n * sizeof(T);
        return p;
    }
    void deallocate(T*, size_t) {} // 不做任何事，池整体回收
};
```

这直接复用了标准库 `std::allocator` 的接口约定——只需实现 `allocate` 和 `deallocate`，即可无缝替换 `std::vector` 等容器的默认分配器。

### 场景 2：DMA 对齐缓冲区

DMA 要求 16 或 32 字节对齐。`new_allocator::allocate` 中对齐感知的代码直接展示了如何确保 DMA 安全：

```cpp
struct alignas(32) DMABuffer {
    uint8_t data[1024];
};
// new_allocator<DMABuffer>::allocate() 自动调用
// operator new(1024, std::align_val_t{32})
auto buf = std::make_unique<DMABuffer>();
```

`make_unique` 内部调用 `new DMABuffer`，后者由于 `alignas(32)` 的声明，触发对齐 new 路径。

### 场景 3：`constexpr` 动态内存 —— 编译期容器

C++20 `constexpr` 动态分配允许在编译期使用 `std::vector`、`std::string`：

```cpp
constexpr auto generate_fir_coefficients() {
    std::vector<float> coeffs;
    for (int i = 0; i < 64; i++)
        coeffs.push_back(sinc_window(i));
    return coeffs;
}
constexpr auto fir_table = generate_fir_coefficients();
```

底层原理：编译器在编译期调用 `operator new` 分配内存，计算结果写入 `.rodata` 段，编译完成后释放编译期堆。GCC 14 中 `std::construct_at` 和 `std::destroy_at` 都标记了 `constexpr` 以支持此场景。

### 场景 4：追踪内存泄漏 —— 替换 `operator new`

在机器人 7x24 运行场景中，通过替换全局 `operator new` 记录分配日志：

```cpp
struct AllocRecord { void* ptr; size_t size; const char* file; int line; };
std::vector<AllocRecord> g_alloc_log;

void* operator new(size_t size, const char* file, int line) {
    void* p = std::malloc(size);
    g_alloc_log.push_back({p, size, file, line});
    return p;
}
#define new new(__FILE__, __LINE__)
```

利用 `operator new` 可替换机制，不修改业务代码即可追踪所有堆分配。配合 RAII guard 记录分配/释放配对，快速定位泄漏点。

### 场景 5：SBO (Small Buffer Optimization) —— `std::function` 的内部实现

从 `bits/std_function.h:83-100` 可以看到 SBO 的物理实现：

```cpp
union [[gnu::may_alias]] _Any_data
{
    void*       _M_access()       noexcept { return &_M_pod_data[0]; }
    template<typename _Tp> _Tp& _M_access() noexcept { ... }

    _Nocopy_types _M_unused;
    char _M_pod_data[sizeof(_Nocopy_types)];
};

// 判断是否本地存储:
static const bool __stored_locally =
    (__is_location_invariant<_Functor>::value
     && sizeof(_Functor) <= _M_max_size        // _M_max_size = sizeof(_Nocopy_types)
     && __alignof__(_Functor) <= _M_max_align
     && (_M_max_align % __alignof__(_Functor) == 0));
```

当捕获对象是 trivially copyable 且足够小（通常在 16-32 字节内），`std::function` 直接将其存储在内联 buffer 中，避免堆分配——这就是为什么捕获 `this` 指针或小对象的 lambda 比捕获大数组快得多的原因。
