---
tags:
  - robotics
  - cpp_and_python
  - subnode
---

# 现代 C/C++ 与脚本自动化

面试官不仅会考察 C 语言的底层内存管理，还会看重现代 C++ 在大型项目中的工程化应用，以及用 Python 快速编写自动化测试脚本的能力。

## 知识地图

| 主题 | 文档 | 核心内容 |
|------|------|---------|
| 内存管理 | [memory_management.md](memory_management.md) | `std::allocator` 源码分析、`operator new`/`delete` 重载、`construct_at`/`destroy_at`、SBO 优化、应用场景 |
| 指针与引用 | [smart_pointers.md](smart_pointers.md) | `unique_ptr` 源码分析、`shared_ptr` 引用计数机制、`weak_ptr` 打破循环引用、`make_unique`/`make_shared` 实现 |
| 并发与多线程 | [concurrency.md](concurrency.md) | `std::thread` 构造与 join/detach 语义、`std::mutex` 实现、`std::scoped_lock` 死锁避免、原子操作内存序 |
| 移动语义 | [move_semantics.md](move_semantics.md) | `std::move` 与 `std::forward` 源码实现、完美转发原理、`std::swap` 三移操作、Rule of Five |
| Lambda 与捕获 | [lambda_and_invoke.md](lambda_and_invoke.md) | `INVOKE` 协议源码、`std::function` 类型擦除与 SBO、Lambda 捕获列表底层、`std::bind` 与占位符 |
| RAII 与异常安全 | [raii_exceptions.md](raii_exceptions.md) | `exception_ptr` 实现、栈展开机制、`noexcept` 与移动构造、强/基本/无异常保证、`scoped_lock` RAII 模板 |
| 关键字应用 | [keywords.md](keywords.md) | `constexpr` 编译期计算、`noexcept` 优化、`decltype`/`decltype(auto)` 推导规则、`explicit`/`override`/`final` 场景 |
| 内存对齐与不对齐 | [memory_alignment.md](memory_alignment.md) | `alignas` / `__attribute__((packed))` 语法、DMA/SIMD/Cache Line 对齐应用、协议栈紧凑布局、`offsetof` 验证、对齐与性能权衡 |
| 多态 | [polymorphism.md](polymorphism.md) | C++ 静态多态(模板/CRTP)、动态多态(虚函数/vtable)、C 语言实现 OOP(函数指针表/共享 vtable/继承模拟) |

## Python 自动化

| 工具 | 用途 |
|------|------|
| `pyserial` | 串口通信与传感器数据采集 |
| `pytest` | 硬件在环 (HIL) 自动化测试 |
| `matplotlib` + `numpy` | 电机电流/位置波形实时可视化 |

## 参考

- 外部资源链接: [reference.md](reference.md)

