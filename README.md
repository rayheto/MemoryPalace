# MemoryPalace

机器人研发岗位知识体系 —— 面向底层系统、软硬件联调与电机控制。

## 目录

### 1. 现代 C/C++ 与脚本自动化

入口：[cpp_and_python/knowledge.md](robotics/cpp_and_python/knowledge.md)

| 文档 | 主题 |
|------|------|
| [memory_management.md](robotics/cpp_and_python/memory_management.md) | std::allocator 源码、operator new/delete、construct_at/destroy_at、内存池、SBO |
| [smart_pointers.md](robotics/cpp_and_python/smart_pointers.md) | unique_ptr/shared_ptr/weak_ptr 源码、引用计数、make_unique |
| [concurrency.md](robotics/cpp_and_python/concurrency.md) | std::thread、std::mutex、atomic 内存序、call_once |
| [move_semantics.md](robotics/cpp_and_python/move_semantics.md) | std::move、std::forward 源码、完美转发、swap 三移操作 |
| [lambda_and_invoke.md](robotics/cpp_and_python/lambda_and_invoke.md) | INVOKE 协议、std::function 类型擦除与 SBO、Lambda 捕获 |
| [raii_exceptions.md](robotics/cpp_and_python/raii_exceptions.md) | exception_ptr、栈展开、noexcept 与移动构造、异常安全三级别 |
| [keywords.md](robotics/cpp_and_python/keywords.md) | const/static/volatile 底层语义、constexpr/noexcept/decltype/explicit/override |
| [memory_alignment.md](robotics/cpp_and_python/memory_alignment.md) | alignas/packed 语法、DMA/SIMD/Cache Line 对齐、协议栈紧凑布局 |

### 2. 嵌入式平台与构建系统

入口：[embedded_rtos_build/knowledge.md](robotics/embedded_rtos_build/knowledge.md)

### 3. Linux 底层开发与调优

入口：[linux_dev_tuning/knowledge.md](robotics/linux_dev_tuning/knowledge.md)

### 4. 硬件调试与信号完整性

入口：[hardware_debug_si/knowledge.md](robotics/hardware_debug_si/knowledge.md)

### 5. 工业通信协议 (EtherCAT)

入口：[industrial_protocols/knowledge.md](robotics/industrial_protocols/knowledge.md)

### 6. 电机控制 (FOC) 与传感器融合

入口：[motor_control_sensor_fusion/knowledge.md](robotics/motor_control_sensor_fusion/knowledge.md)

### 7. 可靠性设计与样机工程化

入口：[reliability_lifecycle/knowledge.md](robotics/reliability_lifecycle/knowledge.md)
