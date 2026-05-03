# 嵌入式平台与构建系统

对于多核 MCU 或高性能芯片，实时性（Real-time）是机器人系统的生命线。

## 面试核心考点

### RTOS 实时操作系统

- **任务调度机制:** 抢占式调度 vs 协作式调度；任务优先级与时间片轮转
- **中断延迟 (Interrupt Latency):** 中断响应时间分析、临界区保护、中断嵌套
- **优先级反转与互斥锁:** 优先级继承 (Priority Inheritance) 机制；`mutex` vs `semaphore` 使用场景
- **任务间通信 (IPC):** 消息队列、信号量、事件组、任务通知 (Task Notification) 的性能对比
- **DMA 原理与配置:** 双缓冲 (Double Buffering)、乒乓缓冲 (Ping-Pong) 模式；DMA 与中断的配合

### FreeRTOS 内存管理

- Heap_1 到 Heap_5 五种分配策略的区别与适用场景
- `pvPortMalloc` / `vPortFree` 的线程安全
- 栈溢出检测 (`configCHECK_FOR_STACK_OVERFLOW`)

### CMake 构建系统

- `target_link_libraries` 的 PUBLIC / PRIVATE / INTERFACE 传递规则
- `add_subdirectory` 模块化管理
- 交叉编译工具链 (Toolchain) 配置 (`CMAKE_TOOLCHAIN_FILE`)
- `find_package` 与 `FetchContent` 的依赖管理
- 多平台条件编译 (`if(CMAKE_SYSTEM_PROCESSOR ...)`)

## 推荐知识库

- **FreeRTOS Official Documentation:** 重点看 Memory Management 和 Task Notification
- **CMake Tutorial (官方):** `cmake.org/cmake/help/latest/guide/tutorial/`
- **STM32CubeMX / ESP-IDF:** 实际项目中 FreeRTOS 配置的最佳入口
