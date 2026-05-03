# 中断 —— 从 FreeRTOS 源码到 ISR 设计原则

> 源码: FreeRTOS Kernel V10.5.1
> 关键文件: `tasks.c`, `portmacro.h`, `queue.c`

## 一、FreeRTOS 中断控制源码

### 1.1 临界区保护：`portDISABLE_INTERRUPTS` / `portENABLE_INTERRUPTS`

```c
// vTaskStartScheduler() 中的使用 (tasks.c:2420)
portDISABLE_INTERRUPTS();    // 关中断
{
    xNextTaskUnblockTime = portMAX_DELAY;
    xSchedulerRunning = pdTRUE;
}
// ... 后续 xPortStartScheduler() 启动第一个任务时, 任务栈帧自动开中断
```

在 ARM Cortex-M 上，`portDISABLE_INTERRUPTS()` 的核心是写 `BASEPRI` 寄存器：

```asm
; portDISABLE_INTERRUPTS() 的等效汇编 (Cortex-M)
    mov  r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY  ; 屏蔽优先级高于此值的中断
    msr  BASEPRI, r0
```

`BASEPRI` 只屏蔽优先级**低于阈值**的中断——SysTick 和 PendSV 的优先级通常比 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 低，因此也被屏蔽。但 NMI 和 HardFault 不受影响。

### 1.2 ISR 专用 API：`xQueueGenericSendFromISR` / `xSemaphoreGiveFromISR`

FreeRTOS 严格区分**任务 API** 和 **ISR API**。ISR 中不能使用阻塞 API（没有"等待"的语义）：

```c
// queue.c:1171-1224 —— ISR 版本不阻塞
BaseType_t xQueueGenericSendFromISR( QueueHandle_t xQueue,
                                      const void * const pvItemToQueue,
                                      BaseType_t * const pxHigherPriorityTaskWoken,
                                      const BaseType_t xCopyPosition )
{
    // 与任务版 xQueueGenericSend 的关键区别:
    // 1. 不阻塞等待空间——队列满直接返回 errQUEUE_FULL
    // 2. 通过 pxHigherPriorityTaskWoken 通知是否需要上下文切换
    //    （ISR 返回后由 PendSV 完成切换, 而非在 ISR 内直接切换）
}
```

---

## 二、中断服务程序设计的核心原则 (Q2)

### 原则 1：ISR 必须短小快

ISR 执行时同级和更低级中断被屏蔽。MCU 上典型规则：ISR 应在 **10-50 μs** 内完成。长时间 ISR 会导致：
- 其他中断丢失或延迟
- 系统节拍丢失（tick 中断被推迟 → 软件定时器漂移）

**危害示例：**

```c
// 错误: 在 ISR 中做浮点矩阵运算
void UART_IRQHandler(void) {
    uint8_t byte = UART->DR;
    float result = inverse_kinematics(byte);  // 浮点运算是入栈大操作
    // ISR 可能在 2000+ 时钟周期后才返回
}
```

### 原则 2：ISR 与任务用 `volatile` 或队列通信

ISR 不能直接用普通变量与任务通信（编译器可能优化掉读/写）。通信方式优先级：

| 方式 | 延迟 | 适用场景 |
|------|------|---------|
| `xQueueSendFromISR` | 1-3 μs | 小数据量/事件通知 |
| `xSemaphoreGiveFromISR` (Binary Semaphore) | <1 μs | 无数据的纯事件通知 |
| `xTaskNotifyFromISR` | <1 μs | 唤醒单个任务, 最快 |
| `volatile` 全局变量 | 单条 `str` | 仅简单标志位 |

### 原则 3：保护临界区数据

任务和 ISR 可能同时访问的数据必须进入临界区：

```c
// 任务端
UBaseType_t uxSavedInterruptStatus;
uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
{
    shared_counter++;
}
taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
```

### 原则 4：中断优先级分层

```
高优先级中断（如电机过流保护） → 不能调用任何 FreeRTOS API
                               （因为 FreeRTOS 只能管理低优先级中断）
────────────────────── configMAX_SYSCALL_INTERRUPT_PRIORITY ──────────
低优先级中断（如 UART 接收） → 可以调用 FromISR() 系列 API
```

ARM Cortex-M 的数值小 = 优先级高。调用 FreeRTOS API 的中断优先级数值必须 **大于等于** `configMAX_SYSCALL_INTERRUPT_PRIORITY`（即逻辑优先级更低）。

### 原则 5：ISR 末尾触发 PendSV 做上下文切换

```c
// ISR 末尾
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );
portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
// 不等 ISR 内切换——如果更高优先级任务被唤醒,
// PendSV 在 ISR 返回时执行实际的上下文切换
```

---

## 三、中断延迟分析

中断延迟 = 硬件延迟 + 最长临界区屏蔽时间 + ISR 入栈时间

- **硬件延迟**：固定（Cortex-M3/M4 约 12 周期）
- **临界区屏蔽**：FreeRTOS 中 `taskENTER_CRITICAL()` 关闭部分中断的最长时间。源码中临界区路径需保持简短。
- **入栈时间**：Cortex-M 自动保存 R0-R3, R12, LR, PC, xPSR（8 个寄存器）

最终延迟通常在 1-5 μs 内（Cortex-M4 @ 168MHz）。

---

## 四、面试要点

**Q2: 中断服务程序设计有哪些核心原则？**

五条原则：(1) **短小快**——ISR 应在 10-50 μs 完成，耗时操作委托给任务；(2) **通信方式选择**——信号量/队列/任务通知，而非裸 `volatile` 变量；(3) **临界区保护**——ISR 和任务共享的数据必须用 `taskENTER_CRITICAL_FROM_ISR()` 保护；(4) **优先级分层**——只有低于 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 优先级的中断可以调用 FreeRTOS API；(5) **延迟上下文切换**——ISR 末尾通过 `portYIELD_FROM_ISR(xHigherPriorityTaskWoken)` 触发 PendSV，在 ISR 返回时安全切换，而非在 ISR 内部直接切换上下文。
