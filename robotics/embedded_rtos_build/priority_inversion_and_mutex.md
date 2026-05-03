# 优先级反转与互斥锁 —— FreeRTOS 源码分析

> 源码: FreeRTOS Kernel V10.5.1
> 关键文件: `queue.c` (mutex 创建), `tasks.c` (优先级继承)

## 一、什么是优先级反转？

三个任务（高/中/低优先级）竞争同一资源时的经典场景：

```
时间 →

高优先级任务 H (prio=3):  ──[等待 mutex]────────────[获得 mutex]──
中优先级任务 M (prio=2):  ──[一直运行──────────────]──
低优先级任务 L (prio=1):  ──[获得 mutex]──[被抢占]──────────────

问题: H 等 L 释放 mutex, 但 M 一直在运行 (M 优先级 > L, M 不需要 mutex)
      → L 得不到 CPU 释放 mutex → H 被无期限阻塞
```

H 的有效优先级被拉低到比 M 还低——这就是**优先级反转**。1997 年火星探路者 (Mars Pathfinder) 复位事故就是这个 bug 的经典案例。

---

## 二、FreeRTOS 的解决: 优先级继承

### 2.1 mutex 初始化: `prvInitialiseMutex()`

```c
// queue.c:631-657 (FreeRTOS Kernel V10.5.1)
static void prvInitialiseMutex( Queue_t * pxNewQueue )
{
    if( pxNewQueue != NULL )
    {
        pxNewQueue->u.xSemaphore.xMutexHolder = NULL;    // ① 无持有者
        pxNewQueue->uxQueueType = queueQUEUE_IS_MUTEX;   // ② 标记为 MUTEX 类型

        pxNewQueue->u.xSemaphore.uxRecursiveCallCount = 0;

        // ③ 以"已有数据"状态初始化——相当于互斥量一开始可用
        ( void ) xQueueGenericSend( pxNewQueue, NULL, ( TickType_t ) 0U,
                                    queueSEND_TO_BACK );
    }
}
```

**关键设计：** mutex 底层复用了**队列**数据结构，但 `uxQueueType = queueQUEUE_IS_MUTEX`。`xQueueGenericSend` 给队列放入一条空数据（`NULL`），表示 mutex 可用。后续 `xSemaphoreTake` 取出这条空数据表示"获取 mutex 成功"。

### 2.2 优先级继承: `xTaskPriorityInherit()`

```c
// tasks.c:5041-5117 (FreeRTOS Kernel V10.5.1)
BaseType_t xTaskPriorityInherit( TaskHandle_t const pxMutexHolder )
{
    TCB_t * const pxMutexHolderTCB = pxMutexHolder;
    BaseType_t xReturn = pdFALSE;

    if( pxMutexHolder != NULL )
    {
        // ★ 核心判断: 持有者的优先级 < 想获取 mutex 的任务的优先级?
        if( pxMutexHolderTCB->uxPriority < pxCurrentTCBs[ xCurCoreID ]->uxPriority )
        {
            // 如果持有者在就绪列表中, 需先移除
            if( listIS_CONTAINED_WITHIN(
                    &( pxReadyTasksLists[ pxMutexHolderTCB->uxPriority ] ),
                    &( pxMutexHolderTCB->xStateListItem ) ) != pdFALSE )
            {
                uxListRemove( &( pxMutexHolderTCB->xStateListItem ) );
                // ★ 提升持有者优先级到获取者优先级
                pxMutexHolderTCB->uxPriority = pxCurrentTCBs[ xCurCoreID ]->uxPriority;
                // 重新插入新优先级的就绪列表
                prvAddTaskToReadyList( pxMutexHolderTCB );
            }
            else
            {
                // 持有者不在就绪列表 (在阻塞), 只改优先级
                pxMutexHolderTCB->uxPriority = pxCurrentTCBs[ xCurCoreID ]->uxPriority;
            }
            xReturn = pdTRUE;  // 发生了继承
        }
    }
    return xReturn;
}
```

**逐行原理：**

- **核心一行**：`if( pxMutexHolderTCB->uxPriority < pxCurrentTCBs[] ->uxPriority )` —— 持有者优先级比等待者低 → 需要继承。把持有者的 `uxPriority` 直接提升到等待者的优先级。这保证了持有者不会被中优先级任务抢占，能尽快释放 mutex。

- **就绪列表迁移**：持有者如果当前在就绪列表（说明它没在阻塞，只是还没被调度），必须从旧优先级的就绪列表移除，重新插入新优先级的就绪列表。否则调度器会找不到它。

- **`xReturn = pdTRUE`**：返回值告诉调用者 `prvCopyDataToQueue()` 发生了继承。释放 mutex 时需要 `xTaskPriorityDisinherit()` 来恢复。

### 2.3 优先级恢复: `xTaskPriorityDisinherit()`

```c
// tasks.c:5141 (FreeRTOS Kernel V10.5.1)
BaseType_t xTaskPriorityDisinherit( TaskHandle_t const pxMutexHolder )
{
    // 检查持有者是否被提升过优先级
    // 如果是, 恢复到 uxBasePriority (原始优先级)
    if( pxMutexHolderTCB->uxPriority != pxMutexHolderTCB->uxBasePriority )
    {
        // 从当前就绪列表移除 → 恢复为 base priority → 重新插入
    }
}
```

### 2.4 触发位置

```c
// queue.c:1812-1821 —— 任务尝试获取 mutex 但被占用时触发继承
if( pxQueue->uxQueueType == queueQUEUE_IS_MUTEX )
{
    xInheritanceOccurred = xTaskPriorityInherit(
        pxQueue->u.xSemaphore.xMutexHolder );
}

// queue.c:2469 —— mutex 释放时触发恢复
xReturn = xTaskPriorityDisinherit(
    pxQueue->u.xSemaphore.xMutexHolder );
pxQueue->u.xSemaphore.xMutexHolder = NULL;
```

---

## 三、面试问题

### Q8: 什么是优先级反转？如何解决？

优先级反转：高优先级任务因等待低优先级任务持有的资源而被阻塞，同时中优先级任务（不涉及该资源）抢占了低优先级任务，导致高优先级任务被**无期限阻塞**。低优先级任务得不到 CPU 也就无法释放资源。

FreeRTOS 的解决方案：**优先级继承**（Priority Inheritance）。核心在 `xTaskPriorityInherit()` 源码：当高优先级任务尝试获取 mutex 时，检查持有者的优先级——若持有者优先级更低，则**将持有者的优先级临时提升到获取者的优先级**。这保证了持有者不会被无关的中优先级任务抢占，能尽快完成并释放 mutex。释放时 `xTaskPriorityDisinherit()` 恢复原始优先级。

**注意：** 二进制信号量没有优先级继承机制（因为 FreeRTOS 信号量不记录持有者）。这就是为什么**保护共享资源必须用 mutex** 的核心原因。

### Q11: 信号量与互斥量的核心区别？

| | Mutex (互斥量) | Semaphore (信号量) |
|---|--------------|-----------------|
| **持有者概念** | 有 (`xMutexHolder` 记录哪个任务持有) | 没有 |
| **优先级继承** | 支持（`xTaskPriorityInherit` 自动处理） | 不支持 |
| **递归获取** | 递归 mutex 支持同任务多次获取 | 不支持（会死锁） |
| **使用场景** | 保护共享资源（互斥） | 任务同步通知（二值信号量）/ 资源计数（计数信号量） |
| **释放规则** | 只有持有者任务可以释放 | 任何任务/ISR 都可以释放 |

源码关键差异：`prvInitialiseMutex()` 设置 `uxQueueType = queueQUEUE_IS_MUTEX`。后续在 `prvCopyDataToQueue()` 和 `xQueueReceive()` 中根据此标志决定是否调用 `xTaskPriorityInherit` / `xTaskPriorityDisinherit`。
