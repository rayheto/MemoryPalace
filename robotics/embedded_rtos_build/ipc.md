# 任务间通信 —— FreeRTOS 队列源码与 IPC 选型

> 源码: FreeRTOS Kernel V10.5.1, `queue.c` (3421 行)

## 一、底层核心：`prvCopyDataToQueue()`

所有 IPC 机制（队列/信号量/mutex）在 FreeRTOS 中都基于同一个数据结构 `Queue_t`。数据入队的核心函数：

```c
// queue.c:2451-2510 (FreeRTOS Kernel V10.5.1)
static BaseType_t prvCopyDataToQueue( Queue_t * const pxQueue,
                                       const void * pvItemToQueue,
                                       const BaseType_t xPosition )
{
    BaseType_t xReturn = pdFALSE;
    UBaseType_t uxMessagesWaiting;

    uxMessagesWaiting = pxQueue->uxMessagesWaiting;

    if( pxQueue->uxItemSize == ( UBaseType_t ) 0 )
    {
        // uxItemSize == 0 → 这是信号量/mutex（不存实际数据）
        #if ( configUSE_MUTEXES == 1 )
        {
            if( pxQueue->uxQueueType == queueQUEUE_IS_MUTEX )
            {
                // mutex 释放: 恢复持有者优先级
                xReturn = xTaskPriorityDisinherit(
                    pxQueue->u.xSemaphore.xMutexHolder );
                pxQueue->u.xSemaphore.xMutexHolder = NULL;
            }
        }
        #endif
    }
    else if( xPosition == queueSEND_TO_BACK )
    {
        // uxItemSize > 0 → 这是真正存数据的队列
        ( void ) memcpy( ( void * ) pxQueue->pcWriteTo,
                         pvItemToQueue,
                         ( size_t ) pxQueue->uxItemSize );  // ★ 数据拷贝
        pxQueue->pcWriteTo += pxQueue->uxItemSize;          // 写指针前进

        if( pxQueue->pcWriteTo >= pxQueue->u.xQueue.pcTail ) // 环形缓冲区回绕
            pxQueue->pcWriteTo = pxQueue->pcHead;
    }
    // ...
    pxQueue->uxMessagesWaiting = uxMessagesWaiting + ( UBaseType_t ) 1;
    xReturn = pdTRUE;
    return xReturn;
}
```

**核心设计原理：**

1. **`uxItemSize == 0` 路径**：用于信号量和 mutex。它们不存储实际数据——"队列中的一条数据"只是表示"一个许可/一次未锁"。没有 memcpy，极快。

2. **`uxItemSize > 0` 路径**：真正的消息队列。`memcpy` 将调用者的数据**拷贝到**队列内部环形缓冲区。FreeRTOS 按值传递消息——数据进队后，调用者可以复用或释放自己的 buffer。

3. **环形缓冲区**：队列用头/尾指针 + `uxItemSize * uxLength` 的连续内存块实现。当 `pcWriteTo` 到达尾部时回绕到 `pcHead`，形成环形。

### 接收端：`xQueueReceive()`

```c
// queue.c:1522 (FreeRTOS Kernel V10.5.1)
BaseType_t xQueueReceive( QueueHandle_t xQueue,
                           void * const pvBuffer,
                           TickType_t xTicksToWait )
{
    // 如果队列空:
    //   - 将当前 TCB 放入队列的等待列表
    //   - 进入 Blocked 状态
    //   - 调用 portYIELD() 切换到其他任务
    // 如果有数据或有超时:
    //   - memcpy 到 pvBuffer
    //   - 返回 pdPASS
}
```

---

## 二、四种 IPC 机制对比

### 2.1 消息队列 (Message Queue)

```c
// 创建: 队列长度 10, 每条消息 16 字节
QueueHandle_t q = xQueueCreate( 10, 16 );

// 发送 (可阻塞)
SensorData data = { .x = 1.0, .y = 2.0 };
xQueueSend( q, &data, portMAX_DELAY );   // 队列满了就阻塞等待

// 接收
SensorData received;
xQueueReceive( q, &received, pdMS_TO_TICKS(100) ); // 100ms 超时
```

**适用:** 传递有实际数据负载的消息（传感器读数、指令包）。

### 2.2 二进制信号量 (Binary Semaphore)

```c
SemaphoreHandle_t sem = xSemaphoreCreateBinary();

// ISR 中 give
xSemaphoreGiveFromISR( sem, &xHigherPriorityTaskWoken );

// 任务中 take
xSemaphoreTake( sem, portMAX_DELAY );  // 阻塞直到 ISR 给信号量
```

**适用:** ISR → Task 的纯事件通知（无数据负载）。底层与队列相同但 `uxItemSize == 0`——"give"就是放一条空数据，"take"就是取出这条空数据。

### 2.3 计数信号量 (Counting Semaphore)

```c
SemaphoreHandle_t sem = xSemaphoreCreateCounting( 5, 0 );
// 最多 5 个许可, 初始 0 个

xSemaphoreGive( sem );  // 释放一个许可 → 计数 +1
xSemaphoreTake( sem, portMAX_DELAY ); // 消耗一个许可 → 计数 -1
```

**适用:** 管理有限数量的相同资源——如 DMA 通道池（5 个 DMA 通道可用）。

### 2.4 任务通知 (Task Notification) —— 最快

```c
// ISR 中唤醒特定任务, 不通过队列
xTaskNotifyFromISR( motor_task_handle,
                     ENCODER_READY_BIT,
                     eSetBits,
                     &xHigherPriorityTaskWoken );

// 任务中等待
uint32_t notified_value;
xTaskNotifyWait( 0, ULONG_MAX, &notified_value, portMAX_DELAY );
```

**适用:** 已知特定接收任务的场景（一对多）。速度快（无队列创建开销、无数据拷贝），但只能唤醒一个任务。

### 2.5 性能对比

| 机制 | 入队延迟 | 内存开销 | 多播能力 | 数据负载 |
|------|---------|---------|---------|---------|
| 队列 | ~2 μs (含 memcpy) | 环形缓冲区 N × itemSize | ✅ 可被多任务读 | ✅ |
| 二值信号量 | <1 μs | Queue_t (约 80B) | ❌ | ❌ |
| 计数信号量 | <1 μs | Queue_t + 计数器 | ❌ | ❌ |
| 任务通知 | <500 ns | 0 (内嵌在 TCB) | ❌ 一对一定点 | 可选 32-bit 值 |

---

## 三、选型建议

```
需要传递实际数据？
  ├── 是, 且数据可能积压 → 消息队列
  ├── 是, 但一次只发一条且不积压 → Stream Buffer (FreeRTOS+)
  └── 否, 纯事件通知
        ├── ISR → Task → 二进制信号量
        ├── Task → Task → 任务通知 (最快)
        └── 管理有限资源 → 计数信号量
```
