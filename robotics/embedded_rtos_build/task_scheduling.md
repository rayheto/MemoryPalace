# 任务调度机制 —— FreeRTOS 源码分析与面试要点

> 源码: FreeRTOS Kernel V10.5.1 (ESP-IDF SMP)
> 关键文件: `tasks.c` (6483 行), `list.c` (230 行)

## 一、任务状态机：从 `eTaskGetState()` 源码看四种状态

```c
// tasks.c:1624-1694 (FreeRTOS Kernel V10.5.1)
eTaskState eTaskGetState( TaskHandle_t xTask )
{
    eTaskState eReturn;
    List_t const * pxStateList;
    const TCB_t * const pxTCB = xTask;

    if( taskIS_CURRENTLY_RUNNING( pxTCB ) == pdTRUE )
    {
        eReturn = eRunning;                // (1) 运行态
    }
    else
    {
        pxStateList = listLIST_ITEM_CONTAINER( &( pxTCB->xStateListItem ) );

        if( ( pxStateList == pxDelayedTaskList ) ||
            ( pxStateList == pxOverflowedDelayedTaskList ) )
        {
            eReturn = eBlocked;            // (2) 阻塞态
        }
        else if( pxStateList == &xSuspendedTaskList )
        {
            // ...检查 Task Notification 等
            eReturn = eSuspended;          // (3) 挂起态
        }
        else
        {
            eReturn = eReady;              // (4) 就绪态
        }
    }
    return eReturn;
}
```

**源码逻辑说明：**

这段 `if/else if/else` 链条揭示了 FreeRTOS 任务状态机的核心判断逻辑：

1. **`eRunning`（运行态）**：当前 CPU 正在执行的 TCB（Task Control Block）就是它自己。通过 `taskIS_CURRENTLY_RUNNING(pxTCB)` 判断——实际是比较 `pxTCB` 是否等于 `pxCurrentTCBs[xCoreID]`。

2. **`eBlocked`（阻塞态）**：TCB 的 `xStateListItem` 被容器持有，且该容器是延迟列表或溢出延迟列表。任务在等待某个事件（延时期满、信号量、队列数据）。

3. **`eSuspended`（挂起态）**：TCB 在挂起列表中，且不等待任何事件。需要注意——一个被 `vTaskSuspend()` 挂起的任务**不在** ready 列表也不在 delayed 列表。

4. **`eReady`（就绪态）**：TCB 在某个优先级的 ready 列表中，但当前 CPU 没在运行它。调度器下一次选择时它可能被切换为运行态。

```
                    ┌──────────┐
         创建任务 → │  Ready   │ ←────────────────────────┐
                    └────┬─────┘                          │
                         │ 调度器选中                        │
                         ▼                                 │
                    ┌──────────┐    事件发生/超时              │
                    │ Running  │───────────────────────────┘
                    └────┬─────┘
                         │ 等事件/延时/等信号量
                         ▼
                    ┌──────────┐    vTaskSuspend()
                    │ Blocked  │ ──────────────────────┐
                    └──────────┘                       │
                                                        ▼
                                                   ┌───────────┐
                                                   │ Suspended │
                                                   └───────────┘
                                          vTaskResume() → Ready
```

---

## 二、抢占式调度核心：`vTaskStartScheduler()` 启动流程

```c
// tasks.c:2382-2449 (FreeRTOS Kernel V10.5.1)
void vTaskStartScheduler( void )
{
    BaseType_t xReturn;

    xReturn = prvCreateIdleTasks();           // ① 创建空闲任务

    if( xReturn == pdPASS )
    {
        portDISABLE_INTERRUPTS();             // ② 关全局中断

        xNextTaskUnblockTime = portMAX_DELAY; // ③ 初始化下次唤醒时间
        xSchedulerRunning = pdTRUE;           // ④ 置调度器运行标志
        xTickCount = configINITIAL_TICK_COUNT; // ⑤ 初始化系统节拍计数

        xPortStartScheduler();                // ⑥ 启动第一个任务（不返回）
    }
}
```

**逐段解释：**

- **① `prvCreateIdleTasks()`**：为每个 CPU 核创建一个优先级最低（0）的空闲任务。CPU 无其他任务可运行时执行空闲任务（通常进入 WFI/WFE 低功耗指令）。
- **② `portDISABLE_INTERRUPTS()`**：启动调度器前关中断——防止在第一个任务启动前发生 tick 中断导致调度器状态不一致。第一个任务的栈帧中包含"中断已开启"的状态字，任务开始执行时自动开中断。
- **④ `xSchedulerRunning = pdTRUE`**：这个标志位被 `vTaskSuspendAll()` 等函数检查——只有调度器已运行时才能做抢占操作。
- **⑥ `xPortStartScheduler()`**：此函数不被返回。它执行一次上下文切换到最高优先级的 ready 任务，CPU 从该任务的第一条指令开始执行。此后全部由调度器和中断驱动。

### 抢占发生的时机

调度器通过 `taskYIELD_IF_USING_PREEMPTION()` 触发抢占：

```c
// tasks.c:72-74
#define taskYIELD_IF_USING_PREEMPTION()
#define taskYIELD_IF_USING_PREEMPTION()    portYIELD_WITHIN_API()
```

`portYIELD_WITHIN_API()` 触发 PendSV 异常（ARM Cortex-M），在异常返回时切换到最高优先级的 ready 任务。抢占发生在：
- 一个更高优先级的任务被唤醒（信号量释放、队列数据到达）
- 当前任务的时间片耗尽（同优先级 round-robin）
- 中断返回时（中断可能唤醒了更高优先级的任务）

---

## 三、就绪列表管理

FreeRTOS 用**双向链表**管理就绪任务。每个优先级有一个链表：

```c
// tasks.c 中的就绪列表数组
List_t pxReadyTasksLists[ configMAX_PRIORITIES ];
```

`configMAX_PRIORITIES` 通常为 25（ESP-IDF 默认），值域 0（最低，idle）到 N-1（最高）。调度器从高到低遍历，找到第一个非空链表，选择其第一个 TCB 运行——O(1) 选择（因为优先级数量固定）。

---

## 四、面试问题

### Q10: RTOS 任务基本状态与抢占式调度原理

RTOS 中任务有四种基本状态：就绪(Ready)、运行(Running)、阻塞(Blocked)、挂起(Suspended)。从 `eTaskGetState()` 源码可见，任务通过 TCB 所在的链表来判断当前状态：在 ready 列表中 → 就绪；是当前运行的 TCB → 运行；在 delayed list 中等待事件 → 阻塞；在 suspended list 中且不等待任何事件 → 挂起。

抢占式调度的核心机制：每个 tick 中断或 API 调用后，调度器检查是否有更高优先级的 ready 任务。如果有，通过 `taskYIELD_IF_USING_PREEMPTION()` 触发 PendSV（在 ARM Cortex-M 上），在 PendSV 异常处理中完成上下文切换——保存当前任务的寄存器到栈、加载新任务的寄存器、切换栈指针。源码 `vTaskStartScheduler()` 在启动时关闭中断、初始化 tick、然后 `xPortStartScheduler()` 做第一次上下文切换。此后 CPU 完全由调度器驱动——任务主动阻塞(tick 延时/等信号量)或被动抢占(tick 中断检测到更高优先级 ready)都会导致任务切换。

**抢占式 vs 协作式的本质区别：** 抢占式中，一个死循环的低优先级任务不会饿死高优先级任务——tick 中断总能触发调度器切换到高优先级的 ready 任务。协作式中，每个任务必须主动释放 CPU（调用 `taskYIELD()` 或阻塞 API），否则其他任务永远得不到执行。
