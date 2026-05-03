# 内存与栈管理 —— 堆/栈/看门狗/栈溢出

> 参考: FreeRTOS Kernel V10.5.1, Cortex-M 架构参考

## 一、Q3: 堆与栈的核心区别

| | 栈 (Stack) | 堆 (Heap) |
|---|-----------|----------|
| **分配方式** | 编译器自动（函数调用时压栈） | 程序员显式调用 `malloc`/`pvPortMalloc` |
| **释放方式** | 函数返回时自动弹出 | 必须手动 `free`/`vPortFree` |
| **内存布局** | 从高地址向低地址增长 | 从低地址向高地址增长 |
| **分配速度** | 单条 `sub sp, #N` 指令（纳秒级） | 需遍历空闲链表 / 合并碎片（微秒到毫秒） |
| **碎片风险** | 无（LIFO 严格） | 有（多次 malloc/free 后产生碎片） |
| **大小限制** | 小（通常几 KB，编译期固定） | 大（取决于链接脚本的 heap 大小） |
| **确定性** | 完全确定 | 不确定（取决于碎片和分配器实现） |

**嵌入式关键约束：**

```c
// 错误: ISR 中用 malloc —— 不确定延迟 + 可能阻塞
void ISR_handler(void) {
    uint8_t* buf = (uint8_t*)malloc(1024);  // 可能花 100 μs 或更多
}

// 正确: ISR 用预分配的静态 buffer
static uint8_t isr_buf[1024];  // 编译期确定, 零运行时开销
void ISR_handler(void) {
    // 直接使用 isr_buf
}
```

FreeRTOS 中每个任务有独立的栈（从 TCB 分配）。栈在任务创建时由 `pxStack` 指针指向——所有局部变量和函数调用链都在这个私有栈上，任务间互不干扰。

---

## 二、Q16: 任务栈溢出检测

FreeRTOS 提供两种栈溢出检测方式（通过 `configCHECK_FOR_STACK_OVERFLOW` 配置）。

### 方法一：检查栈顶魔数

```c
// configCHECK_FOR_STACK_OVERFLOW == 1
// 每次任务切换时检查栈顶的魔数 (0xA5A5A5A5) 是否被覆盖
#define taskCHECK_FOR_STACK_OVERFLOW()                      \
{                                                           \
    const TCB_t * pxTCB = pxCurrentTCB;                     \
    if( *( ( uint32_t * ) pxTCB->pxStack ) != 0xA5A5A5A5 ) \
        vApplicationStackOverflowHook( pxTCB, pxTCB->pcTaskName ); \
}
```

任务创建时在栈底（最低地址，ARM 栈从高到低增长所以"最深处"）写魔数 `0xA5A5A5A5`。每次上下文切换时检查——如果被覆盖，说明栈曾增长到超过此位置。

### 方法二：检查栈指针是否越界

```c
// configCHECK_FOR_STACK_OVERFLOW == 2
// 保存栈的最后一个合法字节的地址, 用当前 SP 与之比较
```

**实际调试技巧：**

```c
// 运行时获取任务栈的使用情况
UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask );
// 返回 "从未被使用过的栈空间最小值"
// 如果返回 < 10 个字 → 栈快用完了, 必须增大
```

**分配多少栈？** 经验值：

| 任务类型 | 推荐栈大小 |
|---------|----------|
| 简单 LED 闪烁 / 标志设置 | 128-256 字节 |
| UART 命令解析 | 512-1024 字节 |
| 浮点运算 / 矩阵计算 | 2048-4096 字节 |
| printf/sprintf 调用 | +1024 字节（printf 栈消耗约 500B+） |

---

## 三、Q17: 内存碎片与 malloc 替代方案

### 碎片如何产生

```
时间 →

初始: [A(100B)][  空闲 900B  ]

分配 B: [A(100B)][B(200B)][空闲 700B]

分配 C: [A(100B)][B(200B)][C(300B)][空闲 400B]

释放 B: [A(100B)][空闲 200B][C(300B)][空闲 400B]
                ↑ 两个空间分别空闲, 不能合并成大块

分配 D 需要 350B: 失败! (虽然总空闲 600B, 但最大连续块只有 400B)
                                   ↑ 这就是碎片
```

### FreeRTOS 的堆方案: Heap_1 ~ Heap_5

| 方案 | 分配策略 | free 支持 | 碎片 | 适用场景 |
|------|---------|----------|------|---------|
| Heap_1 | 顺序分配，永不释放 | ❌ | 无 | 任务只创建不删除的系统 |
| Heap_2 | 最佳匹配，允许释放 | ✅ | 有碎片 | 遗留方案，不推荐 |
| Heap_3 | 封装标准 malloc/free | ✅ | 取决于 malloc 实现 | 有 OS 级堆保护的平台 |
| **Heap_4** | **首次匹配 + 相邻块合并** | ✅ | **碎片少** | **推荐：稳定且碎片可控** |
| Heap_5 | Heap_4 + 跨多块内存 | ✅ | 少 | 有外部 RAM + 内部 SRAM |

**Heap_4 的碎片合并：** `pvPortFree` 后检查相邻块是否可以合并（通过块头中的 `xBlockSize` 和 `pxNextFreeBlock` 链接），尽可能将释放的小块合并为大块。这是生产环境中最常用的方案。

### 工程上避免碎片的策略

1. **静态分配**：所有关键对象在编译期分配——`xTaskCreateStatic()` + `xQueueCreateStatic()` + `xSemaphoreCreateMutexStatic()`。根本不用堆。

2. **固定大小内存池**：机器人电机控制中用固定大小的控制块池（所有 `PIDContext` 对象大小相同），避免不同大小分配的碎片。

3. **只在初始化时用堆**：系统启动时做所有 malloc，运行时不再 malloc/free。碎片为 0。

4. **用 `pvPortMalloc` 而非裸 `malloc`**：FreeRTOS 的 `pvPortMalloc` 是线程安全的，且可以在不同 Heap_N 实现间切换。

---

## 四、Q9: 看门狗的作用与多任务喂狗策略

### 看门狗原理

看门狗是一个硬件计数器，从预设值递减到 0。如果到 0 还没被"喂狗"（重新加载初值），则触发系统复位。

```c
// 独立看门狗 (IWDG) 的典型配置
void IWDG_Init(void) {
    IWDG->KR = 0x5555;     // 解除写保护
    IWDG->PR = 4;          // 预分频 → 1ms 周期
    IWDG->RLR = 1000;      // 重装载值 → 1000ms 超时
    IWDG->KR = 0xAAAA;     // 喂狗一次, 启动
    IWDG->KR = 0xCCCC;     // 启动看门狗
}
```

### 多任务喂狗策略

**错误做法：** 在最高优先级任务中定时喂狗。

```c
// 错误: 高优先级喂狗任务正常运行, 其他任务全卡死也检测不到
void watchdog_task(void *params) {
    while(1) {
        HAL_IWDG_Refresh(&hiwdg);   // 喂狗
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

**正确做法：每个需要监控的任务维护心跳计数。**

```c
typedef struct {
    volatile uint32_t heartbeat;
    uint32_t          max_interval_ms;
} TaskWatch;

TaskWatch g_watches[NUM_TASKS];

// 各任务中定期更新心跳
void motor_task(void *params) {
    while(1) {
        g_watches[0].heartbeat = xTaskGetTickCount();
        // 做实际工作...
    }
}

// 低优先级喂狗任务负责汇总判断
void watchdog_task(void *params) {
    while(1) {
        bool all_alive = true;
        TickType_t now = xTaskGetTickCount();
        for (int i = 0; i < NUM_TASKS; i++) {
            if ( (now - g_watches[i].heartbeat) * portTICK_PERIOD_MS
                 > g_watches[i].max_interval_ms ) {
                all_alive = false;  // 某个任务超时未更新心跳
            }
        }
        if (all_alive)
            HAL_IWDG_Refresh(&hiwdg);
        // 否则不喂狗 → 看门狗超时 → 复位
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

**策略总结：**

- 每个关键任务设置独立的心跳超时
- 喂狗任务在所有心跳有效时才喂狗
- 喂狗任务是系统健康监控的**判定者**，不是无脑定时器
