# DMA 原理与配置 —— 嵌入式高速数据传输

## 一、DMA 是什么？为什么需要它？

DMA (Direct Memory Access) 是一块**独立于 CPU 的硬件控制器**，可以在外设和内存之间搬运数据，不占用 CPU 执行指令的时间。

**有 DMA 的 UART 接收：**

```
CPU:  [计算 FOC] [更新显示] [等 1ms]      [计算 FOC] ...
DMA:     [UART RX → 内存]   [UART RX → 内存]   [UART RX → 内存]
        ↑ DMA 在后台工作, CPU 无需介入
```

**无 DMA（仅中断方式）：**

```
CPU:  [FOC] [ISR:收1B] [显示] [ISR:收1B] [FOC] [ISR:收1B] ...
           ↑ 每个字节触发一次 ISR → 严重打断控制循环
```

对 115200 bps 的 UART：每秒 11520 字节 → 无 DMA 时 CPU 每秒被中断**11520 次**。DMA 可以攒一包数据（如 256 字节）再中断一次，中断次数降低 256 倍。

---

## 二、DMA 控制器架构

以 STM32F4 DMA 为例（每个 DMA 控制器有 8 个 Stream，每个 Stream 有 8 个 Channel）：

```
        DMA1 控制器
        ┌──────────────────────────────────┐
        │  Stream 0 ── Channel 可选 0~7     │
        │  Stream 1 ── Channel 可选 0~7     │      外设
        │  ...                              │ ←── UART RX
        │  Stream 7 ── Channel 可选 0~7     │ ←── SPI TX
        └──────────────────────────────────┘ ←── ADC

关键寄存器:
  DMA_SxCR   — 控制寄存器(数据宽度/方向/模式/中断使能)
  DMA_SxPAR  — 外设地址(固定, 如 &USART1->DR)
  DMA_SxM0AR — 内存地址(自增, 如 buffer 基址)
  DMA_SxNDTR — 剩余传输数量(每次传输减 1, 归零触发完成中断)
```

---

## 三、DMA 传输模式

### 3.1 单缓冲模式

```c
uint8_t rx_buf[256];
HAL_UART_Receive_DMA(&huart1, rx_buf, sizeof(rx_buf));

// 问题: DMA 填满 256 字节后在 ISR 中被处理。
// 处理期间 UART 如果有新数据 → 丢失
```

### 3.2 双缓冲模式（Double Buffer）

```c
uint8_t buf_a[256];
uint8_t buf_b[256];

// 硬件自动在两个 buffer 之间切换
void DMA_IRQHandler(void) {
    if (DMA->SxCR & DMA_SxCR_CT) {  // CT = Current Target bit
        // DMA 当前在用 buf_b → CP U 可以处理 buf_a
        process(buf_a);
        HAL_UART_Receive_DMA(&huart1, buf_b, sizeof(buf_b));
    } else {
        // DMA 当前在用 buf_a → CP U 可以处理 buf_b
        process(buf_b);
        HAL_UART_Receive_DMA(&huart1, buf_a, sizeof(buf_a));
    }
}
```

**原理：** DMA 在 buf_a 和 buf_b 之间自动切换（`DMA_SxM0AR` 和 `DMA_SxM1AR` 两个内存地址寄存器）。当一个 buffer 满时，DMA 自动转向另一个 buffer 继续接收，同时触发中断通知 CPU 处理已满的 buffer。

### 3.3 乒乓缓冲（Ping-Pong Buffer）

双缓冲的特例——在 ADC 连续采样中常用：

```
ADC → DMA (循环模式)
      ├── buf_a (采集第 N 组数据)
      └── buf_b (CPU 正在处理第 N-1 组)
```

### 3.4 循环模式（Circular Mode）

```c
// DMA 填满 buffer 后从头部重新开始, 循环不停
DMA_InitStruct.Mode = DMA_CIRCULAR;
// 适用: 音频采集、电机相电流采样
```

---

## 四、DMA 与中断的配合

```c
// 典型异步 DMA + ISR 流程
void start_dma_transfer(void) {
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;         // 停 DMA
    DMA1_Stream0->NDTR = transfer_size;       // 设置传输数量
    DMA1_Stream0->M0AR = (uint32_t)rx_buffer; // 设置内存地址
    DMA1_Stream0->CR |= DMA_SxCR_EN;          // 启动 DMA

    // 外设启动传输 (如 SPI TX)
    SPI1->CR |= SPI_CR_SPE;
}

// DMA 传输完成 ISR
void DMA1_Stream0_IRQHandler(void) {
    if (DMA1->LISR & DMA_LISR_TCIF0) {        // 检查 TCIF (传输完成标志)
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;       // 清除中断标志

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(dma_done_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// 任务中等待传输完成
void motor_controller_task(void *params) {
    while(1) {
        start_dma_transfer();
        xSemaphoreTake(dma_done_sem, portMAX_DELAY);
        process_received_data(rx_buffer);  // DMA 已填好, 直接处理
    }
}
```

---

## 五、面试问题

### Q15: DMA 的优势与典型应用场景

**优势：**

1. **释放 CPU**：DMA 在后台搬运数据，CPU 可以专注计算（FOC 控制循环、轨迹规划）。UART 115200 bps 场景下，中断次数从 11520/s 降低到 45/s（以 256 字节缓冲区计）。
2. **高速传输保持**：SPI/I2S 外设高速传输（如音频 48kHz × 16bit × 2ch = 1.536 Mbps）时，CPU 逐个字节收发根本来不及。DMA 可以与外设时序完美同步，不丢数据。
3. **低功耗**：DMA 传输时 CPU 可进入睡眠模式（WFI），只在 DMA 完成中断时才唤醒。
4. **确定性延迟**：DMA 传输时间由总线仲裁决定（通常 2-3 个 AHB 周期），不受代码路径分支影响。

**典型应用：**
- ADC 多通道轮询采样（DMA 循环模式 + 触发后读）
- 电机 FOC 中 SPI 读取磁编码器位置
- UART/CAN 接收不定长协议帧
- 摄像头 DCMI 接口图像采集
- DAC 音频播放 (I2S + DMA + 双缓冲)
