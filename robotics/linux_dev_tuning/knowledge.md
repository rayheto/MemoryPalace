# Linux 底层开发与调优

机器人通常是异构系统（MCU 负责实时控制，Linux/SoC 负责上层算法或 AI 交互）。

## 面试核心考点

### Linux 底层开发

- **用户空间与内核空间:** 系统调用过程、上下文切换开销、`mmap` 零拷贝
- **设备树 (Device Tree, DTS):** 节点结构、`compatible` 属性、overlay 机制
- **进程间通信 (IPC):**
  - 管道/Pipe、共享内存 (`shm_open` / `mmap`)
  - Unix Domain Socket、D-Bus
  - MCU ↔ Linux 跨核通信 (OpenAMP / RPMsg)
- **网络编程:** `select` / `poll` / `epoll` 的区别与适用场景；零拷贝 `sendfile` / `splice`

### 调试与优化工具

| 工具 | 用途 |
|------|------|
| `gdb` | 多线程调试、coredump 分析、remote debugging |
| `perf` | CPU 性能瓶颈分析 (top-down)、cache miss、分支预测 |
| `strace` | 系统调用追踪，排查 syscall 级别的失败 |
| `Valgrind` | 内存泄漏、非法访问、竞争条件检测 |
| `ftrace` / `bpftrace` | 内核函数追踪、动态插桩 |
| `iostat` / `iotop` | 磁盘 IO 瓶颈定位 |

### 性能调优

- **实时性:** `PREEMPT_RT` 内核补丁、`isolcpus` CPU 隔离
- **中断亲和性:** `irqbalance`、`/proc/irq/*/smp_affinity`
- **调度策略:** `SCHED_FIFO` vs `SCHED_RR` vs `SCHED_DEADLINE`

## 推荐知识库

- **Bootlin Training Materials:** 全球顶级的嵌入式 Linux 开源培训资料 (`bootlin.com/docs/`)
- **Linux Device Drivers (LDD3):** 驱动开发的经典入门
- **Brendan Gregg's Perf Tools:** `brendangregg.com/perf.html`
