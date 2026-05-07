# EtherCAT 分布式时钟原理与配置步骤详解

## DC 解决什么问题？

EtherCAT 总线上每个从站都有独立的内部时钟。如果不同步，各从站会在稍微不同的时刻执行命令。
分布式时钟（Distributed Clock，DC）让所有从站的时钟在同一时刻走到同一个值，
使主站发出"在时刻 T 执行"的命令时，所有从站能真正同步响应，误差在纳秒级。

---

## 第 1 步：初始化主站网卡

```c
ecx_init(&ctx, "enp4s0");
```

**意义：** 打开指定网卡的 raw socket，主站才能收发 EtherCAT 帧（Ethertype 0x88A4）。此时总线上什么都还没发生。

---

## 第 2 步：扫描并发现从站

```c
ecx_config_init(&ctx);
```

**意义：** 主站广播帧，发现总线上所有从站，给每个从站分配地址（`configadr`），
读取从站基本信息（支持的协议、是否有 DC 能力等）。
执行完成后 `ctx.slavecount` 变为实际发现的从站数量。

---

## 第 3 步：映射过程数据（PDO）

```c
ecx_config_map_group(&ctx, IOmap, 0);
```

**意义：** 配置每个周期需要交换的数据布局（哪些从站寄存器映射到 `IOmap` 的哪个位置）。
**必须在 `ecx_configdc` 之前完成**，因为 DC 配置需要总线拓扑已经确定。

---

## 第 4 步：配置分布式时钟（核心）

```c
ecx_configdc(&ctx);
```

**意义：** DC 配置最复杂的一步，内部分三个子操作：

### 4a. 锁存各从站端口时间戳

```c
ecx_BWR(&ctx.port, 0, ECT_REG_DCTIME0, ...);  // 广播写，触发所有从站同时锁存
```

主站发出一帧广播写，帧沿总线依次经过每个从站时，从站硬件自动把帧到达各端口的时刻
记录下来（Port 0/1/2/3 分别对应 `DCrtA/B/C/D`）。
这是计算传播延迟的原始数据。

### 4b. 对齐从站时钟到主站时间

```c
hrt = htoell(-etohll(hrt) + mastertime64);
ecx_FPWR(..., ECT_REG_DCSYSOFFSET, ...);  // 写入时钟偏移量
```

读取每个从站当前的本地时间，计算它与主站时间的差值，写入从站的 `DCSYSOFFSET` 寄存器。
从此从站的"系统时间 = 硬件本地时间 + 偏移量"，所有从站的系统时间都对齐到主站时钟。

> 注意：EtherCAT 使用 2000-01-01 作为纪元起点，而非 Unix 的 1970-01-01。
> 主站时间需减去 946684800 秒做转换。

### 4c. 计算并写入传播延迟补偿

```c
pdelay = ((dt3 - dt1) / 2) + dt2 + parent_pdelay;
ecx_FPWR(..., ECT_REG_DCSYSDELAY, ...);  // 写入延迟补偿
```

利用 4a 锁存的端口时间戳差值，计算主站到每个从站的单程传播延迟（假设去程等于回程）。
写入 `DCSYSDELAY` 后，从站硬件自动用此值补偿自己的时钟，实现物理意义上的时间对齐。

---

## 第 5 步：启动 SYNC 同步信号

```c
ecx_dcsync0(&ctx, slave_index, TRUE, 1000000, 0);
//                              ↑激活  ↑周期1ms  ↑偏移0ns
```

**意义：** 告诉从站"从下一个整周期时刻起，每隔 1ms 产生一个 SYNC0 脉冲"。

首次触发时刻的计算方式：
```
t = ((当前时间 + 100ms) / CyclTime) * CyclTime + CyclTime + CyclShift
```

此公式保证：**所有使用相同 CyclTime 的从站，第一个脉冲在完全相同的时刻触发**。
`CyclShift` 可以让某个从站故意错开（例如先采样输入再驱动输出）。

---

## 第 6 步：PI 控制器——让主站跟踪 DC 节拍

```c
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    delta = (reftime - syncoffset) % cycletime;
    if (delta > cycletime / 2) delta = delta - cycletime;
    integral += delta;
    *offsettime = (int64)(delta * pgain + integral * igain);
}
```

**意义：** 从站 DC 时钟是基准，但 Linux 系统时钟会漂移。这个 PI 控制器每周期：

- 计算主站唤醒时刻与 DC 基准时刻的误差 `delta`
- 比例项（`pgain = 0.01`）快速纠正大误差
- 积分项（`igain = 0.00002`）消除长期稳态误差
- 输出 `toff` 用于调整下一次睡眠时长，使主站唤醒节拍逐渐锁定到 DC 时钟

---

## 第 7 步：OP 状态下的实时循环

```c
wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
ec_sync(ctx.DCtime, cycletime, &toff);   // 更新时钟补偿量
ecx_send_processdata(&ctx);              // 发送下一帧
```

**意义：** `ctx.DCtime` 从每帧收到的报文中提取（第一个 DC 从站的系统时间），每周期更新。
持续喂给 PI 控制器，维持主站与从站的长期时钟同步。

---

## 最简调用顺序

```c
ecx_init(&ctx, ifname);                          // 1. 打开网卡
ecx_config_init(&ctx);                           // 2. 发现从站
ecx_config_map_group(&ctx, IOmap, 0);            // 3. 映射 PDO
ecx_configdc(&ctx);                              // 4. 配置 DC
ecx_dcsync0(&ctx, 1, TRUE, 1000000, 0);         // 5. 启动 SYNC0
osal_usleep(1000000);                            // 等待 1 秒让时钟稳定
ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
ecx_writestate(&ctx, 0);                         // 6. 进入 OP 状态
// 实时循环：receive → ec_sync → send
```

## 整体示意图

```
主站时间 ──[写DCSYSOFFSET]──► 从站1系统时间 ──► SYNC0脉冲
                                                      │
          [写DCSYSDELAY]──► 从站2系统时间 ──► SYNC0脉冲（同一时刻）
                                                      │
          PI控制器让主站唤醒节拍锁定到 DC 基准时钟
```
