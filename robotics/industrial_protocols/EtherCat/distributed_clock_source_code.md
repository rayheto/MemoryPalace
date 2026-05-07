# SOEM 分布式时钟源码位置

## 核心文件

| 文件 | 作用 |
|------|------|
| `src/ec_dc.c` | 分布式时钟全部实现逻辑 |
| `include/soem/ec_dc.h` | 对外公开的函数声明 |
| `include/soem/ec_type.h` | 寄存器地址宏定义（`ECT_REG_DC*`） |

## 对外接口（ec_dc.h）

```c
boolean ecx_configdc(ecx_contextt *context);
void ecx_dcsync0(ecx_contextt *context, uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift);
void ecx_dcsync01(ecx_contextt *context, uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift);
```

## ec_dc.c 各函数说明

### `ecx_configdc()`（第 250 行）
- 广播写 `ECT_REG_DCTIME0`，触发所有从站同时锁存各端口接收时间戳
- 读取主站当前时间，将 Unix 纪元（1970）转换为 EtherCAT 纪元（2000，差值 946684800 秒）
- 读取每个有 DC 能力从站的本地时间，计算与主站的差值，写入 `ECT_REG_DCSYSOFFSET`
- 利用各端口时间戳差值计算主站到每个从站的传播延迟，写入 `ECT_REG_DCSYSDELAY`
- 建立 DC 从站双向链表（`DCnext` / `DCprevious`）

### `ecx_dcsync0()`（第 33 行）
- 配置从站以固定周期产生 SYNC0 脉冲
- 计算首次触发时刻：`((当前时间 + 100ms) / CyclTime) * CyclTime + CyclTime + CyclShift`
- 保证相同 CyclTime 的所有从站在同一时刻触发第一个脉冲
- 写入寄存器：`ECT_REG_DCSTART0`（起始时间）、`ECT_REG_DCCYCLE0`（周期）、`ECT_REG_DCSYNCACT`（激活）

### `ecx_dcsync01()`（第 92 行）
- 在 SYNC0 基础上额外配置 SYNC1 信号
- SYNC1 周期必须是 SYNC0 周期的整数倍
- 激活字节：`1 + 2 + 4`（周期使能 + SYNC0 + SYNC1）

## 关键寄存器

| 寄存器 | 用途 |
|--------|------|
| `ECT_REG_DCTIME0~3` | 各端口（Port 0-3）接收时间戳锁存值 |
| `ECT_REG_DCSOF` | 从站 64 位系统时间（用于计算偏移量） |
| `ECT_REG_DCSYSOFFSET` | 主站写入的时钟偏移量，用于对齐从站时间 |
| `ECT_REG_DCSYSDELAY` | 传播延迟补偿值 |
| `ECT_REG_DCSYSTIME` | 从站当前系统时间 |
| `ECT_REG_DCSTART0` | SYNC0 首次触发时刻 |
| `ECT_REG_DCCYCLE0/1` | SYNC0/SYNC1 周期 |
| `ECT_REG_DCSYNCACT` | 同步信号激活控制寄存器 |
| `ECT_REG_DCCUC` | EtherCAT 寄存器写访问解锁 |
