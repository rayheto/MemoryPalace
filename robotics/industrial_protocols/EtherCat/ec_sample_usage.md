# ec_sample 示例程序使用说明

## 项目信息
- 项目：SOEM（Simple Open EtherCAT Master）
- 路径：`~/robotic/EtherCat/SOEM`

## 编译

```bash
cd ~/robotic/EtherCat/SOEM
mkdir -p build/default && cd build/default
cmake ../.. && make
```

生成的可执行文件：`build/default/bin/ec_sample`

## 用法

```bash
sudo ./build/default/bin/ec_sample <网卡名> [周期时间(微秒)]
```

| 参数 | 必填 | 说明 |
|------|------|------|
| `网卡名` | 是 | 连接 EtherCAT 从站的网口名，如 `enp4s0` |
| `周期时间` | 否 | PDO 循环周期，单位微秒，默认 1000（即 1ms） |

> **必须加 `sudo`**，因为需要 raw socket 权限才能收发以太网帧。

## 示例

```bash
# 使用 enp4s0，默认 1ms 周期
sudo ./build/default/bin/ec_sample enp4s0

# 使用 enp4s0，指定 2ms 周期
sudo ./build/default/bin/ec_sample enp4s0 2000
```

## 程序运行过程

1. 扫描并配置总线上所有 EtherCAT 从站
2. 配置分布式时钟（`ecx_configdc`）
3. 将有 CoE 邮箱的从站加入周期性邮箱处理器
4. 等待 1 秒，让 DC 时钟稳定
5. 切换到 OP（运行）状态
6. 循环运行 **100 秒**（5000 次 × 20ms），每次打印：
   ```
   Processdata cycle  1234 , Wck   3, DCtime  123456789012, dt      500, O: xx xx  I: xx xx
   ```
   - `Wck`：Working Counter，等于 `expectedWKC` 说明通信正常
   - `DCtime`：当前 DC 时间（纳秒）
   - `dt`：主站时钟与 DC 基准的误差（纳秒），PI 控制器会将其收敛到接近 0
7. 依次退回 SAFE_OP → INIT 并退出

## 线程结构

| 线程 | 类型 | 职责 |
|------|------|------|
| `ecatthread` | 实时线程 | 周期性 PDO 收发 + DC 时钟 PI 同步控制 |
| `ecatcheck` | 普通线程 | 监控从站状态，恢复丢失或故障的从站 |

## 常见问题：为什么直接打印"End program"就退出？

```
EtherCAT Startup
End program
```

原因：`ctx.slavecount == 0`，总线上未发现任何 EtherCAT 从站。

EtherCAT 使用专用帧格式（Ethertype `0x88A4`），普通交换机、PC、路由器**不会响应**这些帧。
必须连接真实的 EtherCAT 从站硬件才能正常运行。

## 本机可用网卡

| 网卡 | 说明 |
|------|------|
| `enp4s0` | 物理以太网口，用于连接真实从站 |
| `wlo1` | 无线网卡，不能用于 EtherCAT |
| `veth-master` | 虚拟网卡，用于软件从站测试 |
| `veth-slave` | 虚拟网卡对端 |
| `docker0` | Docker 网桥，不适用 |
