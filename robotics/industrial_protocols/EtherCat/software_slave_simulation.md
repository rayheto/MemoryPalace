# 在没有真实硬件时模拟 EtherCAT 从站

## 为什么通常需要真实硬件？

EtherCAT 从站的核心是专用硬件 ESC（EtherCAT Slave Controller）芯片（如 ET1100、LAN9252），
负责在纳秒级别处理帧转发和时钟同步。纯软件在 Linux 上很难保证这个时序精度。

## SOES（Simple Open EtherCAT Slave）

- 仓库：https://github.com/OpenEtherCATsociety/SOES
- 本机路径：`~/robotic/EtherCat/SOES`

### 编译

```bash
cd ~/robotic/EtherCat/SOES
mkdir build && cd build
cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..   # CMake 4.x 必须加此参数
make
```

### 重要限制

SOES **没有**基于 Linux 网口（raw socket）的从站实现，所有应用都依赖真实硬件芯片：

| 应用目录 | 依赖硬件 |
|----------|----------|
| `linux_lan9252demo` | LAN9252 芯片，通过 `/dev/lan9252` 访问 |
| `raspberry_lan9252demo` | 树莓派 + LAN9252 |
| `rtl_slavedemo` | RTL 实时内核 |

`linux_lan9252demo/main.c` 中硬编码了 `user_arg = "/dev/lan9252"`，
没有实际芯片无法运行。

## veth 虚拟网卡对搭建测试环境

可以用 veth 对让主站和从站在同一台机器上通信（帧在内核内转发，无需硬件）：

```bash
# 创建 veth 对
sudo ip link add veth-master type veth peer name veth-slave
sudo ip link set veth-master up
sudo ip link set veth-slave up

# 终端 A：从站监听 veth-slave（需要有支持 Linux socket 的从站程序）
sudo ./slave_program veth-slave

# 终端 B：主站监听 veth-master
sudo ./build/default/bin/ec_sample veth-master

# 用完清理（删一端，另一端自动消失）
sudo ip link delete veth-master
```

## 可用的替代方案

| 方案 | 成本 | 说明 |
|------|------|------|
| 真实 EtherCAT 伺服驱动器 | 中高 | 最适合实际开发 |
| ET1100 / LAN9252 评估板 | 低 | 适合测试 SOES |
| 树莓派 + LAN9252 HAT | 低 | 可运行 SOES raspberry demo |
| IgH EtherCAT Master 的 `ec-slave` 工具 | 免费 | 软件从站，时序精度有限 |

## 为什么纯软件模拟很难？

EtherCAT 帧必须在每个从站被处理并转发，延迟要求在几百纳秒以内。
Linux 进程在没有实时内核补丁（PREEMPT_RT）和专用硬件的情况下无法保证此延迟。
