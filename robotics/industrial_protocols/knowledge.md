# 工业通信协议 (EtherCAT)

EtherCAT 在机械臂、人形机器人和高端工控中几乎是标配。

## 面试核心考点

### EtherCAT 工作原理

- **On-the-fly 处理机制:** 从站设备在数据帧经过时"就地"读取/写入数据，不等整个帧接收完毕，实现极低延迟
- **分布时钟 (Distributed Clocks, DC):** 如何实现多个从站间微秒级时间同步；Reference Clock 与 Slave Clock 的校准过程
- **PDO (过程数据对象) vs SDO (服务数据对象):**
  - PDO: 周期性的实时控制数据（位置/速度/力矩指令），低开销、高频率
  - SDO: 非周期性的配置/诊断数据（参数读写），高开销、低频率
- **EtherCAT State Machine:** Init → Pre-Op → Safe-Op → Op 四状态转换

### CiA 402 (CANopen over EtherCAT)

- 伺服驱动器标准状态机
- Control Word (6040h) / Status Word (6041h) 的 bit 定义
- 模式选择 (6060h): PP (Profile Position)、PV (Profile Velocity)、CSP (Cyclic Synchronous Position)、CSV、CST
- 实际位置 (6064h) / 目标位置 (607Ah) / 实际速度 (606Ch)

### 主站实现

- SOEM (Simple Open EtherCAT Master) 架构理解
- 主站发包流程: 配置从站 → 创建 Domain → 注册 PDO 条目 → 周期性发送 EtherCAT 帧
- EtherCAT 帧结构: Ethernet Header + EtherCAT Header + Datagrams + FCS

## 推荐知识库

- **SOEM (GitHub):** `github.com/OpenEtherCATsociety/SOEM` —— 粗读源码了解主站发包和从站配置流程
- **Beckhoff 官方文档:** 了解从站控制器 (ESC) 基础概念和寄存器定义
- **CiA 402 规范:** CAN in Automation 官网
