---
tags:
  - robotics
  - reliability_lifecycle
  - subnode
---

# 可靠性设计与样机工程化

解决"实验室能跑"到"产品能卖"的问题。

## 面试核心考点

### FMEA (失效模式与影响分析)

- **SFMEA (系统级) vs DFMEA (设计级) vs PFMEA (过程级):**
  - 识别潜在失效模式
  - 评估严重度 (Severity)、发生频率 (Occurrence)、可检测度 (Detection)
  - 计算 RPN (Risk Priority Number) = S × O × D
- **机器人关节典型 FMEA:**
  - 电机堵转 → 过流保护 / 电流环异常检测
  - 编码器故障 → 冗余编码器或无传感器估算切换
  - 通信丢帧 → EtherCAT Watchdog / CAN Heartbeat 超时保护
  - 制动器失效 → 重力补偿 + 软制动策略

### 热设计

- 电机高负载发热导致:
  - 永磁体退磁 (超过 Curie 温度)
  - 绕组绝缘层老化加速
  - 热保护策略: NTC 温度传感器 + 降额 (Derating) 曲线
- 散热设计:
  - 自然对流 vs 强制风冷 vs 液冷
  - 热界面材料 (TIM) 选型
  - PCB 铜皮厚度与散热过孔

### 机械可靠性

- **振动环境:** 连接器选型 (带锁扣、IP 等级)、软线束防干涉与应力释放
- **极限工况验证:**
  - 最高速度连续运行 (Velocity Endurance)
  - 最大负载连续运行 (Torque Endurance)
  - 温箱高低温循环 (Thermal Cycling)
  - 跌落与冲击测试

### 工程化落地

- **版本管理:** bootloader 分区、固件回滚机制
- **生产测试:** ICT (在线测试)、FCT (功能测试)、校准工装
- **可维修性:** 模块化设计、故障指示 (LED/Beep Code)

## 推荐知识库

- **GB/T 标准:** 工业机器人环境适应性测试标准 (高低温、振动、EMC)
- **IEC 61508:** 功能安全基础标准 (机器人安全相关系统)
- **IPC-A-610:** 电子组件的可接受性标准

