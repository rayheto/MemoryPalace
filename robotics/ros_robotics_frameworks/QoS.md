
# ROS2 QoS（Quality of Service）策略完全指南

## 概述

QoS策略是ROS2中基于DDS的一项关键特性，允许用户精细控制通信质量，实现在不同网络条件和应用场景下的最优性能。

## 1. 基本用法

修改发布者或订阅者代码，添加QoS配置：

```c++
// 使用预定义的QoS配置文件
auto publisher = node->create_publisher<std_msgs::msg::String>(
  "topic_name",
  rclcpp::QoS(10).reliable());  // 队列大小为10，可靠传输

// 或者使用系统默认的QoS配置文件
auto publisher = node->create_publisher<std_msgs::msg::String>(
  "topic_name", 
  rclcpp::SystemDefaultsQoS());
  
// 订阅者的QoS配置方式类似
auto subscription = node->create_subscription<std_msgs::msg::String>(
  "topic_name", 
  rclcpp::QoS(10).reliable(),
  callback);
```

## 2. 预定义的QoS配置文件

ROS2提供了几个针对特定用例优化的预定义配置文件：

```c++
// 传感器数据 - 优先考虑新数据而非可靠性
// 默认值：深度=5, 可靠性=最佳努力, 持久性=易失性
rclcpp::SensorDataQoS()

// 参数事件 - 可靠通信
// 默认值：深度=1000, 可靠性=可靠, 持久性=易失性
rclcpp::ParameterEventsQoS()

// 系统默认 - 平衡性能
// 默认值：深度=10, 可靠性=可靠, 持久性=易失性
rclcpp::SystemDefaultsQoS()

// 服务默认 - 可靠通信
// 默认值：历史=保留全部, 可靠性=可靠, 持久性=易失性
rclcpp::ServicesQoS()

// ROS2 话题状态QoS
// 默认值：深度=1, 可靠性=最佳努力, 持久性=瞬态本地
rclcpp::TopicStatusQoS()
```

## 3. 自定义QoS参数

```c++
// 完整自定义QoS策略
auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)  // 可靠传输
  .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)  // 瞬态本地持久
  .deadline(std::chrono::milliseconds(200))  // 截止时间
  .lifespan(std::chrono::seconds(1))  // 生命周期
  .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)  // 自动活跃度
  .liveliness_lease_duration(std::chrono::seconds(1));  // 活跃度租约时长

auto publisher = node->create_publisher<std_msgs::msg::String>("topic_name", qos);
```

## 4. QoS策略详解

### 可靠性 (Reliability)
- **RELIABLE**: 确保所有消息都被传递，如果传输失败会尝试重传
- **BEST_EFFORT**: 尽力传递但不保证，适用于可以容忍丢失的高频率数据

```c++
// 简便方法
auto qos = rclcpp::QoS(10).reliable();  // 或 .best_effort()

// 显式设置
auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
```

### 持久性 (Durability)
- **VOLATILE**: 不为迟到加入的订阅者保存数据
- **TRANSIENT_LOCAL**: 发布者会为迟到加入的订阅者保存数据

```c++
// 简便方法
auto qos = rclcpp::QoS(10).transient_local();  // 或 .volatile_durability()

// 显式设置
auto qos = rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
auto qos = rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
```

### 历史记录 (History)
- **KEEP_LAST**: 保留最近的N条消息，N由队列深度指定
- **KEEP_ALL**: 尝试保留所有消息，受限于底层DDS资源限制

```c++
// 简便方法
auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
auto qos = rclcpp::QoS(rclcpp::KeepAll());

// 显式设置
auto qos = rclcpp::QoS(10).history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10);
auto qos = rclcpp::QoS(10).history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
```

### 其他QoS参数

- **截止时间 (Deadline)**: 指定消息应当多久接收一次
- **生命周期 (Lifespan)**: 消息有效期，过期后丢弃
- **活跃度 (Liveliness)**: 确定发布者是否"活着"的策略
- **活跃度租约时长**: 多久没有活动会被认为不活跃

## 5. QoS兼容性

发布者和订阅者的QoS策略必须兼容才能成功通信：

| QoS策略 | 要求 |
|--------|------|
| 可靠性 | 发布者的可靠性级别必须 >= 订阅者 |
| 持久性 | 发布者的持久性级别必须 >= 订阅者 |
| 截止时间 | 发布者的截止时间必须 <= 订阅者 |

不兼容的QoS会导致缺少通信。可以使用`ros2 doctor --report`检测此类问题。

## 6. 实用示例

### 实时传感器数据处理

```c++
// 高频传感器数据优先考虑及时性，可容忍部分丢失
auto sensor_qos = rclcpp::QoS(5)
  .best_effort()            // 最佳努力传输
  .durability_volatile();   // 不保存历史数据

// 用于传感器数据的订阅者
auto sub = node->create_subscription<sensor_msgs::msg::Image>(
  "camera/image", sensor_qos, image_callback);
```

### 系统配置信息发布

```c++
// 配置数据需要可靠传输，并对后加入的节点可见
auto config_qos = rclcpp::QoS(1)
  .reliable()               // 可靠传输
  .transient_local();       // 为后来的订阅者保留

// 用于发布配置数据
auto config_pub = node->create_publisher<std_msgs::msg::String>(
  "system/config", config_qos);
```

## 7. 常用命令行工具

检查话题的QoS配置:
```bash
ros2 topic info --verbose /topic_name
```

检测QoS不匹配问题:
```bash
ros2 doctor --report
```

## 8. 注意事项与最佳实践

- 根据通信需求选择适当的QoS，不要盲目选择最高级别
- 可靠性传输会增加延迟和资源使用，适用于重要但不频繁的数据
- 最佳努力传输适合高频率、可容忍丢失的数据流
- 瞬态本地持久性对于配置、地图等静态数据特别有用
- 测试不同的QoS组合，找到适合特定应用场景的配置
- 注意不同DDS实现对QoS支持可能有细微差异
