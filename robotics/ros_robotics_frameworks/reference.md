# 参考资源

## ROS2 官方

- **ROS2 Documentation (Rolling):** [https://docs.ros.org/en/rolling/](https://docs.ros.org/en/rolling/)
  - 核心概念、教程、API 参考。滚动版本对应最新开发版本，选择对应发行版（Humble/Iron/Jazzy）查看稳定文档
- **ROS2 Design:** [https://design.ros2.org/](https://design.ros2.org/)
  - 架构设计文档，理解"为什么这么做"：Node 生命周期、QoS 设计、DDS 选型原因
- **ROS2 GitHub:** [https://github.com/ros2](https://github.com/ros2)
  - rclcpp/rclpy 源码，适合深入理解 Executor、QoS 底层实现
- **ROS Enhancement Proposals (REP):** [https://ros.org/reps/rep-0000.html](https://ros.org/reps/rep-0000.html)
  - REP-103: 坐标系标准（x前/y左/z上）
  - REP-105: 坐标帧语义（map/odom/base_link）

## DDS

- **eProsima Fast DDS:** [https://github.com/eProsima/Fast-DDS](https://github.com/eProsima/Fast-DDS)
  - ROS2 默认 DDS 实现，学习 Discovery 机制和 QoS 实现
- **Eclipse Cyclone DDS:** [https://github.com/eclipse-cyclonedds/cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds)
  - 轻量化 DDS 替代方案，在某些场景下延迟更低
- **DDS 标准 (OMG DDS Specification):** [https://www.omg.org/spec/DDS/](https://www.omg.org/spec/DDS/)
  - DDS 协议规范原文

## TF2

- **tf2 官方教程:** [https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Tf2-Main.html](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- **tf2 源码 (geometry2):** [https://github.com/ros2/geometry2](https://github.com/ros2/geometry2)

## ros2 bag

- **rosbag2 官方文档:** [https://github.com/ros2/rosbag2](https://github.com/ros2/rosbag2)
- **MCAP 格式:** [https://mcap.dev/](https://mcap.dev/)

## RViz2

- **RViz2 源码:** [https://github.com/ros2/rviz](https://github.com/ros2/rviz)
  - Marker 消息类型参考: `visualization_msgs/msg/Marker`

## 机器人专用框架

### ros2_control
- **ros2_control 文档:** [https://control.ros.org/](https://control.ros.org/)
  - Hardware Interface 架构、Controller 类型、URDF 集成
- **ros2_control 源码:** [https://github.com/ros-controls/ros2_control](https://github.com/ros-controls/ros2_control)

### MoveIt2
- **MoveIt2 文档:** [https://moveit.picknik.ai/](https://moveit.picknik.ai/)
  - 运动规划管线、碰撞检测、SRDF 语义描述
- **MoveIt2 源码:** [https://github.com/ros-planning/moveit2](https://github.com/ros-planning/moveit2)

### Navigation2
- **Nav2 文档:** [https://docs.nav2.org/](https://docs.nav2.org/)
- **Nav2 源码:** [https://github.com/ros-planning/navigation2](https://github.com/ros-planning/navigation2)

## 其他机器人框架

- **Orocos RTT (Real-Time Toolkit):** [https://www.orocos.org/rtt/](https://www.orocos.org/rtt/)
  - 面向硬实时的组件模型，可与 ROS 桥接
- **YARP (Yet Another Robot Platform):** [https://www.yarp.it/](https://www.yarp.it/)
  - iCub 人形机器人平台使用，支持 ROS 互操作
- **LCM (Lightweight Communications and Marshalling):** [https://lcm-proj.github.io/](https://lcm-proj.github.io/)
  - MIT 研发的轻量通信库，UDP 多播，适合嵌入式场景
- **Eclipse zenoh:** [https://zenoh.io/](https://zenoh.io/)
  - 下一代边缘-云统一通信协议，支持 DDS bridge 和 ROS2 桥接
- **ROS1 → ROS2 迁移:** [https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Migration-Guide.html](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Migration-Guide.html)

## 开源参考项目

- **Universal Robots ROS2 Driver:** [https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
  - 工业机械臂 ROS2 驱动参考实现，学习 ros2_control 和硬实时集成
- **ROS2 Real-Time Working Group:** [https://github.com/ros-realtime](https://github.com/ros-realtime)
  - PREEMPT_RT + ROS2 实时性优化参考
