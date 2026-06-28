---
tags:
  - robotics
  - ros_robotics_frameworks
  - subnode
---

# ROS2 与机器人框架

ROS2 是机器人软件工程的事实标准中间件。对于机器人软件工程师岗位，ROS2 不仅是"会用"，更需要理解其通信模型、QoS 策略、DDS 底层以及各工具的工程适配。

---

## 知识体系（按重要性分层）

### 第一层：核心通信模型（必须精通）

| 主题 | 文档 | 核心要点 |
|------|------|---------|
| ROS2 核心概念 | [ros2_core_concepts.md](ros2_core_concepts.md) | Node / Topic / Service / Action 四者语义与适用场景；Executor 回调模型（SingleThreaded vs MultiThreaded）；Node Composition 进程内零拷贝通信 |
| TF2 坐标变换 | [tf2.md](tf2.md) | TF2 树结构与广播/监听机制；静态 vs 动态变换；`lookupTransform` 时间戳匹配与超时；`tf2_ros::Buffer` 缓存策略；常用转换（四元数→欧拉角→旋转矩阵） |

### 第二层：通信质量与数据管理（工程必备）

| 主题 | 文档 | 核心要点 |
|------|------|---------|
| DDS 与 QoS 策略 | [QoS.md](../motor_control_sensor_fusion/QoS.md) | Reliability / Durability / Deadline / Liveliness / Lifespan；预定义 QoS Profile 选型；发布-订阅 QoS 兼容性匹配规则："发布者的可靠性 ≥ 订阅者" |
| ros2 bag 数据录制 | [ros2_bag_rviz2.md](ros2_bag_rviz2.md) | `ros2 bag record` 话题过滤与正则；SQLite3 存储后端；`ros2 bag play` 回放速率与时序控制；bag 文件作为离线算法验证的标准输入源 |

### 第三层：工程化与可视化（开发效率）

| 主题 | 文档 | 核心要点 |
|------|------|---------|
| Launch 系统 | [launch.md](launch.md) | XML vs Python Launch；`launch.actions` (DeclareLaunchArgument / Node / TimerAction)；参数传递与 YAML 配置加载；条件启动与生命周期绑定 |
| RViz2 可视化 | [ros2_bag_rviz2.md](ros2_bag_rviz2.md) | 常用 Display Type（TF / Image / PointCloud2 / MarkerArray）；固定帧配置；位姿在线调试方法；插件扩展 |

### 第四层：机器人专用框架（面试加分项）

| 主题 | 核心要点 |
|------|---------|
| ros2_control | Controller Manager 架构；硬件抽象层（Hardware Interface）：Command / State 接口；Controller 类型（JointTrajectoryController / DiffDriveController）；URDF `ros2_control` 标签配置 |
| MoveIt2 | 运动规划管线（Planning Pipeline）；OMPL 规划器集成；碰撞检测（FCL / Bullet）；SRDF 语义描述；`move_group` C++/Python API 调用流程 |

---

## 面试核心考点速查

### ROS2 核心概念

- **Node → Topic（发布-订阅）:** 单向多对多数据流；异步解耦；典型场景：传感器数据 (`sensor_msgs/Image`)、状态广播 (`nav_msgs/Odometry`)
- **Node → Service（请求-响应）:** 同步一对一 RPC；有反馈有结果；典型场景：参数查询、触发标定、运动学求解
- **Node → Action（带反馈的长时间任务）:** 异步有状态机；支持取消/暂停/反馈；典型场景：导航 (`navigate_to_pose`)、抓取 (`grasp_object`)
- **Executor 模型:**
  - `SingleThreadedExecutor`: 所有回调按顺序执行，无竞态，但高负载回调阻塞其他回调
  - `MultiThreadedExecutor`: 多个线程并行处理回调，需注意数据竞争；通过 `callback_group` 控制回调归属
  - `StaticSingleThreadedExecutor`: 适用于硬实时，减少动态内存分配
- **Node Composition (Component):** 多个 Node 共享同一进程空间，以 `rclcpp::NodeOptions` 加载；零拷贝 intra-process 通信；降低内存占用与上下文切换开销

### DDS 与 QoS

- **DDS (Data Distribution Service):** ROS2 底层通信中间件；默认实现为 Fast DDS (eProsima)；可替换为 Cyclone DDS
- **Discovery 机制:** Simple Discovery Protocol；Participant → Endpoint 自动发现；无需 Master 节点（与 ROS1 的关键区别）
- **QoS 五大策略及选型速查:**

| 策略 | 关键选择 | 传感器数据 | 配置数据 | 控制指令 |
|------|---------|-----------|---------|---------|
| Reliability | RELIABLE / BEST_EFFORT | BEST_EFFORT | RELIABLE | RELIABLE |
| Durability | VOLATILE / TRANSIENT_LOCAL | VOLATILE | TRANSIENT_LOCAL | VOLATILE |
| History | KEEP_LAST(N) / KEEP_ALL | KEEP_LAST(5) | KEEP_LAST(1) | KEEP_LAST(10) |
| Deadline | 期望最大间隔 | 按帧率设定 | — | 按控制周期 |
| Liveliness | AUTOMATIC / MANUAL_BY_TOPIC | AUTOMATIC | AUTOMATIC | MANUAL_BY_TOPIC |

- **QoS 兼容性黄金法则:** 发布者的 QoS 级别必须 ≥ 订阅者；Durability/Reliability 发布者低于订阅者 → 静默通信失败（`ros2 doctor --report` 诊断）

### TF2

- **TF 树:** 有向无环图；每个边表示一个坐标变换；从任意坐标系到任意坐标系的变换通过树上路径推算
- **广播 (Broadcaster):** `tf2_ros::TransformBroadcaster` 周期发送 `geometry_msgs/TransformStamped`
- **监听 (Listener):** `tf2_ros::Buffer` 缓存历史变换；`lookupTransform(target, source, time)` 查询两帧间变换
- **时间戳匹配:** `lookupTransform` 默认取最近可用帧（`tf2::TimePointZero`）；指定 `time` 时系统自动插值
- **静态变换:** 不随时间变化的固定变换，通过 `StaticTransformBroadcaster` 发送，仅在订阅者连接时发送一次，但会被缓存
- **TF2 vs TF (ROS1):** TF2 支持时间回溯查询、缓冲区管理 API 更清晰、C++ API 模板化

### Launch 系统

- **Launch 文件两种写法:**
  - XML：简洁明了，适合简单场景；`<node>` `<param>` `<arg>` `<include>`
  - Python：图灵完备，支持条件逻辑、循环、动态参数计算；适合复杂部署
- **关键 Action:**
  - `DeclareLaunchArgument`: 声明可配置参数，可通过命令行 `ros2 launch ... arg:=value` 覆盖
  - `Node`: 启动一个 ROS2 节点，指定 package、executable、parameters、remappings
  - `IncludeLaunchDescription`: 嵌套其他 Launch 文件，实现模块化部署
  - `TimerAction`: 延迟启动
  - `GroupAction`: 批量设置命名空间/参数作用域
- **参数传递链:** YAML 文件 → `LaunchConfiguration` → Node `parameters`；支持嵌套参数（`ns.param_name`）
- **Remapping:** 重映射话题/服务名称，实现同一节点在不同场景下的复用：`("original_topic", "new_topic")`

### ros2 bag

- **录制:** `ros2 bag record -o <output_dir> <topic1> <topic2>`；`-a` 录制所有话题；`--regex` 正则过滤
- **回放:** `ros2 bag play <bag_dir>`；`--rate 0.5` 半速回放；`--loop` 循环回放；`--topics` 选择性回放
- **信息查看:** `ros2 bag info <bag_dir>` 查看话题类型、消息数量、起止时间戳
- **存储:** 默认 SQLite3 存储后端，`ros2 bag` 也支持 MCAP 格式（ROS2 Iron 起默认）
- **工程价值:** 录制一次现场数据，离线反复迭代算法，避免反复部署硬件；回归测试自动化

### RViz2

- **Display 体系:** 插件化架构，核心类型：TF / Image / PointCloud2 / Marker / MarkerArray / RobotModel / LaserScan
- **固定帧 (Fixed Frame):** 所有可视化的参考坐标系，通常设为 `map` 或 `base_link`；设置错误会导致显示空白
- **Marker:** 程序化绘制基本几何体（箭头/球体/立方体/文字），调试位姿、路径、检测框首选
- **交互:** `Publish Point` 工具发布 `geometry_msgs/PointStamped`；可自定义交互面板（Panel Plugin）

---

## ROS2 工程最佳实践

1. **QoS 选型原则:** 传感器数据用 SensorDataQoS（非可靠 + 低延迟）；配置/参数用 Reliable + Transient Local（后来者可见）；控制指令用 Reliable + Deadline
2. **话题命名规范:** `<namespace>/<semantic_name>`，如 `/camera_left/image_raw`、`/arm/joint_states`；避免全局命名冲突
3. **参数管理:** 所有可调参数放入 YAML 文件，通过 Launch 参数化注入；运行时修改通过 `ros2 param set`
4. **错误处理:** 使用 `rclcpp::Logger` 分级输出（DEBUG/INFO/WARN/ERROR/FATAL）；`RCUTILS_LOG_ERROR` 宏自动记录文件名/行号
5. **实时性考虑:** 控制周期用 `rclcpp::Rate` 或 Timer 精确控制；避免在回调中执行耗时操作（I/O、大内存分配）
6. **坐标系统:** 统一使用右手坐标系；`map`(世界) → `odom`(里程计) → `base_link`(本体) → 各传感器帧；与 REP-105 对齐

---

## 推荐知识库

详见 [reference.md](reference.md)

## 其他机器人框架（快速索引）

| 框架 | 定位 | 关键点 |
|------|------|--------|
| ROS1 (Melodic/Noetic) | 传统工业 | Master 节点中心化；roscpp/rospy；Bag 格式与 ROS2 不兼容；已在 ROS2 中替代 |
| Orocos RTT | 实时控制 | 组件模型；实时工具包；与 ROS 集成 |
| YARP | 人形机器人 | iCub 平台使用；分布式计算；与 ROS2 bridge |
| LCM (Lightweight Communications and Marshalling) | 轻量通信 | MIT 研发；UDP 多播；用于 DRC 机器人挑战赛；比 ROS 轻量但生态小 |
| zenoh | 下一代通信 | Eclipse 项目；支持 ROS2 桥接；DDS 替代方案；边缘-云统一通信 |

