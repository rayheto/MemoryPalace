# ROS2 Launch 系统

Launch 系统负责多节点的编排启动、参数注入和生命周期管理。ROS2 支持 XML 和 Python 两种 Launch 文件写法。

---

## 1. XML Launch

简洁直观，适合大多数常规场景。

```xml
<launch>
  <!-- 声明可配置参数，可通过 ros2 launch 的命令行覆盖 -->
  <arg name="use_sim_time" default="true"/>
  <arg name="world_file" default="$(find-pkg-share my_pkg)/worlds/empty.world"/>

  <!-- 参数服务器：从 YAML 文件加载 -->
  <node pkg="my_robot" exec="controller_node" name="controller" output="screen">
    <param from="$(find-pkg-share my_pkg)/config/controller.yaml"/>
  </node>

  <!-- 话题重映射：隔离命名空间 -->
  <node pkg="camera_driver" exec="camera_node" name="camera_left">
    <remap from="image_raw" to="/camera_left/image_raw"/>
  </node>

  <node pkg="camera_driver" exec="camera_node" name="camera_right">
    <remap from="image_raw" to="/camera_right/image_raw"/>
  </node>

  <!-- 嵌套 Launch 文件：模块化 -->
  <include file="$(find-pkg-share my_pkg)/launch/sensors.xml"/>
</launch>
```

### 命令行参数覆盖

```bash
# launch 参数覆盖
ros2 launch my_pkg main.launch.xml use_sim_time:=false

# node 参数覆盖（在运行时）
ros2 launch my_pkg main.launch.xml controller.param_name:=42
```

---

## 2. Python Launch

图灵完备，支持条件判断、循环、动态参数计算。

```python
# main.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_fps = LaunchConfiguration('camera_fps', default='30')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('camera_fps', default_value=camera_fps,
                              description='Camera frame rate'),

        # 条件启动：仅在 use_sim_time 为 true 时启动 Gazebo
        Node(
            package='gazebo_ros',
            executable='gazebo',
            condition=IfCondition(use_sim_time),
            arguments=['-s', 'libgazebo_ros_factory.so'],
        ),

        # 从 YAML 加载参数
        Node(
            package='my_pkg',
            executable='camera_node',
            name='camera_left',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('my_pkg'), 'config', 'camera.yaml'
                ]),
                {'frame_rate': camera_fps}
            ],
            remappings=[('image_raw', '/camera_left/image_raw')],
        ),

        # 延迟启动（等待 Gazebo 就绪后 5 秒）
        TimerAction(
            period=5.0,
            actions=[
                Node(package='my_pkg', executable='controller_node',
                     name='controller'),
            ],
        ),
    ])
```

### Python Launch 关键 Action 速查

| Action | 作用 |
|--------|------|
| `DeclareLaunchArgument` | 声明可被命令行覆盖的参数 |
| `Node` | 启动一个 ROS2 节点 |
| `IncludeLaunchDescription` | 嵌套其他 Launch 文件 |
| `TimerAction` | 延迟执行 |
| `GroupAction` | 批量设置命名空间/条件/参数作用域 |
| `ExecuteProcess` | 执行任意 shell 命令 |
| `LogInfo` | 输出日志 |
| `PushRosNamespace / SetRemap` | 动态修改命名空间和重映射 |

---

## 3. GroupAction — 批量配置

```python
from launch.actions import GroupAction, PushRosNamespace

GroupAction([
    # 所有子节点自动加上 /robot_a 命名空间
    PushRosNamespace('robot_a'),
    Node(package='my_pkg', executable='controller', name='controller'),
    Node(package='my_pkg', executable='sensor', name='sensor'),
])
# 等价于每个节点手动设置 namespace='robot_a'
```

---

## 4. YAML 参数配置

```yaml
# config/controller.yaml
controller:
  ros__parameters:
    kp: 0.5
    ki: 0.01
    kd: 0.1
    max_velocity: 1.0
    joint_names: ["joint_1", "joint_2", "joint_3"]

    # 嵌套参数
    limits:
      joint_1: {min: -3.14, max: 3.14}
      joint_2: {min: -1.57, max: 1.57}
      joint_3: {min: -3.14, max: 3.14}
```

```cpp
// C++ 端读取参数
node->declare_parameter("kp", 0.5);
node->declare_parameter("limits.joint_1.min", -3.14);  // 嵌套参数用 "." 分隔

double kp = node->get_parameter("kp").as_double();
double j1_min = node->get_parameter("limits.joint_1.min").as_double();
```

---

## 5. 参数传递链

```
命令行 ros2 launch ... arg:=value
        │
        ▼
LaunchConfiguration (字符串形式)
        │
        ▼
Node parameters (YAML + 命令行)
        │
        ▼
Node::declare_parameter / get_parameter
```

---

## 6. 实战：机械臂感知管线 Launch 文件

```python
# arm_perception.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value=use_rviz),

        # TF 静态变换发布
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0', '0.5',
                       '-0.5', '0.5', '-0.5', '0.5',  # 四元数
                       'base_link', 'camera_link'],
        ),

        # 相机驱动节点
        Node(
            package='arm_perception',
            executable='stereo_camera_node',
            name='stereo_camera',
            parameters=[PathJoinSubstitution([
                FindPackageShare('arm_perception'), 'config', 'camera.yaml'
            ])],
        ),

        # AHRS 串口驱动节点
        Node(
            package='arm_perception',
            executable='ahrs_driver_node',
            name='ahrs_driver',
            parameters=[PathJoinSubstitution([
                FindPackageShare('arm_perception'), 'config', 'ahrs.yaml'
            ])],
        ),

        # 姿态融合节点（多传感器→机械臂末端位姿）
        Node(
            package='arm_perception',
            executable='pose_fusion_node',
            name='pose_fusion',
            parameters=[{'publish_rate': 100.0}],
        ),

        # RViz2（可选，延迟启动确保其他节点就绪）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('arm_perception'), 'rviz', 'arm_view.rviz'
            ])],
        ),
    ])
```

启动命令：

```bash
# 完整启动
ros2 launch arm_perception arm_perception.launch.py

# 不启动 RViz2
ros2 launch arm_perception arm_perception.launch.py use_rviz:=false
```

---

## 面试常见追问

1. **"XML vs Python Launch 如何选择？"** — 简单场景用 XML（可读性好）；需要条件逻辑、循环、动态参数计算时用 Python。项目前期 XML 快速原型，后期复杂部署切 Python。
2. **"LaunchConfiguration 和 Python 变量有什么区别？"** — LaunchConfiguration 是一个延迟求值的占位符，在 Launch 系统实际运行时才解析为字符串。不能直接用于 Python 逻辑判断（如 `if`），需使用 `IfCondition` / `UnlessCondition` 或 PythonExpression。
3. **"如何实现节点间的启动顺序控制？"** — TimerAction 延迟启动依赖方；使用 lifecycle nodes（ROS2 Managed Node）管理状态转移（configure → activate → deactivate → cleanup）；`OnProcessStart` 事件驱动。
4. **"Launch 文件中的参数和 YAML 文件优先级？"** — 命令行覆盖（最高）> Launch 文件内联参数 > YAML 文件参数 > 代码默认值（最低）。
