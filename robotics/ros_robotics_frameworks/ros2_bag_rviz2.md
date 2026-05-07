# ros2 bag 与 RViz2

---

# Part A: ros2 bag — 数据录制与回放

ros2 bag 是 ROS2 的标准数据录制工具，用于离线记录和回放话题数据，属于算法调试和回归验证的核心工具。

## 1. 基本用法

```bash
# 录制所有话题
ros2 bag record -a -o my_recording

# 录制指定话题
ros2 bag record -o arm_data /arm/joint_states /camera/image_raw /ahrs/imu

# 正则过滤话题
ros2 bag record --regex "/camera.*/image.*"

# 压缩录制（节省存储，需要 rosbag2 和 zstd 插件）
ros2 bag record -o compressed_data --compression-mode file --compression-format zstd /camera/image_raw

# 限制录制时长或大小
ros2 bag record --max-bag-duration 60          # 60 秒
ros2 bag record --max-bag-size 1073741824      # 1 GB
```

## 2. 回放

```bash
# 正常回放
ros2 bag play my_recording

# 半速回放（便于观察细节）
ros2 bag play my_recording --rate 0.5

# 循环回放
ros2 bag play my_recording --loop

# 仅回放部分话题
ros2 bag play my_recording --topics /camera/image_raw /ahrs/imu

# 从指定时间点开始回放
ros2 bag play my_recording --start-offset 10.0   # 从第 10 秒开始
```

## 3. 信息查看

```bash
# 查看 bag 文件元信息
ros2 bag info my_recording

# 输出示例:
# Files:             my_recording_0.db3
# Bag size:          1.2 GB
# Storage id:        sqlite3
# Duration:          65.43s
# Start:             Jan 01 2024 00:00:00.000
# End:               Jan 01 2024 00:01:05.430
# Messages:          6543
# Topics:
#   /camera/image_raw    : sensor_msgs/msg/Image   (654 msgs, 30.0 Hz)
#   /arm/joint_states    : sensor_msgs/msg/JointState (6543 msgs, 100.0 Hz)
```

## 4. 存储格式

| 版本 | 默认格式 | 说明 |
|------|---------|------|
| ROS2 Humble 及之前 | SQLite3 (.db3) | 单文件存储，兼容性好 |
| ROS2 Iron 起 | MCAP | 支持分块压缩、索引加速、跨语言 |

```bash
# 转换格式
ros2 bag convert my_recording --output-format mcap
```

## 5. 工程最佳实践

- **一次录制，多次使用:** 录制真实传感器数据，离线反复迭代算法，避免反复部署硬件和重复实验
- **录制同步数据:** 将传感器（相机/IMU）、执行器（关节状态）、控制指令同时录制，便于全链路分析
- **命名规范:** `YYYYMMDD_experiment_description`，如 `20240101_arm_pick_place_test1`
- **压缩:** 批量存储时启用压缩以节省磁盘；注意 zstd 压缩/解压会带来额外 CPU 开销

---

# Part B: RViz2 — 三维可视化与调试

RViz2 是 ROS2 的标准三维可视化工具，用于实时显示机器人状态、传感器数据和调试信息。

## 1. 核心 Display 类型

| Display | 消息类型 | 用途 |
|---------|---------|------|
| **TF** | (内部使用 TF2) | 显示坐标系轴，调试 TF 树 |
| **Image** | `sensor_msgs/msg/Image` | 显示相机图像 |
| **PointCloud2** | `sensor_msgs/msg/PointCloud2` | 显示点云 |
| **LaserScan** | `sensor_msgs/msg/LaserScan` | 显示激光雷达扫描线 |
| **Marker** | `visualization_msgs/msg/Marker` | 绘制基本几何体（箭头/球/立方体/文字） |
| **MarkerArray** | `visualization_msgs/msg/MarkerArray` | 批量 Marker（检测框、路径点） |
| **RobotModel** | URDF / `robot_description` | 显示机器人三维模型 |
| **Odometry** | `nav_msgs/msg/Odometry` | 显示里程计轨迹（箭头） |

## 2. Fixed Frame（固定帧）配置

**这是最常见的调试问题来源。** Fixed Frame 是 RViz2 中所有可视化的参考坐标系。

```
错误: 显示空白
原因: Fixed Frame 设为 "map" 但 TF 树中不存在 map → camera_link 路径
修复: 检查 ros2 run tf2_tools view_frames，确保树连通
```

```bash
# 查看当前所有可用的坐标系
ros2 run tf2_tools view_frames
# 会在当前目录生成 frames.pdf
```

## 3. Marker 调试技巧

Marker 是程序化调试的最高效手段，比 print/log 直观得多。

```cpp
#include <visualization_msgs/msg/marker.hpp>

class PoseVisualizer : public rclcpp::Node {
public:
  PoseVisualizer() : Node("pose_visualizer") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 10);
  }

  void visualize_pose(double x, double y, double z, double qx, double qy, double qz, double qw) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "base_link";
    arrow.header.stamp = this->now();
    arrow.ns = "pose_arrow";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    // 箭头尾端
    arrow.points.resize(2);
    arrow.points[0].x = 0;  arrow.points[0].y = 0;  arrow.points[0].z = 0;
    arrow.points[1].x = x;  arrow.points[1].y = y;  arrow.points[1].z = z;

    // 尺寸与颜色
    arrow.scale.x = 0.01;  // shaft diameter
    arrow.scale.y = 0.02;  // head diameter
    arrow.color.r = 1.0; arrow.color.g = 0.0;
    arrow.color.b = 0.0; arrow.color.a = 1.0;

    marker_pub_->publish(arrow);
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};
```

```cpp
// MarkerArray 批量发布（多个检测框/路径点）
visualization_msgs::msg::MarkerArray bboxes;
for (size_t i = 0; i < detections.size(); i++) {
  visualization_msgs::msg::Marker cube;
  cube.header.frame_id = "camera_link";
  cube.ns = "detection";
  cube.id = i;
  cube.type = visualization_msgs::msg::Marker::CUBE;
  cube.pose.position.x = detections[i].center_x;
  cube.pose.position.y = detections[i].center_y;
  cube.pose.position.z = detections[i].center_z;
  cube.scale.x = detections[i].width;
  cube.scale.y = detections[i].height;
  cube.scale.z = detections[i].depth;
  cube.color.a = 0.5;  // 半透明
  cube.color.r = 0.0; cube.color.g = 1.0; cube.color.b = 0.0;
  bboxes.markers.push_back(cube);
}
marker_array_pub_->publish(bboxes);
```

## 4. RViz2 配置文件

保存/加载 RViz2 配置，避免每次手动设置 Display：

```bash
# 启动时加载配置
rviz2 -d path/to/config.rviz
```

RViz2 配置文件记录：已添加的 Display 类型、每个 Display 的话题配置、Fixed Frame、视角、透明度等。纳入版本控制，随项目分发。

## 5. 实战调试流程

```
1. 启动感知管线
   ros2 launch arm_perception arm_perception.launch.py
2. 启动 RViz2（已配置 Fixed Frame = arm_base_link）
   rviz2 -d config/arm_debug.rviz
3. 添加 TF Display → 确认坐标系树连通
4. 添加 Image Display → 确认相机图像正常
5. 添加 MarkerArray Display → 确认检测框/位姿箭头正确
6. 发现异常 → 录制 ros2 bag → 离线回放定位问题
```

---

## 面试常见追问

1. **"为什么 ros2 bag 对算法开发重要？"** — 一次录制可无限次回放，消除硬件依赖；可精确复现特定场景做 A/B 测试；便于团队协作（分享 bag 文件即分享数据）；结合 CI 可做自动化回归测试。
2. **"RViz2 中 Fixed Frame 为空怎么办？"** — TF 树中不存在该坐标系，改为 `base_link` 或已发布的 frame；或先启动发布该坐标系的节点。
3. **"Marker 和 PointCloud2 在调试上如何选择？"** — Marker 适合稀疏几何体（箭头/框/文字/轨迹）；PointCloud2 适合稠密点云。检测框用 MarkerArray，原始传感器数据用 PointCloud2。
4. **"ros2 bag 的存储后端 SQLite3 vs MCAP 怎么选？"** — MCAP 是未来趋势（分块压缩、索引、跨语言）；SQLite3 仍是 Humble 默认且稳定。新项目推荐 MCAP。
