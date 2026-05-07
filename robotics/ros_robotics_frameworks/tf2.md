# TF2 — 坐标变换系统

TF2 是 ROS 生态中管理多个坐标系之间动态变换的库。任何涉及"传感器→本体→世界"坐标转换的机器人应用都依赖它。

---

## 1. 核心概念

### TF 树

一个有向无环图 (DAG)，其中：
- **节点** = 坐标系 (frame)
- **边** = 从一个坐标系到另一个坐标系的变换 (transform)

```
map → odom → base_link → camera_link → camera_optical_frame
                      │
                      └── laser_frame
```

整个树必须是连通的。任意两帧之间的变换，TF2 沿着树上路径自动计算。

### 变换数据类型

```cpp
#include <geometry_msgs/msg/transform_stamped.hpp>
// geometry_msgs::msg::TransformStamped
//   - header: stamp, frame_id (父), child_frame_id (子)
//   - transform: Vector3 translation, Quaternion rotation
```

TF2 内部使用 `tf2::Transform`：
```cpp
#include <tf2/LinearMath/Transform.h>
tf2::Transform t;
t.setOrigin(tf2::Vector3(x, y, z));
t.setRotation(tf2::Quaternion(qx, qy, qz, qw));
```

---

## 2. 广播 Transform

### 动态广播（周期变化）

```cpp
#include <tf2_ros/transform_broadcaster.h>

class OdometryPublisher : public rclcpp::Node {
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void timer_callback() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    // 位姿：base_link 在 odom 坐标系下的位置
    t.transform.translation.x = odom_x_;
    t.transform.translation.y = odom_y_;
    t.transform.translation.z = 0.0;

    // 姿态：从 IMU 或编码器获取
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);  // Roll, Pitch, Yaw
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }
};
```

### 静态广播（固定变换）

适用于不随时间变化的固定变换，如传感器安装位置：

```cpp
#include <tf2_ros/static_transform_broadcaster.h>

auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

geometry_msgs::msg::TransformStamped t;
t.header.stamp = node->now();
t.header.frame_id = "base_link";
t.child_frame_id  = "camera_link";
t.transform.translation.x = 0.1;   // 相机在本体前方 10cm
t.transform.translation.z = 0.5;   // 高度 50cm
t.transform.rotation.x = -0.5;     // 相机向下倾斜（符合 REP-103: x前 y左 z上）
t.transform.rotation.w = 0.5;
broadcaster->sendTransform(t);
```

**静态广播特性:** 只在订阅者连接时发送一次，但 TF2 Buffer 会持久缓存，后加入的节点也能查询到历史静态变换。

### 命令行工具

```bash
# 发布静态变换（调试用）
ros2 run tf2_ros static_transform_publisher \
  --x 0.1 --y 0 --z 0.5 --roll 0 --pitch -1.57 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link
```

---

## 3. 监听和查询 Transform

### 基本查询

```cpp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Localizer : public rclcpp::Node {
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
  Localizer() : Node("localizer") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void lookup() {
    try {
      // 查询 "camera_link" 在 "map" 坐标系下的位姿
      // 参数: target_frame, source_frame, time, timeout
      auto transform = tf_buffer_->lookupTransform(
          "map", "camera_link", tf2::TimePointZero);
      
      auto &t = transform.transform.translation;
      auto &r = transform.transform.rotation;
      RCLCPP_INFO(logger, "Position: [%.2f, %.2f, %.2f]", t.x, t.y, t.z);
      RCLCPP_INFO(logger, "Orientation: [%.2f, %.2f, %.2f, %.2f]", r.x, r.y, r.z, r.w);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(logger, "TF lookup failed: %s", ex.what());
    }
  }
};
```

### 时间戳匹配

```cpp
// 1. 查询最新可用变换（不指定时间）
auto t = buffer->lookupTransform("map", "base_link", tf2::TimePointZero);

// 2. 查询特定时间点的变换（TF2 自动插值）
auto t = buffer->lookupTransform("map", "base_link", msg->header.stamp);

// 3. 带超时的查询
auto t = buffer->lookupTransform(
    "map", "base_link", msg->header.stamp,
    std::chrono::milliseconds(100));
```

**`lookupTransform(target, source, time)` 语义:**
- 查询 `source` 坐标系在 `target` 坐标系下的位姿，在时间 `time` 的取值
- `tf2::TimePointZero` = 取最新可用值
- 如果缓存中没有 `time` 时刻前后的数据 → 无法插值 → 抛异常

### 常用变换操作

```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 对点做坐标变换
geometry_msgs::msg::PointStamped point_in_camera;   // 相机坐标系下的点
geometry_msgs::msg::PointStamped point_in_map;      // 想得到世界坐标系下的点

// 方式1：直接调用 transform
tf2::doTransform(point_in_camera, point_in_map,
                 buffer->lookupTransform("map", "camera_link", tf2::TimePointZero));

// 方式2：手动构建齐次变换矩阵
tf2::Transform tf = tf2::Transform(
    tf2::Quaternion(r.x, r.y, r.z, r.w),
    tf2::Vector3(t.x, t.y, t.z));
tf2::Vector3 p(point.x, point.y, point.z);
tf2::Vector3 p_transformed = tf * p;
```

### 四元数与欧拉角转换

```cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// 欧拉角 → 四元数
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);  // ROS 使用 RPY 顺序（绕固定轴 X-Y-Z）
// 等价于 R_z(yaw) * R_y(pitch) * R_x(roll)

// 四元数 → 欧拉角
double roll, pitch, yaw;
tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

// geometry_msgs → tf2
tf2::Quaternion q2;
tf2::fromMsg(msg.pose.orientation, q2);
```

---

## 4. TF2 Buffer 缓存策略

```
Buffer 内部维护每个变换对的时间序列缓存
默认缓存时长: 10 秒（可通过构造函数参数调整）

时间轴:  [---10s---][---now---]
          可查询范围   当前

lookupTransform("map", "base_link", tf2::TimePointZero)
  → 取缓存中最新的 "map→base_link" 变换

lookupTransform("map", "base_link", target_time)
  → 在 target_time 附近取最近的两帧，线性插值
```

---

## 5. 实战：多传感器坐标融合管线

```cpp
class ArmPoseEstimator : public rclcpp::Node {
public:
  ArmPoseEstimator() : Node("arm_pose_estimator") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 订阅相机检测到的机械臂目标点
    target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "detected_target", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          on_target_detected(msg);
        });
  }

private:
  void on_target_detected(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    try {
      // 相机坐标系下的目标点 → 机械臂基座坐标系
      geometry_msgs::msg::PointStamped target_in_base;
      tf_buffer_->transform(*msg, target_in_base, "arm_base_link");

      // 发送给机械臂控制器
      target_pub_->publish(target_in_base);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
};
```

---

## 6. 坐标系统约定 (REP-103 / REP-105)

ROS 标准坐标系：

```
x → 前方
y → 左方
z → 上方

右手定则:
  拇指(x) = 前
  食指(y) = 左  
  中指(z) = 上
```

| Frame | 含义 | REP |
|-------|------|-----|
| `map` | 世界固定坐标系，不漂移 | REP-105 |
| `odom` | 里程计坐标系，可漂移，保证平滑 | REP-105 |
| `base_link` | 机器人本体参考点 | REP-105 |
| `base_footprint` | `base_link` 在地面投影 | — |

典型变换链：`map → odom → base_footprint → base_link → sensor_frames`

---

## 面试常见追问

1. **"TF2 如何处理变换树中的循环？"** — 树是 DAG，每个 child frame 只能有一个 parent frame。系统会拒绝创建回环边。
2. **"为什么 lookupTransform 会失败？"** — 常见原因：树不连通（缺少中间帧）、时间戳超出缓存、目标时间过于久远无插值点、坐标系名称拼写错误。
3. **"静态变换为什么用 Static 前缀？"** — StaticTransform 只需广播一次并被持久缓存，减少带宽；普通 Transform 需要周期性发布。
4. **"怎么调试 TF 树？"** — `ros2 run tf2_tools view_frames` 生成 PDF 可视化图；`ros2 run tf2_ros tf2_echo <source> <target>` 打印实时变换值；RViz2 中查看 TF Display。
