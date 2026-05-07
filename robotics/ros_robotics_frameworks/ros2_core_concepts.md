# ROS2 核心概念：Node / Topic / Service / Action

ROS2 的通信模型是理解一切 ROS2 应用的基础。四者互不替代，各有明确的适用场景。

---

## 1. Node（节点）

Node 是 ROS2 计算图的基本单元，每个 Node 对应一个进程内的逻辑模块。

```cpp
// 最小 ROS2 Node
#include <rclcpp/rclcpp.hpp>

class MinimalNode : public rclcpp::Node {
public:
  MinimalNode() : Node("minimal_node") {
    RCLCPP_INFO(this->get_logger(), "Node started");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalNode>());
  rclcpp::shutdown();
  return 0;
}
```

### Executor 模型

Executor 负责调度 Node 的回调函数。理解其差异是写出"正确"和"高效"ROS2 代码的关键。

| Executor | 行为 | 适用场景 |
|----------|------|---------|
| `SingleThreadedExecutor` | 所有回调串行执行 | 逻辑简单、需避免竞态 |
| `MultiThreadedExecutor` | 多线程并行执行回调 | 高吞吐、多话题并发 |
| `StaticSingleThreadedExecutor` | 串行，减少动态内存分配 | 硬实时、嵌入式 |

```cpp
// MultiThreadedExecutor 示例
rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

// 通过 Callback Group 控制回调归属
auto group1 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
auto group2 = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

// 同一 group 内回调互斥，不同 group 间可并行
```

### Node Composition（零拷贝进程内通信）

```cpp
// 容器进程加载多个 Component Node，避免 IPC 开销
// 方式1：手动组合
auto node1 = std::make_shared<CameraDriver>(options);
auto node2 = std::make_shared<ImageProcessor>(options);
executor.add_node(node1);
executor.add_node(node2);

// 方式2：Component Manager 动态加载
// ros2 run rclcpp_components component_container
// ros2 component load /ComponentManager my_pkg::MyComponent
```

---

## 2. Topic（话题 — 发布/订阅）

最核心的通信模式：单向、多对多、异步、解耦。

```cpp
// 发布者
auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
std_msgs::msg::String msg;
msg.data = "hello";
pub->publish(msg);

// 订阅者
auto sub = node->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    [](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(rclcpp::get_logger("sub"), "Received: %s", msg->data.c_str());
    });
```

**适用场景:**
- 传感器数据流 (`sensor_msgs/Image`, `sensor_msgs/Imu`, `sensor_msgs/LaserScan`)
- 状态广播 (`nav_msgs/Odometry`, `sensor_msgs/JointState`)
- 心跳/健康检查 (`diagnostic_msgs/DiagnosticStatus`)

**关键点:**
- 发布者和订阅者完全解耦，互不知道对方存在
- 一个 Topic 可以有多个 Publisher 和 Subscriber
- QoS 决定了消息的送达保证和行为（详见 [QoS.md](../motor_control_sensor_fusion/QoS.md)）

---

## 3. Service（服务 — 请求/响应）

同步 RPC 模式：一问一答，有反馈有结果。

```cpp
// 服务端
auto srv = node->create_service<std_srvs::srv::SetBool>(
    "set_enabled",
    [](const std_srvs::srv::SetBool::Request::SharedPtr req,
       std_srvs::srv::SetBool::Response::SharedPtr res) {
      res->success = true;
      res->message = "Enabled set to " + std::to_string(req->data);
    });

// 客户端（同步调用）
auto client = node->create_client<std_srvs::srv::SetBool>("set_enabled");
if (client->wait_for_service(std::chrono::seconds(1))) {
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = true;
  auto future = client->async_send_request(req);
  // 等待结果（阻塞）
  if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
    auto result = future.get();
    RCLCPP_INFO(logger, "Result: %s", result->message.c_str());
  }
}
```

**适用场景:**
- 参数查询/设置（查询 IK 解、获取当前配置）
- 触发式操作（标定、拍照、重置里程计）
- 计算服务（正逆向运动学、点云配准）

**Topic vs Service 选型:**
| 维度 | Topic | Service |
|------|-------|---------|
| 通信方向 | 单向数据流 | 双向请求-响应 |
| 时间特性 | 持续不断 | 一次性触发 |
| 调用频率 | 高频 (10–1000 Hz) | 低频 (按需) |
| 需要结果 | 不需要 | 需要 |

---

## 4. Action（动作 — 带反馈的长时间任务）

Action 是 Service 的增强版：异步、有状态机、支持取消、支持进度反馈。

```cpp
// Action 服务端
#include <rclcpp_action/rclcpp_action.hpp>

class FibonacciServer : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  FibonacciServer() : Node("fibonacci_server") {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
        this, "fibonacci",
        // 1. 处理目标请求（接受/拒绝）
        [](const rclcpp_action::GoalUUID &uuid,
           std::shared_ptr<const Fibonacci::Goal> goal) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // 2. 处理取消请求
        [](std::shared_ptr<GoalHandleFibonacci> handle) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        // 3. 执行目标
        [this](std::shared_ptr<GoalHandleFibonacci> handle) {
          execute(handle);
        });
  }

private:
  void execute(std::shared_ptr<GoalHandleFibonacci> handle) {
    auto goal = handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();
    std::vector<int> sequence;

    for (int i = 0; i < goal->order; i++) {
      // 检查取消请求
      if (handle->is_canceling()) {
        handle->canceled(result);
        return;
      }
      // 计算并发送反馈
      sequence.push_back(/* ... */);
      feedback->partial_sequence = sequence;
      handle->publish_feedback(feedback);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    handle->succeed(result);
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};
```

```cpp
// Action 客户端
auto client = rclcpp_action::create_client<Fibonacci>(node, "fibonacci");

auto goal_msg = Fibonacci::Goal();
goal_msg.order = 10;

auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
send_goal_options.feedback_callback =
    [](auto, const std::shared_ptr<const Fibonacci::Feedback> feedback) {
      RCLCPP_INFO(logger, "Progress: %zu", feedback->partial_sequence.size());
    };
send_goal_options.result_callback =
    [](const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &r) {
      RCLCPP_INFO(logger, "Done, code: %d", static_cast<int>(r.code));
    };

client->async_send_goal(goal_msg, send_goal_options);
```

**适用场景:**
- 导航任务 (`navigate_to_pose`)
- 机械臂操作 (pick-and-place, grasp)
- 长时计算 (建图、路径规划)

**Action 状态机:**
```
          ┌──CANCELING──→CANCELED
          │
ACCEPTED──→EXECUTING──→SUCCEEDED
              │
              └──→ABORTED
```

---

## 四者对比速查

| | Topic | Service | Action |
|---|------|---------|--------|
| 语义 | 发布/订阅 | 请求/响应 | 目标/反馈/结果 |
| 通信方向 | 单向 | 双向 | 双向+持续反馈 |
| 同步性 | 异步 | 同步（也有 async） | 异步 |
| 状态机 | 无 | 无 | 有 |
| 可取消 | 否 | 否 | 是 |
| 典型频率 | 1–1000 Hz | 按需 | 任务级别 |

---

## 面试常见追问

1. **"Node Composition 的优势是什么？"** — 零拷贝进程内通信，减少序列化开销和内存占用；降低上下文切换；适合传感器处理管道（Camera → Detection → Tracking）
2. **"什么时候用 Action 而不是 Service？"** — 任务耗时超过 100ms、需要反馈进度、需要支持取消；Service 适合短时 RPC
3. **"MultiThreadedExecutor 的风险？"** — 数据竞争和死锁；用 MutuallyExclusive Callback Group 保护共享数据；需要注意 `shared_ptr` 引用计数在多线程下的安全
4. **"ROS2 与 ROS1 的最大架构区别？"** — ROS2 无 Master 节点，依赖 DDS 的分布式发现机制；Nodelet 升级为 Component；引入 QoS 策略；支持多平台（Linux/Windows/macOS/RTOS）
