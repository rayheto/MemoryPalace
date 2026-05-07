#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// ================ 发布者：TRANSIENT_LOCAL ================
class TransientPub : public rclcpp::Node {
public:
  TransientPub() : Node("transient_pub") {
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    pub_ = this->create_publisher<std_msgs::msg::String>("cached_topic", qos);

    auto msg = std_msgs::msg::String();
    msg.data = "This was published BEFORE you subscribed!";
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published ONE message, then idle");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// ================ 订阅者：看能不能收到旧消息 ================
class TransientSub : public rclcpp::Node {
public:
  TransientSub() : Node("transient_sub") {
    auto qos = rclcpp::QoS(1).reliable().transient_local(); // 关键在transient_local()
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "cached_topic", qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(),
              "Got msg: '%s'", msg->data.c_str());
        });
    RCLCPP_INFO(this->get_logger(), "Subscriber ready, waiting...");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

// ================ main：根据参数决定跑pub还是sub ================
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
        "Usage: ros2 run qos_demo transient_local_demo [pub|sub]");
    return 1;
  }

  std::string mode(argv[1]);
  if (mode == "pub") {
    auto node = std::make_shared<TransientPub>();
    rclcpp::spin(node);
  } else if (mode == "sub") {
    auto node = std::make_shared<TransientSub>();
    rclcpp::spin(node);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown mode: %s", mode.c_str());
    return 1;
  }

  rclcpp::shutdown();
}
