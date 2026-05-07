#include <chrono>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class FastPublisher : public rclcpp::Node {
public:
  FastPublisher() : Node("fast_pub"), count_(0) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
    timer_ = this->create_wall_timer(20ms, [this]() {
      auto msg = std_msgs::msg::Int32();
      msg.data = count_++;
      pub_->publish(msg);
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

class SlowSubscriber : public rclcpp::Node {
public:
  SlowSubscriber(const std::string &name, uint depth)
      : Node(name), received_(0), last_(-1), lost_(0) {
    auto qos = rclcpp::QoS(depth).reliable();
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "counter", qos,
        [this](std_msgs::msg::Int32::SharedPtr msg) {
          int skipped = 0;
          if (last_ >= 0) {
            skipped = msg->data - last_ - 1;
            lost_ += skipped;
          }
          received_++;
          RCLCPP_INFO(this->get_logger(),
              "depth=%zu | got=#%d | skipped=%d | total_lost=%zu",
              sub_->get_actual_qos().depth(),
              msg->data, skipped, lost_);
          last_ = msg->data;
          std::this_thread::sleep_for(200ms);
        });
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  int received_, last_;
  size_t lost_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);

  auto pub  = std::make_shared<FastPublisher>();
  auto sub1 = std::make_shared<SlowSubscriber>("shallow_sub", 1);
  auto sub2 = std::make_shared<SlowSubscriber>("deep_sub", 100);

  exec->add_node(pub);
  exec->add_node(sub1);
  exec->add_node(sub2);
  exec->spin();
  rclcpp::shutdown();
}
