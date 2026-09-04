#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace route3d_dijkstra_planner
{

class DemoRequestPublisher : public rclcpp::Node
{
public:
  DemoRequestPublisher()
  : Node("route3d_dijkstra_demo_requests")
  {
    const auto topic = declare_parameter<std::string>(
      "request_topic", "/route3d_dijkstra/plan_request");
    const auto values = declare_parameter<std::vector<std::int64_t>>(
      "request_pairs", {1, 20, 5, 16, 4, 17, 3, 13, 13, 3});
    const auto interval_seconds = declare_parameter<double>("interval_seconds", 4.0);
    repeat_ = declare_parameter<bool>("repeat", true);
    if (values.size() < 2U || values.size() % 2U != 0U) {
      throw std::runtime_error("request_pairs must contain start/goal pairs");
    }
    if (interval_seconds <= 0.0) {
      throw std::runtime_error("interval_seconds must be positive");
    }
    for (std::size_t index = 0; index < values.size(); index += 2U) {
      pairs_.emplace_back(
        static_cast<std::int32_t>(values[index]),
        static_cast<std::int32_t>(values[index + 1U]));
    }
    publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      topic, rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(interval_seconds),
      std::bind(&DemoRequestPublisher::publishNext, this));
    RCLCPP_INFO(
      get_logger(),
      "automatic C++ Dijkstra demo ready: topic=%s pairs=%zu interval=%.1fs repeat=%s",
      topic.c_str(), pairs_.size(), interval_seconds, repeat_ ? "true" : "false");
  }

private:
  void publishNext()
  {
    if (index_ >= pairs_.size()) {
      if (!repeat_) {
        timer_->cancel();
        return;
      }
      index_ = 0U;
    }
    const auto pair = pairs_[index_++];
    std_msgs::msg::Int32MultiArray message;
    message.data = {pair.first, pair.second};
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "published demo request: %d->%d", pair.first, pair.second);
  }

  bool repeat_{true};
  std::size_t index_{0U};
  std::vector<std::pair<std::int32_t, std::int32_t>> pairs_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace route3d_dijkstra_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<route3d_dijkstra_planner::DemoRequestPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route3d_dijkstra_demo_requests"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
