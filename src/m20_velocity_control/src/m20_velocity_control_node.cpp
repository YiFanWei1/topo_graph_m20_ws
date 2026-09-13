#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "drdds/msg/motion_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "m20_velocity_control/velocity_gate.hpp"

namespace m20_velocity_control
{
namespace
{

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace

class M20VelocityControlNode : public rclcpp::Node
{
public:
  M20VelocityControlNode()
  : Node("m20_velocity_control")
  {
    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 50.0);

    VelocityGateConfig config;
    config.maximum_vx_mps = declare_parameter<double>("limits.maximum_vx_mps", 0.8);
    config.maximum_vy_mps = declare_parameter<double>("limits.maximum_vy_mps", 0.3);
    config.maximum_wz_radps = declare_parameter<double>("limits.maximum_wz_radps", 0.5);
    config.command_timeout_s = declare_parameter<double>("command_timeout_s", 0.3);
    config.state_timeout_s = declare_parameter<double>("state_timeout_s", 1.0);
    config.allow_lateral_motion = declare_parameter<bool>("allow_lateral_motion", false);
    if (control_rate_hz <= 0.0) {
      throw std::invalid_argument("control_rate_hz must be positive");
    }
    gate_ = std::make_unique<VelocityGate>(config);

    input_topic_ = declare_parameter<std::string>(
      "topics.input_command", "/m20/manual/cmd_vel");
    const auto output_topic = declare_parameter<std::string>(
      "topics.output_command", "/m20/manual/cmd_vel_safe");
    const auto selected_topic = declare_parameter<std::string>(
      "topics.selected_command", "/m20/manual/selected_command");
    const auto ready_topic = declare_parameter<std::string>(
      "topics.m20_ready", "/m20/control/ready");
    const auto motion_state_topic = declare_parameter<std::string>(
      "topics.m20_motion_state", "/m20/state/motion_state");
    const auto emergency_stop_topic = declare_parameter<std::string>(
      "topics.emergency_stop", "/m20/manual/emergency_stop");
    const auto status_topic = declare_parameter<std::string>(
      "topics.status", "/m20/manual/status");

    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(20).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        input_command_ = *message;
        command_stamp_ = now();
      });
    ready_subscription_ = create_subscription<std_msgs::msg::Bool>(
      ready_topic, latchedQos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        ready_ = message->data;
      });
    motion_state_subscription_ = create_subscription<drdds::msg::MotionState>(
      motion_state_topic, rclcpp::QoS(10).reliable(),
      [this](drdds::msg::MotionState::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        motion_state_ = message->data.state;
        motion_state_stamp_ = now();
      });
    emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic, latchedQos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        emergency_stop_ = message->data;
      });

    output_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic, rclcpp::QoS(20).reliable());
    selected_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      selected_topic, rclcpp::QoS(20).reliable());
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, latchedQos());

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&M20VelocityControlNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "M20 manual velocity control ready: input=%s enable_motion=%s lateral=%s",
      input_topic_.c_str(), enable_motion_ ? "true" : "false",
      config.allow_lateral_motion ? "true" : "false");
  }

private:
  void tick()
  {
    VelocityGateInput gate_input;
    {
      std::scoped_lock lock(mutex_);
      const auto stamp = now();
      gate_input.enable_motion = enable_motion_;
      gate_input.emergency_stop = emergency_stop_;
      gate_input.ready = ready_;
      gate_input.state_received = motion_state_stamp_.nanoseconds() > 0;
      gate_input.motion_state = motion_state_;
      gate_input.command_received = command_stamp_.nanoseconds() > 0;
      if (gate_input.state_received) {
        gate_input.state_age_s = (stamp - motion_state_stamp_).seconds();
      }
      if (gate_input.command_received) {
        gate_input.command_age_s = (stamp - command_stamp_).seconds();
      }
      gate_input.command = {
        input_command_.linear.x, input_command_.linear.y, input_command_.angular.z};
    }

    const auto result = gate_->evaluate(gate_input);
    geometry_msgs::msg::Twist output;
    output.linear.x = result.command.vx;
    output.linear.y = result.command.vy;
    output.angular.z = result.command.wz;
    selected_publisher_->publish(output);
    output_publisher_->publish(output);

    std_msgs::msg::String status;
    status.data = nlohmann::json{
      {"enable_motion", gate_input.enable_motion},
      {"emergency_stop", gate_input.emergency_stop},
      {"m20_ready", gate_input.ready},
      {"motion_state", gate_input.motion_state},
      {"motion_state_age_s", gate_input.state_age_s},
      {"command_topic", input_topic_},
      {"command_age_s", gate_input.command_age_s},
      {"blocked", result.blocked},
      {"block_reason", result.block_reason},
      {"clamped", result.clamped},
      {"selected", {
          {"vx", result.command.vx},
          {"vy", result.command.vy},
          {"wz", result.command.wz}}}}.dump();
    status_publisher_->publish(status);
  }

  std::mutex mutex_;
  bool enable_motion_{false};
  bool ready_{false};
  bool emergency_stop_{false};
  std::int32_t motion_state_{-1};
  std::string input_topic_;
  geometry_msgs::msg::Twist input_command_;
  rclcpp::Time command_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time motion_state_stamp_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<VelocityGate> gate_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_subscription_;
  rclcpp::Subscription<drdds::msg::MotionState>::SharedPtr motion_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr selected_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace m20_velocity_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<m20_velocity_control::M20VelocityControlNode>());
  rclcpp::shutdown();
  return 0;
}
