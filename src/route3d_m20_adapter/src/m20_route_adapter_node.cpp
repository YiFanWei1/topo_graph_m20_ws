// Copyright 2026 wei
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "drdds/msg/motion_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace route3d_m20_adapter
{
namespace
{
using namespace std::chrono_literals;

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

double finiteClamp(const double value, const double limit)
{
  if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0) {
    return 0.0;
  }
  return std::clamp(value, -limit, limit);
}

void enforceMinimumEffectiveSpeed(
  geometry_msgs::msg::Twist & command, const double minimum_linear, const double minimum_yaw)
{
  const double planar = std::hypot(command.linear.x, command.linear.y);
  if (planar > 1.0e-9 && planar < minimum_linear) {
    const double scale = minimum_linear / planar;
    command.linear.x *= scale;
    command.linear.y *= scale;
  }
  if (std::abs(command.angular.z) > 1.0e-9 && std::abs(command.angular.z) < minimum_yaw) {
    command.angular.z = std::copysign(minimum_yaw, command.angular.z);
  }
}

}  // namespace

class M20RouteAdapter : public rclcpp::Node
{
public:
  M20RouteAdapter()
  : Node("route3d_m20_adapter")
  {
    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    release_output_when_idle_ = declare_parameter<bool>("release_output_when_idle", true);
    const double rate = declare_parameter<double>("control_rate_hz", 50.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.30);
    state_timeout_s_ = declare_parameter<double>("state_timeout_s", 1.0);
    normal_vx_ = declare_parameter<double>("limits.normal.maximum_vx_mps", 0.80);
    normal_vy_ = declare_parameter<double>("limits.normal.maximum_vy_mps", 0.0);
    normal_wz_ = declare_parameter<double>("limits.normal.maximum_wz_radps", 0.50);
    alignment_vx_ = declare_parameter<double>("limits.alignment.maximum_vx_mps", 0.20);
    alignment_vy_ = declare_parameter<double>("limits.alignment.maximum_vy_mps", 0.30);
    alignment_wz_ = declare_parameter<double>("limits.alignment.maximum_wz_radps", 0.70);
    minimum_linear_speed_ = declare_parameter<double>("limits.minimum_linear_speed_mps", 0.20);
    minimum_yaw_speed_ = declare_parameter<double>("limits.minimum_yaw_speed_radps", 0.32);

    if (rate <= 0.0 || command_timeout_s_ <= 0.0 || state_timeout_s_ <= 0.0 ||
      minimum_linear_speed_ < 0.0 || minimum_yaw_speed_ < 0.0 ||
      minimum_linear_speed_ > std::min(normal_vx_, alignment_vx_) ||
      minimum_yaw_speed_ > std::min(normal_wz_, alignment_wz_))
    {
      throw std::invalid_argument("invalid M20 adapter rate or timeout");
    }

    const auto pid_topic = declare_parameter<std::string>("topics.pid_command", "/cmd_vel_pid");
    const auto efficient_topic = declare_parameter<std::string>(
      "topics.efficient_command", "/cmd_vel_effi");
    const auto source_topic = declare_parameter<std::string>(
      "topics.controller_source", "/route3d_controller/active_source");
    const auto alignment_topic = declare_parameter<std::string>(
      "topics.alignment_active", "/route3d_pid_controller/alignment_active");
    const auto ready_topic = declare_parameter<std::string>(
      "topics.m20_ready", "/m20/control/ready");
    const auto motion_state_topic = declare_parameter<std::string>(
      "topics.m20_motion_state", "/m20/state/motion_state");
    const auto output_topic = declare_parameter<std::string>(
      "topics.output_command", "/cmd_vel_smoothed");
    const auto selected_topic = declare_parameter<std::string>(
      "topics.selected_command", "/route3d_m20_adapter/selected_command");
    const auto status_topic = declare_parameter<std::string>(
      "topics.status", "/route3d_m20_adapter/status");

    pid_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      pid_topic, rclcpp::QoS(20).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        pid_command_ = *message;
        pid_stamp_ = now();
      });
    efficient_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      efficient_topic, rclcpp::QoS(20).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        efficient_command_ = *message;
        efficient_stamp_ = now();
      });
    source_subscription_ = create_subscription<std_msgs::msg::String>(
      source_topic, latchedQos(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        active_source_ = message->data;
      });
    alignment_subscription_ = create_subscription<std_msgs::msg::Bool>(
      alignment_topic, latchedQos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        alignment_active_ = message->data;
      });
    ready_subscription_ = create_subscription<std_msgs::msg::Bool>(
      ready_topic, latchedQos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        m20_ready_ = message->data;
        ready_stamp_ = now();
      });
    motion_state_subscription_ = create_subscription<drdds::msg::MotionState>(
      motion_state_topic, rclcpp::QoS(10).reliable(),
      [this](drdds::msg::MotionState::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        motion_state_ = message->data.state;
        motion_state_stamp_ = now();
      });

    output_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic, rclcpp::QoS(20).reliable());
    selected_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      selected_topic, rclcpp::QoS(20).reliable());
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, latchedQos());

    const auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&M20RouteAdapter::tick, this));
    RCLCPP_INFO(get_logger(), "M20 Route3D adapter started fail-closed; enable_motion=%s",
      enable_motion_ ? "true" : "false");
  }

private:
  void tick()
  {
    geometry_msgs::msg::Twist selected;
    std::string source;
    std::string block_reason;
    bool alignment = false;
    bool ready = false;
    bool output_released = false;
    int motion_state = -1;
    double command_age = -1.0;
    double state_age = -1.0;
    {
      std::scoped_lock lock(mutex_);
      const auto stamp = now();
      source = active_source_;
      alignment = alignment_active_ && source == "pid";
      ready = m20_ready_;
      motion_state = motion_state_;
      if (motion_state_stamp_.nanoseconds() > 0) {
        state_age = (stamp - motion_state_stamp_).seconds();
      }

      const geometry_msgs::msg::Twist * input = nullptr;
      rclcpp::Time input_stamp{0, 0, get_clock()->get_clock_type()};
      if (source == "pid") {
        input = &pid_command_;
        input_stamp = pid_stamp_;
      } else if (source == "efficient_3d_local_planner") {
        input = &efficient_command_;
        input_stamp = efficient_stamp_;
      }
      if (input_stamp.nanoseconds() > 0) {
        command_age = (stamp - input_stamp).seconds();
      }

      if (!enable_motion_) {
        block_reason = "motion_disabled";
        output_released = true;
      } else if (release_output_when_idle_ && (source.empty() || source == "none")) {
        block_reason = "remote_control_released";
        output_released = true;
      } else if (!ready || ready_stamp_.nanoseconds() == 0) {
        block_reason = "m20_not_ready";
      } else if (motion_state != 17 || state_age < 0.0 || state_age > state_timeout_s_) {
        block_reason = "m20_state_not_fresh_rl";
      } else if (input == nullptr) {
        block_reason = source == "none" ? "no_controller_selected" : "unknown_controller";
      } else if (command_age < 0.0 || command_age > command_timeout_s_) {
        block_reason = "controller_command_stale";
      } else {
        const double vx_limit = alignment ? alignment_vx_ : normal_vx_;
        const double vy_limit = alignment ? alignment_vy_ : normal_vy_;
        const double wz_limit = alignment ? alignment_wz_ : normal_wz_;
        selected.linear.x = finiteClamp(input->linear.x, vx_limit);
        selected.linear.y = finiteClamp(input->linear.y, vy_limit);
        selected.angular.z = finiteClamp(input->angular.z, wz_limit);
        enforceMinimumEffectiveSpeed(selected, minimum_linear_speed_, minimum_yaw_speed_);
      }
    }

    selected_publisher_->publish(selected);
    // /cmd_vel_smoothed 是底盘 bridge 的输入。Route3D 空闲时如果仍以 50 Hz
    // 连续发布零速度，会覆盖实体遥控器；只有导航任务接管控制器后才发布。
    // 导航过程中未 ready、通信超时和安全停障仍会持续发布零速度。
    if (!output_released) {
      output_publisher_->publish(selected);
    }
    std_msgs::msg::String status;
    status.data = nlohmann::json{
      {"enable_motion", enable_motion_}, {"m20_ready", ready},
      {"motion_state", motion_state}, {"motion_state_age_s", state_age},
      {"active_controller", source}, {"alignment_active", alignment},
      {"command_age_s", command_age}, {"blocked", !block_reason.empty()},
      {"block_reason", block_reason},
      {"output_released", output_released},
      {"selected", {{"vx", selected.linear.x}, {"vy", selected.linear.y},
          {"wz", selected.angular.z}}}}.dump();
    status_publisher_->publish(status);
  }

  std::mutex mutex_;
  bool enable_motion_{false};
  bool release_output_when_idle_{true};
  bool m20_ready_{false};
  bool alignment_active_{false};
  int motion_state_{-1};
  double command_timeout_s_{0.30};
  double state_timeout_s_{1.0};
  double normal_vx_{0.80};
  double minimum_linear_speed_{0.20};
  double minimum_yaw_speed_{0.32};
  double normal_vy_{0.0};
  double normal_wz_{0.50};
  double alignment_vx_{0.20};
  double alignment_vy_{0.30};
  double alignment_wz_{0.70};
  std::string active_source_{"none"};
  geometry_msgs::msg::Twist pid_command_;
  geometry_msgs::msg::Twist efficient_command_;
  rclcpp::Time pid_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time efficient_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ready_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time motion_state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pid_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr efficient_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alignment_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_subscription_;
  rclcpp::Subscription<drdds::msg::MotionState>::SharedPtr motion_state_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr selected_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace route3d_m20_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<route3d_m20_adapter::M20RouteAdapter>());
  rclcpp::shutdown();
  return 0;
}
