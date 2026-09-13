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

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace route3d_m20_adapter
{

class OdometryTfBroadcasterNode : public rclcpp::Node
{
public:
  OdometryTfBroadcasterNode()
  : Node("route3d_odometry_tf_broadcaster")
  {
    const auto odometry_topic = declare_parameter<std::string>(
      "odometry_topic", "/lio_odom_hf");
    parent_frame_override_ = declare_parameter<std::string>("parent_frame", "");
    child_frame_override_ = declare_parameter<std::string>("child_frame", "base_link");
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::QoS(100).best_effort(),
      std::bind(&OdometryTfBroadcasterNode::odometryCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Broadcasting odometry TF from %s to child frame %s",
      odometry_topic.c_str(), child_frame_override_.c_str());
  }

private:
  void odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const auto & orientation = message->pose.pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(quaternion_norm) ||
      quaternion_norm < 1.0e-12)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring invalid odometry pose for TF");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = message->header;
    if (!parent_frame_override_.empty()) {
      transform.header.frame_id = parent_frame_override_;
    }
    transform.child_frame_id = child_frame_override_.empty() ?
      message->child_frame_id : child_frame_override_;
    if (transform.header.frame_id.empty() || transform.child_frame_id.empty() ||
      transform.header.frame_id == transform.child_frame_id)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring odometry with invalid TF frame IDs");
      return;
    }

    transform.transform.translation.x = position.x;
    transform.transform.translation.y = position.y;
    transform.transform.translation.z = position.z;
    transform.transform.rotation.x = orientation.x / quaternion_norm;
    transform.transform.rotation.y = orientation.y / quaternion_norm;
    transform.transform.rotation.z = orientation.z / quaternion_norm;
    transform.transform.rotation.w = orientation.w / quaternion_norm;
    broadcaster_->sendTransform(transform);
  }

  std::string parent_frame_override_;
  std::string child_frame_override_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace route3d_m20_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<route3d_m20_adapter::OdometryTfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
