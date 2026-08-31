#pragma once

#include <rclcpp/time.hpp>
#include <nav2_msgs/msg/route.hpp>
#include <nav_msgs/msg/path.hpp>

#include "nav2_route3d/graph_3d.hpp"

namespace nav2_route3d
{

std::vector<geometry_msgs::msg::PoseStamped> densifyRoutePoses(
  const Graph3D & graph,
  const Route3D & route,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double density_m);

nav_msgs::msg::Path routeToPath(
  const Graph3D & graph,
  const Route3D & route,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double density_m);

nav2_msgs::msg::Route routeToMsg(
  const Graph3D & graph,
  const Route3D & route,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

}  // namespace nav2_route3d
