#include "nav2_route3d/route_tracker_3d.hpp"

#include <cmath>
#include <utility>

namespace nav2_route3d
{

RouteTracker3D::RouteTracker3D(
  const Graph3D & graph,
  Route3D route,
  const double radius_to_achieve_node)
: graph_(graph), route_(std::move(route)), radius_(radius_to_achieve_node)
{
}

RouteProgress3D RouteTracker3D::update(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (route_.node_ids.empty()) {
    return {};
  }
  while (current_index_ < route_.node_ids.size()) {
    const auto node_id = route_.node_ids[current_index_];
    const auto & node = graph_.node(node_id);
    const auto dx = node.pose.tx - robot_pose.pose.position.x;
    const auto dy = node.pose.ty - robot_pose.pose.position.y;
    const auto dz = node.pose.tz - robot_pose.pose.position.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > radius_) {
      break;
    }
    achieved_.push_back(node_id);
    ++current_index_;
  }
  const bool complete = current_index_ >= route_.node_ids.size();
  const NodeId current_node_id = complete ? route_.node_ids.back() : route_.node_ids[current_index_];
  return {current_index_, current_node_id, achieved_, complete};
}

}  // namespace nav2_route3d
