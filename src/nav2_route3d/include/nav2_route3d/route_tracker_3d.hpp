#pragma once

#include "nav2_route3d/graph_3d.hpp"

namespace nav2_route3d
{

struct RouteProgress3D
{
  size_t current_index{0};
  NodeId current_node_id{0};
  std::vector<NodeId> achieved_node_ids;
  bool complete{false};
};

class RouteTracker3D
{
public:
  RouteTracker3D(const Graph3D & graph, Route3D route, double radius_to_achieve_node);
  RouteProgress3D update(const geometry_msgs::msg::PoseStamped & robot_pose);

private:
  const Graph3D & graph_;
  Route3D route_;
  double radius_{1.0};
  size_t current_index_{0};
  std::vector<NodeId> achieved_;
};

}  // namespace nav2_route3d
