#pragma once

#include "nav2_route3d/graph_3d.hpp"

namespace nav2_route3d
{

class GoalIntentExtractor3D
{
public:
  explicit GoalIntentExtractor3D(const Graph3D & graph);

  GoalIntent3D fromIds(NodeId start_id, NodeId goal_id) const;
  GoalIntent3D fromPoses(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

private:
  const Graph3D & graph_;
};

}  // namespace nav2_route3d
