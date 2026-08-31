#include "nav2_route3d/goal_intent_extractor_3d.hpp"

#include <stdexcept>

namespace nav2_route3d
{

GoalIntentExtractor3D::GoalIntentExtractor3D(const Graph3D & graph)
: graph_(graph)
{
}

GoalIntent3D GoalIntentExtractor3D::fromIds(const NodeId start_id, const NodeId goal_id) const
{
  if (!graph_.hasNode(start_id) || !graph_.hasNode(goal_id)) {
    throw std::runtime_error("Start or goal node id is not in route graph");
  }
  return {start_id, goal_id};
}

GoalIntent3D GoalIntentExtractor3D::fromPoses(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  return {
    graph_.nearestNode(start.pose.position.x, start.pose.position.y, start.pose.position.z),
    graph_.nearestNode(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)
  };
}

}  // namespace nav2_route3d
