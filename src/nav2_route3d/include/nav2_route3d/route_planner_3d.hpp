#pragma once

#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "nav2_route3d/edge_scorer_3d.hpp"

namespace nav2_route3d
{

class RoutePlanner3D
{
public:
  RoutePlanner3D(const Graph3D & graph, const EdgeScorer3D & scorer);
  Route3D plan(
    NodeId start_id,
    NodeId goal_id,
    const std::unordered_set<EdgeId> & blocked_edges = {},
    double max_planning_time_s = 0.0) const;

private:
  double heuristic(NodeId node_id, NodeId goal_id) const;
  Route3D reconstruct(
    NodeId start_id,
    NodeId goal_id,
    const std::unordered_map<NodeId, std::pair<NodeId, Edge3D>> & came_from,
    double cost) const;

  const Graph3D & graph_;
  const EdgeScorer3D & scorer_;
};

}  // namespace nav2_route3d
