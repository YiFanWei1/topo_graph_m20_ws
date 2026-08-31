#include "nav2_route3d/edge_scorer_3d.hpp"

#include <cmath>
#include <limits>

namespace nav2_route3d
{

EdgeScorer3D::EdgeScorer3D(const Graph3D & graph)
: graph_(graph)
{
}

ScoreResult EdgeScorer3D::score(const Edge3D & edge) const
{
  if (getBool(edge.metadata, "closed", false)) {
    return {false, std::numeric_limits<double>::infinity(), "edge closed"};
  }
  if (getString(edge.metadata, "terrain_class", "") == "forbidden") {
    return {false, std::numeric_limits<double>::infinity(), "forbidden terrain"};
  }
  const double base = edge.cost;
  const double penalty = getNumber(edge.metadata, "penalty", 0.0);
  const double risk = getNumber(edge.metadata, "risk", 0.0);
  return {true, base + penalty + risk + slopePenalty(edge) + scenePenalty(edge), ""};
}

double EdgeScorer3D::slopePenalty(const Edge3D & edge) const
{
  const auto & start = graph_.node(edge.start_id);
  const auto & end = graph_.node(edge.end_id);
  const auto dx = start.pose.tx - end.pose.tx;
  const auto dy = start.pose.ty - end.pose.ty;
  const auto xy = std::max(0.001, std::sqrt(dx * dx + dy * dy));
  const auto slope = std::abs(start.pose.tz - end.pose.tz) / xy;
  const auto max_slope = getNumber(edge.metadata, "max_slope", 10.0);
  if (slope > max_slope) {
    return 1.0e6;
  }
  return slope * getNumber(edge.metadata, "slope_cost_scale", 1.0);
}

double EdgeScorer3D::scenePenalty(const Edge3D & edge) const
{
  const auto edge_type = getString(edge.metadata, "edge_type", "normal");
  if (edge_type == "stair_like_transition") {
    return getNumber(edge.metadata, "stair_like_penalty", 2.0);
  }
  if (edge_type == "indoor_outdoor_transition") {
    return getNumber(edge.metadata, "transition_penalty", 1.0);
  }
  if (edge_type == "steep_slope") {
    return getNumber(edge.metadata, "steep_slope_penalty", 3.0);
  }
  return 0.0;
}

}  // namespace nav2_route3d
