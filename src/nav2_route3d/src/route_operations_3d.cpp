#include "nav2_route3d/route_operations_3d.hpp"

namespace nav2_route3d
{

OperationResult RouteOperation3D::onNode(const Node3D &) const {return {};}
OperationResult RouteOperation3D::onEdgeEnter(const Edge3D &) const {return {};}
OperationResult RouteOperation3D::onEdgeExit(const Edge3D &) const {return {};}

std::string StairLikeTransitionOperation::name() const {return "stair_like_transition";}

OperationResult StairLikeTransitionOperation::onEdgeEnter(const Edge3D & edge) const
{
  if (getString(edge.metadata, "edge_type", "") != "stair_like_transition") {
    return {};
  }
  OperationResult result;
  result.triggered = true;
  result.scenario_profile = getString(edge.metadata, "enter_profile", "stair_like_transition");
  result.speed_limit_mps = getNumber(edge.metadata, "speed_limit_mps", 0.25);
  result.message = "Entering stair-like transition";
  return result;
}

OperationResult StairLikeTransitionOperation::onEdgeExit(const Edge3D & edge) const
{
  if (getString(edge.metadata, "edge_type", "") != "stair_like_transition") {
    return {};
  }
  OperationResult result;
  result.triggered = true;
  result.scenario_profile = getString(edge.metadata, "exit_profile", "factory_indoor");
  result.message = "Leaving stair-like transition";
  return result;
}

std::string IndoorOutdoorTransitionOperation::name() const {return "indoor_outdoor_transition";}

OperationResult IndoorOutdoorTransitionOperation::onEdgeEnter(const Edge3D & edge) const
{
  if (getString(edge.metadata, "edge_type", "") != "indoor_outdoor_transition") {
    return {};
  }
  OperationResult result;
  result.triggered = true;
  result.scenario_profile = getString(edge.metadata, "target_profile", "campus_road");
  result.speed_limit_mps = getNumber(edge.metadata, "speed_limit_mps", 0.4);
  result.message = "Indoor/outdoor route transition";
  return result;
}

std::string SteepSlopeOperation::name() const {return "steep_slope";}

OperationResult SteepSlopeOperation::onEdgeEnter(const Edge3D & edge) const
{
  if (getString(edge.metadata, "edge_type", "") != "steep_slope") {
    return {};
  }
  OperationResult result;
  result.triggered = true;
  result.scenario_profile = getString(edge.metadata, "target_profile", "stair_like_transition");
  result.speed_limit_mps = getNumber(edge.metadata, "speed_limit_mps", 0.2);
  result.message = "Entering steep-slope cautious mode";
  return result;
}

std::vector<std::shared_ptr<RouteOperation3D>> defaultOperations()
{
  return {
    std::make_shared<StairLikeTransitionOperation>(),
    std::make_shared<IndoorOutdoorTransitionOperation>(),
    std::make_shared<SteepSlopeOperation>()
  };
}

}  // namespace nav2_route3d
