#pragma once

#include <memory>

#include "nav2_route3d/graph_3d.hpp"

namespace nav2_route3d
{

class RouteOperation3D
{
public:
  virtual ~RouteOperation3D() = default;
  virtual std::string name() const = 0;
  virtual OperationResult onNode(const Node3D & node) const;
  virtual OperationResult onEdgeEnter(const Edge3D & edge) const;
  virtual OperationResult onEdgeExit(const Edge3D & edge) const;
};

class StairLikeTransitionOperation : public RouteOperation3D
{
public:
  std::string name() const override;
  OperationResult onEdgeEnter(const Edge3D & edge) const override;
  OperationResult onEdgeExit(const Edge3D & edge) const override;
};

class IndoorOutdoorTransitionOperation : public RouteOperation3D
{
public:
  std::string name() const override;
  OperationResult onEdgeEnter(const Edge3D & edge) const override;
};

class SteepSlopeOperation : public RouteOperation3D
{
public:
  std::string name() const override;
  OperationResult onEdgeEnter(const Edge3D & edge) const override;
};

std::vector<std::shared_ptr<RouteOperation3D>> defaultOperations();

}  // namespace nav2_route3d
