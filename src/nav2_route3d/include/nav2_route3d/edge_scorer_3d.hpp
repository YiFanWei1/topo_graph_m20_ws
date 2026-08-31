#pragma once

#include "nav2_route3d/graph_3d.hpp"

namespace nav2_route3d
{

class EdgeScorer3D
{
public:
  explicit EdgeScorer3D(const Graph3D & graph);
  ScoreResult score(const Edge3D & edge) const;

private:
  double slopePenalty(const Edge3D & edge) const;
  double scenePenalty(const Edge3D & edge) const;

  const Graph3D & graph_;
};

}  // namespace nav2_route3d
