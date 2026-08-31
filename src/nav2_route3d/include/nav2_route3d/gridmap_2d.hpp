#pragma once

#include <cstdint>
#include <vector>

#include "nav2_route3d/kdtree_3d.hpp"

namespace nav2_route3d
{

class OccupancyGrid2D
{
public:
  OccupancyGrid2D() = default;
  OccupancyGrid2D(const std::vector<Point3D> & points, double resolution, int padding_cells = 2);

  bool occupiedOnSegment(double x0, double y0, double x1, double y1) const;
  bool empty() const;

private:
  std::pair<int, int> worldToCell(double x, double y) const;

  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.2};
  int width_{1};
  int height_{1};
  std::vector<uint8_t> occupied_;
};

std::vector<std::pair<int, int>> bresenham(int r0, int c0, int r1, int c1);

}  // namespace nav2_route3d
