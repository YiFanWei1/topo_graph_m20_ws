#pragma once

#include <array>
#include <memory>
#include <vector>

namespace nav2_route3d
{

using Point3D = std::array<double, 3>;

class KDTree3D
{
public:
  explicit KDTree3D(const std::vector<Point3D> & points);
  std::vector<size_t> radiusSearch(const Point3D & center, double radius) const;
  size_t nearest(const Point3D & center) const;

private:
  struct Node
  {
    Point3D point;
    size_t index{0};
    int axis{0};
    std::unique_ptr<Node> left;
    std::unique_ptr<Node> right;
  };

  using IndexedPoint = std::pair<Point3D, size_t>;

  std::unique_ptr<Node> build(std::vector<IndexedPoint> points, int depth);
  void radiusSearch(
    const Node * node,
    const Point3D & center,
    double radius_sq,
    std::vector<size_t> & result) const;
  void nearest(
    const Node * node,
    const Point3D & center,
    size_t & best_index,
    double & best_distance_sq) const;

  std::unique_ptr<Node> root_;
};

double distanceSquared(const Point3D & a, const Point3D & b);

}  // namespace nav2_route3d
