#include "nav2_route3d/kdtree_3d.hpp"

// 本文件实现一个轻量级三维 KD-Tree，供离线骨架构图阶段进行空间索引：
// - radiusSearch()：找到给定三维半径内的所有 Anchor；
// - nearest()：找到离查询点最近的一个 Anchor。
//
// 树中保存的 index 始终是构造函数输入 points 的原始下标，而不是树节点的遍历序号。
// 在 graph_builder_3d.cpp 中，points 只包含稀疏 Anchor，因此查询结果还需要通过
// anchor_node_indices_ 再映射到 poses_ 和 Graph3D 的真实节点编号。

#include <algorithm>
#include <cmath>
// numeric_limits<double>::max() 用作最近邻搜索的初始“无穷大”距离。
#include <limits>
#include <stdexcept>
#include <utility>

namespace nav2_route3d
{

// 计算两点的三维欧氏距离平方：|a-b|^2 = dx^2 + dy^2 + dz^2。
// 搜索过程中只需比较距离大小，不需要开平方；使用平方距离可以减少大量 sqrt 计算。
double distanceSquared(const Point3D & a, const Point3D & b)
{
  const auto dx = a[0] - b[0];
  const auto dy = a[1] - b[1];
  const auto dz = a[2] - b[2];
  return dx * dx + dy * dy + dz * dz;
}

// 从输入点集建立一棵三维 KD-Tree。
KDTree3D::KDTree3D(const std::vector<Point3D> & points)
{
  // 将每个坐标和它在原输入数组中的下标绑定在一起。
  // 后续建树会重排坐标，index 则保证查询结果仍能指回调用方的数据。
  std::vector<IndexedPoint> indexed;
  indexed.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    indexed.emplace_back(points[i], i);
  }
  // 根节点深度为 0，因此首先沿 X 轴切分。
  // indexed 通过 move 交给递归函数，避免这里再复制整个数组。
  root_ = build(std::move(indexed), 0);
}

// 递归建立 KD-Tree：每层选择一个坐标轴，用该轴的中位点把点集分成左右两半。
std::unique_ptr<KDTree3D::Node> KDTree3D::build(std::vector<IndexedPoint> points, const int depth)
{
  // 空区间对应空子树，也是递归终止条件。
  if (points.empty()) {
    return nullptr;
  }
  // 切分轴按深度循环：0=X、1=Y、2=Z、3 又回到 X。
  const int axis = depth % 3;
  // 使用中位位置作为当前子树根；偶数个点时，左右数量最多相差 1。
  const auto mid = points.size() / 2U;
  // nth_element 只保证：
  // - mid 位置是按当前轴排序后本应位于该位置的元素；
  // - 左侧元素不大于它，右侧元素不小于它；
  // 它不会完整排序，因此平均复杂度比每层全排序更低。
  std::nth_element(
    points.begin(), points.begin() + static_cast<std::ptrdiff_t>(mid), points.end(),
    [axis](const auto & lhs, const auto & rhs) {
      return lhs.first[static_cast<size_t>(axis)] < rhs.first[static_cast<size_t>(axis)];
    });

  // 中位点成为当前树节点，并保存本层使用的切分轴。
  auto node = std::make_unique<Node>();
  node->point = points[mid].first;
  // 这里保存的是构造函数输入 points 的原始下标。
  node->index = points[mid].second;
  node->axis = axis;

  // 中位点之前的元素构成左子树，中位点之后的元素构成右子树；
  // 当前中位点本身不再放入任一子树，避免重复节点。
  std::vector<IndexedPoint> left(points.begin(), points.begin() + static_cast<std::ptrdiff_t>(mid));
  std::vector<IndexedPoint> right(
    points.begin() + static_cast<std::ptrdiff_t>(mid + 1U), points.end());
  // 子区间通过 move 传入下一层；depth+1 会让下一层更换切分轴。
  node->left = build(std::move(left), depth + 1);
  node->right = build(std::move(right), depth + 1);
  return node;
}

// 返回以 center 为球心、radius 为半径的闭球内所有点的原始下标。
std::vector<size_t> KDTree3D::radiusSearch(const Point3D & center, const double radius) const
{
  std::vector<size_t> result;
  // 将半径预先平方，递归中即可始终使用平方距离比较。
  // 边界采用 <=，所以恰好落在球面上的点也会被返回。
  radiusSearch(root_.get(), center, radius * radius, result);
  // 结果顺序来自树的递归访问顺序，不等于输入顺序，也没有按距离排序。
  return result;
}

// 返回距离 center 最近的点在原输入数组中的下标。
size_t KDTree3D::nearest(const Point3D & center) const
{
  // 空树没有合法下标，明确抛出异常，避免悄悄返回无意义的 0。
  if (!root_) {
    throw std::runtime_error("Cannot query empty KDTree3D");
  }
  // best_index 会在访问根节点时被更新；先把最优距离初始化为 double 最大值。
  size_t best_index = 0;
  double best_distance_sq = std::numeric_limits<double>::max();
  nearest(root_.get(), center, best_index, best_distance_sq);
  return best_index;
}

// 半径搜索的递归实现。radius_sq 在整个递归过程中保持不变。
void KDTree3D::radiusSearch(
  const Node * node,
  const Point3D & center,
  const double radius_sq,
  std::vector<size_t> & result) const
{
  // 到达空孩子，当前搜索分支结束。
  if (!node) {
    return;
  }
  // 当前节点位于查询球内部或边界上时，记录其原始输入下标。
  if (distanceSquared(center, node->point) <= radius_sq) {
    result.push_back(node->index);
  }
  // delta 是查询中心到当前切分平面的有符号距离：
  // 节点使用 axis 坐标定义平面，delta<=0 表示查询中心位于左侧。
  const auto axis = static_cast<size_t>(node->axis);
  const auto delta = center[axis] - node->point[axis];
  if (delta <= 0.0) {
    // 先搜索查询中心所在一侧，这一侧最有可能包含球内点。
    radiusSearch(node->left.get(), center, radius_sq, result);
    // 只有查询球与切分平面相交或相切时，另一侧才可能出现球内点。
    // 若 delta^2 > radius^2，整个右子树都可安全剪掉。
    if (delta * delta <= radius_sq) {
      radiusSearch(node->right.get(), center, radius_sq, result);
    }
  } else {
    // 查询中心在右侧时逻辑对称：先右后左。
    radiusSearch(node->right.get(), center, radius_sq, result);
    if (delta * delta <= radius_sq) {
      radiusSearch(node->left.get(), center, radius_sq, result);
    }
  }
}

// 最近邻搜索的递归实现。best_index 和 best_distance_sq 以引用传递，
// 所有递归分支共享并持续收紧当前已知的最优结果。
void KDTree3D::nearest(
  const Node * node,
  const Point3D & center,
  size_t & best_index,
  double & best_distance_sq) const
{
  // 空子树不可能改善当前最优结果。
  if (!node) {
    return;
  }
  // 用当前节点更新已知最近邻。
  const auto dist = distanceSquared(center, node->point);
  if (dist < best_distance_sq) {
    best_distance_sq = dist;
    best_index = node->index;
  }
  // 根据查询中心位于切分平面的哪一侧，决定优先搜索的近侧子树。
  const auto axis = static_cast<size_t>(node->axis);
  const auto delta = center[axis] - node->point[axis];
  const Node * first = delta < 0.0 ? node->left.get() : node->right.get();
  const Node * second = delta < 0.0 ? node->right.get() : node->left.get();
  // 先搜索近侧，通常能较早得到更小的 best_distance_sq，从而增强后面的剪枝效果。
  nearest(first, center, best_index, best_distance_sq);
  // 查询点到切分平面的最短距离为 |delta|。
  // 只有该距离仍小于当前最近距离时，远侧子树才可能包含更近的点。
  // 使用严格 <：若远侧最多只能找到等距离点，就保留先访问到的那个下标。
  if (delta * delta < best_distance_sq) {
    nearest(second, center, best_index, best_distance_sq);
  }
}

}  // namespace nav2_route3d
