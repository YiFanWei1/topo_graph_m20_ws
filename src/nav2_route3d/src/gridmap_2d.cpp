#include "nav2_route3d/gridmap_2d.hpp"

// 本文件实现骨架 shortcut 可见性检测使用的轻量二维占用栅格：
// 1. 输入是 graph_builder_3d.cpp 已完成高度过滤的全局三维点；
// 2. 构造函数忽略 z，把点投影到 XY 栅格并将对应单元标为占用；
// 3. occupiedOnSegment() 使用 Bresenham 遍历两个节点中心之间的栅格线；
// 4. 线段经过任意占用单元，就认为候选 shortcut 被障碍遮挡。
//
// 该类没有代价渐变、未知区域、地面拟合或机器人足迹模型；每个障碍点只占一个栅格，
// padding_cells 仅扩展地图边界，并不会对障碍物本身进行膨胀。

#include <algorithm>
#include <cmath>
// numeric_limits 用于初始化包围盒的正无穷/负无穷近似值。
#include <limits>

namespace nav2_route3d
{

// 由全局三维障碍点建立二维二值占用栅格。
OccupancyGrid2D::OccupancyGrid2D(
  const std::vector<Point3D> & points,
  const double resolution,
  const int padding_cells)
: resolution_(resolution)
{
  // 没有输入点时仍保留一个未占用单元，使成员保持可用状态；empty() 会返回 true。
  if (points.empty()) {
    occupied_.assign(1, 0U);
    return;
  }

  // 先计算所有点在 XY 平面上的轴对齐包围盒。
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & point : points) {
    // point[2]（z）在这里被有意忽略，因为调用方已经按导航高度带过滤过点云。
    min_x = std::min(min_x, point[0]);
    min_y = std::min(min_y, point[1]);
    max_x = std::max(max_x, point[0]);
    max_y = std::max(max_y, point[1]);
  }

  // 原点设在最小 XY 外侧 padding_cells 个栅格处，给包围盒左侧和下侧留边。
  origin_x_ = min_x - static_cast<double>(padding_cells) * resolution_;
  origin_y_ = min_y - static_cast<double>(padding_cells) * resolution_;
  // floor(range/resolution)+1 覆盖包含最大值的那个单元，再加 padding_cells 给右侧/上侧留边。
  // max(1, ...) 保证即使所有点重合，栅格宽高也至少为 1。
  width_ = std::max(1, static_cast<int>(std::floor((max_x - origin_x_) / resolution_)) + padding_cells + 1);
  height_ = std::max(1, static_cast<int>(std::floor((max_y - origin_y_) / resolution_)) + padding_cells + 1);
  // 使用一维 row-major 数组保存栅格：index = row * width + col；0=空闲，1=占用。
  occupied_.assign(static_cast<size_t>(width_ * height_), 0U);

  for (const auto & point : points) {
    // 将全局 XY 坐标量化为栅格行列；y 对应 row，x 对应 col。
    const auto [row, col] = worldToCell(point[0], point[1]);
    // 正常情况下由该批点构造出的包围盒会包含所有点；这里仍做边界检查以防数值边界误差。
    if (row >= 0 && row < height_ && col >= 0 && col < width_) {
      // 多个点落入同一单元时重复写 1，不累计点数，也不产生更高代价。
      occupied_[static_cast<size_t>(row * width_ + col)] = 1U;
    }
  }
}

// 检查世界坐标中的线段 (x0,y0)->(x1,y1) 是否经过至少一个占用栅格。
bool OccupancyGrid2D::occupiedOnSegment(
  const double x0,
  const double y0,
  const double x1,
  const double y1) const
{
  // 先把两个端点转换为整数栅格坐标。
  const auto [r0, c0] = worldToCell(x0, y0);
  const auto [r1, c1] = worldToCell(x1, y1);
  // Bresenham 返回的序列包含起点单元、终点单元以及二者之间的离散中心线单元。
  for (const auto & [row, col] : bresenham(r0, c0, r1, c1)) {
    // 栅格范围外的线段部分被跳过，相当于本类不把地图外区域自动视为障碍。
    if (row >= 0 && row < height_ && col >= 0 && col < width_ &&
      occupied_[static_cast<size_t>(row * width_ + col)] != 0U)
    {
      // 找到第一个占用单元即可提前结束，无需遍历剩余线段。
      return true;
    }
  }
  return false;
}

// 判断当前栅格是否完全没有占用单元，而不是判断 occupied_ 容器是否为空。
bool OccupancyGrid2D::empty() const
{
  // 只要找到任意非零值，none_of 就返回 false。
  return std::none_of(occupied_.begin(), occupied_.end(), [](const auto v) {return v != 0U;});
}

// 将全局连续坐标转换为离散栅格行列号。
std::pair<int, int> OccupancyGrid2D::worldToCell(const double x, const double y) const
{
  // 先减栅格原点，再除以分辨率；floor 表示每个单元采用左闭右开的坐标区间。
  // 位于 origin 左侧/下侧的坐标会得到负列号/行号，随后由调用方决定如何处理。
  const int col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return {row, col};
}

// 用整数 Bresenham 算法离散化从 (r0,c0) 到 (r1,c1) 的栅格线段。
// 算法适用于任意方向和斜率，返回值包含首尾两个单元。
std::vector<std::pair<int, int>> bresenham(const int r0, const int c0, const int r1, const int c1)
{
  std::vector<std::pair<int, int>> cells;
  // dr/dc 是行、列方向需要跨越的绝对单元数量。
  const int dr = std::abs(r1 - r0);
  const int dc = std::abs(c1 - c0);
  // step_r/step_c 决定每次修正时向正方向还是负方向移动。
  const int step_r = r0 < r1 ? 1 : -1;
  const int step_c = c0 < c1 ? 1 : -1;
  // 误差项同时统一处理“更水平”和“更垂直”的线段，不需要按斜率拆分函数。
  int err = dc - dr;
  int row = r0;
  int col = c0;
  while (true) {
    // 先记录当前单元，因此起点一定包含在结果中。
    cells.emplace_back(row, col);
    if (row == r1 && col == c1) {
      // 到达终点后停止，终点也已经在本轮开头加入结果。
      break;
    }
    // 使用两倍误差避免引入 0.5 和浮点运算。
    const int err2 = 2 * err;
    if (err2 > -dr) {
      // 误差允许沿列方向前进时，更新误差并把 col 向终点移动一步。
      err -= dr;
      col += step_c;
    }
    if (err2 < dc) {
      // 误差允许沿行方向前进时，更新误差并把 row 向终点移动一步。
      // 两个 if 可能在同一次循环都执行，对应一次对角移动。
      err += dc;
      row += step_r;
    }
  }
  return cells;
}

}  // namespace nav2_route3d
