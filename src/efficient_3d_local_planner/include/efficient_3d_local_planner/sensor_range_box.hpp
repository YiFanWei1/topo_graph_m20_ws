#pragma once

#include <Eigen/Core>

namespace efficient_3d_local_planner
{

// 雷达坐标系中的两层轴对齐长方体量程过滤器。外框之外和内框之内的点都被拒绝；
// 内外框之间的点继续进入世界坐标变换、滚动地图裁剪、raycast 和占据更新。
struct SensorRangeBox
{
  Eigen::Vector3d outer_min{-4.0, -3.0, -3.0};
  Eigen::Vector3d outer_max{4.0, 3.0, 2.0};
  Eigen::Vector3d inner_half{0.30, 0.30, 0.30};

  bool valid() const noexcept
  {
    if (!outer_min.allFinite() || !outer_max.allFinite() || !inner_half.allFinite() ||
      (outer_min.array() >= outer_max.array()).any() ||
      (inner_half.array() < 0.0).any())
    {
      return false;
    }

    // 内框以雷达原点为中心，必须完整包含在外框内；同时要求雷达原点本身位于外框。
    return (outer_min.array() <= -inner_half.array()).all() &&
           (outer_max.array() >= inner_half.array()).all();
  }

  bool accepts(const Eigen::Vector3d & point_in_lidar) const noexcept
  {
    if (!point_in_lidar.allFinite()) {
      return false;
    }
    const bool inside_outer =
      (point_in_lidar.array() >= outer_min.array()).all() &&
      (point_in_lidar.array() <= outer_max.array()).all();
    const bool inside_inner =
      (point_in_lidar.array().abs() <= inner_half.array()).all();
    return inside_outer && !inside_inner;
  }
};

}  // namespace efficient_3d_local_planner
