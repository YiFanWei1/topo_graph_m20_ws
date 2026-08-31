#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace efficient_3d_local_planner
{

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};
  bool operator==(const VoxelKey & other) const noexcept
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept
  {
    std::uint64_t value = static_cast<std::uint32_t>(key.x);
    value = value * 0x9e3779b185ebca87ULL + static_cast<std::uint32_t>(key.y);
    value = value * 0xc2b2ae3d27d4eb4fULL + static_cast<std::uint32_t>(key.z);
    value ^= value >> 29U;
    return static_cast<std::size_t>(value);
  }
};

inline VoxelKey pointToKey(const Eigen::Vector3d & point, const double resolution)
{
  return VoxelKey{
    static_cast<int>(std::floor(point.x() / resolution)),
    static_cast<int>(std::floor(point.y() / resolution)),
    static_cast<int>(std::floor(point.z() / resolution))};
}

inline Eigen::Vector3d keyToCenter(const VoxelKey & key, const double resolution)
{
  return resolution * Eigen::Vector3d(
    static_cast<double>(key.x) + 0.5,
    static_cast<double>(key.y) + 0.5,
    static_cast<double>(key.z) + 0.5);
}

}  // namespace efficient_3d_local_planner
