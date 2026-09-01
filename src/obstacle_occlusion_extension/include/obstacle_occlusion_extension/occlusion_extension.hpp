#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace obstacle_occlusion_extension
{

struct GridGeometry
{
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};
  double resolution{0.0};
  std::uint32_t size_x{0U};
  std::uint32_t size_y{0U};
  std::uint32_t size_z{0U};

  std::size_t cellCount() const noexcept
  {
    return static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y) *
           static_cast<std::size_t>(size_z);
  }

  bool valid() const noexcept
  {
    return std::isfinite(origin_x) && std::isfinite(origin_y) && std::isfinite(origin_z) &&
           std::isfinite(resolution) && resolution > 0.0 && size_x > 0U && size_y > 0U &&
           size_z > 0U && cellCount() <= std::numeric_limits<std::uint32_t>::max();
  }

  std::uint32_t index(
    const std::uint32_t x, const std::uint32_t y, const std::uint32_t z) const noexcept
  {
    return (z * size_y + y) * size_x + x;
  }
};

struct ExtensionConfig
{
  double distance{0.60};
  double minimum_obstacle_range{0.20};
  double maximum_obstacle_range{8.0};

  bool valid() const noexcept
  {
    return std::isfinite(distance) && distance >= 0.0 &&
           std::isfinite(minimum_obstacle_range) && minimum_obstacle_range >= 0.0 &&
           std::isfinite(maximum_obstacle_range) && maximum_obstacle_range >= 0.0 &&
           (maximum_obstacle_range == 0.0 || maximum_obstacle_range >= minimum_obstacle_range);
  }
};

inline std::vector<std::uint32_t> computeOcclusionExtension(
  const GridGeometry & grid, const std::vector<std::uint32_t> & hard_indices,
  const double observer_x, const double observer_y, const ExtensionConfig & config)
{
  std::vector<std::uint32_t> added;
  if (!grid.valid() || !config.valid() || config.distance <= 0.0 ||
    !std::isfinite(observer_x) || !std::isfinite(observer_y))
  {
    return added;
  }

  constexpr std::uint8_t kAdded = 1U;
  constexpr std::uint8_t kHard = 2U;
  const std::size_t cell_count = grid.cellCount();
  std::vector<std::uint8_t> state(cell_count, 0U);
  for (const std::uint32_t index : hard_indices) {
    if (index < cell_count) {
      state[index] = kHard;
    }
  }

  const int sample_count = static_cast<int>(std::ceil(config.distance / grid.resolution));
  added.reserve(std::min(
      cell_count, hard_indices.size() * static_cast<std::size_t>(sample_count)));

  for (const std::uint32_t hard_index : hard_indices) {
    if (hard_index >= cell_count) {continue;}
    const std::uint32_t hard_x = hard_index % grid.size_x;
    const std::uint32_t yz = hard_index / grid.size_x;
    const std::uint32_t hard_y = yz % grid.size_y;
    const std::uint32_t hard_z = yz / grid.size_y;
    const double hard_world_x =
      grid.origin_x + (static_cast<double>(hard_x) + 0.5) * grid.resolution;
    const double hard_world_y =
      grid.origin_y + (static_cast<double>(hard_y) + 0.5) * grid.resolution;
    const double ray_x = hard_world_x - observer_x;
    const double ray_y = hard_world_y - observer_y;
    const double planar_range = std::hypot(ray_x, ray_y);
    if (planar_range < std::max(config.minimum_obstacle_range, 1e-9) ||
      (config.maximum_obstacle_range > 0.0 && planar_range > config.maximum_obstacle_range))
    {
      continue;
    }
    const double unit_x = ray_x / planar_range;
    const double unit_y = ray_y / planar_range;

    for (int sample = 1; sample <= sample_count; ++sample) {
      const double travel = std::min(
        config.distance, static_cast<double>(sample) * grid.resolution);
      const double candidate_world_x = hard_world_x + unit_x * travel;
      const double candidate_world_y = hard_world_y + unit_y * travel;
      const int candidate_x = static_cast<int>(std::floor(
          (candidate_world_x - grid.origin_x) / grid.resolution));
      const int candidate_y = static_cast<int>(std::floor(
          (candidate_world_y - grid.origin_y) / grid.resolution));
      if (candidate_x < 0 || candidate_y < 0 ||
        candidate_x >= static_cast<int>(grid.size_x) ||
        candidate_y >= static_cast<int>(grid.size_y))
      {
        continue;
      }
      const std::uint32_t candidate = grid.index(
        static_cast<std::uint32_t>(candidate_x),
        static_cast<std::uint32_t>(candidate_y), hard_z);
      if (state[candidate] != 0U) {continue;}
      state[candidate] = kAdded;
      added.push_back(candidate);
    }
  }
  return added;
}

}  // namespace obstacle_occlusion_extension
