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

struct InflationConfig
{
  int horizontal_fill_cells{1};
  double hard_z_down{0.40};
  double hard_z_up{0.0};
  double soft_radius{0.60};

  bool valid() const noexcept
  {
    return horizontal_fill_cells >= 0 && horizontal_fill_cells <= 16 &&
           std::isfinite(hard_z_down) && hard_z_down >= 0.0 &&
           std::isfinite(hard_z_up) && hard_z_up >= 0.0 &&
           std::isfinite(soft_radius) && soft_radius >= 0.0;
  }
};

struct InflatedLayers
{
  std::vector<std::uint32_t> hard_indices;
  std::vector<std::uint32_t> soft_indices;
  std::vector<std::uint8_t> soft_costs;
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

inline std::vector<std::uint32_t> mergeHardIndices(
  const std::vector<std::uint32_t> & hard_indices,
  const std::vector<std::uint32_t> & added_indices,
  const std::size_t cell_count)
{
  std::vector<std::uint32_t> merged;
  if (cell_count == 0U) {
    return merged;
  }
  merged.reserve(std::min(cell_count, hard_indices.size() + added_indices.size()));
  std::vector<std::uint8_t> present(cell_count, 0U);
  const auto append_unique = [&merged, &present, cell_count](
      const std::vector<std::uint32_t> & indices) {
      for (const std::uint32_t index : indices) {
        if (index >= cell_count || present[index] != 0U) {
          continue;
        }
        present[index] = 1U;
        merged.push_back(index);
      }
    };
  append_unique(hard_indices);
  append_unique(added_indices);
  return merged;
}

inline InflatedLayers inflateObstacleSeeds(
  const GridGeometry & grid, const std::vector<std::uint32_t> & seed_indices,
  const InflationConfig & config)
{
  InflatedLayers layers;
  if (!grid.valid() || !config.valid() || seed_indices.empty()) {
    return layers;
  }

  constexpr std::uint8_t kHard = 255U;
  const std::size_t cell_count = grid.cellCount();
  std::vector<std::uint8_t> state(cell_count, 0U);
  const int down = static_cast<int>(std::ceil(config.hard_z_down / grid.resolution));
  const int up = static_cast<int>(std::ceil(config.hard_z_up / grid.resolution));
  const double half_voxel = 0.5 * grid.resolution;

  for (const std::uint32_t seed : seed_indices) {
    if (seed >= cell_count) {continue;}
    const int seed_x = static_cast<int>(seed % grid.size_x);
    const std::uint32_t yz = seed / grid.size_x;
    const int seed_y = static_cast<int>(yz % grid.size_y);
    const int seed_z = static_cast<int>(yz / grid.size_y);
    for (int dx = -config.horizontal_fill_cells; dx <= config.horizontal_fill_cells; ++dx) {
      for (int dy = -config.horizontal_fill_cells; dy <= config.horizontal_fill_cells; ++dy) {
        const int x = seed_x + dx;
        const int y = seed_y + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(grid.size_x) ||
          y >= static_cast<int>(grid.size_y))
        {
          continue;
        }
        for (int dz = -down; dz <= up; ++dz) {
          const double vertical = static_cast<double>(dz) * grid.resolution;
          if (vertical < -config.hard_z_down - half_voxel ||
            vertical > config.hard_z_up + half_voxel)
          {
            continue;
          }
          const int z = seed_z + dz;
          if (z < 0 || z >= static_cast<int>(grid.size_z)) {continue;}
          const std::uint32_t index = grid.index(
            static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
            static_cast<std::uint32_t>(z));
          if (state[index] == kHard) {continue;}
          state[index] = kHard;
          layers.hard_indices.push_back(index);
        }
      }
    }
  }

  if (config.soft_radius <= 1e-9 || layers.hard_indices.empty()) {
    return layers;
  }
  const int maximum_xy = static_cast<int>(std::ceil(config.soft_radius / grid.resolution));
  std::vector<std::uint32_t> touched;
  for (const std::uint32_t hard : layers.hard_indices) {
    const int hard_x = static_cast<int>(hard % grid.size_x);
    const std::uint32_t yz = hard / grid.size_x;
    const int hard_y = static_cast<int>(yz % grid.size_y);
    const std::uint32_t hard_z = yz / grid.size_y;
    for (int dx = -maximum_xy; dx <= maximum_xy; ++dx) {
      for (int dy = -maximum_xy; dy <= maximum_xy; ++dy) {
        const double radial = grid.resolution * std::hypot(dx, dy);
        if (radial > config.soft_radius + half_voxel) {continue;}
        const int x = hard_x + dx;
        const int y = hard_y + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(grid.size_x) ||
          y >= static_cast<int>(grid.size_y))
        {
          continue;
        }
        const std::uint32_t index = grid.index(
          static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), hard_z);
        if (state[index] == kHard) {continue;}
        const double normalized = std::clamp(
          (config.soft_radius - radial) / config.soft_radius, 0.0, 1.0);
        const std::uint8_t cost = static_cast<std::uint8_t>(
          std::clamp(std::lround(254.0 * normalized * normalized), 1L, 254L));
        if (state[index] == 0U) {touched.push_back(index);}
        state[index] = std::max(state[index], cost);
      }
    }
  }
  layers.soft_indices.reserve(touched.size());
  layers.soft_costs.reserve(touched.size());
  for (const std::uint32_t index : touched) {
    layers.soft_indices.push_back(index);
    layers.soft_costs.push_back(state[index]);
  }
  return layers;
}

inline InflatedLayers mergeInflatedLayers(
  const std::vector<std::uint32_t> & base_hard,
  const std::vector<std::uint32_t> & base_soft_indices,
  const std::vector<std::uint8_t> & base_soft_costs,
  const InflatedLayers & extension, const std::size_t cell_count)
{
  InflatedLayers merged;
  merged.hard_indices = mergeHardIndices(base_hard, extension.hard_indices, cell_count);
  if (cell_count == 0U) {return merged;}

  std::vector<std::uint8_t> hard(cell_count, 0U);
  for (const std::uint32_t index : merged.hard_indices) {
    hard[index] = 1U;
  }
  std::vector<std::uint8_t> costs(cell_count, 0U);
  std::vector<std::uint32_t> touched;
  const auto merge_soft = [&costs, &touched, cell_count](
      const std::vector<std::uint32_t> & indices,
      const std::vector<std::uint8_t> & values) {
      const std::size_t count = std::min(indices.size(), values.size());
      for (std::size_t i = 0; i < count; ++i) {
        const std::uint32_t index = indices[i];
        if (index >= cell_count || values[i] == 0U) {continue;}
        if (costs[index] == 0U) {touched.push_back(index);}
        costs[index] = std::max(costs[index], values[i]);
      }
    };
  merge_soft(base_soft_indices, base_soft_costs);
  merge_soft(extension.soft_indices, extension.soft_costs);
  merged.soft_indices.reserve(touched.size());
  merged.soft_costs.reserve(touched.size());
  for (const std::uint32_t index : touched) {
    if (hard[index] != 0U) {continue;}
    merged.soft_indices.push_back(index);
    merged.soft_costs.push_back(costs[index]);
  }
  return merged;
}

}  // namespace obstacle_occlusion_extension
