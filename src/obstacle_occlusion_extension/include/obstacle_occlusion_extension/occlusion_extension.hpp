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

namespace detail
{

// Exact one-dimensional squared Euclidean distance transform using the lower
// envelope of parabolas (Felzenszwalb/Huttenlocher). Inputs are zero at feature
// cells and kInfiniteDistance elsewhere. Scratch buffers are owned by the caller
// so processing every row/column does not allocate repeatedly.
inline void squaredDistanceTransform1D(
  const std::vector<int> & input, std::vector<int> & output, const int length,
  std::vector<int> & sites, std::vector<double> & boundaries,
  const int infinite_distance)
{
  int envelope_size = 0;
  sites[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < length; ++q) {
    double intersection = 0.0;
    do {
      const int previous = sites[envelope_size];
      const auto numerator = static_cast<double>(
        static_cast<long long>(input[q]) + static_cast<long long>(q) * q -
        static_cast<long long>(input[previous]) -
        static_cast<long long>(previous) * previous);
      intersection = numerator / static_cast<double>(2 * (q - previous));
      if (envelope_size == 0 || intersection > boundaries[envelope_size]) {
        break;
      }
      --envelope_size;
    } while (true);
    ++envelope_size;
    sites[envelope_size] = q;
    boundaries[envelope_size] = intersection;
    boundaries[envelope_size + 1] = std::numeric_limits<double>::infinity();
  }

  envelope_size = 0;
  for (int q = 0; q < length; ++q) {
    while (boundaries[envelope_size + 1] < static_cast<double>(q)) {
      ++envelope_size;
    }
    const int nearest = sites[envelope_size];
    const long long delta = static_cast<long long>(q - nearest);
    output[q] = static_cast<int>(std::min<long long>(
      infinite_distance, static_cast<long long>(input[nearest]) + delta * delta));
  }
}

}  // namespace detail

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

  // The old implementation drew one soft disk around every hard voxel. Dense
  // obstacles therefore revisited the same cells millions of times. The maximum
  // disk cost at a cell depends only on its nearest hard voxel, so an exact 2-D
  // Euclidean distance transform per Z layer produces identical occupancy/costs
  // in O(size_x * size_y * size_z), independent of hard-voxel density.
  constexpr int kInfiniteDistance = 1 << 28;
  const int width = static_cast<int>(grid.size_x);
  const int height = static_cast<int>(grid.size_y);
  const int maximum_dimension = std::max(width, height);
  const std::size_t layer_size =
    static_cast<std::size_t>(grid.size_x) * static_cast<std::size_t>(grid.size_y);
  std::vector<int> horizontal_distance(layer_size, kInfiniteDistance);
  std::vector<int> squared_distance(layer_size, kInfiniteDistance);
  std::vector<int> input(static_cast<std::size_t>(maximum_dimension), kInfiniteDistance);
  std::vector<int> output(static_cast<std::size_t>(maximum_dimension), kInfiniteDistance);
  std::vector<int> sites(static_cast<std::size_t>(maximum_dimension), 0);
  std::vector<double> boundaries(static_cast<std::size_t>(maximum_dimension + 1), 0.0);

  layers.soft_indices.reserve(std::min(cell_count, layers.hard_indices.size() * 2U));
  layers.soft_costs.reserve(layers.soft_indices.capacity());
  for (std::uint32_t z = 0U; z < grid.size_z; ++z) {
    const std::size_t layer_offset = static_cast<std::size_t>(z) * layer_size;
    bool layer_has_hard = false;
    for (std::size_t local = 0U; local < layer_size; ++local) {
      if (state[layer_offset + local] == kHard) {
        layer_has_hard = true;
        break;
      }
    }
    if (!layer_has_hard) {continue;}

    for (int y = 0; y < height; ++y) {
      const std::size_t row_offset = static_cast<std::size_t>(y) * grid.size_x;
      for (int x = 0; x < width; ++x) {
        input[static_cast<std::size_t>(x)] =
          state[layer_offset + row_offset + static_cast<std::size_t>(x)] == kHard ?
          0 : kInfiniteDistance;
      }
      detail::squaredDistanceTransform1D(
        input, output, width, sites, boundaries, kInfiniteDistance);
      for (int x = 0; x < width; ++x) {
        horizontal_distance[row_offset + static_cast<std::size_t>(x)] =
          output[static_cast<std::size_t>(x)];
      }
    }

    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
        input[static_cast<std::size_t>(y)] = horizontal_distance[
          static_cast<std::size_t>(y) * grid.size_x + static_cast<std::size_t>(x)];
      }
      detail::squaredDistanceTransform1D(
        input, output, height, sites, boundaries, kInfiniteDistance);
      for (int y = 0; y < height; ++y) {
        squared_distance[
          static_cast<std::size_t>(y) * grid.size_x + static_cast<std::size_t>(x)] =
          output[static_cast<std::size_t>(y)];
      }
    }

    for (std::size_t local = 0U; local < layer_size; ++local) {
      const std::size_t index = layer_offset + local;
      if (state[index] == kHard) {continue;}
      const double radial = grid.resolution * std::sqrt(
        static_cast<double>(squared_distance[local]));
      if (radial > config.soft_radius + half_voxel) {continue;}
      const double normalized = std::clamp(
        (config.soft_radius - radial) / config.soft_radius, 0.0, 1.0);
      const std::uint8_t cost = static_cast<std::uint8_t>(
        std::clamp(std::lround(254.0 * normalized * normalized), 1L, 254L));
      layers.soft_indices.push_back(static_cast<std::uint32_t>(index));
      layers.soft_costs.push_back(cost);
    }
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
