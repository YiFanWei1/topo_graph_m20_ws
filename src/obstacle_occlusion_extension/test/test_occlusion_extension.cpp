#include "obstacle_occlusion_extension/occlusion_extension.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

using obstacle_occlusion_extension::ExtensionConfig;
using obstacle_occlusion_extension::GridGeometry;
using obstacle_occlusion_extension::InflationConfig;
using obstacle_occlusion_extension::InflatedLayers;
using obstacle_occlusion_extension::computeOcclusionExtension;
using obstacle_occlusion_extension::inflateObstacleSeeds;
using obstacle_occlusion_extension::mergeHardIndices;
using obstacle_occlusion_extension::mergeInflatedLayers;

namespace
{

GridGeometry testGrid()
{
  return GridGeometry{0.0, 0.0, 0.0, 0.10, 12U, 8U, 3U};
}

bool contains(const std::vector<std::uint32_t> & values, const std::uint32_t value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

// Reference implementation retained in the test so the optimized distance
// transform can be checked against the original disk-per-hard-voxel algorithm.
InflatedLayers referenceInflateObstacleSeeds(
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

struct DenseLayers
{
  std::vector<std::uint8_t> hard;
  std::vector<std::uint8_t> soft_cost;
};

DenseLayers toDense(const InflatedLayers & layers, const std::size_t cell_count)
{
  DenseLayers dense{
    std::vector<std::uint8_t>(cell_count, 0U),
    std::vector<std::uint8_t>(cell_count, 0U)};
  for (const std::uint32_t index : layers.hard_indices) {
    if (index < cell_count) {dense.hard[index] = 1U;}
  }
  const std::size_t soft_count = std::min(layers.soft_indices.size(), layers.soft_costs.size());
  for (std::size_t i = 0U; i < soft_count; ++i) {
    const std::uint32_t index = layers.soft_indices[i];
    if (index < cell_count) {
      dense.soft_cost[index] = std::max(dense.soft_cost[index], layers.soft_costs[i]);
    }
  }
  return dense;
}

void expectEquivalent(
  const GridGeometry & grid, const std::vector<std::uint32_t> & seeds,
  const InflationConfig & config)
{
  const auto reference = toDense(
    referenceInflateObstacleSeeds(grid, seeds, config), grid.cellCount());
  const auto optimized = toDense(inflateObstacleSeeds(grid, seeds, config), grid.cellCount());
  EXPECT_EQ(optimized.hard, reference.hard);
  EXPECT_EQ(optimized.soft_cost, reference.soft_cost);
}

}  // namespace

TEST(OcclusionExtension, ExtendsAwayFromObserverAlongHorizontalRay)
{
  const auto grid = testGrid();
  const std::uint32_t hard = grid.index(4U, 3U, 1U);
  ExtensionConfig config;
  config.distance = 0.30;
  config.minimum_obstacle_range = 0.0;
  config.maximum_obstacle_range = 0.0;

  const auto added = computeOcclusionExtension(
    grid, {hard}, 0.05, 0.35, config);

  ASSERT_EQ(added.size(), 3U);
  EXPECT_TRUE(contains(added, grid.index(5U, 3U, 1U)));
  EXPECT_TRUE(contains(added, grid.index(6U, 3U, 1U)));
  EXPECT_TRUE(contains(added, grid.index(7U, 3U, 1U)));
  EXPECT_FALSE(contains(added, hard));
}

TEST(OcclusionExtension, NeverPublishesExistingHardVoxels)
{
  const auto grid = testGrid();
  const std::uint32_t front = grid.index(4U, 3U, 1U);
  const std::uint32_t rear = grid.index(6U, 3U, 1U);
  ExtensionConfig config;
  config.distance = 0.30;
  config.minimum_obstacle_range = 0.0;
  config.maximum_obstacle_range = 0.0;

  const auto added = computeOcclusionExtension(
    grid, {front, rear}, 0.05, 0.35, config);

  EXPECT_FALSE(contains(added, front));
  EXPECT_FALSE(contains(added, rear));
  EXPECT_EQ(std::adjacent_find(added.begin(), added.end()), added.end());
}

TEST(OcclusionExtension, PreservesHeightAndClipsAtRollingWindowBoundary)
{
  const auto grid = testGrid();
  const std::uint32_t hard = grid.index(10U, 3U, 2U);
  ExtensionConfig config;
  config.distance = 0.50;
  config.minimum_obstacle_range = 0.0;
  config.maximum_obstacle_range = 0.0;

  const auto added = computeOcclusionExtension(
    grid, {hard}, 0.05, 0.35, config);

  ASSERT_EQ(added.size(), 1U);
  EXPECT_EQ(added.front(), grid.index(11U, 3U, 2U));
}

TEST(OcclusionExtension, AppliesConfiguredObstacleRange)
{
  const auto grid = testGrid();
  ExtensionConfig config;
  config.distance = 0.30;
  config.minimum_obstacle_range = 0.50;
  config.maximum_obstacle_range = 0.80;
  const std::uint32_t near = grid.index(2U, 3U, 1U);
  const std::uint32_t accepted = grid.index(6U, 3U, 1U);
  const std::uint32_t far = grid.index(10U, 3U, 1U);

  const auto added = computeOcclusionExtension(
    grid, {near, accepted, far}, 0.05, 0.35, config);

  EXPECT_FALSE(contains(added, grid.index(3U, 3U, 1U)));
  EXPECT_TRUE(contains(added, grid.index(7U, 3U, 1U)));
  EXPECT_FALSE(contains(added, grid.index(11U, 3U, 1U)));
}

TEST(OcclusionExtension, MergesAddedVoxelsIntoHardWithoutDuplicates)
{
  const auto merged = mergeHardIndices({1U, 3U, 3U}, {2U, 3U, 99U}, 8U);

  EXPECT_EQ(merged, (std::vector<std::uint32_t>{1U, 3U, 2U}));
}

TEST(OcclusionExtension, InflatesAddedSeedsThroughHorizontalVerticalAndSoftLayers)
{
  const GridGeometry grid{0.0, 0.0, 0.0, 0.10, 15U, 11U, 6U};
  InflationConfig config;
  config.horizontal_fill_cells = 1;
  config.hard_z_down = 0.20;
  config.hard_z_up = 0.0;
  config.soft_radius = 0.30;
  const std::uint32_t seed = grid.index(8U, 5U, 3U);

  const auto layers = inflateObstacleSeeds(grid, {seed}, config);

  EXPECT_TRUE(contains(layers.hard_indices, grid.index(8U, 5U, 3U)));
  EXPECT_TRUE(contains(layers.hard_indices, grid.index(9U, 6U, 3U)));
  EXPECT_TRUE(contains(layers.hard_indices, grid.index(8U, 5U, 1U)));
  const auto soft = std::find(layers.soft_indices.begin(), layers.soft_indices.end(),
      grid.index(11U, 5U, 3U));
  ASSERT_NE(soft, layers.soft_indices.end());
  const std::size_t soft_offset = static_cast<std::size_t>(
    std::distance(layers.soft_indices.begin(), soft));
  EXPECT_GT(layers.soft_costs[soft_offset], 0U);
}

TEST(OcclusionExtension, MergesSoftCostsAndRemovesCellsPromotedToHard)
{
  const auto grid = testGrid();
  obstacle_occlusion_extension::InflatedLayers extension;
  extension.hard_indices = {grid.index(5U, 3U, 1U)};
  extension.soft_indices = {
    grid.index(4U, 3U, 1U), grid.index(6U, 3U, 1U)};
  extension.soft_costs = {180U, 120U};
  const std::vector<std::uint32_t> base_hard{grid.index(3U, 3U, 1U)};
  const std::vector<std::uint32_t> base_soft{
    grid.index(5U, 3U, 1U), grid.index(6U, 3U, 1U)};
  const std::vector<std::uint8_t> base_costs{90U, 200U};

  const auto merged = mergeInflatedLayers(
    base_hard, base_soft, base_costs, extension, grid.cellCount());

  EXPECT_TRUE(contains(merged.hard_indices, grid.index(5U, 3U, 1U)));
  EXPECT_FALSE(contains(merged.soft_indices, grid.index(5U, 3U, 1U)));
  const auto overlap = std::find(
    merged.soft_indices.begin(), merged.soft_indices.end(), grid.index(6U, 3U, 1U));
  ASSERT_NE(overlap, merged.soft_indices.end());
  EXPECT_EQ(merged.soft_costs[static_cast<std::size_t>(
      std::distance(merged.soft_indices.begin(), overlap))], 200U);
}

TEST(OcclusionExtension, OptimizedInflationExactlyMatchesReferenceAtEdgesAndAcrossLayers)
{
  const GridGeometry grid{-1.0, -2.0, -0.5, 0.08, 31U, 23U, 9U};
  const std::vector<std::uint32_t> seeds{
    grid.index(0U, 0U, 0U), grid.index(30U, 22U, 8U),
    grid.index(15U, 11U, 4U), grid.index(16U, 11U, 4U),
    grid.index(2U, 20U, 7U), grid.index(15U, 11U, 4U),
    static_cast<std::uint32_t>(grid.cellCount() + 10U)};

  InflationConfig config;
  config.horizontal_fill_cells = 1;
  config.hard_z_down = 0.40;
  config.hard_z_up = 0.16;
  config.soft_radius = 0.60;
  expectEquivalent(grid, seeds, config);

  config.horizontal_fill_cells = 0;
  config.hard_z_down = 0.13;
  config.hard_z_up = 0.07;
  config.soft_radius = 0.37;
  expectEquivalent(grid, seeds, config);
}

TEST(OcclusionExtension, OptimizedInflationMatchesReferenceForProductionSizedDenseMap)
{
  const GridGeometry grid{-4.0, -4.0, -1.2, 0.08, 100U, 100U, 38U};
  std::vector<std::uint32_t> seeds;
  for (std::uint32_t z = 8U; z < 34U; z += 4U) {
    for (std::uint32_t y = 8U; y < 94U; y += 5U) {
      for (std::uint32_t x = 7U; x < 95U; x += 5U) {
        seeds.push_back(grid.index(x, y, z));
      }
    }
  }

  InflationConfig config;
  config.horizontal_fill_cells = 1;
  config.hard_z_down = 0.40;
  config.hard_z_up = 0.0;
  config.soft_radius = 0.60;

  const auto reference_start = std::chrono::steady_clock::now();
  const auto reference_layers = referenceInflateObstacleSeeds(grid, seeds, config);
  const auto reference_elapsed = std::chrono::steady_clock::now() - reference_start;
  const auto optimized_start = std::chrono::steady_clock::now();
  const auto optimized_layers = inflateObstacleSeeds(grid, seeds, config);
  const auto optimized_elapsed = std::chrono::steady_clock::now() - optimized_start;

  const auto reference = toDense(reference_layers, grid.cellCount());
  const auto optimized = toDense(optimized_layers, grid.cellCount());
  EXPECT_EQ(optimized.hard, reference.hard);
  EXPECT_EQ(optimized.soft_cost, reference.soft_cost);

  const double reference_ms =
    std::chrono::duration<double, std::milli>(reference_elapsed).count();
  const double optimized_ms =
    std::chrono::duration<double, std::milli>(optimized_elapsed).count();
  std::cout << "production inflation benchmark: reference=" << reference_ms
            << " ms optimized=" << optimized_ms << " ms speedup="
            << reference_ms / std::max(optimized_ms, std::numeric_limits<double>::epsilon())
            << "x\n";
  EXPECT_LT(optimized_ms, reference_ms);
}
