#include "obstacle_occlusion_extension/occlusion_extension.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

using obstacle_occlusion_extension::ExtensionConfig;
using obstacle_occlusion_extension::GridGeometry;
using obstacle_occlusion_extension::InflationConfig;
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
