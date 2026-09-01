#include "obstacle_occlusion_extension/occlusion_extension.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

using obstacle_occlusion_extension::ExtensionConfig;
using obstacle_occlusion_extension::GridGeometry;
using obstacle_occlusion_extension::computeOcclusionExtension;
using obstacle_occlusion_extension::mergeHardIndices;

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
