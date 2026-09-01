#include "efficient_3d_local_planner/rolling_voxel_map.hpp"
#include "efficient_3d_local_planner/sensor_range_box.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using efficient_3d_local_planner::RollingVoxelMap;
using efficient_3d_local_planner::SensorRangeBox;

namespace
{

struct ReferenceInflation
{
  std::unordered_map<std::uint32_t, std::uint8_t> soft;
};

ReferenceInflation buildReferenceInflation(
  const RollingVoxelMap::Layers & layers, const RollingVoxelMap::Config & config)
{
  ReferenceInflation result;
  const int maximum_xy = static_cast<int>(std::ceil(
      config.soft_inflation_radius / config.resolution));
  const auto inside = [&layers](const int x, const int y, const int z) {
      return x >= 0 && y >= 0 && z >= 0 && x < layers.dimensions.x() &&
             y < layers.dimensions.y() && z < layers.dimensions.z();
    };
  const auto linear = [&layers](const int x, const int y, const int z) {
      return static_cast<std::uint32_t>(
        (z * layers.dimensions.y() + y) * layers.dimensions.x() + x);
    };

  const std::unordered_set<std::uint32_t> hard(layers.hard.begin(), layers.hard.end());
  // 用 hard 索引恢复局部坐标后直接构造单一 soft 膨胀，验证优化实现保持两层语义。
  for (const std::uint32_t obstacle_index : layers.hard) {
    const int obstacle_x = static_cast<int>(obstacle_index % layers.dimensions.x());
    const int yz = static_cast<int>(obstacle_index / layers.dimensions.x());
    const int obstacle_y = yz % layers.dimensions.y();
    const int obstacle_z = yz / layers.dimensions.y();
    for (int dx = -maximum_xy; dx <= maximum_xy; ++dx) {
      for (int dy = -maximum_xy; dy <= maximum_xy; ++dy) {
        const double radial = config.resolution * std::hypot(dx, dy);
        if (config.soft_inflation_radius <= 1e-9 ||
          radial > config.soft_inflation_radius + 0.5 * config.resolution)
        {
          continue;
        }
        const double normalized = std::clamp(
          (config.soft_inflation_radius - radial) / config.soft_inflation_radius, 0.0, 1.0);
        const std::uint8_t cost = static_cast<std::uint8_t>(
          std::clamp(std::lround(254.0 * normalized * normalized), 1L, 254L));
        const int x = obstacle_x + dx;
        const int y = obstacle_y + dy;
        const int z = obstacle_z;
        if (!inside(x, y, z)) {
          continue;
        }
        const std::uint32_t index = linear(x, y, z);
        if (hard.count(index) != 0U) {continue;}
        auto [iterator, inserted] = result.soft.try_emplace(index, cost);
        if (!inserted) {
          iterator->second = std::max(iterator->second, cost);
        }
      }
    }
  }
  return result;
}

}  // namespace

TEST(RollingVoxelMap, RequiresConfirmedHitAndBuildsTwoLayers)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(4.0, 4.0, 2.0);
  config.soft_inflation_radius = 0.40;
  config.hard_inflation_z_down = 0.00;
  RollingVoxelMap map(config);

  const auto update = map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(1.0, 0.0, 0.0)}, 1.0);
  EXPECT_TRUE(map.buildLayers().hard.empty());
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(1.0, 0.0, 0.0)}, 1.1);
  const auto layers = map.buildLayers();
  EXPECT_EQ(update.unique_endpoints, 1U);
  EXPECT_EQ(layers.hard.size(), 9U);
  EXPECT_FALSE(layers.soft_indices.empty());
  EXPECT_EQ(layers.soft_indices.size(), layers.soft_costs.size());
  EXPECT_TRUE(std::all_of(
      layers.soft_costs.begin(), layers.soft_costs.end(),
      [](const std::uint8_t value) {return value > 0U && value < 255U;}));
  EXPECT_EQ(
    std::count(layers.soft_indices.begin(), layers.soft_indices.end(), layers.hard.front()), 0);
}

TEST(RollingVoxelMap, SoftCostIsGradedOnlyByHorizontalDistanceFromHard)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(4.0, 4.0, 2.0);
  config.soft_inflation_radius = 0.60;
  config.hard_inflation_z_down = 0.00;
  config.hard_inflation_z_up = 0.00;
  config.hit_confirmation_count = 1;
  config.raycast_enabled = false;
  config.decay_enabled = false;
  RollingVoxelMap map(config);
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.5, 0.0, 0.0)}, 1.0);
  const auto layers = map.buildLayers();
  ASSERT_EQ(layers.hard.size(), 9U);

  std::unordered_map<std::uint32_t, std::uint8_t> soft;
  for (std::size_t i = 0; i < layers.soft_indices.size(); ++i) {
    soft.emplace(layers.soft_indices[i], layers.soft_costs[i]);
  }
  const Eigen::Vector3d origin = map.origin(layers);
  const Eigen::Vector3i raw_local =
    ((Eigen::Vector3d(0.5, 0.0, 0.0) - origin) / config.resolution)
    .array().floor().cast<int>();
  const auto linear = [&layers](const int x, const int y, const int z) {
      return static_cast<std::uint32_t>(
        (z * layers.dimensions.y() + y) * layers.dimensions.x() + x);
    };
  // 原始体素左右一格已经成为 hard；从补空 hard 的外侧开始检查 soft 梯度。
  const std::uint32_t near = linear(raw_local.x() + 2, raw_local.y(), raw_local.z());
  const std::uint32_t far = linear(raw_local.x() + 5, raw_local.y(), raw_local.z());
  ASSERT_NE(soft.count(near), 0U);
  ASSERT_NE(soft.count(far), 0U);
  EXPECT_GT(soft.at(near), soft.at(far));
  EXPECT_EQ(
    soft.count(linear(raw_local.x() + 2, raw_local.y(), raw_local.z() + 1)), 0U);
}

TEST(SensorRangeBox, AcceptsOnlyPointsBetweenInnerAndOuterBoxes)
{
  SensorRangeBox filter;
  filter.outer_min = Eigen::Vector3d(-4.0, -3.0, -3.0);
  filter.outer_max = Eigen::Vector3d(4.0, 3.0, 1.2);
  filter.inner_half = Eigen::Vector3d(0.30, 0.20, 0.10);
  ASSERT_TRUE(filter.valid());

  EXPECT_FALSE(filter.accepts(Eigen::Vector3d(0.10, 0.10, 0.05)));
  EXPECT_TRUE(filter.accepts(Eigen::Vector3d(0.31, 0.10, 0.05)));
  EXPECT_TRUE(filter.accepts(Eigen::Vector3d(4.0, 3.0, 1.2)));
  EXPECT_FALSE(filter.accepts(Eigen::Vector3d(4.01, 0.0, 0.0)));
  EXPECT_FALSE(filter.accepts(Eigen::Vector3d(0.0, 0.0, 1.21)));
  EXPECT_FALSE(filter.accepts(Eigen::Vector3d(
    0.0, 0.0, std::numeric_limits<double>::quiet_NaN())));
}

TEST(SensorRangeBox, RejectsInvalidOrEscapingInnerBoxConfiguration)
{
  SensorRangeBox filter;
  EXPECT_TRUE(filter.valid());
  filter.inner_half.x() = 5.0;
  EXPECT_FALSE(filter.valid());
}

TEST(RollingVoxelMap, StaleObstacleDecaysOut)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(4.0, 4.0, 2.0);
  config.decay_start = 0.10;
  config.decay_rate = 10.0;
  config.hit_confirmation_count = 1;
  RollingVoxelMap map(config);
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(1.0, 0.0, 0.0)}, 1.0);
  ASSERT_FALSE(map.buildLayers().hard.empty());
  map.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, 2.0);
  EXPECT_TRUE(map.buildLayers().hard.empty());
}

TEST(RollingVoxelMap, DisabledDecayRetainsStaleObstacle)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(4.0, 4.0, 2.0);
  config.decay_enabled = false;
  config.decay_start = 0.10;
  config.decay_rate = 10.0;
  config.hit_confirmation_count = 1;
  RollingVoxelMap map(config);
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(1.0, 0.0, 0.0)}, 1.0);
  ASSERT_FALSE(map.buildLayers().hard.empty());
  map.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, 2.0);
  EXPECT_FALSE(map.buildLayers().hard.empty());
}

TEST(RollingVoxelMap, DisabledRaycastDoesNotClearTraversedObstacle)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(6.0, 4.0, 2.0);
  config.hard_inflation_z_down = 0.00;
  config.raycast_enabled = false;
  config.decay_enabled = false;
  config.hit_confirmation_count = 1;
  RollingVoxelMap map(config);
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(1.0, 0.0, 0.0)}, 1.0);
  ASSERT_EQ(map.buildLayers().hard.size(), 9U);

  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(2.0, 0.0, 0.0)}, 1.1);
  EXPECT_EQ(map.buildLayers().hard.size(), 18U);
}

TEST(RollingVoxelMap, FrontOnlyDecayRetainsRearUntilBodyTurnsTowardIt)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(4.0, 4.0, 2.0);
  config.hard_inflation_z_down = 0.00;
  config.decay_start = 0.10;
  config.decay_rate = 10.0;
  config.decay_front_only = true;
  config.hit_confirmation_count = 1;
  RollingVoxelMap map(config);

  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-1.0, 0.0, 0.0)}, 1.0,
    Eigen::Quaterniond::Identity());
  ASSERT_EQ(map.buildLayers().hard.size(), 18U);

  // Facing +X: the front obstacle decays, while the rear obstacle remains.
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, 2.0,
    Eigen::Quaterniond::Identity());
  EXPECT_EQ(map.buildLayers().hard.size(), 9U);

  // After turning around, the retained -X obstacle is now in front and may decay.
  constexpr double kPi = 3.14159265358979323846;
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, 3.0,
    Eigen::Quaterniond(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitZ())));
  EXPECT_TRUE(map.buildLayers().hard.empty());
}

TEST(RollingVoxelMap, OmnidirectionalDecayCanStillBeSelected)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(4.0, 4.0, 2.0);
  config.hard_inflation_z_down = 0.00;
  config.decay_start = 0.10;
  config.decay_rate = 10.0;
  config.decay_front_only = false;
  config.hit_confirmation_count = 1;
  RollingVoxelMap map(config);

  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(-1.0, 0.0, 0.0)}, 1.0,
    Eigen::Quaterniond::Identity());
  ASSERT_FALSE(map.buildLayers().hard.empty());
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, 2.0,
    Eigen::Quaterniond::Identity());
  EXPECT_TRUE(map.buildLayers().hard.empty());
}

TEST(RollingVoxelMap, FrontDecayUsesConfiguredAngularSector)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(4.0, 4.0, 2.0);
  config.hard_inflation_z_down = 0.00;
  config.decay_start = 0.10;
  config.decay_rate = 10.0;
  config.decay_front_only = true;
  config.decay_front_fov_deg = 120.0;
  config.hit_confirmation_count = 1;
  RollingVoxelMap map(config);

  constexpr double kPi = 3.14159265358979323846;
  const auto point_at_angle = [](const double angle) {
      return Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
    };
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {point_at_angle(30.0 * kPi / 180.0), point_at_angle(80.0 * kPi / 180.0)},
    1.0, Eigen::Quaterniond::Identity());
  ASSERT_EQ(map.buildLayers().hard.size(), 18U);

  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, 2.0,
    Eigen::Quaterniond::Identity());
  // The 30-degree obstacle decays; the 80-degree obstacle is retained.
  EXPECT_EQ(map.buildLayers().hard.size(), 9U);
}

TEST(RollingVoxelMap, RollingWindowPrunesOldCells)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(2.0, 2.0, 1.0);
  config.hit_confirmation_count = 1;
  RollingVoxelMap map(config);
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.5, 0.0, 0.0)}, 1.0);
  ASSERT_FALSE(map.buildLayers().hard.empty());
  map.update(
    Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d(10.0, 0.0, 0.0), {}, 1.1);
  EXPECT_TRUE(map.buildLayers().hard.empty());
}

TEST(RollingVoxelMap, BodyExclusionClearsPreviouslyOccupiedVoxels)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(4.0, 4.0, 2.0);
  config.hard_inflation_z_down = 0.00;
  config.hit_confirmation_count = 1;
  RollingVoxelMap map(config);
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.4, 0.05, 0.0), Eigen::Vector3d(0.4, 0.25, 0.0)}, 1.0);
  ASSERT_EQ(map.buildLayers().hard.size(), 15U);

  const std::size_t cleared = map.clearBodyExclusion(
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
    Eigen::Vector3d(0.55, 0.10, 0.35));

  EXPECT_EQ(cleared, 1U);
  EXPECT_EQ(map.buildLayers().hard.size(), 9U);
}

TEST(RollingVoxelMap, VerticalInflationUsesIndependentMetricLimits)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(4.0, 4.0, 2.0);
  config.soft_inflation_radius = 0.25;
  config.hard_inflation_z_down = 0.40;
  config.hard_inflation_z_up = 0.10;
  config.hit_confirmation_count = 1;
  config.raycast_enabled = false;
  config.decay_enabled = false;
  RollingVoxelMap map(config);
  const Eigen::Vector3d obstacle(0.55, 0.05, 0.05);
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {obstacle}, 1.0);
  const auto layers = map.buildLayers();
  const Eigen::Vector3d origin = map.origin(layers);
  const Eigen::Vector3i raw_local =
    ((obstacle - origin) / config.resolution).array().floor().cast<int>();
  const auto linear = [&layers](const Eigen::Vector3i & local) {
      return static_cast<std::uint32_t>(
        (local.z() * layers.dimensions.y() + local.y()) * layers.dimensions.x() + local.x());
    };
  const std::unordered_set<std::uint32_t> hard(layers.hard.begin(), layers.hard.end());

  ASSERT_EQ(hard.size(), 54U);  // 3x3 水平补空 * [-4, +1] 六层 Z 体素
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      EXPECT_NE(hard.count(linear(raw_local + Eigen::Vector3i(dx, dy, 0))), 0U);
      EXPECT_NE(hard.count(linear(raw_local + Eigen::Vector3i(dx, dy, -4))), 0U);
    }
  }
  EXPECT_EQ(hard.count(linear(raw_local + Eigen::Vector3i(2, 0, 0))), 0U);
  EXPECT_EQ(hard.count(linear(raw_local + Eigen::Vector3i(0, 2, -4))), 0U);
  EXPECT_NE(hard.count(linear(raw_local)), 0U);
  EXPECT_NE(hard.count(linear(raw_local + Eigen::Vector3i(0, 0, -4))), 0U);
  EXPECT_NE(hard.count(linear(raw_local + Eigen::Vector3i(0, 0, 1))), 0U);
  EXPECT_EQ(hard.count(linear(raw_local + Eigen::Vector3i(0, 0, -5))), 0U);
  EXPECT_EQ(hard.count(linear(raw_local + Eigen::Vector3i(0, 0, 2))), 0U);
}

TEST(RollingVoxelMap, OptimizedLayerAssemblyMatchesSingleSoftInflationRules)
{
  RollingVoxelMap::Config config;
  config.resolution = 0.10;
  config.size = Eigen::Vector3d(2.0, 2.0, 1.0);
  config.soft_inflation_radius = 0.425;
  config.hard_inflation_z_down = 0.15;
  config.hard_inflation_z_up = 0.10;
  config.hit_confirmation_count = 1;
  config.raycast_enabled = false;
  config.decay_enabled = false;
  RollingVoxelMap map(config);

  // 相邻障碍用于覆盖 soft 代价重叠，窗口边缘障碍用于覆盖边界裁剪。
  map.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    {Eigen::Vector3d(0.05, 0.05, 0.05), Eigen::Vector3d(0.25, 0.05, 0.05),
      Eigen::Vector3d(0.85, 0.85, 0.35)}, 1.0);
  const auto layers = map.buildLayers();
  const auto reference = buildReferenceInflation(layers, config);

  std::unordered_map<std::uint32_t, std::uint8_t> actual_soft;
  ASSERT_EQ(layers.soft_indices.size(), layers.soft_costs.size());
  for (std::size_t i = 0; i < layers.soft_indices.size(); ++i) {
    actual_soft.emplace(layers.soft_indices[i], layers.soft_costs[i]);
  }
  ASSERT_EQ(actual_soft.size(), layers.soft_indices.size());
  EXPECT_EQ(actual_soft, reference.soft);
  for (const auto index : layers.hard) {
    EXPECT_EQ(actual_soft.count(index), 0U);
  }
}
