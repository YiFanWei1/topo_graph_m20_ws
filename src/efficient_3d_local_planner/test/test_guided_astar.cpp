#include "efficient_3d_local_planner/guided_astar.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using efficient_3d_local_planner::GridSnapshot;
using efficient_3d_local_planner::GuidedAStar;
using efficient_3d_local_planner::projectStartZToLiftedPath;

namespace
{

GridSnapshot makeMap()
{
  GridSnapshot map;
  map.origin = Eigen::Vector3d(-3.0, -3.0, -0.6);
  map.resolution = 0.10;
  map.dimensions = Eigen::Vector3i(60, 60, 12);
  map.revision = 1U;
  map.hard.assign(map.cellCount(), 0U);
  map.footprint_hard.assign(map.cellCount(), 0U);
  map.soft_cost.assign(map.cellCount(), 0U);
  return map;
}

std::vector<Eigen::Vector3d> straightGuide()
{
  std::vector<Eigen::Vector3d> guide;
  for (double x = -2.0; x <= 2.01; x += 0.10) {
    guide.emplace_back(x, 0.0, 0.0);
  }
  return guide;
}

void setVerticalBand(GridSnapshot & map, const double x, const bool hard, const bool soft)
{
  for (std::size_t index = 0; index < map.cellCount(); ++index) {
    const Eigen::Vector3d point = map.cellCenter(map.cellFromLinear(static_cast<int>(index)));
    if (std::abs(point.x() - x) <= 0.35 && std::abs(point.y()) <= 1.7) {
      if (hard) {map.footprint_hard[index] = 1U;}
      if (soft) {map.soft_cost[index] = 220U;}
    }
  }
}

}  // namespace

TEST(GuidedAStar, FindsStrictPathInOpenSpace)
{
  GridSnapshot map = makeMap();
  GuidedAStar planner;
  const auto result = planner.search(map, Eigen::Vector3d(-2.0, 0.0, 0.0), 0.0, straightGuide());
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_FALSE(result.used_soft);
  EXPECT_GE(result.path.size(), 2U);
  EXPECT_NEAR((result.path.back() - result.selected_goal).norm(), 0.0, 1e-9);
}

TEST(GuidedAStar, UsesContinuousDirectShortcutForNonLatticeHeading)
{
  GridSnapshot map = makeMap();
  const Eigen::Vector3d start(-2.0, -0.20, 0.0);
  const Eigen::Vector3d goal(2.0, 0.80, 0.0);
  std::vector<Eigen::Vector3d> guide;
  for (int i = 0; i <= 40; ++i) {
    const double ratio = static_cast<double>(i) / 40.0;
    guide.push_back(Eigen::Vector3d(-2.0 + 4.0 * ratio, 0.80 * ratio, 0.0));
  }
  GuidedAStar planner;
  const auto result = planner.search(map, start, 0.0, guide);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_TRUE(result.direct_shortcut_used);
  EXPECT_EQ(result.reason, "direct_strict_success");
  ASSERT_EQ(result.path.size(), 2U);
  EXPECT_NEAR((result.path.front() - start).norm(), 0.0, 1e-9);
  EXPECT_NEAR((result.path.back() - goal).norm(), 0.0, 1e-9);
}

TEST(GuidedAStar, DirectShortcutNeverTouchesSoftInflationLayer)
{
  GridSnapshot map = makeMap();
  setVerticalBand(map, 0.0, false, true);
  GuidedAStar planner;
  const auto result = planner.search(
    map, Eigen::Vector3d(-2.0, 0.0, 0.0), 0.0, straightGuide());
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_FALSE(result.direct_shortcut_used);
  EXPECT_TRUE(result.used_soft);
}

TEST(GuidedAStar, DirectShortcutNeverCutsReferenceCorner)
{
  GridSnapshot map = makeMap();
  const std::vector<Eigen::Vector3d> guide{
    Eigen::Vector3d(-2.0, 0.0, 0.0),
    Eigen::Vector3d(-1.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(0.0, 2.0, 0.0)};
  GuidedAStar planner;
  const auto result = planner.search(map, guide.front(), 0.0, guide);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_FALSE(result.direct_shortcut_used);
}

TEST(GuidedAStar, DirectShortcutChecksInPlaceRotationSweep)
{
  GridSnapshot map = makeMap();
  const Eigen::Vector3d start(-2.0, 0.0, 0.0);
  const Eigen::Vector3d rotation_collision = start +
    0.18 * Eigen::Vector3d(std::sqrt(0.5), -std::sqrt(0.5), 0.0);
  const int collision_index = map.indexAt(rotation_collision);
  ASSERT_GE(collision_index, 0);
  map.soft_cost[static_cast<std::size_t>(collision_index)] = 220U;
  GuidedAStar planner;
  const auto result = planner.search(map, start, -0.5 * M_PI, straightGuide());
  EXPECT_FALSE(result.direct_shortcut_used);
}

TEST(GuidedAStar, ProjectsOnlyStartHeightOntoLiftedStairPath)
{
  const std::vector<Eigen::Vector3d> lifted_stairs{
    Eigen::Vector3d(0.0, 0.0, 0.40),
    Eigen::Vector3d(1.0, 0.0, 0.90),
    Eigen::Vector3d(2.0, 0.0, 1.40)};
  const Eigen::Vector3d odometry(0.8, 0.2, 0.45);
  bool applied = false;
  const Eigen::Vector3d projected = projectStartZToLiftedPath(
    odometry, lifted_stairs, 0.60, &applied);
  EXPECT_TRUE(applied);
  EXPECT_DOUBLE_EQ(projected.x(), odometry.x());
  EXPECT_DOUBLE_EQ(projected.y(), odometry.y());
  EXPECT_GT(projected.z(), odometry.z());
  EXPECT_LT(std::abs(projected.z() - 0.8), 0.15);
}

TEST(GuidedAStar, FallsBackToSoftLayerForPassableNarrowRegion)
{
  GridSnapshot map = makeMap();
  setVerticalBand(map, 0.0, false, true);
  GuidedAStar planner;
  ASSERT_GT(map.softAt(Eigen::Vector3d(0.0, 0.0, 0.0)), 0.5);
  const auto result = planner.search(map, Eigen::Vector3d(-2.0, 0.0, 0.0), 0.0, straightGuide());
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_TRUE(result.used_soft);
}

TEST(GuidedAStar, StrictSimplificationRejectsShortcutThroughSoftLayer)
{
  GridSnapshot map = makeMap();
  const auto guide = straightGuide();
  const auto guide_distance = efficient_3d_local_planner::cumulativeDistance(guide);
  setVerticalBand(map, 0.0, false, true);
  GuidedAStar planner;
  const Eigen::Vector3d from(-1.0, 0.0, 0.0);
  const Eigen::Vector3d to(1.0, 0.0, 0.0);

  EXPECT_TRUE(planner.edgeValid(
      map, from, 0.0, to, 0.0, guide, guide_distance, false));
  EXPECT_FALSE(planner.edgeValid(
      map, from, 0.0, to, 0.0, guide, guide_distance, true));
}

TEST(GuidedAStar, NeverCrossesPhysicalFootprintWall)
{
  GridSnapshot map = makeMap();
  setVerticalBand(map, 0.0, true, false);
  GuidedAStar planner;
  ASSERT_FALSE(planner.poseValid(map, Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, false));
  const auto result = planner.search(map, Eigen::Vector3d(-2.0, 0.0, 0.0), 0.0, straightGuide());
  EXPECT_FALSE(result.success);
}

TEST(GuidedAStar, RejectsPureVerticalOrCrossFloorShortcut)
{
  GridSnapshot map = makeMap();
  GuidedAStar::Config config;
  config.corridor_z = 0.20;
  GuidedAStar planner(config);
  auto guide = straightGuide();
  for (std::size_t i = guide.size() / 2U; i < guide.size(); ++i) {
    guide[i].z() = 0.8;
  }
  const auto result = planner.search(map, Eigen::Vector3d(-2.0, 0.0, 0.0), 0.0, guide);
  EXPECT_FALSE(result.success);
}

TEST(GuidedAStar, SnapsOccupiedLookaheadGoalToNearbyFreePose)
{
  GridSnapshot map = makeMap();
  const auto guide = straightGuide();
  const Eigen::Vector3d requested_goal = guide.back();
  for (std::size_t index = 0; index < map.cellCount(); ++index) {
    const Eigen::Vector3d point = map.cellCenter(map.cellFromLinear(static_cast<int>(index)));
    if ((point - requested_goal).norm() < 0.22) {
      map.footprint_hard[index] = 1U;
    }
  }
  GuidedAStar planner;
  const auto result = planner.search(map, Eigen::Vector3d(-2.0, 0.0, 0.0), 0.0, guide);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_NEAR((result.path.back() - result.selected_goal).norm(), 0.0, 1e-9);
  EXPECT_GT((result.path.back() - requested_goal).norm(), 0.15);
  EXPECT_LT((result.path.back() - requested_goal).norm(), 0.75);
}

TEST(GuidedAStar, RecoversFromOccupiedRequestedStartLikeScanPlanner)
{
  GridSnapshot map = makeMap();
  const auto guide = straightGuide();
  const Eigen::Vector3d requested_start(-2.0, 0.0, 0.0);
  const int front = map.indexAt(requested_start + Eigen::Vector3d(0.18, 0.0, 0.0));
  ASSERT_GE(front, 0);
  map.footprint_hard[static_cast<std::size_t>(front)] = 1U;

  GuidedAStar planner;
  ASSERT_FALSE(planner.poseValid(map, requested_start, 0.0, false));
  const auto result = planner.search(map, requested_start, 0.0, guide);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_GT((result.selected_start - requested_start).norm(), 0.05);
  EXPECT_LE((result.selected_start - requested_start).norm(), 0.50 + 1e-9);
}
