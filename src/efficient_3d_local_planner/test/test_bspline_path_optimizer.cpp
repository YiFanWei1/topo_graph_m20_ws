#include "efficient_3d_local_planner/bspline_path_optimizer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{

using efficient_3d_local_planner::BsplinePathOptimizer;
using efficient_3d_local_planner::GridSnapshot;
using Eigen::Vector3d;

GridSnapshot emptyMap()
{
  GridSnapshot map;
  map.origin = Vector3d(-2.0, -2.0, -0.5);
  map.resolution = 0.1;
  map.dimensions = Eigen::Vector3i(50, 50, 10);
  map.hard.assign(map.cellCount(), 0U);
  map.soft_cost.assign(map.cellCount(), 0U);
  return map;
}

double maximumHeadingChange(const std::vector<Vector3d> & path)
{
  double maximum = 0.0;
  for (std::size_t i = 1U; i + 1U < path.size(); ++i) {
    const Eigen::Vector2d before = (path[i] - path[i - 1U]).head<2>();
    const Eigen::Vector2d after = (path[i + 1U] - path[i]).head<2>();
    if (before.norm() < 1e-6 || after.norm() < 1e-6) {continue;}
    maximum = std::max(maximum, std::acos(std::clamp(
          before.normalized().dot(after.normalized()), -1.0, 1.0)));
  }
  return maximum;
}

TEST(BsplinePathOptimizer, SmoothsCornerWhileKeepingEndpoints)
{
  BsplinePathOptimizer::Config config;
  config.control_point_spacing = 0.20;
  config.sample_spacing = 0.04;
  config.lambda_collision = 0.0;
  config.lambda_reference = 2.0;
  config.lambda_smooth = 2.0;
  config.allow_goal_adjustment = false;
  BsplinePathOptimizer optimizer(config);
  const std::vector<Vector3d> reference{
    Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0), Vector3d(1.0, 1.0, 0.0)};

  const auto result = optimizer.optimize(emptyMap(), reference);

  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_GT(result.path.size(), reference.size());
  EXPECT_NEAR((result.path.front() - reference.front()).norm(), 0.0, 1e-9);
  EXPECT_NEAR((result.path.back() - reference.back()).norm(), 0.0, 1e-9);
  EXPECT_LT(maximumHeadingChange(result.path), 0.45);
  EXPECT_LT(result.final_cost, result.initial_cost);
}

TEST(BsplinePathOptimizer, SoftRepulsionMovesMiddleAwayFromWall)
{
  GridSnapshot map = emptyMap();
  // A short obstacle row beside the middle of the reference path.  The hard
  // path remains free, but the desired clearance band should push the smooth
  // candidate toward negative Y.
  for (double x = 0.65; x <= 1.35; x += map.resolution) {
    const int index = map.indexAt(Vector3d(x, 0.25, 0.0));
    ASSERT_GE(index, 0);
    map.hard[static_cast<std::size_t>(index)] = 1U;
  }

  BsplinePathOptimizer::Config config;
  config.control_point_spacing = 0.20;
  config.sample_spacing = 0.05;
  config.lambda_smooth = 0.5;
  config.lambda_collision = 120.0;
  config.lambda_reference = 1.0;
  config.clearance_distance = 0.45;
  config.reference_deadband = 0.02;
  config.max_deviation = 0.50;
  BsplinePathOptimizer optimizer(config);
  const std::vector<Vector3d> reference{
    Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0), Vector3d(2.0, 0.0, 0.0)};

  const auto result = optimizer.optimize(map, reference);

  ASSERT_FALSE(result.path.empty());
  double minimum_y = 0.0;
  for (const auto & point : result.path) {minimum_y = std::min(minimum_y, point.y());}
  EXPECT_LT(minimum_y, -0.03);
  EXPECT_EQ(result.hard_collision_samples, 0);
  EXPECT_NEAR(result.collision_cost, 0.0, 1e-5);
  EXPECT_GE(result.minimum_movable_clearance, config.minimum_acceptable_clearance);
}

TEST(BsplinePathOptimizer, NarrowRetryAcceptsHardFreePathWithReducedClearance)
{
  GridSnapshot map = emptyMap();
  for (double x = -0.4; x <= 2.4; x += map.resolution) {
    const int index = map.indexAt(Vector3d(x, 0.20, 0.0));
    ASSERT_GE(index, 0);
    map.hard[static_cast<std::size_t>(index)] = 1U;
  }
  const std::vector<Vector3d> reference{
    Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0), Vector3d(2.0, 0.0, 0.0)};

  BsplinePathOptimizer::Config normal_config;
  normal_config.max_deviation = 0.02;
  normal_config.goal_max_deviation = 0.02;
  normal_config.allow_goal_adjustment = false;
  const auto normal = BsplinePathOptimizer(normal_config).optimize(map, reference);
  ASSERT_FALSE(normal.success);
  EXPECT_EQ(normal.reason, "insufficient_clearance");
  EXPECT_EQ(normal.hard_collision_samples, 0);

  BsplinePathOptimizer::Config narrow_config = normal_config;
  narrow_config.control_point_spacing = 0.10;
  narrow_config.sample_spacing = 0.03;
  narrow_config.minimum_acceptable_clearance = 0.10;
  narrow_config.max_z_deviation = 0.05;
  const auto narrow = BsplinePathOptimizer(narrow_config).optimize(map, reference);
  ASSERT_TRUE(narrow.success) << narrow.reason;
  EXPECT_EQ(narrow.hard_collision_samples, 0);
  EXPECT_GE(
    narrow.minimum_movable_clearance,
    narrow_config.minimum_acceptable_clearance);
}

TEST(BsplinePathOptimizer, ReducedClearanceNeverAllowsHardCollision)
{
  GridSnapshot map = emptyMap();
  for (double x = -0.4; x <= 2.4; x += map.resolution) {
    const int index = map.indexAt(Vector3d(x, 0.0, 0.0));
    ASSERT_GE(index, 0);
    map.hard[static_cast<std::size_t>(index)] = 1U;
  }
  BsplinePathOptimizer::Config config;
  config.minimum_acceptable_clearance = 0.0;
  config.max_deviation = 0.02;
  config.goal_max_deviation = 0.02;
  config.allow_goal_adjustment = false;
  const std::vector<Vector3d> reference{
    Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0), Vector3d(2.0, 0.0, 0.0)};

  const auto result = BsplinePathOptimizer(config).optimize(map, reference);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "hard_collision");
  EXPECT_GT(result.hard_collision_samples, 0);
}

}  // namespace
