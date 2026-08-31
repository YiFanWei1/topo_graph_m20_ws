#include "path_tracking_benchmark/benchmark_math.hpp"

#include <gtest/gtest.h>

using path_tracking_benchmark::Point3;
using path_tracking_benchmark::makeLocalTrajectory;
using path_tracking_benchmark::projectToPath;
using path_tracking_benchmark::transformTrajectory;

TEST(BenchmarkMath, GeneratesAllRequiredTrajectories)
{
  for (const char * type : {"straight", "arc", "s_curve", "right_angle"}) {
    const auto path = makeLocalTrajectory(type, 0.05);
    ASSERT_GT(path.size(), 2U) << type;
    EXPECT_NEAR(path.front().x, 0.0, 1e-9);
    EXPECT_NEAR(path.front().y, 0.0, 1e-9);
  }
  const auto straight = makeLocalTrajectory("straight", 0.05);
  EXPECT_NEAR(straight.back().x, 3.0, 1e-9);
  const auto corner = makeLocalTrajectory("right_angle", 0.05);
  EXPECT_NEAR(corner.back().x, 1.5, 1e-9);
  EXPECT_NEAR(corner.back().y, 1.5, 1e-9);
}

TEST(BenchmarkMath, TransformKeepsStartAndRotatesPath)
{
  const std::vector<Point3> local{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const auto world = transformTrajectory(local, {2.0, 3.0, 4.0}, path_tracking_benchmark::kPi / 2.0);
  ASSERT_EQ(world.size(), 2U);
  EXPECT_NEAR(world.front().x, 2.0, 1e-9);
  EXPECT_NEAR(world.back().x, 2.0, 1e-9);
  EXPECT_NEAR(world.back().y, 4.0, 1e-9);
  EXPECT_NEAR(world.back().z, 4.0, 1e-9);
}

TEST(BenchmarkMath, ProjectionReportsCrossTrackAndYaw)
{
  const std::vector<Point3> path{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  const auto projection = projectToPath({1.0, 0.2, 0.0}, path);
  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.point.x, 1.0, 1e-9);
  EXPECT_NEAR(projection.distance_xy, 0.2, 1e-9);
  EXPECT_NEAR(projection.signed_lateral, 0.2, 1e-9);
  EXPECT_NEAR(projection.yaw, 0.0, 1e-9);
}
