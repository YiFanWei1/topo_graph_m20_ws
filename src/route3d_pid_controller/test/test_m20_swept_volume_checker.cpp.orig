#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "route3d_pid_controller/m20_swept_volume_checker.hpp"

using route3d_pid_controller::M20SweptVolumeChecker;
using route3d_pid_controller::M20SweptVolumeConfig;
using route3d_pid_controller::Point3d;
using route3d_pid_controller::Pose3d;
using route3d_pid_controller::SweptVolumePose;

TEST(M20SweptVolumeChecker, UsesM20BodyAndSafetyDimensions)
{
  M20SweptVolumeChecker checker;
  EXPECT_TRUE(checker.isSelfPoint({0.41, 0.215, -0.285}));
  EXPECT_TRUE(checker.isSelfPoint({0.0, 0.0, 0.01}));
  EXPECT_FALSE(checker.isSelfPoint({0.0, 0.0, 0.286}));
  EXPECT_FALSE(checker.isSelfPoint({0.411, 0.0, 0.0}));
  const auto & config = checker.config();
  EXPECT_DOUBLE_EQ(config.detection_half_length_m * 2.0, 0.92);
  EXPECT_DOUBLE_EQ(config.detection_half_width_m * 2.0, 0.50);
  EXPECT_DOUBLE_EQ(config.detection_max_z_m - config.detection_min_z_m, 0.57);
}

TEST(M20SweptVolumeChecker, DetectsObstacleAlongPitchedPath)
{
  M20SweptVolumeChecker checker;
  const double pitch = std::atan2(1.0, 2.0);
  const std::vector<SweptVolumePose> path{{{2.0, 0.0, 1.57}, 0.0, pitch, 2.0}};
  const auto blocked = checker.check({{2.0, 0.0, 1.57}}, path);
  EXPECT_TRUE(blocked.collision());
  EXPECT_EQ(blocked.collision_points, 1U);
  EXPECT_DOUBLE_EQ(blocked.nearest_hit_distance_m, 2.0);
  const auto clear = checker.check({{2.0, 0.40, 1.57}}, path);
  EXPECT_FALSE(clear.collision());
}

TEST(M20SweptVolumeChecker, ExcludesPathSurfaceAndFirstTenCentimetres)
{
  M20SweptVolumeConfig config;
  config.detection_min_z_m = -0.57;
  config.detection_max_z_m = 0.0;
  M20SweptVolumeChecker checker(config);
  const std::vector<SweptVolumePose> path{{{0.0, 0.0, 0.57}, 0.0, 0.0, 0.0}};
  EXPECT_FALSE(checker.check({{0.0, 0.0, 0.0}}, path).collision());
  EXPECT_FALSE(checker.check({{0.0, 0.0, 0.10}}, path).collision());
  EXPECT_TRUE(checker.check({{0.0, 0.0, 0.11}}, path).collision());
}

TEST(M20SweptVolumeChecker, ExcludesSupportedStairTreadButDetectsObstacleAboveIt)
{
  M20SweptVolumeConfig config;
  config.detection_min_z_m = -0.57;
  config.detection_max_z_m = 0.0;
  M20SweptVolumeChecker checker(config);
  const std::vector<SweptVolumePose> path{{{0.0, 0.0, 0.57}, 0.0, 0.0, 0.0}};
  const std::vector<Point3d> tread{
    {-0.05, -0.05, 0.15}, {0.05, -0.05, 0.15},
    {-0.05, 0.05, 0.15}, {0.05, 0.05, 0.15}};
  const auto clear = checker.check(tread, path);
  EXPECT_FALSE(clear.collision());
  EXPECT_EQ(clear.surface_points_excluded, tread.size());
  auto with_obstacle = tread;
  with_obstacle.push_back({0.0, 0.0, 0.26});
  const auto blocked = checker.check(with_obstacle, path);
  EXPECT_TRUE(blocked.collision());
  EXPECT_EQ(blocked.collision_points, 1U);
}

TEST(M20SweptVolumeChecker, DoesNotTreatVerticalOrLinearReturnsAsStepSurface)
{
  M20SweptVolumeChecker checker;
  const std::vector<SweptVolumePose> path{{{0.0, 0.0, 0.57}, 0.0, 0.0, 0.0}};
  const std::vector<Point3d> obstacle{
    {0.0, -0.15, 0.30}, {0.0, -0.05, 0.30},
    {0.0, 0.05, 0.30}, {0.0, 0.15, 0.30}};
  const auto blocked = checker.check(obstacle, path);
  EXPECT_TRUE(blocked.collision());
  EXPECT_EQ(blocked.collision_points, obstacle.size());
}

TEST(M20SweptVolumeChecker, TransformsBodyCloudWithFullQuaternion)
{
  constexpr double root_half = 0.7071067811865476;
  Pose3d pose;
  pose.position = {1.0, 2.0, 3.0};
  pose.qy = root_half;
  pose.qw = root_half;
  const Point3d world = M20SweptVolumeChecker::transformPoint(pose, {1.0, 0.0, 0.0});
  EXPECT_NEAR(world.x, 1.0, 1.0e-12);
  EXPECT_NEAR(world.y, 2.0, 1.0e-12);
  EXPECT_NEAR(world.z, 2.0, 1.0e-12);
}
