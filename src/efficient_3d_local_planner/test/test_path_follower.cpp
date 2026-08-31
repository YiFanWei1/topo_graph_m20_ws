#include "efficient_3d_local_planner/path_follower.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using efficient_3d_local_planner::PathFollowerConfig;
using efficient_3d_local_planner::applyMinimumYawSpeedDeadzone;
using efficient_3d_local_planner::computePathFollowerCommand;
using efficient_3d_local_planner::samplePathForTracking;

TEST(PathFollower, InterpolatesLookaheadAlongPolyline)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 1.0, 0.0)};
  const auto sample = samplePathForTracking(path, Eigen::Vector3d(0.2, 0.1, 0.0), 1.1);
  ASSERT_TRUE(sample.valid);
  EXPECT_NEAR(sample.nearest.x(), 0.2, 1e-6);
  EXPECT_NEAR(sample.lookahead.x(), 1.0, 1e-6);
  EXPECT_NEAR(sample.lookahead.y(), 0.3, 1e-6);
  EXPECT_NEAR(sample.remaining_arc, 1.8, 1e-6);
}

TEST(PathFollower, SelectsCorrectLevelUsingThreeDimensionalProjection)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(2.0, 0.0, 0.0),
    Eigen::Vector3d(2.0, 0.0, 3.0),
    Eigen::Vector3d(0.0, 0.0, 3.0)};
  const auto sample = samplePathForTracking(path, Eigen::Vector3d(1.0, 0.0, 3.0), 0.20);
  ASSERT_TRUE(sample.valid);
  EXPECT_NEAR(sample.nearest.x(), 1.0, 1e-9);
  EXPECT_NEAR(sample.nearest.z(), 3.0, 1e-9);
  EXPECT_NEAR(sample.projection_distance_3d, 0.0, 1e-9);
  EXPECT_NEAR(sample.projected_arc, 6.0, 1e-9);
  EXPECT_NEAR(sample.remaining_arc, 1.0, 1e-9);
  EXPECT_LT(sample.tangent.x(), 0.0);
}

TEST(PathFollower, ProducesBodyFrameForwardVelocity)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)};
  PathFollowerConfig config;
  const auto command = computePathFollowerCommand(
    path, Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, config);
  ASSERT_TRUE(command.valid);
  EXPECT_GT(command.vx, 0.0);
  EXPECT_NEAR(command.vy, 0.0, 1e-9);
  EXPECT_NEAR(command.wz, 0.0, 1e-9);
}

TEST(PathFollower, DisabledMinimumYawSpeedPreservesExistingOutput)
{
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(0.10, 0.10, false, 0.25), 0.10);
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(-0.10, -0.10, false, 0.25), -0.10);
}

TEST(PathFollower, MinimumYawSpeedAppliesInBothDirections)
{
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(0.10, 0.10, true, 0.25), 0.25);
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(-0.10, -0.10, true, 0.25), -0.25);
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(0.40, 0.40, true, 0.25), 0.40);
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(-0.40, -0.40, true, 0.25), -0.40);
}

TEST(PathFollower, MinimumYawSpeedKeepsTrueZeroOutsideDeadzone)
{
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(0.0, 0.0, true, 0.25), 0.0);
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(0.10, 0.0, true, 0.25), 0.0);
}

TEST(PathFollower, MinimumYawSpeedCanCrossZeroWhenDirectionReverses)
{
  // 限幅结果仍带旧的正方向，但已落入死区时先输出零；下一周期再从负向下限起步。
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(0.20, -0.30, true, 0.25), 0.0);
  EXPECT_DOUBLE_EQ(applyMinimumYawSpeedDeadzone(-0.01, -0.30, true, 0.25), -0.25);
}

TEST(PathFollower, TurnsInPlaceWhenHeadingErrorIsLarge)
{
  constexpr double half_pi = 1.5707963267948966;
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)};
  PathFollowerConfig config;
  const auto command = computePathFollowerCommand(
    path, Eigen::Vector3d(0.0, 0.0, 0.0), half_pi, config);
  ASSERT_TRUE(command.valid);
  EXPECT_NEAR(command.vx, 0.0, 1e-9);
  EXPECT_NEAR(command.vy, 0.0, 1e-9);
  EXPECT_LT(command.wz, 0.0);
}

TEST(PathFollower, SuppressesCrossTrackCorrectionDuringLargeHeadingError)
{
  constexpr double half_pi = 1.5707963267948966;
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)};
  PathFollowerConfig config;
  const auto command = computePathFollowerCommand(
    path, Eigen::Vector3d(0.0, 0.5, 0.0), half_pi, config);
  ASSERT_TRUE(command.valid);
  EXPECT_NEAR(command.vx, 0.0, 1e-9);
  EXPECT_NEAR(command.vy, 0.0, 1e-9);
  EXPECT_LT(command.wz, 0.0);
}

TEST(PathFollower, StopsAtLocalPathEnd)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)};
  PathFollowerConfig config;
  const auto command = computePathFollowerCommand(
    path, Eigen::Vector3d(0.95, 0.0, 0.0), 0.0, config);
  ASSERT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
}

TEST(PathFollower, CorrectsLateralErrorBeforeDeclaringEndpointReached)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)};
  PathFollowerConfig config;
  const auto command = computePathFollowerCommand(
    path, Eigen::Vector3d(0.86, 0.10, 0.0), 0.0, config);
  ASSERT_TRUE(command.valid);
  EXPECT_LT(command.sample.remaining_arc, config.finish_distance);
  EXPECT_GT(command.endpoint_distance_3d, config.finish_distance);
  EXPECT_LT(command.vy, 0.0);
}

TEST(PathFollower, UsesThreeDimensionalEndpointDistance)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 1.0)};
  PathFollowerConfig config;
  const auto command = computePathFollowerCommand(
    path, Eigen::Vector3d(1.0, 0.0, 0.8), 0.0, config);
  ASSERT_TRUE(command.valid);
  EXPECT_LT(command.sample.remaining_arc, config.finish_distance);
  EXPECT_NEAR(command.endpoint_distance_3d, 0.20, 1e-9);
  EXPECT_TRUE(command.target_ray_fallback);
}

TEST(PathFollower, UsesRobotToLookaheadRayInsteadOfCurveTangentForYaw)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.2, 0.0, 0.0),
    Eigen::Vector3d(0.2, 1.0, 0.0)};
  PathFollowerConfig config;
  const Eigen::Vector3d robot = Eigen::Vector3d::Zero();
  const auto command = computePathFollowerCommand(path, robot, 0.0, config);
  ASSERT_TRUE(command.valid);
  const Eigen::Vector2d ray = command.sample.lookahead.head<2>() - robot.head<2>();
  const double ray_yaw = std::atan2(ray.y(), ray.x());
  const double tangent_yaw = std::atan2(command.sample.tangent.y(), command.sample.tangent.x());
  EXPECT_NEAR(command.target_yaw, ray_yaw, 1e-9);
  EXPECT_NEAR(command.yaw_error, ray_yaw, 1e-9);
  EXPECT_GT(std::abs(tangent_yaw - ray_yaw), 0.30);
}

TEST(PathFollower, PreservesDirectedObtuseTurnTowardLookahead)
{
  constexpr double pi = 3.14159265358979323846;
  const double target_yaw = 120.0 * pi / 180.0;
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d(2.0 * std::cos(target_yaw), 2.0 * std::sin(target_yaw), 0.0)};
  PathFollowerConfig config;
  const auto command = computePathFollowerCommand(path, Eigen::Vector3d::Zero(), 0.0, config);
  ASSERT_TRUE(command.valid);
  EXPECT_NEAR(command.yaw_error, target_yaw, 1e-9);
  EXPECT_TRUE(command.aligning_in_place);
  EXPECT_GT(command.wz, 0.0);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
}

TEST(PathFollower, ChoosesShortestSignedTurnAcrossAngleWrap)
{
  constexpr double pi = 3.14159265358979323846;
  const double robot_yaw = 170.0 * pi / 180.0;
  const double target_yaw = -170.0 * pi / 180.0;
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d(2.0 * std::cos(target_yaw), 2.0 * std::sin(target_yaw), 0.0)};
  PathFollowerConfig config;
  const auto command = computePathFollowerCommand(
    path, Eigen::Vector3d::Zero(), robot_yaw, config);
  ASSERT_TRUE(command.valid);
  EXPECT_NEAR(command.yaw_error, 20.0 * pi / 180.0, 1e-9);
  EXPECT_GT(command.wz, 0.0);
}

TEST(PathFollower, UsesSixtyDegreeAlignmentThreshold)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)};
  PathFollowerConfig config;
  const auto at_limit = computePathFollowerCommand(
    path, Eigen::Vector3d::Zero(), -config.translation_yaw_limit, config);
  const auto below_limit = computePathFollowerCommand(
    path, Eigen::Vector3d::Zero(), -config.translation_yaw_limit + 1e-3, config);
  ASSERT_TRUE(at_limit.valid);
  ASSERT_TRUE(below_limit.valid);
  EXPECT_TRUE(at_limit.aligning_in_place);
  EXPECT_DOUBLE_EQ(at_limit.vx, 0.0);
  EXPECT_DOUBLE_EQ(at_limit.vy, 0.0);
  EXPECT_FALSE(below_limit.aligning_in_place);
  EXPECT_GT(below_limit.vx, 0.0);
}

TEST(PathFollower, LinearlyReducesTranslationBetweenTenAndSixtyDegrees)
{
  constexpr double kPi = 3.14159265358979323846;
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(10.0, 0.0, 0.0)};
  PathFollowerConfig config;
  const auto at_ten = computePathFollowerCommand(
    path, Eigen::Vector3d::Zero(), -10.0 * kPi / 180.0, config);
  const auto at_thirty_five = computePathFollowerCommand(
    path, Eigen::Vector3d::Zero(), -35.0 * kPi / 180.0, config);
  const auto at_sixty = computePathFollowerCommand(
    path, Eigen::Vector3d::Zero(), -60.0 * kPi / 180.0, config);
  ASSERT_TRUE(at_ten.valid);
  ASSERT_TRUE(at_thirty_five.valid);
  ASSERT_TRUE(at_sixty.valid);
  EXPECT_FALSE(at_ten.aligning_in_place);
  EXPECT_FALSE(at_thirty_five.aligning_in_place);
  EXPECT_TRUE(at_sixty.aligning_in_place);

  const double ten_world_speed = at_ten.vx / std::cos(10.0 * kPi / 180.0);
  const double thirty_five_world_speed =
    at_thirty_five.vx / std::cos(35.0 * kPi / 180.0);
  EXPECT_NEAR(thirty_five_world_speed, 0.5 * ten_world_speed, 1e-9);
  EXPECT_DOUBLE_EQ(at_sixty.vx, 0.0);
  EXPECT_DOUBLE_EQ(at_sixty.vy, 0.0);
}

TEST(PathFollower, FallsBackToTangentWhenLookaheadRayIsTooShort)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)};
  PathFollowerConfig config;
  const auto command = computePathFollowerCommand(
    path, Eigen::Vector3d(1.0, 0.0, 0.0), 0.0, config);
  ASSERT_TRUE(command.valid);
  EXPECT_TRUE(command.target_ray_fallback);
  EXPECT_NEAR(command.target_yaw, 0.0, 1e-9);
}

TEST(PathFollower, NeverRequestsNegativeBodyForwardVelocity)
{
  constexpr double pi = 3.14159265358979323846;
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)};
  PathFollowerConfig config;
  for (int degrees = -180; degrees <= 180; degrees += 5) {
    const auto command = computePathFollowerCommand(
      path, Eigen::Vector3d(0.25, 0.30, 0.0), degrees * pi / 180.0, config);
    ASSERT_TRUE(command.valid);
    EXPECT_GE(command.vx, 0.0) << "heading=" << degrees;
  }
}

TEST(PathFollower, UsesScanLookaheadAndShrinksItForEndpointBraking)
{
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)};
  PathFollowerConfig config;
  config.nominal_speed = 0.65;

  const auto far_command = computePathFollowerCommand(
    path, Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, config);
  const auto corner_command = computePathFollowerCommand(
    path, Eigen::Vector3d(1.80, 0.0, 0.0), 0.0, config);

  ASSERT_TRUE(far_command.valid);
  ASSERT_TRUE(corner_command.valid);
  EXPECT_NEAR(far_command.lookahead_distance, 0.35, 1e-9);
  EXPECT_GT(corner_command.lookahead_distance, config.lookahead_min);
  EXPECT_LT(corner_command.lookahead_distance, far_command.lookahead_distance);
}
