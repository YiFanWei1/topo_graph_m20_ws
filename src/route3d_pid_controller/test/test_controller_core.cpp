#include <gtest/gtest.h>

#include <cmath>

#include "route3d_pid_controller/controller_core.hpp"
#include "route3d_pid_controller/elevation_collision_checker.hpp"

namespace
{

using route3d_pid_controller::ElevationCollisionChecker;
using route3d_pid_controller::ElevationCollisionConfig;
using route3d_pid_controller::ElevationPoint;
using route3d_pid_controller::PidAxis;
using route3d_pid_controller::PidAxisConfig;
using route3d_pid_controller::Pose2d;
using route3d_pid_controller::RouteTracker;
using route3d_pid_controller::TrackerConfig;
using route3d_pid_controller::TrackingTask;
using route3d_pid_controller::Waypoint;

TrackerConfig fastTestConfig()
{
  TrackerConfig config;
  config.lookahead_distance_m = 0.6;
  config.maximum_linear_acceleration_mps2 = 100.0;
  config.maximum_yaw_acceleration_radps2 = 100.0;
  return config;
}

TEST(PidAxis, SaturatesAndResetsWithoutDerivativeKick)
{
  PidAxis axis(PidAxisConfig{2.0, 1.0, 0.5, 0.2, 0.4, 1.3, 0.04});
  EXPECT_DOUBLE_EQ(axis.update(1.0, 0.02), 0.4);
  axis.reset();
  EXPECT_DOUBLE_EQ(axis.update(0.0, 0.02), 0.0);
}

TEST(RouteTracker, HardCornerClampsLookaheadUntilGateIsEntered)
{
  RouteTracker tracker(fastTestConfig());
  TrackingTask task;
  task.endpoint_tolerance_m = 0.1;
  task.maximum_speed_mps = 0.4;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 1.0, 0.0, 1.57, true, 0.20},
    Waypoint{3, 1.0, 2.0, 1.57, false, 0.45}};
  tracker.setTask(task);

  const auto before = tracker.update(Pose2d{0.75, 0.0, 0.0}, 0.02);
  EXPECT_EQ(before.active_gate_index, 1U);
  EXPECT_TRUE(before.gate_is_mandatory);
  EXPECT_NEAR(before.lookahead.x, 1.0, 1.0e-9);
  EXPECT_NEAR(before.lookahead.y, 0.0, 1.0e-9);

  const auto after = tracker.update(Pose2d{1.0, 0.10, 1.57}, 0.02);
  EXPECT_EQ(after.active_gate_index, 2U);
  EXPECT_FALSE(after.gate_is_mandatory);
  EXPECT_GT(after.lookahead.y, 0.10);
}

TEST(RouteTracker, OrdinaryIntermediatePointDoesNotBecomeGoal)
{
  RouteTracker tracker(fastTestConfig());
  TrackingTask task;
  task.endpoint_tolerance_m = 0.1;
  task.maximum_speed_mps = 0.4;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 1.0, 0.0, 0.0, false, 0.45},
    Waypoint{3, 2.0, 0.0, 0.0, false, 0.45}};
  tracker.setTask(task);

  const auto output = tracker.update(Pose2d{1.0, 0.0, 0.0}, 0.02);
  EXPECT_FALSE(output.reached);
  EXPECT_EQ(output.active_gate_index, 2U);
  EXPECT_GT(output.lookahead.x, 1.0);
}

TEST(RouteTracker, OrdinaryTrackingDoesNotCommandLateralMotion)
{
  RouteTracker tracker(fastTestConfig());
  TrackingTask task;
  task.endpoint_tolerance_m = 0.1;
  task.maximum_speed_mps = 0.8;
  task.waypoints = {
    Waypoint{1, 0.0, 1.0, 0.0, false, 0.45},
    Waypoint{2, 4.0, 1.0, 0.0, false, 0.45}};
  tracker.setTask(task);

  const auto tracking = tracker.update(Pose2d{0.0, 0.0, 0.0}, 0.02);
  EXPECT_FALSE(tracking.adjusting);
  EXPECT_DOUBLE_EQ(tracking.command.vy, 0.0);
  EXPECT_GT(tracking.command.wz, 0.0);
}

TEST(RouteTracker, GoalCanRequireFinalYawAlignment)
{
  RouteTracker tracker(fastTestConfig());
  TrackingTask task;
  task.endpoint_tolerance_m = 0.15;
  task.maximum_speed_mps = 0.4;
  task.align_goal_yaw = true;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 1.0, 0.0, 1.57, false, 0.45}};
  tracker.setTask(task);

  (void)tracker.update(Pose2d{0.9, 0.0, 0.0}, 0.02);
  const auto rotating = tracker.update(Pose2d{1.0, 0.0, 0.0}, 0.02);
  EXPECT_FALSE(rotating.reached);
  EXPECT_DOUBLE_EQ(rotating.command.vx, 0.0);
  EXPECT_GT(rotating.command.wz, 0.0);

  const auto reached = tracker.update(Pose2d{1.0, 0.0, 1.57}, 0.02);
  EXPECT_TRUE(reached.reached);
}

TEST(RouteTracker, UsesReferenceStyleLowSpeedFinalAdjustmentWithLateralMotion)
{
  auto config = fastTestConfig();
  config.adjustment_entry_distance_m = 0.30;
  config.adjustment_route_goal_position_tolerance_m = 0.10;
  config.adjustment_route_goal_yaw_tolerance_rad = 0.15;
  config.adjustment_maximum_vx_mps = 0.20;
  config.adjustment_maximum_vy_mps = 0.20;
  config.adjustment_maximum_wz_radps = 0.90;
  RouteTracker tracker(config);
  TrackingTask task;
  task.endpoint_tolerance_m = 0.5;
  task.maximum_speed_mps = 0.8;
  task.align_goal_yaw = true;
  task.is_route_goal = true;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 1.0, 0.0, 1.57, false, 0.45}};
  tracker.setTask(task);

  const auto adjusting = tracker.update(Pose2d{0.80, -0.15, 0.0}, 0.02);
  EXPECT_TRUE(adjusting.adjusting);
  EXPECT_FALSE(adjusting.reached);
  EXPECT_GT(adjusting.command.vx, 0.0);
  EXPECT_GT(adjusting.command.vy, 0.0);
  EXPECT_LE(std::abs(adjusting.command.vx), 0.20);
  EXPECT_LE(std::abs(adjusting.command.vy), 0.20);
  EXPECT_LE(std::abs(adjusting.command.wz), 0.90);

  const auto reached = tracker.update(Pose2d{0.95, -0.05, 1.57}, 0.02);
  EXPECT_TRUE(reached.adjusting);
  EXPECT_TRUE(reached.reached);
}

TEST(RouteTracker, FinalPoseUsesTenCentimetresFiveDegreesAndMinimumEffectiveSpeeds)
{
  auto config = fastTestConfig();
  config.adjustment_entry_distance_m = 0.30;
  config.adjustment_route_goal_position_tolerance_m = 0.10;
  config.adjustment_route_goal_yaw_tolerance_rad = 0.08726646259971647;
  RouteTracker tracker(config);
  TrackingTask task;
  task.endpoint_tolerance_m = 0.50;
  task.maximum_speed_mps = 0.80;
  task.align_goal_yaw = true;
  task.is_route_goal = true;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 1.0, 0.0, 0.0, false, 0.45}};
  tracker.setTask(task);

  const auto outside_position = tracker.update(Pose2d{0.91, -0.09, 0.0}, 0.02);
  EXPECT_TRUE(outside_position.adjusting);
  EXPECT_FALSE(outside_position.reached);
  EXPECT_GE(std::hypot(outside_position.command.vx, outside_position.command.vy), 0.20);

  constexpr double six_degrees = 0.10471975511965977;
  const auto outside_yaw = tracker.update(Pose2d{1.0, 0.0, -six_degrees}, 0.02);
  EXPECT_TRUE(outside_yaw.adjusting);
  EXPECT_FALSE(outside_yaw.reached);
  EXPECT_DOUBLE_EQ(std::hypot(outside_yaw.command.vx, outside_yaw.command.vy), 0.0);
  EXPECT_GE(std::abs(outside_yaw.command.wz), 0.25);

  const auto inside = tracker.update(Pose2d{0.94, -0.06, -0.06981317007977318}, 0.02);
  EXPECT_TRUE(inside.adjusting);
  EXPECT_TRUE(inside.reached);
  EXPECT_DOUBLE_EQ(std::hypot(inside.command.vx, inside.command.vy), 0.0);
  EXPECT_DOUBLE_EQ(inside.command.wz, 0.0);
}

TEST(RouteTracker, RouteGoalDoesNotStopBetweenCoarseToleranceAndAdjustmentEntry)
{
  auto config = fastTestConfig();
  config.adjustment_entry_distance_m = 0.30;
  config.adjustment_route_goal_position_tolerance_m = 0.10;
  RouteTracker tracker(config);
  TrackingTask task;
  task.endpoint_tolerance_m = 0.50;
  task.maximum_speed_mps = 0.80;
  task.is_route_goal = true;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 1.0, 0.0, 0.0, false, 0.45}};
  tracker.setTask(task);

  const auto tracking = tracker.update(Pose2d{0.60, 0.0, 0.0}, 0.02);
  EXPECT_FALSE(tracking.adjusting);
  EXPECT_FALSE(tracking.reached);
  EXPECT_GT(tracking.command.vx, 0.0);
}

TEST(RouteTracker, ControllerSwitchBoundaryDoesNotStopBeforeAdjustmentEntry)
{
  auto config = fastTestConfig();
  config.adjustment_entry_distance_m = 0.30;
  RouteTracker tracker(config);
  TrackingTask task;
  task.endpoint_tolerance_m = 0.45;
  task.maximum_speed_mps = 0.80;
  task.is_route_goal = false;
  task.align_goal_yaw = false;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{4, 1.0, 0.0, 0.0, false, 0.45}};
  tracker.setTask(task);

  const auto tracking = tracker.update(Pose2d{0.675, 0.0, 0.0}, 0.02);
  EXPECT_NEAR(tracking.goal_distance_m, 0.325, 1.0e-9);
  EXPECT_FALSE(tracking.adjusting);
  EXPECT_FALSE(tracking.reached);
  EXPECT_GT(tracking.command.vx, 0.0);
}

TEST(RouteTracker, CapsPlanarTrackingSpeedAtEightTenths)
{
  auto config = fastTestConfig();
  config.maximum_vx_mps = 0.8;
  config.maximum_vy_mps = 0.8;
  config.longitudinal_pid.output_limit = 2.0;
  config.lateral_pid.output_limit = 2.0;
  RouteTracker tracker(config);
  TrackingTask task;
  task.maximum_speed_mps = 2.0;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 4.0, 4.0, 0.0, false, 0.45}};
  tracker.setTask(task);

  const auto output = tracker.update(Pose2d{0.0, 0.0, 0.7853981633974483}, 0.02);
  EXPECT_LE(std::hypot(output.command.vx, output.command.vy), 0.8 + 1.0e-9);
}

TEST(RouteTracker, ReverseTaskCommandsNegativeLongitudinalVelocity)
{
  RouteTracker tracker(fastTestConfig());
  TrackingTask task;
  task.maximum_speed_mps = 0.4;
  task.reverse_motion = true;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 1.0, 0.0, 0.0, false, 0.45}};
  tracker.setTask(task);

  const auto output = tracker.update(Pose2d{0.0, 0.0, 3.14159265358979323846}, 0.02);
  EXPECT_LT(output.command.vx, 0.0);
}

TEST(RouteTracker, SamplesRemainingPathAtCollisionResolution)
{
  RouteTracker tracker(fastTestConfig());
  TrackingTask task;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 2.0, 0.0, 0.0, false, 0.45}};
  tracker.setTask(task);
  (void)tracker.update(Pose2d{0.45, 0.0, 0.0}, 0.02);

  const auto samples = tracker.sampleRemainingPath(0.4, 15U);
  ASSERT_EQ(samples.size(), 5U);
  EXPECT_NEAR(samples.front().x, 0.45, 1.0e-9);
  EXPECT_NEAR(samples[1].x, 0.85, 1.0e-9);
  EXPECT_NEAR(samples.back().x, 2.0, 1.0e-9);
}

TEST(RouteTracker, DetectsOnlyYawStoppedTrackingAsInPlaceRotation)
{
  RouteTracker tracker(fastTestConfig());
  TrackingTask task;
  task.waypoints = {
    Waypoint{1, 0.0, 0.0, 0.0, false, 0.45},
    Waypoint{2, 2.0, 0.0, 0.0, false, 0.45}};
  tracker.setTask(task);

  EXPECT_FALSE(tracker.requiresInPlaceRotation(Pose2d{0.0, 0.0, 0.0}));
  EXPECT_TRUE(tracker.requiresInPlaceRotation(
    Pose2d{0.0, 0.0, 3.14159265358979323846 / 2.0}));
}

TEST(ElevationCollisionChecker, FlatOrEmptyMapIsSafe)
{
  ElevationCollisionConfig config;
  config.minimum_obstacle_height_m = -0.50;
  ElevationCollisionChecker checker(config);
  checker.update({
    ElevationPoint{0.35, 0.0, -0.40},
    ElevationPoint{0.55, 0.1, -0.40}});

  EXPECT_TRUE(checker.ready());
  EXPECT_EQ(checker.roughCells(), 0U);
  EXPECT_FALSE(checker.checkTrajectory({Pose2d{0.0, 0.0, 0.0}}).collision());
}

TEST(ElevationCollisionChecker, SparseFlatReturnsDoNotCreateUnknownCellEdges)
{
  ElevationCollisionChecker checker;
  checker.update({
    ElevationPoint{0.35, 0.00, 0.02},
    ElevationPoint{0.65, 0.20, 0.01},
    ElevationPoint{0.95, -0.20, 0.03}});

  EXPECT_EQ(checker.occupiedCells(), 3U);
  EXPECT_EQ(checker.roughCells(), 0U);
  EXPECT_FALSE(checker.checkTrajectory({Pose2d{0.0, 0.0, 0.0}}).collision());
}

TEST(ElevationCollisionChecker, HeightDiscontinuityOnSweptFootprintStops)
{
  ElevationCollisionChecker checker;
  checker.update({
    ElevationPoint{0.35, 0.0, 0.00},
    ElevationPoint{0.45, 0.0, 0.30}});

  EXPECT_GT(checker.roughCells(), 0U);
  const auto result = checker.checkTrajectory({Pose2d{0.0, 0.0, 0.0}});
  EXPECT_TRUE(result.collision());
  EXPECT_GT(result.collision_samples, 0U);
}

TEST(ElevationCollisionChecker, ObstacleOutsideSweptRouteIsIgnored)
{
  ElevationCollisionChecker checker;
  checker.update({
    ElevationPoint{-1.0, 1.0, 0.00},
    ElevationPoint{-0.9, 1.0, 0.30}});

  EXPECT_GT(checker.roughCells(), 0U);
  EXPECT_FALSE(checker.checkTrajectory({Pose2d{0.0, 0.0, 0.0}}).collision());
}

}  // namespace
