#ifndef ROUTE3D_PID_CONTROLLER__CONTROLLER_CORE_HPP_
#define ROUTE3D_PID_CONTROLLER__CONTROLLER_CORE_HPP_

#include <cstddef>
#include <vector>

namespace route3d_pid_controller
{

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct PidAxisConfig
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double integral_limit{0.2};
  double output_limit{1.0};
  double integral_separation{1.3};
  double derivative_filter_tau{0.04};
};

class PidAxis
{
public:
  explicit PidAxis(PidAxisConfig config = {});

  double update(double error, double dt);
  void reset();

private:
  PidAxisConfig config_;
  double integral_{0.0};
  double previous_error_{0.0};
  double filtered_derivative_{0.0};
  bool initialized_{false};
};

struct Waypoint
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool must_pass_through{false};
  double pass_radius_m{0.45};
};

struct TrackingTask
{
  std::vector<Waypoint> waypoints;
  double endpoint_tolerance_m{0.28};
  double maximum_speed_mps{0.40};
  bool align_goal_yaw{false};
  bool reverse_motion{false};
  bool is_route_goal{false};
};

struct TrackerConfig
{
  double lookahead_distance_m{0.60};
  double goal_yaw_tolerance_rad{0.12};
  double corner_slowdown_distance_m{0.70};
  double corner_speed_mps{0.20};
  double braking_deceleration_mps2{0.50};
  double full_speed_yaw_error_rad{0.17};
  double stop_translation_yaw_error_rad{1.05};
  double maximum_vx_mps{0.80};
  double maximum_vy_mps{0.25};
  double maximum_wz_radps{0.80};
  double minimum_linear_speed_mps{0.20};
  double minimum_yaw_speed_radps{0.25};
  double maximum_linear_acceleration_mps2{0.50};
  double maximum_yaw_acceleration_radps2{1.20};
  double projection_backtrack_m{0.10};
  double adjustment_entry_distance_m{0.30};
  double adjustment_route_goal_position_tolerance_m{0.10};
  double adjustment_route_goal_yaw_tolerance_rad{0.08726646259971647};
  double adjustment_maximum_vx_mps{0.20};
  double adjustment_maximum_vy_mps{0.20};
  double adjustment_maximum_wz_radps{0.90};
  PidAxisConfig longitudinal_pid{1.2, 0.0, 0.05, 0.20, 0.80, 1.3, 0.04};
  PidAxisConfig lateral_pid{1.5, 0.0, 0.04, 0.20, 0.25, 1.3, 0.04};
  PidAxisConfig yaw_pid{2.0, 0.0, 0.05, 0.20, 0.80, 1.3, 0.04};
  PidAxisConfig adjustment_longitudinal_pid{1.2, 0.0, 0.05, 0.20, 0.20, 1.3, 0.04};
  PidAxisConfig adjustment_lateral_pid{1.5, 0.0, 0.04, 0.20, 0.30, 1.3, 0.04};
  PidAxisConfig adjustment_yaw_pid{2.0, 0.0, 0.05, 0.20, 0.50, 1.3, 0.04};
};

struct TrackingOutput
{
  VelocityCommand command;
  Pose2d lookahead;
  double progress_m{0.0};
  double remaining_m{0.0};
  double goal_distance_m{0.0};
  double yaw_error_rad{0.0};
  std::size_t active_gate_index{0U};
  bool gate_is_mandatory{false};
  bool reached{false};
  bool adjusting{false};
};

class RouteTracker
{
public:
  explicit RouteTracker(TrackerConfig config = {});

  void setTask(TrackingTask task);
  TrackingOutput update(const Pose2d & robot, double dt);
  std::vector<Pose2d> sampleRemainingPath(double spacing_m, std::size_t maximum_samples) const;
  bool requiresInPlaceRotation(const Pose2d & robot) const;
  void stopAndResetControllers();
  bool hasTask() const noexcept {return !task_.waypoints.empty();}

private:
  std::size_t nextGateAfter(std::size_t index) const;
  Pose2d sampleAt(double arc_length) const;
  double tangentYawAt(double arc_length) const;
  void updateProgress(const Pose2d & robot);
  VelocityCommand limitAcceleration(const VelocityCommand & desired, double dt);

  TrackerConfig config_;
  TrackingTask task_;
  std::vector<double> cumulative_lengths_;
  double progress_m_{0.0};
  std::size_t active_segment_{0U};
  std::size_t gate_index_{0U};
  bool adjusting_{false};
  PidAxis longitudinal_pid_;
  PidAxis lateral_pid_;
  PidAxis yaw_pid_;
  PidAxis adjustment_longitudinal_pid_;
  PidAxis adjustment_lateral_pid_;
  PidAxis adjustment_yaw_pid_;
  VelocityCommand previous_command_;
};

double normalizeAngle(double angle);

}  // namespace route3d_pid_controller

#endif  // ROUTE3D_PID_CONTROLLER__CONTROLLER_CORE_HPP_
