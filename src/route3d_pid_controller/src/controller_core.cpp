#include "route3d_pid_controller/controller_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace route3d_pid_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumDt = 1.0e-4;
constexpr double kMinimumSegmentLength = 1.0e-9;

double clamp(const double value, const double limit)
{
  return std::clamp(value, -limit, limit);
}

double distance(const Pose2d & pose, const Waypoint & waypoint)
{
  return std::hypot(pose.x - waypoint.x, pose.y - waypoint.y);
}

void validateFinite(const double value, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

}  // namespace

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

PidAxis::PidAxis(PidAxisConfig config)
: config_(std::move(config))
{
  if (config_.integral_limit < 0.0 || config_.output_limit < 0.0 ||
    config_.integral_separation < 0.0 || config_.derivative_filter_tau < 0.0)
  {
    throw std::invalid_argument("PID limits and filter time must be non-negative");
  }
}

double PidAxis::update(const double error, const double dt)
{
  validateFinite(error, "PID error");
  if (!std::isfinite(dt) || dt < kMinimumDt) {
    throw std::invalid_argument("PID dt must be finite and positive");
  }

  if (std::abs(error) <= config_.integral_separation) {
    integral_ = std::clamp(
      integral_ + error * dt, -config_.integral_limit, config_.integral_limit);
  } else {
    integral_ = 0.0;
  }

  const double raw_derivative = initialized_ ? (error - previous_error_) / dt : 0.0;
  const double alpha = config_.derivative_filter_tau <= 0.0 ? 1.0 :
    dt / (config_.derivative_filter_tau + dt);
  filtered_derivative_ += alpha * (raw_derivative - filtered_derivative_);
  previous_error_ = error;
  initialized_ = true;

  return clamp(
    config_.kp * error + config_.ki * integral_ + config_.kd * filtered_derivative_,
    config_.output_limit);
}

void PidAxis::reset()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  filtered_derivative_ = 0.0;
  initialized_ = false;
}

RouteTracker::RouteTracker(TrackerConfig config)
: config_(std::move(config)),
  longitudinal_pid_(config_.longitudinal_pid),
  lateral_pid_(config_.lateral_pid),
  yaw_pid_(config_.yaw_pid)
{
  if (config_.lookahead_distance_m <= 0.0 || config_.goal_yaw_tolerance_rad < 0.0 ||
    config_.corner_slowdown_distance_m < 0.0 || config_.corner_speed_mps < 0.0 ||
    config_.braking_deceleration_mps2 <= 0.0 || config_.maximum_vx_mps < 0.0 ||
    config_.maximum_vy_mps < 0.0 || config_.maximum_wz_radps < 0.0 ||
    config_.maximum_linear_acceleration_mps2 <= 0.0 ||
    config_.maximum_yaw_acceleration_radps2 <= 0.0 ||
    config_.projection_backtrack_m < 0.0 ||
    config_.adjustment_entry_distance_m <= 0.0 ||
    config_.adjustment_route_goal_position_tolerance_m <= 0.0 ||
    config_.adjustment_route_goal_yaw_tolerance_rad < 0.0 ||
    config_.adjustment_maximum_vx_mps < 0.0 || config_.adjustment_maximum_vy_mps < 0.0 ||
    config_.adjustment_maximum_wz_radps < 0.0 ||
    config_.stop_translation_yaw_error_rad <= config_.full_speed_yaw_error_rad)
  {
    throw std::invalid_argument("invalid route tracker configuration");
  }
}

void RouteTracker::setTask(TrackingTask task)
{
  if (task.waypoints.empty()) {
    throw std::invalid_argument("tracking task must contain at least one waypoint");
  }
  if (task.endpoint_tolerance_m < 0.0 || task.maximum_speed_mps < 0.0) {
    throw std::invalid_argument("task tolerances and speed must be non-negative");
  }
  for (const auto & waypoint : task.waypoints) {
    validateFinite(waypoint.x, "waypoint x");
    validateFinite(waypoint.y, "waypoint y");
    validateFinite(waypoint.yaw, "waypoint yaw");
    if (waypoint.pass_radius_m < 0.0) {
      throw std::invalid_argument("waypoint pass radius must be non-negative");
    }
  }

  task_ = std::move(task);
  cumulative_lengths_.assign(task_.waypoints.size(), 0.0);
  for (std::size_t index = 1U; index < task_.waypoints.size(); ++index) {
    const double segment_length = std::hypot(
      task_.waypoints[index].x - task_.waypoints[index - 1U].x,
      task_.waypoints[index].y - task_.waypoints[index - 1U].y);
    cumulative_lengths_[index] = cumulative_lengths_[index - 1U] + segment_length;
  }
  progress_m_ = 0.0;
  active_segment_ = 0U;
  gate_index_ = nextGateAfter(0U);
  adjusting_ = false;
  stopAndResetControllers();
}

std::size_t RouteTracker::nextGateAfter(const std::size_t index) const
{
  const std::size_t last = task_.waypoints.size() - 1U;
  for (std::size_t candidate = index + 1U; candidate < last; ++candidate) {
    if (task_.waypoints[candidate].must_pass_through) {
      return candidate;
    }
  }
  return last;
}

Pose2d RouteTracker::sampleAt(const double arc_length) const
{
  if (task_.waypoints.size() == 1U) {
    const auto & only = task_.waypoints.front();
    return {only.x, only.y, only.yaw};
  }
  const double bounded = std::clamp(arc_length, 0.0, cumulative_lengths_.back());
  auto upper = std::upper_bound(cumulative_lengths_.begin(), cumulative_lengths_.end(), bounded);
  std::size_t second = upper == cumulative_lengths_.end() ?
    cumulative_lengths_.size() - 1U : static_cast<std::size_t>(upper - cumulative_lengths_.begin());
  second = std::max<std::size_t>(second, 1U);
  const std::size_t first = second - 1U;
  const double length = cumulative_lengths_[second] - cumulative_lengths_[first];
  const double ratio = length > kMinimumSegmentLength ?
    (bounded - cumulative_lengths_[first]) / length : 0.0;
  const auto & a = task_.waypoints[first];
  const auto & b = task_.waypoints[second];
  const double yaw_delta = normalizeAngle(b.yaw - a.yaw);
  return {
    a.x + ratio * (b.x - a.x),
    a.y + ratio * (b.y - a.y),
    normalizeAngle(a.yaw + ratio * yaw_delta)};
}

std::vector<Pose2d> RouteTracker::sampleRemainingPath(
  const double spacing_m, const std::size_t maximum_samples) const
{
  if (!hasTask() || !std::isfinite(spacing_m) || spacing_m <= 0.0 || maximum_samples == 0U) {
    throw std::invalid_argument("path collision sampling requires a task, positive spacing and samples");
  }
  std::vector<Pose2d> result;
  result.reserve(maximum_samples);
  const double total_length = cumulative_lengths_.back();
  for (std::size_t index = 0U; index < maximum_samples; ++index) {
    const double arc = std::min(
      progress_m_ + spacing_m * static_cast<double>(index), total_length);
    Pose2d pose = sampleAt(arc);
    pose.yaw = tangentYawAt(arc);
    result.push_back(pose);
    if (arc >= total_length) {
      break;
    }
  }
  return result;
}

bool RouteTracker::requiresInPlaceRotation(const Pose2d & robot) const
{
  if (!hasTask()) {
    return false;
  }
  validateFinite(robot.x, "robot x");
  validateFinite(robot.y, "robot y");
  validateFinite(robot.yaw, "robot yaw");

  const auto & goal = task_.waypoints.back();
  if (adjusting_) {
    if (!task_.align_goal_yaw) {
      return false;
    }
    const double cosine = std::cos(robot.yaw);
    const double sine = std::sin(robot.yaw);
    const double goal_dx = goal.x - robot.x;
    const double goal_dy = goal.y - robot.y;
    const double local_x = cosine * goal_dx + sine * goal_dy;
    const double local_y = -sine * goal_dx + cosine * goal_dy;
    const double position_tolerance = task_.is_route_goal ?
      config_.adjustment_route_goal_position_tolerance_m :
      std::clamp(task_.endpoint_tolerance_m > 0.01 ? task_.endpoint_tolerance_m : 0.20, 0.01, 0.31);
    const double yaw_tolerance = task_.is_route_goal ?
      config_.adjustment_route_goal_yaw_tolerance_rad :
      std::min(task_.endpoint_tolerance_m > 0.01 ? task_.endpoint_tolerance_m : 0.20, 0.21);
    return std::abs(local_x) <= position_tolerance &&
           std::abs(local_y) <= position_tolerance &&
           std::abs(normalizeAngle(goal.yaw - robot.yaw)) > yaw_tolerance;
  }

  const double desired_arc = std::min(
    progress_m_ + config_.lookahead_distance_m, cumulative_lengths_[gate_index_]);
  const Pose2d lookahead = sampleAt(desired_arc);
  const double target_dx = lookahead.x - robot.x;
  const double target_dy = lookahead.y - robot.y;
  const double target_distance = std::hypot(target_dx, target_dy);
  double target_heading = target_distance > 0.03 ?
    std::atan2(target_dy, target_dx) : tangentYawAt(desired_arc);
  if (task_.reverse_motion) {
    target_heading = normalizeAngle(target_heading + kPi);
  }
  return std::abs(normalizeAngle(target_heading - robot.yaw)) >=
         config_.stop_translation_yaw_error_rad;
}

double RouteTracker::tangentYawAt(const double arc_length) const
{
  if (task_.waypoints.size() < 2U) {
    return task_.waypoints.front().yaw;
  }
  const double bounded = std::clamp(arc_length, 0.0, cumulative_lengths_.back());
  auto upper = std::upper_bound(cumulative_lengths_.begin(), cumulative_lengths_.end(), bounded);
  std::size_t second = upper == cumulative_lengths_.end() ?
    cumulative_lengths_.size() - 1U : static_cast<std::size_t>(upper - cumulative_lengths_.begin());
  second = std::max<std::size_t>(second, 1U);
  std::size_t first = second - 1U;
  while (second < task_.waypoints.size() &&
    cumulative_lengths_[second] - cumulative_lengths_[first] <= kMinimumSegmentLength)
  {
    ++second;
  }
  if (second >= task_.waypoints.size()) {
    return task_.waypoints.back().yaw;
  }
  return std::atan2(
    task_.waypoints[second].y - task_.waypoints[first].y,
    task_.waypoints[second].x - task_.waypoints[first].x);
}

void RouteTracker::updateProgress(const Pose2d & robot)
{
  if (task_.waypoints.size() < 2U) {
    progress_m_ = 0.0;
    return;
  }
  const std::size_t last_segment = std::max<std::size_t>(gate_index_, 1U) - 1U;
  const std::size_t first_segment = active_segment_ > 0U ? active_segment_ - 1U : 0U;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  double best_arc = progress_m_;
  std::size_t best_segment = active_segment_;

  for (std::size_t segment = first_segment;
    segment <= last_segment && segment + 1U < task_.waypoints.size(); ++segment)
  {
    const auto & a = task_.waypoints[segment];
    const auto & b = task_.waypoints[segment + 1U];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    const double ratio = length_sq > kMinimumSegmentLength ?
      std::clamp(((robot.x - a.x) * dx + (robot.y - a.y) * dy) / length_sq, 0.0, 1.0) :
      0.0;
    const double projected_x = a.x + ratio * dx;
    const double projected_y = a.y + ratio * dy;
    const double error_x = robot.x - projected_x;
    const double error_y = robot.y - projected_y;
    const double distance_sq = error_x * error_x + error_y * error_y;
    const double arc = cumulative_lengths_[segment] + ratio * std::sqrt(length_sq);
    if (arc + config_.projection_backtrack_m >= progress_m_ && distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_arc = arc;
      best_segment = segment;
    }
  }
  progress_m_ = std::max(progress_m_, best_arc);
  active_segment_ = best_segment;
}

VelocityCommand RouteTracker::limitAcceleration(
  const VelocityCommand & desired, const double dt)
{
  const double linear_delta = config_.maximum_linear_acceleration_mps2 * dt;
  const double yaw_delta = config_.maximum_yaw_acceleration_radps2 * dt;
  VelocityCommand limited;
  limited.vx = std::clamp(
    desired.vx, previous_command_.vx - linear_delta, previous_command_.vx + linear_delta);
  limited.vy = std::clamp(
    desired.vy, previous_command_.vy - linear_delta, previous_command_.vy + linear_delta);
  limited.wz = std::clamp(
    desired.wz, previous_command_.wz - yaw_delta, previous_command_.wz + yaw_delta);
  previous_command_ = limited;
  return limited;
}

TrackingOutput RouteTracker::update(const Pose2d & robot, const double dt)
{
  if (!hasTask()) {
    throw std::logic_error("route tracker has no task");
  }
  validateFinite(robot.x, "robot x");
  validateFinite(robot.y, "robot y");
  validateFinite(robot.yaw, "robot yaw");
  if (!std::isfinite(dt) || dt < kMinimumDt) {
    throw std::invalid_argument("tracker dt must be finite and positive");
  }

  const std::size_t last = task_.waypoints.size() - 1U;
  while (gate_index_ < last && task_.waypoints[gate_index_].must_pass_through &&
    distance(robot, task_.waypoints[gate_index_]) <=
    task_.waypoints[gate_index_].pass_radius_m)
  {
    progress_m_ = std::max(progress_m_, cumulative_lengths_[gate_index_]);
    gate_index_ = nextGateAfter(gate_index_);
  }
  updateProgress(robot);

  const double total_length = cumulative_lengths_.back();
  const auto & goal = task_.waypoints.back();
  TrackingOutput output;
  output.progress_m = progress_m_;
  output.remaining_m = std::max(0.0, total_length - progress_m_);
  output.goal_distance_m = distance(robot, goal);
  output.active_gate_index = gate_index_;
  output.gate_is_mandatory = gate_index_ < last &&
    task_.waypoints[gate_index_].must_pass_through;

  const double progress_finish_window = std::max(task_.endpoint_tolerance_m, 0.25);
  const bool progressed_to_goal = total_length <= progress_finish_window ||
    progress_m_ >= total_length - progress_finish_window;
  const double final_yaw_error = normalizeAngle(goal.yaw - robot.yaw);
  if (!adjusting_ && progressed_to_goal &&
    output.goal_distance_m <= config_.adjustment_entry_distance_m)
  {
    adjusting_ = true;
    longitudinal_pid_.reset();
    lateral_pid_.reset();
    yaw_pid_.reset();
    previous_command_ = {};
  }
  if (adjusting_) {
    output.lookahead = {goal.x, goal.y, goal.yaw};
    output.yaw_error_rad = final_yaw_error;
    output.adjusting = true;
    const double cosine = std::cos(robot.yaw);
    const double sine = std::sin(robot.yaw);
    const double goal_dx = goal.x - robot.x;
    const double goal_dy = goal.y - robot.y;
    const double local_x = cosine * goal_dx + sine * goal_dy;
    const double local_y = -sine * goal_dx + cosine * goal_dy;
    const double position_tolerance = task_.is_route_goal ?
      config_.adjustment_route_goal_position_tolerance_m :
      std::clamp(task_.endpoint_tolerance_m > 0.01 ? task_.endpoint_tolerance_m : 0.20, 0.01, 0.31);
    const double yaw_tolerance = task_.is_route_goal ?
      config_.adjustment_route_goal_yaw_tolerance_rad :
      std::min(task_.endpoint_tolerance_m > 0.01 ? task_.endpoint_tolerance_m : 0.20, 0.21);
    if (std::abs(local_x) <= position_tolerance && std::abs(local_y) <= position_tolerance &&
      (!task_.align_goal_yaw || std::abs(final_yaw_error) <= yaw_tolerance))
    {
      output.reached = true;
      stopAndResetControllers();
      return output;
    }
    VelocityCommand adjustment;
    adjustment.vx = std::abs(local_x) <= position_tolerance ? 0.0 : clamp(
      longitudinal_pid_.update(local_x, dt), config_.adjustment_maximum_vx_mps);
    adjustment.vy = std::abs(local_y) <= position_tolerance ? 0.0 : clamp(
      lateral_pid_.update(local_y, dt), config_.adjustment_maximum_vy_mps);
    if (task_.align_goal_yaw) {
      adjustment.wz = clamp(
        yaw_pid_.update(final_yaw_error, dt), config_.adjustment_maximum_wz_radps);
    }
    output.command = limitAcceleration(adjustment, dt);
    return output;
  }

  const double desired_arc = std::min(
    progress_m_ + config_.lookahead_distance_m, cumulative_lengths_[gate_index_]);
  output.lookahead = sampleAt(desired_arc);
  const double target_dx = output.lookahead.x - robot.x;
  const double target_dy = output.lookahead.y - robot.y;
  const double target_distance = std::hypot(target_dx, target_dy);
  double target_heading = target_distance > 0.03 ?
    std::atan2(target_dy, target_dx) : tangentYawAt(desired_arc);
  if (task_.reverse_motion) {
    target_heading = normalizeAngle(target_heading + kPi);
  }
  output.yaw_error_rad = normalizeAngle(target_heading - robot.yaw);

  const double cosine = std::cos(robot.yaw);
  const double sine = std::sin(robot.yaw);
  const double local_x = cosine * target_dx + sine * target_dy;
  VelocityCommand desired;
  desired.vx = longitudinal_pid_.update(local_x, dt);
  // During ordinary route tracking the Go2 should steer into the path with
  // yaw instead of translating sideways.  Lateral motion is intentionally
  // reserved for the low-speed final adjustment branch above, where it is
  // needed to converge precisely to the route goal without another approach.
  desired.vy = 0.0;
  desired.wz = yaw_pid_.update(output.yaw_error_rad, dt);

  const double yaw_abs = std::abs(output.yaw_error_rad);
  const double translation_scale = yaw_abs <= config_.full_speed_yaw_error_rad ? 1.0 :
    (yaw_abs >= config_.stop_translation_yaw_error_rad ? 0.0 :
    (config_.stop_translation_yaw_error_rad - yaw_abs) /
    (config_.stop_translation_yaw_error_rad - config_.full_speed_yaw_error_rad));
  desired.vx *= translation_scale;
  desired.vy *= translation_scale;

  double speed_limit = std::min(config_.maximum_vx_mps, task_.maximum_speed_mps);
  // A route goal is completed by the low-speed adjustment phase, whose
  // position tolerance is deliberately tighter than the topology vertex's
  // coarse arrival radius.  Braking against the coarse radius can reduce the
  // tracking speed to zero before the robot reaches the adjustment entry
  // distance (for example, 0.50 m endpoint tolerance versus 0.30 m entry).
  const double braking_tolerance = task_.is_route_goal ?
    config_.adjustment_route_goal_position_tolerance_m : task_.endpoint_tolerance_m;
  speed_limit = std::min(
    speed_limit,
    std::sqrt(2.0 * config_.braking_deceleration_mps2 *
    std::max(output.goal_distance_m - braking_tolerance, 0.0)));
  if (output.gate_is_mandatory &&
    distance(robot, task_.waypoints[gate_index_]) <= config_.corner_slowdown_distance_m)
  {
    speed_limit = std::min(speed_limit, config_.corner_speed_mps);
  }
  const double planar_speed = std::hypot(desired.vx, desired.vy);
  if (planar_speed > speed_limit && planar_speed > 1.0e-9) {
    const double scale = speed_limit / planar_speed;
    desired.vx *= scale;
    desired.vy *= scale;
  }
  desired.vx = clamp(desired.vx, config_.maximum_vx_mps);
  desired.vy = clamp(desired.vy, config_.maximum_vy_mps);
  desired.wz = clamp(desired.wz, config_.maximum_wz_radps);
  output.command = limitAcceleration(desired, dt);
  return output;
}

void RouteTracker::stopAndResetControllers()
{
  longitudinal_pid_.reset();
  lateral_pid_.reset();
  yaw_pid_.reset();
  previous_command_ = {};
}

}  // namespace route3d_pid_controller
