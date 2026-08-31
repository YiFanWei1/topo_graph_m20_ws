#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace efficient_3d_local_planner
{

inline double normalizeFollowerAngle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {angle -= 2.0 * pi;}
  while (angle < -pi) {angle += 2.0 * pi;}
  return angle;
}

// 将连续角速度量化为“0 或不小于 minimum_speed”的输出，用于跨过底盘角速度死区。
// limited_speed 是已经经过角加速度限制的本周期结果，desired_speed 是跟踪器期望方向。
// 换向时不能简单对 limited_speed 做 copysign(max(abs(w), min), w)，否则历史正速度会
// 永远被钳在 +min、无法穿过零点。这里在旧方向速度降入死区后先输出 0，下一周期再
// 按新方向从 minimum_speed 起步，因此正反方向都能可靠切换。
inline double applyMinimumYawSpeedDeadzone(
  const double limited_speed, const double desired_speed,
  const bool enabled, const double minimum_speed)
{
  const double minimum = std::max(0.0, minimum_speed);
  if (!enabled || minimum <= 0.0 || !std::isfinite(limited_speed) ||
    !std::isfinite(desired_speed))
  {
    return limited_speed;
  }

  constexpr double kZeroTolerance = 1e-12;
  const double limited_magnitude = std::abs(limited_speed);
  const double desired_magnitude = std::abs(desired_speed);
  if (limited_magnitude >= minimum) {return limited_speed;}
  if (desired_magnitude <= kZeroTolerance) {return 0.0;}
  if (limited_magnitude <= kZeroTolerance) {
    return std::copysign(minimum, desired_speed);
  }
  if (limited_speed * desired_speed > 0.0) {
    return std::copysign(minimum, desired_speed);
  }
  return 0.0;
}

struct PathTrackingSample
{
  bool valid{false};
  Eigen::Vector3d nearest{Eigen::Vector3d::Zero()};
  Eigen::Vector3d lookahead{Eigen::Vector3d::Zero()};
  Eigen::Vector2d tangent{Eigen::Vector2d::UnitX()};
  double projected_arc{0.0};
  double remaining_arc{0.0};
  double projection_distance_3d{std::numeric_limits<double>::infinity()};
};

inline std::vector<double> followerCumulativeDistance3D(
  const std::vector<Eigen::Vector3d> & path)
{
  std::vector<double> distance(path.size(), 0.0);
  for (std::size_t i = 1U; i < path.size(); ++i) {
    distance[i] = distance[i - 1U] + (path[i] - path[i - 1U]).norm();
  }
  return distance;
}

inline Eigen::Vector3d followerInterpolate(
  const std::vector<Eigen::Vector3d> & path, const std::vector<double> & cumulative,
  const double requested_arc, std::size_t * segment = nullptr)
{
  if (path.empty()) {return Eigen::Vector3d::Zero();}
  if (path.size() == 1U || cumulative.size() != path.size()) {return path.front();}
  const double arc = std::clamp(requested_arc, 0.0, cumulative.back());
  auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), arc);
  std::size_t index = upper == cumulative.begin() ? 0U :
    static_cast<std::size_t>(std::distance(cumulative.begin(), upper) - 1);
  index = std::min(index, path.size() - 2U);
  while (index + 1U < path.size() && cumulative[index + 1U] - cumulative[index] < 1e-9) {
    ++index;
  }
  if (index + 1U >= path.size()) {
    if (segment) {*segment = path.size() - 2U;}
    return path.back();
  }
  if (segment) {*segment = index;}
  const double length = cumulative[index + 1U] - cumulative[index];
  const double ratio = length > 1e-9 ? (arc - cumulative[index]) / length : 0.0;
  return path[index] + std::clamp(ratio, 0.0, 1.0) * (path[index + 1U] - path[index]);
}

inline PathTrackingSample samplePathForTracking(
  const std::vector<Eigen::Vector3d> & path, const Eigen::Vector3d & robot,
  const double lookahead_distance, const double tangent_window = 0.30)
{
  PathTrackingSample result;
  if (path.size() < 2U || !robot.allFinite()) {return result;}
  const auto cumulative = followerCumulativeDistance3D(path);
  if (cumulative.back() < 1e-6) {return result;}

  double best_squared_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0U; i + 1U < path.size(); ++i) {
    const Eigen::Vector3d delta = path[i + 1U] - path[i];
    const double denominator = delta.squaredNorm();
    if (denominator < 1e-12) {continue;}
    const double ratio = std::clamp(
      (robot - path[i]).dot(delta) / denominator, 0.0, 1.0);
    const Eigen::Vector3d candidate = path[i] + ratio * (path[i + 1U] - path[i]);
    const double squared_distance = (robot - candidate).squaredNorm();
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      result.nearest = candidate;
      result.projected_arc = cumulative[i] +
        ratio * (cumulative[i + 1U] - cumulative[i]);
    }
  }
  if (!std::isfinite(best_squared_distance)) {return result;}
  result.projection_distance_3d = std::sqrt(best_squared_distance);

  const double target_arc = std::min(
    cumulative.back(), result.projected_arc + std::max(0.0, lookahead_distance));
  std::size_t target_segment = 0U;
  result.lookahead = followerInterpolate(path, cumulative, target_arc, &target_segment);
  const double half_window = 0.5 * std::max(0.0, tangent_window);
  const Eigen::Vector3d tangent_begin = followerInterpolate(
    path, cumulative, std::max(result.projected_arc, target_arc - half_window));
  const Eigen::Vector3d tangent_end = followerInterpolate(
    path, cumulative, std::min(cumulative.back(), target_arc + half_window));
  result.tangent = tangent_end.head<2>() - tangent_begin.head<2>();
  if (result.tangent.norm() < 1e-6 && target_segment + 1U < path.size()) {
    result.tangent = path[target_segment + 1U].head<2>() - path[target_segment].head<2>();
  }
  if (result.tangent.norm() < 1e-6) {
    result.tangent = result.lookahead.head<2>() - result.nearest.head<2>();
  }
  if (result.tangent.norm() < 1e-6) {return result;}
  result.tangent.normalize();
  result.remaining_arc = std::max(0.0, cumulative.back() - result.projected_arc);
  result.valid = true;
  return result;
}

struct PathFollowerConfig
{
  double nominal_speed{0.45};
  double max_vx{0.80};
  double max_vy{0.50};
  double max_vyaw{0.30};
  double position_gain{0.80};
  double yaw_gain{1.30};
  double lookahead_base{0.15};
  double lookahead_speed_gain{0.40};
  double lookahead_min{0.15};
  double lookahead_max{0.35};
  double tangent_window{0.30};
  double finish_distance{0.15};
  double braking_deceleration{0.60};
  double translation_yaw_limit{1.0471975511965976};
  double full_speed_yaw_limit{0.17453292519943295};
  double yaw_deadband{0.04};
};

struct PathFollowerCommand
{
  bool valid{false};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double yaw_error{0.0};
  double target_yaw{0.0};
  double lookahead_distance{0.0};
  double endpoint_distance_3d{std::numeric_limits<double>::infinity()};
  bool target_ray_fallback{false};
  bool aligning_in_place{false};
  bool reverse_clamped{false};
  PathTrackingSample sample;
};

inline PathFollowerCommand computePathFollowerCommand(
  const std::vector<Eigen::Vector3d> & path, const Eigen::Vector3d & robot,
  const double robot_yaw, const PathFollowerConfig & config)
{
  PathFollowerCommand output;
  // SCAN bases lookahead on the speed planned at the nearest trajectory point,
  // rather than on the delayed measured/commanded robot speed.  A geometric
  // path has no time parameterization, so use this follower's target speed
  // profile: nominal speed away from the endpoint and braking speed near it.
  const PathTrackingSample projection = samplePathForTracking(
    path, robot, 0.0, config.tangent_window);
  if (!projection.valid) {return output;}
  const double braking_speed = std::sqrt(std::max(
      0.0, 2.0 * config.braking_deceleration *
      std::max(0.0, projection.remaining_arc - config.finish_distance)));
  const double tracking_speed = std::min(config.nominal_speed, braking_speed);
  output.lookahead_distance = std::clamp(
    config.lookahead_base + config.lookahead_speed_gain * tracking_speed,
    config.lookahead_min, config.lookahead_max);
  output.sample = samplePathForTracking(
    path, robot, output.lookahead_distance, config.tangent_window);
  if (!output.sample.valid) {return output;}
  output.endpoint_distance_3d = (path.back() - robot).norm();

  Eigen::Vector2d target_ray = output.sample.lookahead.head<2>() - robot.head<2>();
  if (target_ray.norm() < 1e-6) {
    target_ray = output.sample.tangent;
    output.target_ray_fallback = true;
  }
  if (target_ray.norm() < 1e-6) {return PathFollowerCommand{};}
  target_ray.normalize();
  output.target_yaw = std::atan2(target_ray.y(), target_ray.x());
  const Eigen::Vector2d heading(std::cos(robot_yaw), std::sin(robot_yaw));
  const double cross = heading.x() * target_ray.y() - heading.y() * target_ray.x();
  const double dot = heading.dot(target_ray);
  output.yaw_error = std::atan2(cross, dot);
  const double yaw_outside_deadband = std::copysign(
    std::max(0.0, std::abs(output.yaw_error) - config.yaw_deadband), output.yaw_error);
  output.wz = std::clamp(
    config.yaw_gain * yaw_outside_deadband, -config.max_vyaw, config.max_vyaw);
  const double absolute_yaw_error = std::abs(output.yaw_error);
  output.aligning_in_place = absolute_yaw_error >= config.translation_yaw_limit;
  // Match SCAN's measured endpoint check.  Arc progress alone is insufficient:
  // a laterally displaced robot can project onto the final path segment while
  // still being farther than the requested position tolerance.
  if (output.sample.remaining_arc <= config.finish_distance &&
    output.endpoint_distance_3d <= config.finish_distance)
  {
    output.valid = true;
    return output;
  }

  // 10 度以内保持完整线速度；10~60 度之间线性减速；达到 60 度后只原地转向。
  const double yaw_slowdown_span = std::max(
    1e-6, config.translation_yaw_limit - config.full_speed_yaw_limit);
  const double heading_scale = std::clamp(
    (config.translation_yaw_limit - absolute_yaw_error) / yaw_slowdown_span, 0.0, 1.0);
  // Match SCAN-Planner's controller: heading alignment scales both the path
  // feed-forward term and the projection-error correction.  Keeping the
  // correction active during an in-place turn makes a holonomic base drift.
  Eigen::Vector2d world_velocity = heading_scale *
    (tracking_speed * output.sample.tangent +
    config.position_gain * (output.sample.nearest.head<2>() - robot.head<2>()));
  const double maximum_linear_speed = std::hypot(config.max_vx, config.max_vy);
  if (world_velocity.norm() > maximum_linear_speed && world_velocity.norm() > 1e-9) {
    world_velocity *= maximum_linear_speed / world_velocity.norm();
  }
  const double cosine = std::cos(robot_yaw);
  const double sine = std::sin(robot_yaw);
  output.vx = std::clamp(
    cosine * world_velocity.x() + sine * world_velocity.y(), 0.0, config.max_vx);
  output.vy = std::clamp(
    -sine * world_velocity.x() + cosine * world_velocity.y(), -config.max_vy, config.max_vy);
  const double unclamped_vx = cosine * world_velocity.x() + sine * world_velocity.y();
  output.reverse_clamped = unclamped_vx < 0.0;
  output.valid = true;
  return output;
}

}  // namespace efficient_3d_local_planner
