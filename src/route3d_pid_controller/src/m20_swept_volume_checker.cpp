#include "route3d_pid_controller/m20_swept_volume_checker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace route3d_pid_controller
{
namespace
{

bool finitePositive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finiteOrdered(const double minimum, const double maximum)
{
  return std::isfinite(minimum) && std::isfinite(maximum) && minimum < maximum;
}

double dot(const Point3d & left, const Point3d & right)
{
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

}  // namespace

M20SweptVolumeChecker::M20SweptVolumeChecker(M20SweptVolumeConfig config)
: config_(std::move(config))
{
  if (!finitePositive(config_.body_half_length_m) ||
    !finitePositive(config_.body_half_width_m) ||
    !finiteOrdered(config_.body_min_z_m, config_.body_max_z_m) ||
    !finitePositive(config_.detection_half_length_m) ||
    !finitePositive(config_.detection_half_width_m) ||
    !finiteOrdered(config_.detection_min_z_m, config_.detection_max_z_m) ||
    !finitePositive(config_.path_center_height_m) ||
    !std::isfinite(config_.surface_exclusion_height_m) ||
    config_.surface_exclusion_height_m < 0.0 ||
    !finitePositive(config_.step_surface_max_deviation_m) ||
    !finitePositive(config_.step_surface_support_radius_m) ||
    !finitePositive(config_.step_surface_height_tolerance_m) ||
    !finitePositive(config_.step_surface_min_planar_spread_m) ||
    config_.step_surface_min_support_points == 0U)
  {
    throw std::invalid_argument("invalid M20 swept-volume configuration");
  }
}

bool M20SweptVolumeChecker::isSelfPoint(const Point3d & point) const noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         std::abs(point.x) <= config_.body_half_length_m &&
         std::abs(point.y) <= config_.body_half_width_m &&
         point.z >= config_.body_min_z_m && point.z <= config_.body_max_z_m;
}

Point3d M20SweptVolumeChecker::transformPoint(
  const Pose3d & pose, const Point3d & point)
{
  // Quaternion-vector rotation, with normalization so slightly imperfect
  // odometry quaternions cannot scale a cloud.
  const double norm = std::sqrt(
    pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
  const double qx = norm > 1.0e-12 ? pose.qx / norm : 0.0;
  const double qy = norm > 1.0e-12 ? pose.qy / norm : 0.0;
  const double qz = norm > 1.0e-12 ? pose.qz / norm : 0.0;
  const double qw = norm > 1.0e-12 ? pose.qw / norm : 1.0;
  const double tx = 2.0 * (qy * point.z - qz * point.y);
  const double ty = 2.0 * (qz * point.x - qx * point.z);
  const double tz = 2.0 * (qx * point.y - qy * point.x);
  return {
    pose.position.x + point.x + qw * tx + (qy * tz - qz * ty),
    pose.position.y + point.y + qw * ty + (qz * tx - qx * tz),
    pose.position.z + point.z + qw * tz + (qx * ty - qy * tx)};
}

M20SweptVolumeResult M20SweptVolumeChecker::check(
  const std::vector<Point3d> & points_world,
  const std::vector<SweptVolumePose> & trajectory) const
{
  M20SweptVolumeResult result;
  result.points_examined = points_world.size();
  result.poses_checked = trajectory.size();
  std::vector<bool> inside(points_world.size(), false);
  std::vector<bool> surface(points_world.size(), false);
  std::vector<double> nearest_distance(
    points_world.size(), std::numeric_limits<double>::infinity());
  struct LocalPoint
  {
    std::size_t index;
    double x;
    double y;
    double z;
    double height_above_expected_surface;
  };
  for (const auto & pose : trajectory) {
    const double cosine_yaw = std::cos(pose.yaw);
    const double sine_yaw = std::sin(pose.yaw);
    const double cosine_pitch = std::cos(pose.pitch);
    const double sine_pitch = std::sin(pose.pitch);
    const Point3d forward{
      cosine_pitch * cosine_yaw, cosine_pitch * sine_yaw, sine_pitch};
    const Point3d left{-sine_yaw, cosine_yaw, 0.0};
    const Point3d up{
      -sine_pitch * cosine_yaw, -sine_pitch * sine_yaw, cosine_pitch};
    std::vector<LocalPoint> local_points;
    for (std::size_t index = 0U; index < points_world.size(); ++index) {
      const auto & point = points_world[index];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const Point3d delta{
        point.x - pose.center.x, point.y - pose.center.y, point.z - pose.center.z};
      const double local_x = dot(delta, forward);
      const double local_y = dot(delta, left);
      const double local_z = dot(delta, up);
      if (std::abs(local_x) > config_.detection_half_length_m ||
        std::abs(local_y) > config_.detection_half_width_m ||
        local_z < config_.detection_min_z_m ||
        local_z > config_.detection_max_z_m)
      {
        continue;
      }
      const double height_above_surface = local_z + config_.path_center_height_m;
      inside[index] = true;
      nearest_distance[index] = std::min(nearest_distance[index], pose.distance_m);
      local_points.push_back({index, local_x, local_y, local_z, height_above_surface});
      // Smooth path and ramp returns are measured relative to the locally
      // pitched path plane.
      if (height_above_surface <= config_.surface_exclusion_height_m) {
        surface[index] = true;
      }
    }

    // A discrete stair tread can sit above the continuous path plane. Treat a
    // low, locally horizontal cluster as a step surface, but never classify an
    // isolated point as terrain. This preserves single-point obstacle
    // detection while removing the tread and the first 10 cm above it.
    std::vector<const LocalPoint *> supported_surfaces;
    const double support_radius_squared =
      config_.step_surface_support_radius_m * config_.step_surface_support_radius_m;
    for (const auto & candidate : local_points) {
      if (candidate.height_above_expected_surface <= config_.surface_exclusion_height_m ||
        candidate.height_above_expected_surface > config_.step_surface_max_deviation_m)
      {
        continue;
      }
      std::size_t support = 0U;
      double mean_x = 0.0;
      double mean_y = 0.0;
      for (const auto & neighbour : local_points) {
        const double dx = neighbour.x - candidate.x;
        const double dy = neighbour.y - candidate.y;
        if (dx * dx + dy * dy <= support_radius_squared &&
          std::abs(neighbour.z - candidate.z) <=
          config_.step_surface_height_tolerance_m)
        {
          ++support;
          mean_x += neighbour.x;
          mean_y += neighbour.y;
        }
      }
      if (support < config_.step_surface_min_support_points) {
        continue;
      }
      mean_x /= static_cast<double>(support);
      mean_y /= static_cast<double>(support);
      double covariance_xx = 0.0;
      double covariance_xy = 0.0;
      double covariance_yy = 0.0;
      for (const auto & neighbour : local_points) {
        const double dx = neighbour.x - candidate.x;
        const double dy = neighbour.y - candidate.y;
        if (dx * dx + dy * dy > support_radius_squared ||
          std::abs(neighbour.z - candidate.z) >
          config_.step_surface_height_tolerance_m)
        {
          continue;
        }
        const double centered_x = neighbour.x - mean_x;
        const double centered_y = neighbour.y - mean_y;
        covariance_xx += centered_x * centered_x;
        covariance_xy += centered_x * centered_y;
        covariance_yy += centered_y * centered_y;
      }
      covariance_xx /= static_cast<double>(support);
      covariance_xy /= static_cast<double>(support);
      covariance_yy /= static_cast<double>(support);
      const double trace = covariance_xx + covariance_yy;
      const double discriminant = std::sqrt(std::max(
        0.0, (covariance_xx - covariance_yy) * (covariance_xx - covariance_yy) +
        4.0 * covariance_xy * covariance_xy));
      const double minor_planar_spread = std::sqrt(std::max(0.0, 0.5 * (trace - discriminant)));
      if (minor_planar_spread >= config_.step_surface_min_planar_spread_m) {
        supported_surfaces.push_back(&candidate);
      }
    }
    for (const auto & candidate : local_points) {
      for (const auto * supported : supported_surfaces) {
        const double dx = supported->x - candidate.x;
        const double dy = supported->y - candidate.y;
        const double height = candidate.z - supported->z;
        if (dx * dx + dy * dy <= support_radius_squared &&
          height >= -config_.step_surface_height_tolerance_m &&
          height <= config_.surface_exclusion_height_m)
        {
          surface[candidate.index] = true;
          break;
        }
      }
    }
  }
  for (std::size_t index = 0U; index < points_world.size(); ++index) {
    if (!inside[index]) {
      continue;
    }
    if (surface[index]) {
      ++result.surface_points_excluded;
      continue;
    }
    result.hits.push_back(points_world[index]);
    ++result.collision_points;
    result.nearest_hit_distance_m = std::min(
      result.nearest_hit_distance_m, nearest_distance[index]);
  }
  return result;
}

}  // namespace route3d_pid_controller
