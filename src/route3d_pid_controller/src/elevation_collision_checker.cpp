#include "route3d_pid_controller/elevation_collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace route3d_pid_controller
{
namespace
{

bool finite(const double value)
{
  return std::isfinite(value);
}

std::vector<double> samples(
  const double minimum, const double maximum, const std::size_t count)
{
  std::vector<double> result;
  result.reserve(count);
  if (count == 1U) {
    result.push_back(0.5 * (minimum + maximum));
    return result;
  }
  const double step = (maximum - minimum) / static_cast<double>(count - 1U);
  for (std::size_t index = 0U; index < count; ++index) {
    result.push_back(minimum + step * static_cast<double>(index));
  }
  return result;
}

}  // namespace

ElevationCollisionChecker::ElevationCollisionChecker(ElevationCollisionConfig config)
: config_(std::move(config))
{
  const auto & footprint = config_.warning_footprint;
  if (!finite(config_.resolution_m) || config_.resolution_m <= 0.0 ||
    !finite(config_.width_m) || config_.width_m <= config_.resolution_m ||
    !finite(config_.default_ground_height_m) ||
    !finite(config_.minimum_obstacle_height_m) ||
    !finite(config_.maximum_obstacle_height_m) ||
    config_.minimum_obstacle_height_m >= config_.maximum_obstacle_height_m ||
    !finite(config_.roughness_threshold_m) || config_.roughness_threshold_m <= 0.0 ||
    config_.body_min_x >= config_.body_max_x || config_.body_min_y >= config_.body_max_y ||
    footprint.min_x >= footprint.max_x || footprint.min_y >= footprint.max_y ||
    footprint.samples_x < 2U || footprint.samples_y < 2U ||
    footprint.lateral_inset_m < 0.0 ||
    2.0 * footprint.lateral_inset_m >= footprint.max_y - footprint.min_y)
  {
    throw std::invalid_argument("invalid elevation collision configuration");
  }

  cells_per_side_ = static_cast<std::size_t>(std::ceil(config_.width_m / config_.resolution_m));
  half_width_m_ = 0.5 * static_cast<double>(cells_per_side_) * config_.resolution_m;
  const std::size_t cell_count = cells_per_side_ * cells_per_side_;
  elevation_.assign(cell_count, config_.default_ground_height_m);
  observed_.assign(cell_count, false);
  rough_.assign(cell_count, false);
}

void ElevationCollisionChecker::clear()
{
  std::fill(elevation_.begin(), elevation_.end(), config_.default_ground_height_m);
  std::fill(observed_.begin(), observed_.end(), false);
  std::fill(rough_.begin(), rough_.end(), false);
  ready_ = false;
  occupied_cells_ = 0U;
  rough_cells_ = 0U;
}

bool ElevationCollisionChecker::inside(const double x, const double y) const noexcept
{
  return x >= -half_width_m_ && x < half_width_m_ &&
         y >= -half_width_m_ && y < half_width_m_;
}

std::size_t ElevationCollisionChecker::index(const int x, const int y) const noexcept
{
  return static_cast<std::size_t>(y) * cells_per_side_ + static_cast<std::size_t>(x);
}

void ElevationCollisionChecker::update(const std::vector<ElevationPoint> & points)
{
  std::fill(elevation_.begin(), elevation_.end(), config_.default_ground_height_m);
  std::fill(observed_.begin(), observed_.end(), false);
  std::fill(rough_.begin(), rough_.end(), false);
  occupied_cells_ = 0U;
  rough_cells_ = 0U;

  // The reference map first clips the cloud to the inscribed square of its
  // rolling map. This keeps every transformed footprint sample inside the map.
  const double input_half_width = config_.width_m / (2.01 * std::sqrt(2.0));
  for (const auto & point : points) {
    if (!finite(point.x) || !finite(point.y) || !finite(point.z) ||
      std::abs(point.x) > input_half_width || std::abs(point.y) > input_half_width ||
      point.z < config_.minimum_obstacle_height_m ||
      point.z > config_.maximum_obstacle_height_m)
    {
      continue;
    }
    if (point.x > config_.body_min_x && point.x < config_.body_max_x &&
      point.y > config_.body_min_y && point.y < config_.body_max_y)
    {
      continue;
    }
    const int grid_x = static_cast<int>(std::floor((point.x + half_width_m_) /
      config_.resolution_m));
    const int grid_y = static_cast<int>(std::floor((point.y + half_width_m_) /
      config_.resolution_m));
    if (grid_x < 0 || grid_y < 0 ||
      grid_x >= static_cast<int>(cells_per_side_) ||
      grid_y >= static_cast<int>(cells_per_side_))
    {
      continue;
    }
    const std::size_t cell = index(grid_x, grid_y);
    if (!observed_[cell]) {
      elevation_[cell] = point.z;
      observed_[cell] = true;
      ++occupied_cells_;
    } else {
      // Voxel filtering in the reference leaves one representative height per
      // cell. Taking the maximum is deterministic and conservative.
      elevation_[cell] = std::max(elevation_[cell], point.z);
    }
  }

  // Compute the 3x3 sample standard deviation from actual observations only.
  // Treating an unobserved cell as default_ground_height_m creates a false
  // height edge around every sparse return and was the source of flat-floor
  // emergency stops on the robot.
  for (int y = 1; y + 1 < static_cast<int>(cells_per_side_); ++y) {
    for (int x = 1; x + 1 < static_cast<int>(cells_per_side_); ++x) {
      double sum = 0.0;
      double squared_sum = 0.0;
      std::size_t count = 0U;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const auto neighbor = index(x + dx, y + dy);
          if (!observed_[neighbor]) {
            continue;
          }
          const double height = elevation_[neighbor];
          sum += height;
          squared_sum += height * height;
          ++count;
        }
      }
      if (count < 2U) {
        continue;
      }
      const double sample_count = static_cast<double>(count);
      const double variance = std::max(
        0.0, (squared_sum - sum * sum / sample_count) / (sample_count - 1.0));
      if (std::sqrt(variance) > config_.roughness_threshold_m) {
        rough_[index(x, y)] = true;
        ++rough_cells_;
      }
    }
  }
  ready_ = true;
}

double ElevationCollisionChecker::heightAt(const double x, const double y) const noexcept
{
  if (!inside(x, y)) {
    return 0.0;
  }
  const int grid_x = static_cast<int>(std::floor((x + half_width_m_) / config_.resolution_m));
  const int grid_y = static_cast<int>(std::floor((y + half_width_m_) / config_.resolution_m));
  if (grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(cells_per_side_) || grid_y >= static_cast<int>(cells_per_side_))
  {
    return 0.0;
  }
  const auto cell = index(grid_x, grid_y);
  return observed_[cell] ? elevation_[cell] : 0.0;
}

std::vector<ElevationCell> ElevationCollisionChecker::cellData() const
{
  std::vector<ElevationCell> cells;
  cells.reserve(occupied_cells_ + rough_cells_);
  for (std::size_t y = 0U; y < cells_per_side_; ++y) {
    for (std::size_t x = 0U; x < cells_per_side_; ++x) {
      const auto cell = y * cells_per_side_ + x;
      if (!observed_[cell] && !rough_[cell]) {
        continue;
      }
      cells.push_back({
        -half_width_m_ + (static_cast<double>(x) + 0.5) * config_.resolution_m,
        -half_width_m_ + (static_cast<double>(y) + 0.5) * config_.resolution_m,
        observed_[cell] ? elevation_[cell] : 0.0, observed_[cell], rough_[cell]});
    }
  }
  return cells;
}

std::vector<FootprintSample> ElevationCollisionChecker::sampleTrajectory(
  const std::vector<Pose2d> & trajectory, const FootprintGrid * footprint_override) const
{
  std::vector<FootprintSample> result;
  if (!ready_) {
    return result;
  }
  const FootprintGrid & footprint = footprint_override == nullptr ?
    config_.warning_footprint : *footprint_override;
  const auto x_samples = samples(footprint.min_x, footprint.max_x, footprint.samples_x);
  const auto y_samples = samples(
    footprint.min_y + footprint.lateral_inset_m,
    footprint.max_y - footprint.lateral_inset_m, footprint.samples_y);
  result.reserve(trajectory.size() * x_samples.size() * y_samples.size());
  for (const auto & pose : trajectory) {
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    for (const double local_x : x_samples) {
      for (const double local_y : y_samples) {
        const double x = pose.x + local_x * cosine - local_y * sine;
        const double y = pose.y + local_x * sine + local_y * cosine;
        result.push_back({x, y, roughAt(x, y)});
      }
    }
  }
  return result;
}

bool ElevationCollisionChecker::roughAt(const double x, const double y) const noexcept
{
  if (!ready_ || !inside(x, y)) {
    return false;
  }
  const int grid_x = static_cast<int>(std::floor((x + half_width_m_) / config_.resolution_m));
  const int grid_y = static_cast<int>(std::floor((y + half_width_m_) / config_.resolution_m));
  if (grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(cells_per_side_) || grid_y >= static_cast<int>(cells_per_side_))
  {
    return false;
  }
  return rough_[index(grid_x, grid_y)];
}

ElevationCollisionResult ElevationCollisionChecker::checkTrajectory(
  const std::vector<Pose2d> & trajectory, const FootprintGrid * footprint_override) const
{
  ElevationCollisionResult result;
  if (!ready_) {
    return result;
  }
  const FootprintGrid & footprint = footprint_override == nullptr ?
    config_.warning_footprint : *footprint_override;
  if (footprint.min_x >= footprint.max_x || footprint.min_y >= footprint.max_y ||
    footprint.samples_x < 2U || footprint.samples_y < 2U ||
    2.0 * footprint.lateral_inset_m >= footprint.max_y - footprint.min_y)
  {
    throw std::invalid_argument("invalid trajectory footprint");
  }
  const auto x_samples = samples(footprint.min_x, footprint.max_x, footprint.samples_x);
  const auto y_samples = samples(
    footprint.min_y + footprint.lateral_inset_m,
    footprint.max_y - footprint.lateral_inset_m, footprint.samples_y);
  result.trajectory_poses = trajectory.size();
  for (const auto & pose : trajectory) {
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    for (const double local_x : x_samples) {
      for (const double local_y : y_samples) {
        const double x = pose.x + local_x * cosine - local_y * sine;
        const double y = pose.y + local_x * sine + local_y * cosine;
        ++result.footprint_samples;
        if (roughAt(x, y)) {
          ++result.collision_samples;
        }
      }
    }
  }
  return result;
}

}  // namespace route3d_pid_controller
