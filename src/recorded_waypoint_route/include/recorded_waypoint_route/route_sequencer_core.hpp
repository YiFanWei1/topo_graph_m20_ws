#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace recorded_waypoint_route
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline double distance3d(const Point3 & first, const Point3 & second)
{
  return std::hypot(std::hypot(first.x - second.x, first.y - second.y), first.z - second.z);
}

inline std::pair<std::size_t, double> nearestTarget(
  const Point3 & body_position, const std::vector<Point3> & body_targets)
{
  if (body_targets.empty()) {
    throw std::invalid_argument("at least one target is required");
  }
  std::size_t nearest_index = 0;
  double nearest_distance = distance3d(body_position, body_targets.front());
  for (std::size_t index = 1; index < body_targets.size(); ++index) {
    const double distance = distance3d(body_position, body_targets[index]);
    if (distance < nearest_distance) {
      nearest_index = index;
      nearest_distance = distance;
    }
  }
  return {nearest_index, nearest_distance};
}

inline std::vector<std::size_t> orderedTargetIndices(
  std::size_t start_index, std::size_t goal_index, std::size_t target_count)
{
  if (target_count == 0 || start_index >= target_count || goal_index >= target_count) {
    throw std::invalid_argument("start and goal target indices must be in range");
  }
  std::vector<std::size_t> result;
  if (goal_index >= start_index) {
    for (std::size_t index = start_index; index <= goal_index; ++index) {
      result.push_back(index);
    }
  } else {
    for (std::size_t index = start_index;; --index) {
      result.push_back(index);
      if (index == goal_index) {
        break;
      }
    }
  }
  return result;
}

inline std::vector<Point3> interpolateTargets(
  const std::vector<Point3> & targets, double spacing)
{
  if (targets.size() < 2) {
    throw std::invalid_argument("route requires at least two targets");
  }
  if (!std::isfinite(spacing) || spacing <= 0.0) {
    throw std::invalid_argument("route spacing must be finite and positive");
  }
  std::vector<Point3> result{targets.front()};
  for (std::size_t index = 0; index + 1 < targets.size(); ++index) {
    const Point3 & start = targets[index];
    const Point3 & end = targets[index + 1];
    const double length = distance3d(start, end);
    if (length <= 1e-9) {
      throw std::invalid_argument("adjacent route targets must not be duplicates");
    }
    const auto intervals = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(length / spacing)));
    for (std::size_t step = 1; step <= intervals; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(intervals);
      result.push_back({
        start.x + ratio * (end.x - start.x),
        start.y + ratio * (end.y - start.y),
        start.z + ratio * (end.z - start.z)});
    }
  }
  return result;
}

inline std::vector<std::vector<Point3>> splitInterpolatedPath(
  const std::vector<Point3> & targets, const std::vector<Point3> & path,
  double tolerance = 1e-7)
{
  if (targets.size() < 2 || path.size() < targets.size()) {
    throw std::invalid_argument("interpolated path has fewer points than targets");
  }
  std::vector<std::size_t> target_indices;
  std::size_t search_start = 0;
  for (const Point3 & target : targets) {
    bool found = false;
    for (std::size_t index = search_start; index < path.size(); ++index) {
      if (distance3d(target, path[index]) <= tolerance) {
        target_indices.push_back(index);
        search_start = index + 1;
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::invalid_argument("interpolated path does not contain every target in order");
    }
  }
  std::vector<std::vector<Point3>> segments;
  for (std::size_t index = 0; index + 1 < target_indices.size(); ++index) {
    const auto begin = path.begin() + static_cast<std::ptrdiff_t>(target_indices[index]);
    const auto end = path.begin() + static_cast<std::ptrdiff_t>(target_indices[index + 1] + 1);
    segments.emplace_back(begin, end);
    if (segments.back().size() < 2) {
      throw std::invalid_argument("interpolated segment must contain at least two points");
    }
  }
  return segments;
}

enum class TargetType : std::uint8_t {Normal = 0, Corner = 1};

enum class RouteState {Tracking, Complete};

struct ProgressUpdate
{
  RouteState state{RouteState::Tracking};
  double distance{std::numeric_limits<double>::infinity()};
  bool changed{false};
  const char * event{"none"};
  std::optional<std::pair<std::size_t, std::size_t>> active_leg;
  std::size_t required_target{0};
  TargetType target_type{TargetType::Normal};
  double arrival_tolerance{0.15};
};

class RouteProgress
{
public:
  RouteProgress(std::vector<std::size_t> target_indices, double arrival_tolerance)
  : RouteProgress(
      std::move(target_indices), {}, arrival_tolerance, arrival_tolerance, arrival_tolerance)
  {}

  RouteProgress(
    std::vector<std::size_t> target_indices, std::vector<TargetType> target_types,
    double normal_arrival_tolerance, double corner_arrival_tolerance,
    double goal_arrival_tolerance)
  : target_indices_(std::move(target_indices)), target_types_(std::move(target_types)),
    normal_arrival_tolerance_(normal_arrival_tolerance),
    corner_arrival_tolerance_(corner_arrival_tolerance),
    goal_arrival_tolerance_(goal_arrival_tolerance)
  {
    if (target_indices_.empty()) {
      throw std::invalid_argument("selected route requires at least one target");
    }
    if (!validTolerance(normal_arrival_tolerance_) ||
      !validTolerance(corner_arrival_tolerance_) ||
      !validTolerance(goal_arrival_tolerance_))
    {
      throw std::invalid_argument("arrival tolerances must be finite and positive");
    }
    for (std::size_t index = 0; index + 1 < target_indices_.size(); ++index) {
      const auto left = static_cast<long long>(target_indices_[index]);
      const auto right = static_cast<long long>(target_indices_[index + 1]);
      if (std::llabs(right - left) != 1) {
        throw std::invalid_argument("selected route targets must be adjacent and ordered");
      }
    }
    if (!target_types_.empty()) {
      for (const std::size_t target_index : target_indices_) {
        if (target_index >= target_types_.size()) {
          throw std::invalid_argument("target type list does not cover selected route");
        }
      }
    }
    required_target_ = target_indices_.front();
  }

  ProgressUpdate update(const Point3 & body_position, const std::vector<Point3> & body_targets)
  {
    if (required_target_ >= body_targets.size()) {
      throw std::invalid_argument("selected target is outside the current route");
    }
    ProgressUpdate update;
    update.state = state_;
    update.distance = distance3d(body_position, body_targets[required_target_]);
    update.active_leg = active_leg_;
    update.required_target = required_target_;
    update.target_type = requiredTargetType();
    update.arrival_tolerance = effectiveTolerance();
    if (state_ == RouteState::Complete || update.distance > update.arrival_tolerance) {
      return update;
    }

    update.changed = true;
    if (sequence_position_ + 1 == target_indices_.size()) {
      state_ = RouteState::Complete;
      update.event = "goal_reached";
    } else {
      const std::size_t previous = required_target_;
      ++sequence_position_;
      required_target_ = target_indices_[sequence_position_];
      active_leg_ = std::make_pair(previous, required_target_);
      update.event = "target_advanced";
    }
    update.state = state_;
    update.active_leg = active_leg_;
    update.required_target = required_target_;
    update.target_type = requiredTargetType();
    update.arrival_tolerance = effectiveTolerance();
    return update;
  }

  std::size_t requiredTarget() const {return required_target_;}
  std::size_t sequencePosition() const {return sequence_position_;}
  const std::optional<std::pair<std::size_t, std::size_t>> & activeLeg() const
  {
    return active_leg_;
  }

  TargetType requiredTargetType() const
  {
    if (target_types_.empty()) {
      return TargetType::Normal;
    }
    return target_types_.at(required_target_);
  }

  bool requiredTargetIsGoal() const
  {
    return sequence_position_ + 1 == target_indices_.size();
  }

  double effectiveTolerance() const
  {
    if (requiredTargetIsGoal()) {
      return goal_arrival_tolerance_;
    }
    return requiredTargetType() == TargetType::Corner ?
      corner_arrival_tolerance_ : normal_arrival_tolerance_;
  }

private:
  static bool validTolerance(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  std::vector<std::size_t> target_indices_;
  std::vector<TargetType> target_types_;
  double normal_arrival_tolerance_{2.80};
  double corner_arrival_tolerance_{0.28};
  double goal_arrival_tolerance_{0.15};
  RouteState state_{RouteState::Tracking};
  std::size_t sequence_position_{0};
  std::size_t required_target_{0};
  std::optional<std::pair<std::size_t, std::size_t>> active_leg_;
};

}  // namespace recorded_waypoint_route
