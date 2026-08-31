#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace efficient_3d_local_planner
{

constexpr double kPi = 3.14159265358979323846;

inline double wrapAngle(double value)
{
  while (value > kPi) {value -= 2.0 * kPi;}
  while (value < -kPi) {value += 2.0 * kPi;}
  return value;
}

struct GridSnapshot
{
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  double resolution{0.15};
  Eigen::Vector3i dimensions{Eigen::Vector3i::Zero()};
  std::uint64_t revision{0};
  std::vector<std::uint8_t> hard;
  std::vector<std::uint8_t> footprint_hard;
  std::vector<std::uint8_t> soft_cost;

  std::size_t cellCount() const
  {
    return static_cast<std::size_t>(dimensions.x()) * dimensions.y() * dimensions.z();
  }

  bool valid() const
  {
    return resolution > 0.0 && (dimensions.array() > 0).all() &&
           footprint_hard.size() == cellCount() && soft_cost.size() == cellCount();
  }

  int linear(const Eigen::Vector3i & cell) const
  {
    if ((cell.array() < 0).any() || (cell.array() >= dimensions.array()).any()) {
      return -1;
    }
    return (cell.z() * dimensions.y() + cell.y()) * dimensions.x() + cell.x();
  }

  Eigen::Vector3i cellFromLinear(const int index) const
  {
    const int x = index % dimensions.x();
    const int yz = index / dimensions.x();
    return Eigen::Vector3i(x, yz % dimensions.y(), yz / dimensions.y());
  }

  Eigen::Vector3i pointToCell(const Eigen::Vector3d & point) const
  {
    return ((point - origin) / resolution).array().floor().cast<int>();
  }

  Eigen::Vector3d cellCenter(const Eigen::Vector3i & cell) const
  {
    return origin + resolution * (cell.cast<double>() + Eigen::Vector3d::Constant(0.5));
  }

  int indexAt(const Eigen::Vector3d & point) const {return linear(pointToCell(point));}

  bool footprintAt(const Eigen::Vector3d & point) const
  {
    const int index = indexAt(point);
    return index < 0 || footprint_hard[static_cast<std::size_t>(index)] != 0U;
  }

  double softAt(const Eigen::Vector3d & point) const
  {
    const int index = indexAt(point);
    return index < 0 ? 1.0 :
           static_cast<double>(soft_cost[static_cast<std::size_t>(index)]) / 254.0;
  }
};

struct GuideProjection
{
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  double distance_xy{std::numeric_limits<double>::infinity()};
  double z_error{std::numeric_limits<double>::infinity()};
  double arc{0.0};
  std::size_t segment{0U};
};

inline std::vector<double> cumulativeDistance(const std::vector<Eigen::Vector3d> & path)
{
  std::vector<double> distance(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    distance[i] = distance[i - 1U] + (path[i] - path[i - 1U]).norm();
  }
  return distance;
}

inline Eigen::Vector3d interpolatePath(
  const std::vector<Eigen::Vector3d> & path, const std::vector<double> & distance,
  const double target)
{
  if (path.empty()) {return Eigen::Vector3d::Zero();}
  if (path.size() == 1U || target <= 0.0) {return path.front();}
  if (target >= distance.back()) {return path.back();}
  const auto upper = std::upper_bound(distance.begin(), distance.end(), target);
  const std::size_t high = static_cast<std::size_t>(std::distance(distance.begin(), upper));
  const std::size_t low = high - 1U;
  const double span = distance[high] - distance[low];
  const double ratio = span > 1e-9 ? (target - distance[low]) / span : 0.0;
  return path[low] + ratio * (path[high] - path[low]);
}

inline GuideProjection projectToGuide(
  const Eigen::Vector3d & query, const std::vector<Eigen::Vector3d> & guide,
  const std::vector<double> & distance)
{
  GuideProjection best;
  double best_score = std::numeric_limits<double>::infinity();
  if (guide.empty()) {return best;}
  if (guide.size() == 1U) {
    best.point = guide.front();
    best.distance_xy = (query.head<2>() - guide.front().head<2>()).norm();
    best.z_error = query.z() - guide.front().z();
    return best;
  }
  for (std::size_t i = 0; i + 1U < guide.size(); ++i) {
    const Eigen::Vector3d delta = guide[i + 1U] - guide[i];
    const double denominator = delta.squaredNorm();
    const double ratio = denominator > 1e-12 ?
      std::clamp((query - guide[i]).dot(delta) / denominator, 0.0, 1.0) : 0.0;
    const Eigen::Vector3d point = guide[i] + ratio * delta;
    const double xy = (query.head<2>() - point.head<2>()).norm();
    const double z = query.z() - point.z();
    const double score = xy * xy + z * z;
    if (score < best_score) {
      best_score = score;
      best.point = point;
      best.distance_xy = xy;
      best.z_error = z;
      best.segment = i;
      best.arc = distance[i] + ratio * (distance[i + 1U] - distance[i]);
    }
  }
  return best;
}

inline Eigen::Vector3d projectStartZToLiftedPath(
  const Eigen::Vector3d & odometry_position,
  const std::vector<Eigen::Vector3d> & lifted_path,
  const double maximum_z_correction, bool * applied = nullptr)
{
  Eigen::Vector3d result = odometry_position;
  if (applied) {*applied = false;}
  if (lifted_path.size() < 2U) {return result;}
  const auto distance = cumulativeDistance(lifted_path);
  const GuideProjection projection = projectToGuide(odometry_position, lifted_path, distance);
  const double correction = projection.point.z() - odometry_position.z();
  if (!std::isfinite(correction) ||
    (maximum_z_correction > 0.0 && std::abs(correction) > maximum_z_correction))
  {
    return result;
  }
  result.z() = projection.point.z();
  if (applied) {*applied = true;}
  return result;
}

inline std::vector<Eigen::Vector3d> extractLocalGuide(
  const std::vector<Eigen::Vector3d> & lifted_path, const Eigen::Vector3d & robot,
  const double horizon, const double spacing)
{
  if (lifted_path.size() < 2U) {return {};}
  const auto cumulative = cumulativeDistance(lifted_path);
  const auto projection = projectToGuide(robot, lifted_path, cumulative);
  const double end = std::min(cumulative.back(), projection.arc + horizon);
  std::vector<Eigen::Vector3d> guide;
  guide.push_back(projection.point);
  for (double arc = projection.arc + spacing; arc < end; arc += spacing) {
    guide.push_back(interpolatePath(lifted_path, cumulative, arc));
  }
  guide.push_back(interpolatePath(lifted_path, cumulative, end));
  return guide;
}

class GuidedAStar
{
public:
  struct Config
  {
    double corridor_xy{1.50};
    double corridor_z{0.40};
    double cylinder_offset{0.18};
    double soft_weight{12.0};
    double path_weight{0.35};
    double vertical_weight{0.60};
    double turn_weight{0.08};
    double rotation_cost{0.12};
    double goal_tolerance{0.30};
    double start_snap_radius{0.50};
    double goal_snap_radius{0.60};
    bool direct_shortcut_enabled{true};
    double direct_shortcut_sample_spacing{0.05};
    double direct_shortcut_max_guide_deviation_xy{0.15};
    double direct_shortcut_max_guide_deviation_z{0.15};
    int strict_max_expansions{8000};
    int max_expansions{120000};
  };

  struct Result
  {
    bool success{false};
    bool used_soft{false};
    bool used_reference_start_yaw{false};
    bool direct_shortcut_used{false};
    bool cancelled{false};
    std::string reason{"not_run"};
    int expansions{0};
    int rejected_outside_map{0};
    int rejected_corridor_xy{0};
    int rejected_corridor_z{0};
    int rejected_diagonal_corner{0};
    int rejected_hard_collision{0};
    int rejected_soft_layer{0};
    bool start_front_hard{false};
    bool start_rear_hard{false};
    bool goal_front_hard{false};
    bool goal_rear_hard{false};
    double maximum_soft_cost{0.0};
    double closest_goal_distance{std::numeric_limits<double>::infinity()};
    double furthest_guide_arc{0.0};
    double reached_z_min{std::numeric_limits<double>::infinity()};
    double reached_z_max{-std::numeric_limits<double>::infinity()};
    Eigen::Vector3d requested_start{Eigen::Vector3d::Zero()};
    Eigen::Vector3d selected_start{Eigen::Vector3d::Zero()};
    Eigen::Vector3d selected_goal{Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> path;
  };

  GuidedAStar() : GuidedAStar(Config{}) {}

  explicit GuidedAStar(Config config) : config_(std::move(config)) {}

  const Config & config() const {return config_;}

  bool poseValid(
    const GridSnapshot & map, const Eigen::Vector3d & position, const double yaw,
    const bool strict_soft, double * soft_cost = nullptr) const
  {
    const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
    const Eigen::Vector3d front = position + config_.cylinder_offset * heading;
    const Eigen::Vector3d rear = position - config_.cylinder_offset * heading;
    if (map.footprintAt(front) || map.footprintAt(rear)) {return false;}
    const double cost = std::max(map.softAt(front), map.softAt(rear));
    if (soft_cost) {*soft_cost = cost;}
    return !strict_soft || cost <= 0.0;
  }

  void hardCollisionAtPose(
    const GridSnapshot & map, const Eigen::Vector3d & position, const double yaw,
    bool & front_hard, bool & rear_hard) const
  {
    const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
    front_hard = map.footprintAt(position + config_.cylinder_offset * heading);
    rear_hard = map.footprintAt(position - config_.cylinder_offset * heading);
  }

  bool insideCorridor(
    const Eigen::Vector3d & point, const std::vector<Eigen::Vector3d> & guide,
    const std::vector<double> & distance, GuideProjection * projection = nullptr) const
  {
    const GuideProjection local = projectToGuide(point, guide, distance);
    if (projection) {*projection = local;}
    return local.distance_xy <= config_.corridor_xy + 1e-9 &&
           std::abs(local.z_error) <= config_.corridor_z + 1e-9;
  }

  bool edgeValid(
    const GridSnapshot & map, const Eigen::Vector3d & from, const double from_yaw,
    const Eigen::Vector3d & to, const double to_yaw,
    const std::vector<Eigen::Vector3d> & guide, const std::vector<double> & guide_distance,
    const bool strict_soft, double * maximum_soft = nullptr,
    std::string * failure_reason = nullptr) const
  {
    const double length = (to - from).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(
          length / std::max(1e-3, 0.5 * map.resolution))));
    const double yaw_delta = wrapAngle(to_yaw - from_yaw);
    double max_soft = 0.0;
    for (int i = 0; i <= samples; ++i) {
      const double ratio = static_cast<double>(i) / samples;
      const Eigen::Vector3d point = from + ratio * (to - from);
      if (!insideCorridor(point, guide, guide_distance)) {
        if (failure_reason) {*failure_reason = "outside_search_corridor";}
        return false;
      }
      double soft = 0.0;
      if (!poseValid(map, point, from_yaw + ratio * yaw_delta, strict_soft, &soft)) {
        if (failure_reason) {
          *failure_reason = strict_soft && poseValid(
            map, point, from_yaw + ratio * yaw_delta, false) ?
            "soft_layer" : "hard_footprint_collision";
        }
        return false;
      }
      max_soft = std::max(max_soft, soft);
    }
    if (maximum_soft) {*maximum_soft = max_soft;}
    return true;
  }

  Result search(
    const GridSnapshot & map, const Eigen::Vector3d & requested_start,
    const double start_yaw, const std::vector<Eigen::Vector3d> & guide,
    const std::function<bool()> & cancelled = []() {return false;}) const
  {
    Result result;
    result.requested_start = requested_start;
    if (!map.valid() || guide.size() < 2U) {
      result.reason = "invalid_map_or_guide";
      return result;
    }
    const auto guide_distance = cumulativeDistance(guide);

    // 空旷直线路段不需要受 8 邻域（每 45 度一个运动方向）的栅格折线限制。只有当
    // 起步旋转扫掠和整条三维直线都完全避开硬碰撞层、软膨胀层，并且始终贴近参考
    // 路线时，才直接返回连续空间直线；任一条件不满足就继续执行原有 A*。
    if (config_.direct_shortcut_enabled &&
      directShortcutValid(map, requested_start, start_yaw, guide, guide_distance))
    {
      Result direct;
      direct.success = true;
      direct.direct_shortcut_used = true;
      direct.reason = "direct_strict_success";
      direct.requested_start = requested_start;
      direct.selected_start = requested_start;
      direct.selected_goal = guide.back();
      direct.path = {requested_start, guide.back()};
      direct.closest_goal_distance = 0.0;
      direct.furthest_guide_arc = guide_distance.back();
      direct.reached_z_min = std::min(requested_start.z(), guide.back().z());
      direct.reached_z_max = std::max(requested_start.z(), guide.back().z());
      const Eigen::Vector2d direction = guide.back().head<2>() - requested_start.head<2>();
      const double travel_yaw = direction.norm() > 1e-9 ?
        std::atan2(direction.y(), direction.x()) : start_yaw;
      hardCollisionAtPose(
        map, requested_start, start_yaw, direct.start_front_hard, direct.start_rear_hard);
      hardCollisionAtPose(
        map, guide.back(), travel_yaw, direct.goal_front_hard, direct.goal_rear_hard);
      return direct;
    }

    Eigen::Vector3d start = requested_start;
    Eigen::Vector3d goal = guide.back();
    double start_soft = 0.0;
    double goal_soft = 0.0;
    const bool start_hard_valid = poseValid(map, requested_start, start_yaw, false, &start_soft);
    hardCollisionAtPose(
      map, requested_start, start_yaw, result.start_front_hard, result.start_rear_hard);
    bool start_valid = start_hard_valid;
    if (!start_valid && config_.start_snap_radius > 0.0) {
      Eigen::Vector3d tangent = guide[1] - guide[0];
      if (tangent.norm() > 1e-6) {tangent.normalize();}
      const int steps = static_cast<int>(std::ceil(
          config_.start_snap_radius / map.resolution));
      for (int step = 1; step <= steps && !start_valid; ++step) {
        const Eigen::Vector3d candidate = requested_start - step * map.resolution * tangent;
        if ((candidate - requested_start).norm() > config_.start_snap_radius ||
          !insideCorridor(candidate, guide, guide_distance))
        {
          continue;
        }
        double candidate_soft = 0.0;
        if (poseValid(map, candidate, start_yaw, false, &candidate_soft)) {
          start = candidate;
          start_soft = candidate_soft;
          start_valid = true;
        }
      }

      if (!start_valid) {
        const Eigen::Vector3i requested_cell = map.pointToCell(requested_start);
        const int radius = static_cast<int>(std::ceil(
            config_.start_snap_radius / map.resolution));
        double best_score = std::numeric_limits<double>::infinity();
        for (int dx = -radius; dx <= radius; ++dx) {
          for (int dy = -radius; dy <= radius; ++dy) {
            for (int dz = -radius; dz <= radius; ++dz) {
              const Eigen::Vector3i cell = requested_cell + Eigen::Vector3i(dx, dy, dz);
              if (map.linear(cell) < 0) {continue;}
              const Eigen::Vector3d candidate = map.cellCenter(cell);
              const double displacement = (candidate - requested_start).norm();
              if (displacement > config_.start_snap_radius) {continue;}
              GuideProjection projection;
              if (!insideCorridor(candidate, guide, guide_distance, &projection)) {continue;}
              double candidate_soft = 0.0;
              if (!poseValid(map, candidate, start_yaw, false, &candidate_soft)) {continue;}
              const double score = displacement + 0.25 * projection.distance_xy +
                0.25 * std::abs(projection.z_error);
              if (score < best_score) {
                best_score = score;
                start = candidate;
                start_soft = candidate_soft;
                start_valid = true;
              }
            }
          }
        }
      }
    }
    result.selected_start = start;
    const double goal_yaw = std::atan2(
      guide.back().y() - guide[guide.size() - 2U].y(),
      guide.back().x() - guide[guide.size() - 2U].x());
    bool goal_hard_valid = poseValid(map, goal, goal_yaw, false, &goal_soft);
    hardCollisionAtPose(map, goal, goal_yaw, result.goal_front_hard, result.goal_rear_hard);
    if (!goal_hard_valid) {
      const Eigen::Vector3i requested_cell = map.pointToCell(goal);
      const int radius = static_cast<int>(std::ceil(config_.goal_snap_radius / map.resolution));
      double best_score = std::numeric_limits<double>::infinity();
      Eigen::Vector3d best = goal;
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dz = -radius; dz <= radius; ++dz) {
            const Eigen::Vector3i cell = requested_cell + Eigen::Vector3i(dx, dy, dz);
            if (map.linear(cell) < 0) {continue;}
            const Eigen::Vector3d candidate = map.cellCenter(cell);
            const double displacement = (candidate - goal).norm();
            if (displacement > config_.goal_snap_radius) {continue;}
            GuideProjection projection;
            if (!insideCorridor(candidate, guide, guide_distance, &projection)) {continue;}
            double candidate_soft = 0.0;
            if (!poseValid(map, candidate, goal_yaw, false, &candidate_soft)) {continue;}
            const double score = displacement + 0.25 * projection.distance_xy +
              0.10 * std::abs(projection.z_error);
            if (score < best_score) {
              best_score = score;
              best = candidate;
              goal_soft = candidate_soft;
            }
          }
        }
      }
      if (best_score < std::numeric_limits<double>::infinity()) {
        goal = best;
        goal_hard_valid = true;
        hardCollisionAtPose(map, goal, goal_yaw, result.goal_front_hard, result.goal_rear_hard);
      }
    }
    result.selected_goal = goal;
    if (!start_valid || !goal_hard_valid) {
      result.reason = !start_valid ? "start_snap_failed" : "goal_snap_failed";
      return result;
    }

    if (start_soft <= 0.0 && goal_soft <= 0.0) {
      result = searchPhase(
        map, start, start_yaw, goal, guide, guide_distance, true,
        config_.strict_max_expansions, cancelled);
      result.requested_start = requested_start;
      result.selected_start = start;
      if (result.success || result.cancelled) {return result;}
    }
    result = searchPhase(
      map, start, start_yaw, goal, guide, guide_distance, false,
      config_.max_expansions, cancelled);
    result.requested_start = requested_start;
    result.selected_start = start;
    result.used_soft = result.success;
    if (!result.success && !result.cancelled && result.expansions <= 1) {
      const double reference_start_yaw = std::atan2(
        guide[1].y() - guide[0].y(), guide[1].x() - guide[0].x());
      double reference_start_soft = 0.0;
      if (poseValid(map, start, reference_start_yaw, false, &reference_start_soft)) {
        if (reference_start_soft <= 0.0 && goal_soft <= 0.0) {
          result = searchPhase(
            map, start, reference_start_yaw, goal, guide, guide_distance, true,
            config_.strict_max_expansions, cancelled);
        }
        if (!result.success && !result.cancelled) {
          result = searchPhase(
            map, start, reference_start_yaw, goal, guide, guide_distance, false,
            config_.max_expansions, cancelled);
        }
        result.requested_start = requested_start;
        result.selected_start = start;
        result.used_soft = result.success && result.reason == "relaxed_success";
        result.used_reference_start_yaw = true;
      }
    }
    return result;
  }

  bool validatePath(
    const GridSnapshot & map, const std::vector<Eigen::Vector3d> & path,
    const std::vector<Eigen::Vector3d> & guide, std::size_t * invalid_segment = nullptr,
    std::string * failure_reason = nullptr) const
  {
    if (path.size() < 2U || guide.size() < 2U) {return false;}
    const auto distance = cumulativeDistance(guide);
    double previous_yaw = std::atan2(
      path[1].y() - path[0].y(), path[1].x() - path[0].x());
    for (std::size_t i = 1; i < path.size(); ++i) {
      const double yaw = std::atan2(
        path[i].y() - path[i - 1U].y(), path[i].x() - path[i - 1U].x());
      std::string edge_failure;
      if (!edgeValid(
          map, path[i - 1U], previous_yaw, path[i], yaw, guide, distance, false))
      {
        if (invalid_segment) {*invalid_segment = i - 1U;}
        if (failure_reason) {
          edgeValid(
            map, path[i - 1U], previous_yaw, path[i], yaw, guide, distance, false,
            nullptr, &edge_failure);
          *failure_reason = edge_failure.empty() ? "invalid_edge" : edge_failure;
        }
        return false;
      }
      previous_yaw = yaw;
    }
    return true;
  }

private:
  bool directShortcutValid(
    const GridSnapshot & map, const Eigen::Vector3d & start, const double start_yaw,
    const std::vector<Eigen::Vector3d> & guide,
    const std::vector<double> & guide_distance) const
  {
    if (!map.valid() || guide.size() < 2U || guide_distance.size() != guide.size()) {
      return false;
    }
    const Eigen::Vector3d goal = guide.back();
    const Eigen::Vector3d delta = goal - start;
    const double length = delta.norm();
    if (!std::isfinite(length) || length < 1e-6) {return false;}

    const Eigen::Vector2d direction_xy = delta.head<2>();
    const double travel_yaw = direction_xy.norm() > 1e-9 ?
      std::atan2(direction_xy.y(), direction_xy.x()) : start_yaw;

    // 机器人禁止倒着走，因此进入直线前必须原地转向。用 5 度角步长检查前后两个
    // 圆柱的完整旋转扫掠，并把软膨胀层与硬层一样视为不可进入区域。
    const double yaw_delta = wrapAngle(travel_yaw - start_yaw);
    constexpr double kRotationSample = kPi / 36.0;
    const int rotation_samples = std::max(
      1, static_cast<int>(std::ceil(std::abs(yaw_delta) / kRotationSample)));
    for (int sample = 0; sample <= rotation_samples; ++sample) {
      const double ratio = static_cast<double>(sample) / rotation_samples;
      if (!poseValid(map, start, start_yaw + ratio * yaw_delta, true)) {return false;}
    }

    const GuideProjection start_projection = projectToGuide(start, guide, guide_distance);
    const double spacing = std::max(1e-3, config_.direct_shortcut_sample_spacing);
    const int translation_samples = std::max(
      1, static_cast<int>(std::ceil(length / spacing)));
    for (int sample = 0; sample <= translation_samples; ++sample) {
      const double ratio = static_cast<double>(sample) / translation_samples;
      const Eigen::Vector3d point = start + ratio * delta;
      GuideProjection projection;
      if (!insideCorridor(point, guide, guide_distance, &projection)) {return false;}

      // 允许机器人从当前的小幅横向/高度偏差逐渐汇入参考路线，但不能借直连切掉
      // 后续拐角。越靠近目标，允许偏差越严格地收敛到配置值。
      const double allowed_xy = std::max(
        config_.direct_shortcut_max_guide_deviation_xy,
        (1.0 - ratio) * start_projection.distance_xy);
      const double allowed_z = std::max(
        config_.direct_shortcut_max_guide_deviation_z,
        (1.0 - ratio) * std::abs(start_projection.z_error));
      if (projection.distance_xy > allowed_xy + 1e-9 ||
        std::abs(projection.z_error) > allowed_z + 1e-9)
      {
        return false;
      }
      if (!poseValid(map, point, travel_yaw, true)) {return false;}
    }
    return true;
  }

  struct QueueEntry
  {
    float f{0.0F};
    int state{0};
    bool operator<(const QueueEntry & other) const {return f > other.f;}
  };

  static constexpr std::array<int, 8> kDx{{1, 1, 0, -1, -1, -1, 0, 1}};
  static constexpr std::array<int, 8> kDy{{0, 1, 1, 1, 0, -1, -1, -1}};

  static double headingYaw(const int heading)
  {
    return std::atan2(static_cast<double>(kDy[heading]), static_cast<double>(kDx[heading]));
  }

  static int yawToHeading(const double yaw)
  {
    int best = 0;
    double error = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 8; ++i) {
      const double candidate = std::abs(wrapAngle(yaw - headingYaw(i)));
      if (candidate < error) {error = candidate; best = i;}
    }
    return best;
  }

  Result searchPhase(
    const GridSnapshot & map, const Eigen::Vector3d & requested_start,
    const double start_yaw, const Eigen::Vector3d & goal,
    const std::vector<Eigen::Vector3d> & guide, const std::vector<double> & guide_distance,
    const bool strict_soft, const int expansion_limit,
    const std::function<bool()> & cancelled) const
  {
    Result result;
    result.requested_start = requested_start;
    result.selected_start = requested_start;
    result.selected_goal = goal;
    result.reason = strict_soft ? "strict_no_path" : "relaxed_no_path";
    const int total_cells = static_cast<int>(map.cellCount());
    const int total_states = total_cells * 8;
    const int start_cell = map.indexAt(requested_start);
    if (start_cell < 0) {result.reason = "start_outside_map"; return result;}
    const int start_heading = yawToHeading(start_yaw);
    const int start_state = start_cell * 8 + start_heading;

    std::vector<float> g(static_cast<std::size_t>(total_states),
      std::numeric_limits<float>::infinity());
    std::vector<int> parent(static_cast<std::size_t>(total_states), -1);
    std::vector<std::uint8_t> closed(static_cast<std::size_t>(total_states), 0U);
    std::vector<float> projection_xy(map.cellCount(),
      std::numeric_limits<float>::quiet_NaN());
    std::vector<float> projection_z(map.cellCount(),
      std::numeric_limits<float>::quiet_NaN());
    auto projection_for = [&](const int cell_index, const Eigen::Vector3d & position) {
        GuideProjection projection;
        if (std::isnan(projection_xy[static_cast<std::size_t>(cell_index)])) {
          projection = projectToGuide(position, guide, guide_distance);
          projection_xy[static_cast<std::size_t>(cell_index)] =
            static_cast<float>(projection.distance_xy);
          projection_z[static_cast<std::size_t>(cell_index)] =
            static_cast<float>(projection.z_error);
        } else {
          projection.distance_xy = projection_xy[static_cast<std::size_t>(cell_index)];
          projection.z_error = projection_z[static_cast<std::size_t>(cell_index)];
        }
        return projection;
      };
    std::priority_queue<QueueEntry> open;
    g[static_cast<std::size_t>(start_state)] = 0.0F;
    open.push(QueueEntry{static_cast<float>((requested_start - goal).norm()), start_state});
    int goal_state = -1;

    while (!open.empty() && result.expansions < expansion_limit) {
      if ((result.expansions & 0x7f) == 0 && cancelled()) {
        result.cancelled = true;
        result.reason = "cancelled_by_new_map";
        return result;
      }
      const QueueEntry entry = open.top();
      open.pop();
      if (closed[static_cast<std::size_t>(entry.state)] != 0U) {continue;}
      closed[static_cast<std::size_t>(entry.state)] = 1U;
      ++result.expansions;

      const int current_cell_index = entry.state / 8;
      const int current_heading = entry.state % 8;
      const Eigen::Vector3i current_cell = map.cellFromLinear(current_cell_index);
      const Eigen::Vector3d current_position =
        current_cell_index == start_cell ? requested_start : map.cellCenter(current_cell);
      const double goal_distance = (current_position - goal).norm();
      result.closest_goal_distance = std::min(result.closest_goal_distance, goal_distance);
      result.reached_z_min = std::min(result.reached_z_min, current_position.z());
      result.reached_z_max = std::max(result.reached_z_max, current_position.z());
      const GuideProjection current_projection = projectToGuide(
        current_position, guide, guide_distance);
      result.furthest_guide_arc = std::max(
        result.furthest_guide_arc, current_projection.arc);
      if (goal_distance <= config_.goal_tolerance) {
        const Eigen::Vector3d connector = goal - current_position;
        const double current_yaw = headingYaw(current_heading);
        const double connector_yaw = connector.head<2>().norm() > 1e-9 ?
          std::atan2(connector.y(), connector.x()) : current_yaw;
        // The tolerance is only a search accelerator.  As in SCAN's exact
        // trajectory endpoint, a successful result must still contain the
        // selected goal itself and a collision-checked final connection.
        if (goal_distance <= 1e-9 || edgeValid(
            map, current_position, current_yaw, goal, connector_yaw,
            guide, guide_distance, strict_soft))
        {
          goal_state = entry.state;
          break;
        }
      }

      // A heading-aware footprint must be able to rotate before translating at
      // a tight corner. Without these states every successor couples a 45-degree
      // yaw change to a one-voxel translation, which can leave a valid start
      // with zero successors even though an in-place turn is collision-free.
      for (const int heading_delta : {-1, 1}) {
        const int next_heading = (current_heading + heading_delta + 8) % 8;
        const int next_state = current_cell_index * 8 + next_heading;
        if (closed[static_cast<std::size_t>(next_state)] != 0U) {continue;}
        const double from_yaw = headingYaw(current_heading);
        const double yaw_delta = wrapAngle(headingYaw(next_heading) - from_yaw);
        bool collision = false;
        double max_soft = 0.0;
        for (int sample = 1; sample <= 4; ++sample) {
          double soft = 0.0;
          const double yaw = from_yaw + yaw_delta * static_cast<double>(sample) / 4.0;
          if (!poseValid(map, current_position, yaw, strict_soft, &soft)) {
            if (strict_soft && poseValid(map, current_position, yaw, false)) {
              ++result.rejected_soft_layer;
            } else {
              ++result.rejected_hard_collision;
            }
            collision = true;
            break;
          }
          max_soft = std::max(max_soft, soft);
        }
        if (collision) {continue;}
        const double soft_multiplier = strict_soft ? 1.0 :
          1.0 + config_.soft_weight * max_soft;
        const float tentative = g[static_cast<std::size_t>(entry.state)] +
          static_cast<float>(config_.rotation_cost * soft_multiplier);
        if (tentative >= g[static_cast<std::size_t>(next_state)]) {continue;}
        g[static_cast<std::size_t>(next_state)] = tentative;
        parent[static_cast<std::size_t>(next_state)] = entry.state;
        const float heuristic = static_cast<float>((current_position - goal).norm());
        open.push(QueueEntry{tentative + heuristic, next_state});
      }

      for (int heading = 0; heading < 8; ++heading) {
        for (int dz = -1; dz <= 1; ++dz) {
          const Eigen::Vector3i next_cell = current_cell +
            Eigen::Vector3i(kDx[heading], kDy[heading], dz);
          const int next_cell_index = map.linear(next_cell);
          if (next_cell_index < 0) {
            ++result.rejected_outside_map;
            continue;
          }
          const Eigen::Vector3d next_position = map.cellCenter(next_cell);
          const GuideProjection projection = projection_for(next_cell_index, next_position);
          if (projection.distance_xy > config_.corridor_xy + 1e-9) {
            ++result.rejected_corridor_xy;
            continue;
          }
          if (std::abs(projection.z_error) > config_.corridor_z + 1e-9) {
            ++result.rejected_corridor_z;
            continue;
          }

          // The cached z_error is measured against the local guide rather than
          // a straight start-goal plane, preventing cross-floor shortcuts.
          if (kDx[heading] != 0 && kDy[heading] != 0) {
            const Eigen::Vector3d side_x = map.cellCenter(
              current_cell + Eigen::Vector3i(kDx[heading], 0, 0));
            const Eigen::Vector3d side_y = map.cellCenter(
              current_cell + Eigen::Vector3i(0, kDy[heading], 0));
            if (map.footprintAt(side_x) || map.footprintAt(side_y)) {
              ++result.rejected_diagonal_corner;
              continue;
            }
          }

          double max_soft = 0.0;
          const double edge_length = (next_position - current_position).norm();
          const int edge_samples = std::max(1, static_cast<int>(std::ceil(
                edge_length / std::max(1e-3, 0.5 * map.resolution))));
          const double from_yaw = headingYaw(current_heading);
          const double yaw_delta = wrapAngle(headingYaw(heading) - from_yaw);
          bool collision = false;
          for (int sample = 0; sample <= edge_samples; ++sample) {
            const double ratio = static_cast<double>(sample) / edge_samples;
            double soft = 0.0;
            if (!poseValid(
                map, current_position + ratio * (next_position - current_position),
                from_yaw + ratio * yaw_delta, strict_soft, &soft))
            {
              if (strict_soft && poseValid(
                  map, current_position + ratio * (next_position - current_position),
                  from_yaw + ratio * yaw_delta, false))
              {
                ++result.rejected_soft_layer;
              } else {
                ++result.rejected_hard_collision;
              }
              collision = true;
              break;
            }
            max_soft = std::max(max_soft, soft);
          }
          if (collision) {continue;}
          const int next_state = next_cell_index * 8 + heading;
          if (closed[static_cast<std::size_t>(next_state)] != 0U) {continue;}
          const Eigen::Vector3d delta = next_position - current_position;
          const double geometric = delta.norm();
          const double turn = std::abs(wrapAngle(
              headingYaw(heading) - headingYaw(current_heading))) / (kPi / 4.0);
          const double path_cost = config_.path_weight * projection.distance_xy * geometric;
          const double vertical_cost = config_.vertical_weight *
            (std::abs(delta.z()) + std::abs(projection.z_error) * geometric);
          const double soft_multiplier = strict_soft ? 1.0 :
            1.0 + config_.soft_weight * max_soft;
          const float tentative = g[static_cast<std::size_t>(entry.state)] +
            static_cast<float>(geometric * soft_multiplier + path_cost + vertical_cost +
            config_.turn_weight * turn);
          if (tentative >= g[static_cast<std::size_t>(next_state)]) {continue;}
          g[static_cast<std::size_t>(next_state)] = tentative;
          parent[static_cast<std::size_t>(next_state)] = entry.state;
          const float heuristic = static_cast<float>((next_position - goal).norm());
          open.push(QueueEntry{tentative + heuristic, next_state});
        }
      }
    }

    if (goal_state < 0) {
      if (result.expansions >= expansion_limit) {
        result.reason = strict_soft ? "strict_expansion_limit" : "relaxed_expansion_limit";
      }
      return result;
    }
    std::vector<Eigen::Vector3d> reversed;
    for (int state = goal_state; state >= 0; state = parent[static_cast<std::size_t>(state)]) {
      const Eigen::Vector3d point = state / 8 == start_cell ? requested_start :
        map.cellCenter(map.cellFromLinear(state / 8));
      if (reversed.empty() || (point - reversed.back()).norm() > 1e-9) {
        reversed.push_back(point);
      }
      if (state == start_state) {break;}
    }
    if (reversed.empty() || (reversed.back() - requested_start).norm() > map.resolution * 2.0) {
      result.reason = "broken_parent_chain";
      return result;
    }
    result.path.assign(reversed.rbegin(), reversed.rend());
    if ((result.path.back() - goal).norm() > 1e-9) {
      result.path.push_back(goal);
    }
    result.success = true;
    result.reason = strict_soft ? "strict_success" : "relaxed_success";
    result.used_soft = !strict_soft;
    for (const auto & point : result.path) {
      result.maximum_soft_cost = std::max(result.maximum_soft_cost, map.softAt(point));
    }
    return result;
  }

  Config config_;
};

}  // namespace efficient_3d_local_planner
