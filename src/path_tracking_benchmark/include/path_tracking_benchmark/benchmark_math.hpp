#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace path_tracking_benchmark
{

constexpr double kPi = 3.14159265358979323846;

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Projection
{
  bool valid{false};
  Point3 point;
  double arc{0.0};
  double distance_xy{std::numeric_limits<double>::infinity()};
  double distance_3d{std::numeric_limits<double>::infinity()};
  double signed_lateral{0.0};
  double yaw{0.0};
};

inline double norm3(const Point3 & point)
{
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

inline Point3 subtract(const Point3 & left, const Point3 & right)
{
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

inline double distance3d(const Point3 & left, const Point3 & right)
{
  return norm3(subtract(left, right));
}

inline double normalizeAngle(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}

inline std::vector<double> cumulativeDistance(const std::vector<Point3> & path)
{
  std::vector<double> result(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    result[i] = result[i - 1] + distance3d(path[i], path[i - 1]);
  }
  return result;
}

inline Projection projectToPath(const Point3 & query, const std::vector<Point3> & path)
{
  Projection best;
  if (path.size() < 2) {return best;}
  const auto cumulative = cumulativeDistance(path);
  double best_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Point3 delta = subtract(path[i + 1], path[i]);
    const double denominator = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
    if (denominator < 1e-12) {continue;}
    const Point3 to_query = subtract(query, path[i]);
    const double ratio = std::clamp(
      (to_query.x * delta.x + to_query.y * delta.y + to_query.z * delta.z) / denominator,
      0.0, 1.0);
    const Point3 candidate{
      path[i].x + ratio * delta.x, path[i].y + ratio * delta.y,
      path[i].z + ratio * delta.z};
    const Point3 error = subtract(query, candidate);
    const double squared = error.x * error.x + error.y * error.y + error.z * error.z;
    if (squared >= best_squared) {continue;}
    best_squared = squared;
    best.valid = true;
    best.point = candidate;
    best.arc = cumulative[i] + ratio * (cumulative[i + 1] - cumulative[i]);
    best.distance_xy = std::hypot(error.x, error.y);
    best.distance_3d = std::sqrt(squared);
    const double tangent_xy = std::hypot(delta.x, delta.y);
    best.yaw = tangent_xy > 1e-9 ? std::atan2(delta.y, delta.x) : 0.0;
    best.signed_lateral = tangent_xy > 1e-9 ?
      (delta.x * error.y - delta.y * error.x) / tangent_xy : 0.0;
  }
  return best;
}

inline std::vector<Point3> resampleParametric(
  const double length, const double spacing,
  const std::function<Point3(double)> & evaluator)
{
  const int steps = std::max(1, static_cast<int>(std::ceil(length / std::max(1e-3, spacing))));
  std::vector<Point3> result;
  result.reserve(static_cast<std::size_t>(steps + 1));
  for (int i = 0; i <= steps; ++i) {
    result.push_back(evaluator(static_cast<double>(i) / static_cast<double>(steps)));
  }
  return result;
}

inline std::vector<Point3> makeLocalTrajectory(
  const std::string & type, const double spacing)
{
  if (type == "straight") {
    return resampleParametric(3.0, spacing, [](double u) {return Point3{3.0 * u, 0.0, 0.0};});
  }
  if (type == "arc") {
    return resampleParametric(0.5 * kPi, spacing, [](double u) {
      const double theta = 0.5 * kPi * u;
      return Point3{std::sin(theta), 1.0 - std::cos(theta), 0.0};
    });
  }
  if (type == "s_curve") {
    return resampleParametric(3.0, spacing, [](double u) {
      const double sine = std::sin(2.0 * kPi * u);
      return Point3{3.0 * u, 0.5 * sine * sine * sine, 0.0};
    });
  }
  if (type == "right_angle") {
    const int steps = std::max(1, static_cast<int>(std::ceil(1.5 / std::max(1e-3, spacing))));
    std::vector<Point3> result;
    result.reserve(static_cast<std::size_t>(2 * steps + 1));
    for (int i = 0; i <= steps; ++i) {result.push_back({1.5 * i / steps, 0.0, 0.0});}
    for (int i = 1; i <= steps; ++i) {result.push_back({1.5, 1.5 * i / steps, 0.0});}
    return result;
  }
  return {};
}

inline std::vector<Point3> transformTrajectory(
  const std::vector<Point3> & local, const Point3 & origin, const double yaw)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  std::vector<Point3> result;
  result.reserve(local.size());
  for (const Point3 & point : local) {
    result.push_back({
      origin.x + cosine * point.x - sine * point.y,
      origin.y + sine * point.x + cosine * point.y,
      origin.z + point.z});
  }
  return result;
}

inline double percentile(std::vector<double> values, const double fraction)
{
  if (values.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(std::floor(
    std::clamp(fraction, 0.0, 1.0) * static_cast<double>(values.size() - 1)));
  return values[index];
}

}  // namespace path_tracking_benchmark
