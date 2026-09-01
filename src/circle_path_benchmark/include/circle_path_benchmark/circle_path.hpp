#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace circle_path_benchmark
{

constexpr double kPi = 3.14159265358979323846;

struct CircleSpec
{
  double center_x{0.0};
  double center_y{0.0};
  double path_z{0.0};
  double radius{0.8};
  double sample_spacing{0.05};
  double start_angle{0.0};
  bool clockwise{false};
};

struct CirclePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

inline double normalizeAngle(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}

inline double startAngleForTangentYaw(const CircleSpec & spec, double tangent_yaw)
{
  const double direction_sign = spec.clockwise ? -1.0 : 1.0;
  return normalizeAngle(tangent_yaw - direction_sign * 0.5 * kPi);
}

inline void validateCircleSpec(const CircleSpec & spec)
{
  if (!std::isfinite(spec.center_x) || !std::isfinite(spec.center_y) ||
    !std::isfinite(spec.path_z) || !std::isfinite(spec.radius) ||
    !std::isfinite(spec.sample_spacing) || !std::isfinite(spec.start_angle) ||
    spec.radius <= 0.0 || spec.sample_spacing <= 0.0)
  {
    throw std::invalid_argument("circle parameters must be finite and positive");
  }
}

inline std::vector<CirclePoint> generateCircle(const CircleSpec & spec)
{
  validateCircleSpec(spec);
  const double circumference = 2.0 * kPi * spec.radius;
  const int segments = std::max(8, static_cast<int>(std::ceil(
        circumference / spec.sample_spacing)));
  std::vector<CirclePoint> points;
  points.reserve(static_cast<std::size_t>(segments) + 1U);
  const double sign = spec.clockwise ? -1.0 : 1.0;
  for (int index = 0; index <= segments; ++index) {
    const double angle = spec.start_angle + sign * 2.0 * kPi *
      static_cast<double>(index) / static_cast<double>(segments);
    const double tangent_yaw = angle + sign * 0.5 * kPi;
    points.push_back(CirclePoint{
      spec.center_x + spec.radius * std::cos(angle),
      spec.center_y + spec.radius * std::sin(angle),
      spec.path_z,
      normalizeAngle(tangent_yaw),
    });
  }
  return points;
}

inline double radialError(const CircleSpec & spec, double x, double y)
{
  return std::hypot(x - spec.center_x, y - spec.center_y) - spec.radius;
}

inline double directedAngularDelta(
  const CircleSpec & spec, double previous_angle, double current_angle)
{
  const double physical_delta = normalizeAngle(current_angle - previous_angle);
  return spec.clockwise ? -physical_delta : physical_delta;
}

}  // namespace circle_path_benchmark
