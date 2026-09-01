#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include "recorded_waypoint_route/route_sequencer_core.hpp"

namespace recorded_waypoint_route
{

// Keep all route-attribute-to-runtime-parameter decisions in this resolver.
// Adding a future door/charging/narrow-corridor profile should only require
// extending RoutePointAttributes and this one resolve() function.
struct RoutePointAttributes
{
  TargetType target_type{TargetType::Normal};
  bool is_slope{false};
};

struct PlanningProfile
{
  std::string name{"normal"};
  double path_height{0.0};
  bool extension_enabled{true};
  double soft_radius{0.60};

  bool operator==(const PlanningProfile & other) const
  {
    return name == other.name &&
           std::abs(path_height - other.path_height) <= 1e-9 &&
           extension_enabled == other.extension_enabled &&
           std::abs(soft_radius - other.soft_radius) <= 1e-9;
  }
};

class PlanningProfileResolver
{
public:
  PlanningProfileResolver(PlanningProfile normal, PlanningProfile slope)
  : normal_(std::move(normal)), slope_(std::move(slope))
  {
    validate(normal_);
    validate(slope_);
  }

  PlanningProfile resolve(const RoutePointAttributes & attributes) const
  {
    // Corner remains part of the extensible input even though the first profile
    // rule only distinguishes normal terrain from slope terrain.
    (void)attributes.target_type;
    return attributes.is_slope ? slope_ : normal_;
  }

private:
  static void validate(const PlanningProfile & profile)
  {
    if (profile.name.empty() || !std::isfinite(profile.path_height) ||
      profile.path_height < 0.0 || !std::isfinite(profile.soft_radius) ||
      profile.soft_radius < 0.0)
    {
      throw std::invalid_argument(
              "planning profiles require a name, non-negative height, and soft radius");
    }
  }

  PlanningProfile normal_;
  PlanningProfile slope_;
};

}  // namespace recorded_waypoint_route
