#include "route3d_route_slicer/route_slicer.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace route3d_route_slicer
{
namespace
{

using route3d_dijkstra_planner::EdgeId;
using route3d_dijkstra_planner::TopologyEdge;
using route3d_dijkstra_planner::TopologyGraph;
using route3d_dijkstra_planner::TopologyVertex;
using route3d_dijkstra_planner::VertexId;

struct EffectiveAttributes
{
  std::string configured_controller_mode;
  std::string resolved_controller_mode;
  std::string gait_command;
  int locomotion_mode{0};
  double linear_speed_mps{0.0};
  double angular_speed_radps{0.0};
  double height_offset_m{0.0};
  int obstacle_mode{0};
  std::array<double, 4> obstacle_box_m{};
  std::string grid_map_name;
  bool rotation_allowed{true};
  bool contains_slope{false};
};

bool near(const double left, const double right, const double epsilon)
{
  return std::abs(left - right) <= epsilon;
}

std::string businessMode(const int current_type, const int next_type)
{
  if (current_type == 2 && next_type == 1) {
    return "normal_charging";
  }
  if (current_type == 6 && next_type == 6) {
    return "map_change";
  }
  if (current_type == 7 && next_type == 3) {
    return "retreat_with_door";
  }
  if (current_type == 3 && next_type == 7) {
    return "charging_with_door";
  }
  if (current_type != 7 && next_type == 7) {
    return "single_charging";
  }
  if (current_type == 3 && next_type == 3) {
    return "door";
  }
  if (current_type == 3 && next_type == 8) {
    return "open_door";
  }
  if (current_type == 8 && next_type == 3) {
    return "close_door";
  }
  return "normal";
}

std::string resolveController(
  const TopologyEdge & edge, const bool contains_slope, const SliceOptions & options)
{
  if (edge.controller_mode != "auto") {
    return edge.controller_mode;
  }
  if (contains_slope) {
    return options.auto_slope_controller;
  }
  if (edge.obstacle_mode == 4 || !edge.grid_map_name.empty()) {
    return options.auto_grid_controller;
  }
  return options.auto_default_controller;
}

EffectiveAttributes effectiveAttributes(
  const TopologyEdge & edge, const SliceOptions & options)
{
  EffectiveAttributes result;
  result.configured_controller_mode = edge.controller_mode;
  // TopologyGraph performs one-pass point/edge synchronization when it is
  // constructed.  Use that frozen edge result here instead of re-deriving it
  // from promoted boundary vertices, which would spread a slope recursively.
  result.contains_slope = edge.is_slope;
  result.resolved_controller_mode = resolveController(edge, result.contains_slope, options);
  result.locomotion_mode = edge.locomotion_mode;
  if (options.enable_slope_gait && result.contains_slope &&
    result.locomotion_mode == options.normal_locomotion_mode)
  {
    result.locomotion_mode = options.slope_locomotion_mode;
  }
  result.gait_command =
    result.locomotion_mode == options.slope_locomotion_mode ?
    options.slope_gait_command : options.normal_gait_command;
  result.linear_speed_mps = edge.linear_speed_mps;
  result.angular_speed_radps = edge.angular_speed_radps;
  result.height_offset_m = edge.height_offset_m;
  result.obstacle_mode = edge.obstacle_mode;
  // The reference controller enters its stair/climbing tasks with obs=3.  A
  // generated graph only knows that the end vertices are slope vertices, so
  // reproduce that effective task attribute here.  Explicit non-default
  // obstacle policies (replan/grid navigation) remain authoritative.
  if (options.slope_ignore_ordinary_obstacles && result.contains_slope &&
    result.obstacle_mode == 0)
  {
    result.obstacle_mode = 3;
  }
  result.obstacle_box_m = edge.obstacle_box_m;
  result.grid_map_name = edge.grid_map_name;
  result.rotation_allowed = edge.rotation_allowed;
  return result;
}

std::vector<std::string> differences(
  const EffectiveAttributes & previous, const EffectiveAttributes & current,
  const double epsilon)
{
  std::vector<std::string> result;
  if (previous.configured_controller_mode != current.configured_controller_mode ||
    previous.resolved_controller_mode != current.resolved_controller_mode)
  {
    result.emplace_back("controller_mode_changed");
  }
  if (previous.locomotion_mode != current.locomotion_mode ||
    previous.gait_command != current.gait_command)
  {
    result.emplace_back("locomotion_mode_changed");
  }
  if (!near(previous.linear_speed_mps, current.linear_speed_mps, epsilon)) {
    result.emplace_back("linear_speed_changed");
  }
  if (!near(previous.angular_speed_radps, current.angular_speed_radps, epsilon)) {
    result.emplace_back("angular_speed_changed");
  }
  if (!near(previous.height_offset_m, current.height_offset_m, epsilon)) {
    result.emplace_back("height_offset_changed");
  }
  if (previous.obstacle_mode != current.obstacle_mode) {
    result.emplace_back("obstacle_mode_changed");
  }
  for (std::size_t index = 0; index < previous.obstacle_box_m.size(); ++index) {
    if (!near(previous.obstacle_box_m[index], current.obstacle_box_m[index], epsilon)) {
      result.emplace_back("obstacle_box_changed");
      break;
    }
  }
  if (previous.grid_map_name != current.grid_map_name) {
    result.emplace_back("grid_map_changed");
  }
  if (previous.rotation_allowed != current.rotation_allowed) {
    result.emplace_back("rotation_permission_changed");
  }
  return result;
}

WaypointConstraint waypoint(const TopologyVertex & vertex, const SliceOptions & options)
{
  WaypointConstraint result;
  result.vertex_id = vertex.id;
  result.position = vertex.position;
  result.rpy = vertex.rpy;
  result.semantic_type = vertex.semantic_type;
  result.semantic_type_id = vertex.semantic_type_id;
  result.charging_mode = vertex.charging_mode;
  result.is_corner = vertex.is_corner;
  result.is_slope = vertex.is_slope;
  result.is_junction = vertex.is_junction;
  result.turn_degrees = vertex.turn_degrees;
  result.must_pass_through = vertex.must_pass_through_explicit ?
    vertex.must_pass_through : vertex.is_corner;
  result.pass_radius_m = vertex.pass_radius_explicit ? vertex.pass_radius_m :
    (vertex.is_corner ? options.default_corner_pass_radius_m :
    options.default_normal_pass_radius_m);
  result.goal_tolerance_m = vertex.goal_tolerance_m;
  result.align_final_yaw = vertex.align_final_yaw;
  result.turnable = vertex.turnable;
  result.pcd_name = vertex.pcd_name;
  return result;
}

void assignAttributes(RouteTask & task, const EffectiveAttributes & attributes)
{
  task.configured_controller_mode = attributes.configured_controller_mode;
  task.resolved_controller_mode = attributes.resolved_controller_mode;
  task.gait_command = attributes.gait_command;
  task.locomotion_mode = attributes.locomotion_mode;
  task.linear_speed_mps = attributes.linear_speed_mps;
  task.angular_speed_radps = attributes.angular_speed_radps;
  task.height_offset_m = attributes.height_offset_m;
  task.obstacle_mode = attributes.obstacle_mode;
  task.obstacle_box_m = attributes.obstacle_box_m;
  task.grid_map_name = attributes.grid_map_name;
  task.rotation_allowed = attributes.rotation_allowed;
  task.contains_slope = attributes.contains_slope;
  task.reverse_motion = task.locomotion_mode == 1 || task.locomotion_mode == 3;
}

bool isObstacleSingleton(const EffectiveAttributes & attributes)
{
  return attributes.obstacle_mode == 1 || attributes.obstacle_mode == 4;
}

void validateTraversal(
  const TopologyEdge & edge, const VertexId from, const VertexId to)
{
  const bool stored_forward = edge.first == from && edge.second == to;
  const bool stored_reverse = edge.first == to && edge.second == from;
  if (!stored_forward && !stored_reverse) {
    throw std::invalid_argument(
            "edge " + std::to_string(edge.id) + " does not connect route vertices " +
            std::to_string(from) + " and " + std::to_string(to));
  }
  if ((stored_forward && edge.travel_mode == "second_to_first") ||
    (stored_reverse && edge.travel_mode == "first_to_second"))
  {
    throw std::invalid_argument(
            "route traverses edge " + std::to_string(edge.id) + " against travelMode");
  }
}

}  // namespace

RouteSlicer::RouteSlicer(SliceOptions options)
: options_(std::move(options))
{
  if (options_.attribute_epsilon < 0.0 ||
    options_.default_corner_pass_radius_m < 0.0 ||
    options_.default_normal_pass_radius_m < 0.0)
  {
    throw std::invalid_argument("route slicer tolerances must be non-negative");
  }
  if (options_.auto_default_controller.empty() || options_.auto_grid_controller.empty() ||
    options_.auto_slope_controller.empty() ||
    options_.normal_gait_command.empty() || options_.slope_gait_command.empty())
  {
    throw std::invalid_argument("controller and gait command names must not be empty");
  }
}

SliceResult RouteSlicer::slice(
  const TopologyGraph & graph, const std::vector<VertexId> & vertex_ids,
  const std::vector<EdgeId> & edge_ids) const
{
  if (vertex_ids.empty() || edge_ids.size() + 1U != vertex_ids.size()) {
    throw std::invalid_argument("route must contain N vertices and N-1 edges, with N >= 1");
  }

  SliceResult result;
  result.route_start_id = vertex_ids.front();
  result.route_goal_id = vertex_ids.back();
  if (vertex_ids.size() == 1U) {
    const auto & goal = graph.vertex(vertex_ids.front());
    RouteTask task;
    task.task_index = 0U;
    task.task_mode = "normal";
    task.configured_controller_mode = "auto";
    task.resolved_controller_mode = options_.auto_default_controller;
    task.gait_command = options_.normal_gait_command;
    task.completion_policy = CompletionPolicy::kRouteGoal;
    task.is_route_goal = true;
    task.requires_stop_at_end = true;
    task.endpoint_tolerance_m = goal.goal_tolerance_m;
    task.align_goal_yaw = goal.align_final_yaw;
    task.locomotion_mode = options_.normal_locomotion_mode;
    task.waypoints.push_back(waypoint(goal, options_));
    task.split_reasons.emplace_back("same_vertex_goal");
    result.tasks.push_back(std::move(task));
    return result;
  }
  std::optional<RouteTask> current_task;
  std::optional<EffectiveAttributes> previous_attributes;
  EdgeId previous_edge_id{0};

  auto finish_current = [&]() {
      if (!current_task.has_value()) {
        return;
      }
      if (current_task->edge_ids.empty() ||
        current_task->waypoints.size() != current_task->edge_ids.size() + 1U)
      {
        throw std::logic_error("internal slicer task invariant failed");
      }
      result.tasks.push_back(std::move(*current_task));
      current_task.reset();
    };

  auto start_task = [&](const std::string & mode, const EffectiveAttributes & attributes,
      const TopologyVertex & from, const TopologyVertex & to, const EdgeId edge_id,
      std::vector<std::string> reasons) {
      RouteTask task;
      task.task_mode = mode;
      task.split_reasons = std::move(reasons);
      assignAttributes(task, attributes);
      task.waypoints.push_back(waypoint(from, options_));
      task.waypoints.push_back(waypoint(to, options_));
      task.edge_ids.push_back(edge_id);
      current_task = std::move(task);
    };

  for (std::size_t index = 0; index < edge_ids.size(); ++index) {
    const auto & from = graph.vertex(vertex_ids[index]);
    const auto & to = graph.vertex(vertex_ids[index + 1U]);
    const auto & edge = graph.edge(edge_ids[index]);
    validateTraversal(edge, from.id, to.id);
    if (edge.obstacle_mode == 4 && edge.grid_map_name.empty()) {
      throw std::invalid_argument(
              "edge " + std::to_string(edge.id) +
              " uses obstacleMode=4 but gridMapName is empty");
    }

    const auto attributes = effectiveAttributes(edge, options_);
    const auto business_mode = businessMode(from.semantic_type, to.semantic_type);
    const bool special_business = business_mode != "normal";
    const bool singleton_obstacle = isObstacleSingleton(attributes);

    if (special_business || singleton_obstacle) {
      finish_current();
      std::vector<std::string> reasons;
      reasons.emplace_back(special_business ? "business_node_pair" : "isolated_obstacle_edge");
      start_task(business_mode, attributes, from, to, edge.id, std::move(reasons));
      finish_current();
      previous_attributes = attributes;
      previous_edge_id = edge.id;
      continue;
    }

    std::vector<std::string> split_reasons;
    if (previous_attributes.has_value()) {
      split_reasons = differences(*previous_attributes, attributes, options_.attribute_epsilon);
      if (previous_edge_id == edge.id) {
        split_reasons.emplace_back("immediate_edge_retrace");
      }
    }

    if (!current_task.has_value() || !split_reasons.empty()) {
      finish_current();
      if (split_reasons.empty()) {
        split_reasons.emplace_back(index == 0U ? "route_start" : "after_singleton_task");
      }
      start_task("normal", attributes, from, to, edge.id, std::move(split_reasons));
    } else {
      current_task->edge_ids.push_back(edge.id);
      current_task->waypoints.push_back(waypoint(to, options_));
      current_task->contains_slope = current_task->contains_slope || attributes.contains_slope;
    }
    previous_attributes = attributes;
    previous_edge_id = edge.id;
  }
  finish_current();

  if (result.tasks.empty()) {
    throw std::logic_error("slicer produced no tasks");
  }
  for (std::size_t index = 0; index < result.tasks.size(); ++index) {
    auto & task = result.tasks[index];
    task.task_index = index;
    task.is_route_goal = index + 1U == result.tasks.size();
    const bool business_task = task.task_mode != "normal";
    task.completion_policy = business_task ? CompletionPolicy::kBusinessStop :
      (task.is_route_goal ? CompletionPolicy::kRouteGoal : CompletionPolicy::kTransition);
    task.endpoint_tolerance_m = task.is_route_goal || business_task ?
      task.waypoints.back().goal_tolerance_m : task.waypoints.back().pass_radius_m;
    task.align_goal_yaw = (task.is_route_goal || business_task) &&
      task.waypoints.back().align_final_yaw;
    // The robot's gait before a new route is unknown.  Always establish the
    // first task's requested gait explicitly; later tasks only switch when the
    // locomotion mode changes.
    const bool gait_changed = index == 0U ||
      result.tasks[index - 1U].locomotion_mode != task.locomotion_mode;
    task.requires_gait_switch_at_start = gait_changed;
    bool next_requires_hard_switch = false;
    if (index + 1U < result.tasks.size()) {
      const auto & next = result.tasks[index + 1U];
      next_requires_hard_switch =
        task.locomotion_mode != next.locomotion_mode ||
        task.resolved_controller_mode != next.resolved_controller_mode ||
        next.task_mode != "normal";
    }
    task.requires_stop_at_end = task.is_route_goal || business_task || next_requires_hard_switch;
  }
  return result;
}

}  // namespace route3d_route_slicer
