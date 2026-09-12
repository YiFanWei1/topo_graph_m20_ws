#ifndef ROUTE3D_ROUTE_SLICER__ROUTE_SLICER_HPP_
#define ROUTE3D_ROUTE_SLICER__ROUTE_SLICER_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "route3d_dijkstra_planner/topology_graph.hpp"

namespace route3d_route_slicer
{

struct SliceOptions
{
  bool enable_slope_gait{true};
  bool slope_ignore_ordinary_obstacles{true};
  int normal_locomotion_mode{0};
  int slope_locomotion_mode{2};
  double attribute_epsilon{1.0e-6};
  bool split_at_corners{true};
  double default_corner_pass_radius_m{0.20};
  double default_normal_pass_radius_m{0.45};
  std::string auto_default_controller{"pid"};
  std::string auto_grid_controller{"local_planner"};
  std::string auto_slope_controller{"efficient_3d_local_planner"};
  std::string normal_gait_command{"static_walk"};
  std::string slope_gait_command{"switch_gait_3"};
};

struct WaypointConstraint
{
  route3d_dijkstra_planner::VertexId vertex_id{0};
  route3d_dijkstra_planner::Point3 position{};
  route3d_dijkstra_planner::Point3 rpy{};
  int semantic_type{0};
  int semantic_type_id{0};
  int charging_mode{0};
  bool is_corner{false};
  bool is_slope{false};
  bool is_junction{false};
  double turn_degrees{0.0};
  bool must_pass_through{false};
  double pass_radius_m{0.45};
  double goal_tolerance_m{0.5};
  bool align_final_yaw{true};
  bool turnable{true};
  std::string pcd_name;
};

enum class CompletionPolicy : std::uint8_t
{
  kTransition = 0,
  kRouteGoal = 1,
  kBusinessStop = 2,
};

struct RouteTask
{
  std::size_t task_index{0};
  std::string task_mode{"normal"};
  std::string configured_controller_mode{"auto"};
  std::string resolved_controller_mode{"pid"};
  std::string gait_command{"static_walk"};
  CompletionPolicy completion_policy{CompletionPolicy::kTransition};
  bool is_route_goal{false};
  bool requires_stop_at_end{false};
  bool requires_gait_switch_at_start{false};
  bool contains_slope{false};
  bool reverse_motion{false};
  bool align_goal_yaw{false};
  double endpoint_tolerance_m{0.5};
  int locomotion_mode{0};
  double linear_speed_mps{1.0};
  double angular_speed_radps{0.0};
  double height_offset_m{0.0};
  int obstacle_mode{0};
  std::array<double, 4> obstacle_box_m{};
  std::string grid_map_name;
  bool rotation_allowed{true};
  std::vector<WaypointConstraint> waypoints;
  std::vector<route3d_dijkstra_planner::EdgeId> edge_ids;
  std::vector<std::string> split_reasons;
};

struct SliceResult
{
  route3d_dijkstra_planner::VertexId route_start_id{0};
  route3d_dijkstra_planner::VertexId route_goal_id{0};
  std::vector<RouteTask> tasks;
};

class RouteSlicer
{
public:
  explicit RouteSlicer(SliceOptions options = {});

  SliceResult slice(
    const route3d_dijkstra_planner::TopologyGraph & graph,
    const std::vector<route3d_dijkstra_planner::VertexId> & vertex_ids,
    const std::vector<route3d_dijkstra_planner::EdgeId> & edge_ids) const;

private:
  SliceOptions options_;
};

}  // namespace route3d_route_slicer

#endif  // ROUTE3D_ROUTE_SLICER__ROUTE_SLICER_HPP_
