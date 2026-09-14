#include <gtest/gtest.h>

#include <unordered_map>
#include <utility>
#include <vector>

#include "route3d_dijkstra_planner/topology_graph.hpp"
#include "route3d_route_slicer/route_slicer.hpp"

namespace
{

using route3d_dijkstra_planner::EdgeId;
using route3d_dijkstra_planner::TopologyEdge;
using route3d_dijkstra_planner::TopologyGraph;
using route3d_dijkstra_planner::TopologyVertex;
using route3d_dijkstra_planner::VertexId;
using route3d_route_slicer::CompletionPolicy;
using route3d_route_slicer::RouteSlicer;

TopologyVertex vertex(const VertexId id, const bool corner = false, const bool slope = false)
{
  TopologyVertex result;
  result.id = id;
  result.position = {static_cast<double>(id - 1), 0.0, slope ? 0.2 : 0.0};
  result.is_corner = corner;
  result.is_slope = slope;
  result.turn_degrees = corner ? 90.0 : 0.0;
  return result;
}

TopologyEdge edge(const EdgeId id, const VertexId first, const VertexId second)
{
  TopologyEdge result;
  result.id = id;
  result.first = first;
  result.second = second;
  result.weight = 1.0;
  result.travel_mode = "bidirectional";
  result.controller_mode = "auto";
  result.linear_speed_mps = 1.0;
  return result;
}

TopologyGraph lineGraph(
  std::vector<TopologyVertex> vertices, std::vector<TopologyEdge> edges)
{
  std::unordered_map<VertexId, TopologyVertex> vertex_map;
  std::unordered_map<EdgeId, TopologyEdge> edge_map;
  for (auto & item : vertices) {
    vertex_map.emplace(item.id, std::move(item));
  }
  for (auto & item : edges) {
    edge_map.emplace(item.id, std::move(item));
  }
  return TopologyGraph("map", "normal", std::move(vertex_map), std::move(edge_map));
}

TEST(RouteSlicer, SlopeAnnotationsDoNotSelectAController)
{
  auto graph = lineGraph(
    {vertex(1), vertex(2), vertex(3, false, true), vertex(4, false, true),
      vertex(5), vertex(6), vertex(7)},
    {edge(1, 1, 2), edge(2, 2, 3), edge(3, 3, 4), edge(4, 4, 5),
      edge(5, 5, 6), edge(6, 6, 7)});

  EXPECT_FALSE(graph.edge(1).is_slope);
  EXPECT_TRUE(graph.edge(2).is_slope);
  EXPECT_TRUE(graph.edge(3).is_slope);
  EXPECT_TRUE(graph.edge(4).is_slope);
  EXPECT_FALSE(graph.edge(5).is_slope);
  EXPECT_TRUE(graph.vertex(2).is_slope);
  EXPECT_TRUE(graph.vertex(5).is_slope);
  EXPECT_FALSE(graph.vertex(1).is_slope);
  EXPECT_FALSE(graph.vertex(6).is_slope);

  const auto result = RouteSlicer().slice(graph, {1, 2, 3, 4, 5, 6, 7}, {1, 2, 3, 4, 5, 6});

  ASSERT_EQ(result.tasks.size(), 1U);
  EXPECT_EQ(result.tasks[0].edge_ids, (std::vector<EdgeId>{1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(result.tasks[0].contains_slope);
  EXPECT_EQ(result.tasks[0].resolved_controller_mode, "pid");
  EXPECT_EQ(result.tasks[0].obstacle_mode, 0);
  EXPECT_TRUE(result.tasks[0].is_route_goal);
  EXPECT_TRUE(result.tasks[0].align_goal_yaw);
}

TEST(RouteSlicer, ObstacleModesHaveM20ControllerPrecedence)
{
  auto avoid = edge(1, 1, 2);
  avoid.obstacle_mode = 1;
  avoid.controller_mode = "pid";
  auto blind_pid = edge(2, 2, 3);
  blind_pid.obstacle_mode = 2;
  blind_pid.controller_mode = "efficient_3d_local_planner";
  auto legacy_ignore = edge(3, 3, 4);
  legacy_ignore.obstacle_mode = 3;
  legacy_ignore.controller_mode = "efficient_3d_local_planner";
  auto grid = edge(4, 4, 5);
  grid.obstacle_mode = 4;
  grid.grid_map_name = "m20_test_grid";
  auto graph = lineGraph(
    {vertex(1), vertex(2), vertex(3), vertex(4), vertex(5)},
    {avoid, blind_pid, legacy_ignore, grid});

  const auto result = RouteSlicer().slice(graph, {1, 2, 3, 4, 5}, {1, 2, 3, 4});

  ASSERT_EQ(result.tasks.size(), 5U);
  EXPECT_EQ(result.tasks[0].resolved_controller_mode, "efficient_3d_local_planner");
  EXPECT_EQ(result.tasks[0].obstacle_mode, 1);
  EXPECT_EQ(result.tasks[1].resolved_controller_mode, "pid");
  EXPECT_EQ(result.tasks[1].obstacle_mode, 2);
  EXPECT_EQ(result.tasks[2].resolved_controller_mode, "efficient_3d_local_planner");
  EXPECT_EQ(result.tasks[2].obstacle_mode, 3);
  EXPECT_EQ(result.tasks[3].resolved_controller_mode, "external_grid");
  EXPECT_EQ(result.tasks[3].obstacle_mode, 4);
  EXPECT_EQ(result.tasks[3].grid_map_name, "m20_test_grid");
  EXPECT_EQ(result.tasks[4].resolved_controller_mode, "pid");
  EXPECT_TRUE(result.tasks[4].is_route_goal);
}

TEST(RouteSlicer, SlopeGoalAlwaysAligns)
{
  auto graph = lineGraph(
    {vertex(1), vertex(2, false, true)},
    {edge(1, 1, 2)});

  const auto result = RouteSlicer().slice(graph, {1, 2}, {1});

  ASSERT_EQ(result.tasks.size(), 1U);
  const auto & task = result.tasks.front();
  EXPECT_TRUE(task.is_route_goal);
  EXPECT_EQ(task.resolved_controller_mode, "pid");
  EXPECT_TRUE(task.align_goal_yaw);
}

TEST(RouteSlicer, EfficientApproachToConfiguredFlatGoalAddsPidAlignment)
{
  auto slope_edge = edge(1, 1, 2);
  slope_edge.obstacle_mode = 1;
  auto graph = lineGraph({vertex(1), vertex(2)}, {slope_edge});

  ASSERT_FALSE(graph.vertex(2).configured_is_slope);
  ASSERT_FALSE(graph.vertex(2).is_slope);

  const auto result = RouteSlicer().slice(graph, {1, 2}, {1});

  ASSERT_EQ(result.tasks.size(), 2U);
  const auto & approach = result.tasks[0];
  EXPECT_EQ(approach.resolved_controller_mode, "efficient_3d_local_planner");
  EXPECT_FALSE(approach.is_route_goal);
  EXPECT_FALSE(approach.align_goal_yaw);
  EXPECT_TRUE(approach.requires_stop_at_end);

  const auto & alignment = result.tasks[1];
  EXPECT_EQ(alignment.resolved_controller_mode, "pid");
  EXPECT_TRUE(alignment.is_route_goal);
  EXPECT_TRUE(alignment.align_goal_yaw);
  ASSERT_EQ(alignment.waypoints.size(), 1U);
  EXPECT_TRUE(alignment.edge_ids.empty());
  EXPECT_EQ(alignment.waypoints.front().vertex_id, 2);
}

TEST(RouteSlicer, ExplicitIntermediateYawCreatesPidAlignmentTask)
{
  auto middle = vertex(2);
  middle.align_final_yaw = true;
  auto avoid_first = edge(1, 1, 2);
  avoid_first.obstacle_mode = 1;
  auto graph = lineGraph(
    {vertex(1), middle, vertex(3)}, {avoid_first, edge(2, 2, 3)});

  const auto result = RouteSlicer().slice(graph, {1, 2, 3}, {1, 2});

  ASSERT_EQ(result.tasks.size(), 3U);
  EXPECT_EQ(result.tasks[0].resolved_controller_mode, "efficient_3d_local_planner");
  EXPECT_FALSE(result.tasks[0].align_goal_yaw);
  EXPECT_EQ(result.tasks[1].resolved_controller_mode, "pid");
  EXPECT_TRUE(result.tasks[1].align_goal_yaw);
  EXPECT_FALSE(result.tasks[1].is_route_goal);
  EXPECT_EQ(result.tasks[1].waypoints.front().vertex_id, 2);
  EXPECT_TRUE(result.tasks[2].is_route_goal);
  EXPECT_TRUE(result.tasks[2].align_goal_yaw);
}

TEST(RouteSlicer, ExplicitSlopeObstaclePolicyIsNotOverwritten)
{
  auto slope_edge = edge(1, 1, 2);
  slope_edge.obstacle_mode = 1;
  auto graph = lineGraph({vertex(1), vertex(2, false, true)}, {slope_edge});

  const auto result = RouteSlicer().slice(graph, {1, 2}, {1});

  ASSERT_EQ(result.tasks.size(), 2U);
  EXPECT_TRUE(result.tasks.front().contains_slope);
  EXPECT_EQ(result.tasks.front().obstacle_mode, 1);
  EXPECT_EQ(result.tasks.front().resolved_controller_mode, "efficient_3d_local_planner");
  EXPECT_EQ(result.tasks.back().resolved_controller_mode, "pid");
}

TEST(RouteSlicer, CornerCreatesTaskBoundaryWithItsPassRadius)
{
  auto goal = vertex(5);
  goal.goal_tolerance_m = 0.08;
  goal.align_final_yaw = false;
  auto graph = lineGraph(
    {vertex(1), vertex(2), vertex(3, true, false), vertex(4), goal},
    {edge(1, 1, 2), edge(2, 2, 3), edge(3, 3, 4), edge(4, 4, 5)});

  const auto result = RouteSlicer().slice(graph, {1, 2, 3, 4, 5}, {1, 2, 3, 4});

  ASSERT_EQ(result.tasks.size(), 2U);
  ASSERT_EQ(result.tasks[0].waypoints.size(), 3U);
  const auto & corner = result.tasks[0].waypoints.back();
  EXPECT_TRUE(corner.is_corner);
  EXPECT_TRUE(corner.must_pass_through);
  EXPECT_DOUBLE_EQ(corner.pass_radius_m, 0.28);
  EXPECT_EQ(result.tasks[0].edge_ids, (std::vector<EdgeId>{1, 2}));
  EXPECT_EQ(result.tasks[0].completion_policy, CompletionPolicy::kTransition);
  EXPECT_DOUBLE_EQ(result.tasks[0].endpoint_tolerance_m, 0.28);
  EXPECT_FALSE(result.tasks[0].align_goal_yaw);
  EXPECT_FALSE(result.tasks[0].requires_stop_at_end);

  ASSERT_EQ(result.tasks[1].waypoints.size(), 3U);
  EXPECT_EQ(result.tasks[1].waypoints.front().vertex_id, 3);
  EXPECT_EQ(result.tasks[1].edge_ids, (std::vector<EdgeId>{3, 4}));
  EXPECT_EQ(result.tasks[1].completion_policy, CompletionPolicy::kRouteGoal);
  EXPECT_DOUBLE_EQ(result.tasks[1].endpoint_tolerance_m, 0.08);
  EXPECT_TRUE(result.tasks[1].align_goal_yaw);
  EXPECT_NE(
    std::find(
      result.tasks[1].split_reasons.begin(), result.tasks[1].split_reasons.end(),
      "corner_waypoint"),
    result.tasks[1].split_reasons.end());
}

TEST(RouteSlicer, CornerTaskSplittingCanBeDisabled)
{
  auto graph = lineGraph(
    {vertex(1), vertex(2, true, false), vertex(3)},
    {edge(1, 1, 2), edge(2, 2, 3)});
  route3d_route_slicer::SliceOptions options;
  options.split_at_corners = false;

  const auto result = RouteSlicer(options).slice(graph, {1, 2, 3}, {1, 2});

  ASSERT_EQ(result.tasks.size(), 1U);
  ASSERT_EQ(result.tasks.front().waypoints.size(), 3U);
  EXPECT_TRUE(result.tasks.front().waypoints[1].is_corner);
}

TEST(RouteSlicer, ExplicitViaPropertiesOverrideCornerDefaults)
{
  auto custom = vertex(2, true, false);
  custom.must_pass_through_explicit = true;
  custom.must_pass_through = false;
  custom.pass_radius_explicit = true;
  custom.pass_radius_m = 0.12;
  auto graph = lineGraph({vertex(1), custom, vertex(3)}, {edge(1, 1, 2), edge(2, 2, 3)});

  const auto result = RouteSlicer().slice(graph, {1, 2, 3}, {1, 2});

  ASSERT_EQ(result.tasks.size(), 1U);
  EXPECT_FALSE(result.tasks.front().waypoints[1].must_pass_through);
  EXPECT_DOUBLE_EQ(result.tasks.front().waypoints[1].pass_radius_m, 0.12);
}

TEST(RouteSlicer, PreservesBusinessAndObstacleSingletonProcessing)
{
  auto door_a = vertex(2);
  door_a.semantic_type = 3;
  auto door_b = vertex(3);
  door_b.semantic_type = 3;
  auto grid_edge = edge(3, 3, 4);
  grid_edge.obstacle_mode = 4;
  grid_edge.grid_map_name = "floor5_grid";
  auto graph = lineGraph(
    {vertex(1), door_a, door_b, vertex(4), vertex(5)},
    {edge(1, 1, 2), edge(2, 2, 3), grid_edge, edge(4, 4, 5)});

  const auto result = RouteSlicer().slice(graph, {1, 2, 3, 4, 5}, {1, 2, 3, 4});

  ASSERT_EQ(result.tasks.size(), 4U);
  EXPECT_EQ(result.tasks[0].task_mode, "normal");
  EXPECT_EQ(result.tasks[1].task_mode, "door");
  EXPECT_EQ(result.tasks[1].completion_policy, CompletionPolicy::kBusinessStop);
  EXPECT_EQ(result.tasks[2].obstacle_mode, 4);
  EXPECT_EQ(result.tasks[2].resolved_controller_mode, "external_grid");
  EXPECT_EQ(result.tasks[2].edge_ids.size(), 1U);
  EXPECT_EQ(result.tasks[3].task_mode, "normal");
}

TEST(RouteSlicer, RejectsMismatchedAndDirectionallyInvalidRoutes)
{
  auto directed = edge(1, 1, 2);
  directed.travel_mode = "first_to_second";
  auto graph = lineGraph({vertex(1), vertex(2)}, {directed});
  RouteSlicer slicer;

  EXPECT_THROW(slicer.slice(graph, {1, 2}, {}), std::invalid_argument);
  EXPECT_THROW(slicer.slice(graph, {2, 1}, {1}), std::invalid_argument);
}

TEST(RouteSlicer, SameVertexRouteProducesGoalOnlyTask)
{
  auto goal = vertex(7);
  goal.goal_tolerance_m = 0.06;
  auto graph = lineGraph({goal}, {});

  const auto result = RouteSlicer().slice(graph, {7}, {});

  ASSERT_EQ(result.tasks.size(), 1U);
  const auto & task = result.tasks.front();
  EXPECT_TRUE(task.is_route_goal);
  EXPECT_EQ(task.completion_policy, CompletionPolicy::kRouteGoal);
  EXPECT_TRUE(task.edge_ids.empty());
  ASSERT_EQ(task.waypoints.size(), 1U);
  EXPECT_EQ(task.waypoints.front().vertex_id, 7);
  EXPECT_DOUBLE_EQ(task.endpoint_tolerance_m, 0.06);
  EXPECT_EQ(task.split_reasons, (std::vector<std::string>{"same_vertex_goal"}));
}

TEST(RouteSlicer, SpeedAndControllerChangesAreDeterministicBoundaries)
{
  auto second = edge(2, 2, 3);
  second.linear_speed_mps = 0.4;
  auto third = edge(3, 3, 4);
  third.linear_speed_mps = 0.4;
  third.controller_mode = "local_planner";
  auto graph = lineGraph(
    {vertex(1), vertex(2), vertex(3), vertex(4)},
    {edge(1, 1, 2), second, third});

  const auto result = RouteSlicer().slice(graph, {1, 2, 3, 4}, {1, 2, 3});

  ASSERT_EQ(result.tasks.size(), 4U);
  EXPECT_EQ(result.tasks[1].split_reasons.front(), "linear_speed_changed");
  EXPECT_EQ(result.tasks[2].split_reasons.front(), "controller_mode_changed");
  EXPECT_TRUE(result.tasks[1].requires_stop_at_end);
  EXPECT_EQ(result.tasks[3].resolved_controller_mode, "pid");
  EXPECT_TRUE(result.tasks[3].is_route_goal);
  EXPECT_TRUE(result.tasks[3].align_goal_yaw);
}

}  // namespace
