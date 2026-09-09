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

TEST(RouteSlicer, SlopeVerticesCreateClimbAndRestoreGaitTasks)
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

  ASSERT_EQ(result.tasks.size(), 3U);
  EXPECT_EQ(result.tasks[0].edge_ids, (std::vector<EdgeId>{1}));
  EXPECT_EQ(result.tasks[0].locomotion_mode, 0);
  EXPECT_TRUE(result.tasks[0].requires_gait_switch_at_start);
  EXPECT_TRUE(result.tasks[0].requires_stop_at_end);
  EXPECT_EQ(result.tasks[1].edge_ids, (std::vector<EdgeId>{2, 3, 4}));
  EXPECT_EQ(result.tasks[1].locomotion_mode, 2);
  EXPECT_EQ(result.tasks[1].gait_command, "switch_gait_3");
  EXPECT_TRUE(result.tasks[1].contains_slope);
  EXPECT_EQ(result.tasks[1].resolved_controller_mode, "efficient_3d_local_planner");
  EXPECT_EQ(result.tasks[1].obstacle_mode, 3);
  EXPECT_TRUE(result.tasks[1].requires_gait_switch_at_start);
  EXPECT_TRUE(result.tasks[1].requires_stop_at_end);
  EXPECT_EQ(result.tasks[2].edge_ids, (std::vector<EdgeId>{5, 6}));
  EXPECT_EQ(result.tasks[2].gait_command, "static_walk");
  EXPECT_TRUE(result.tasks[2].requires_gait_switch_at_start);
  EXPECT_TRUE(result.tasks[2].is_route_goal);
  EXPECT_EQ(result.tasks[2].obstacle_mode, 0);
}

TEST(RouteSlicer, ExplicitSlopeEdgePromotesEndpointsWithoutRecursiveSpread)
{
  auto slope_edge = edge(2, 2, 3);
  slope_edge.locomotion_mode = 2;
  auto graph = lineGraph(
    {vertex(1), vertex(2), vertex(3), vertex(4)},
    {edge(1, 1, 2), slope_edge, edge(3, 3, 4)});

  EXPECT_FALSE(graph.edge(1).is_slope);
  EXPECT_TRUE(graph.edge(2).is_slope);
  EXPECT_FALSE(graph.edge(3).is_slope);
  EXPECT_FALSE(graph.vertex(1).is_slope);
  EXPECT_TRUE(graph.vertex(2).is_slope);
  EXPECT_TRUE(graph.vertex(3).is_slope);
  EXPECT_FALSE(graph.vertex(4).is_slope);

  const auto result = RouteSlicer().slice(graph, {1, 2, 3, 4}, {1, 2, 3});

  ASSERT_EQ(result.tasks.size(), 3U);
  EXPECT_EQ(result.tasks[0].gait_command, "static_walk");
  EXPECT_EQ(result.tasks[1].gait_command, "switch_gait_3");
  EXPECT_EQ(result.tasks[1].resolved_controller_mode, "efficient_3d_local_planner");
  EXPECT_EQ(result.tasks[1].obstacle_mode, 3);
  EXPECT_EQ(result.tasks[2].gait_command, "static_walk");
}

TEST(RouteSlicer, ExplicitSlopeObstaclePolicyIsNotOverwritten)
{
  auto slope_edge = edge(1, 1, 2);
  slope_edge.obstacle_mode = 1;
  auto graph = lineGraph({vertex(1), vertex(2, false, true)}, {slope_edge});

  const auto result = RouteSlicer().slice(graph, {1, 2}, {1});

  ASSERT_EQ(result.tasks.size(), 1U);
  EXPECT_TRUE(result.tasks.front().contains_slope);
  EXPECT_EQ(result.tasks.front().obstacle_mode, 1);
}

TEST(RouteSlicer, CornerIsHardViaPointWithoutSemanticTaskSplit)
{
  auto goal = vertex(5);
  goal.goal_tolerance_m = 0.08;
  goal.align_final_yaw = false;
  auto graph = lineGraph(
    {vertex(1), vertex(2), vertex(3, true, false), vertex(4), goal},
    {edge(1, 1, 2), edge(2, 2, 3), edge(3, 3, 4), edge(4, 4, 5)});

  const auto result = RouteSlicer().slice(graph, {1, 2, 3, 4, 5}, {1, 2, 3, 4});

  ASSERT_EQ(result.tasks.size(), 1U);
  ASSERT_EQ(result.tasks.front().waypoints.size(), 5U);
  const auto & corner = result.tasks.front().waypoints[2];
  EXPECT_TRUE(corner.is_corner);
  EXPECT_TRUE(corner.must_pass_through);
  EXPECT_DOUBLE_EQ(corner.pass_radius_m, 0.20);
  EXPECT_EQ(result.tasks.front().completion_policy, CompletionPolicy::kRouteGoal);
  EXPECT_DOUBLE_EQ(result.tasks.front().endpoint_tolerance_m, 0.08);
  EXPECT_FALSE(result.tasks.front().align_goal_yaw);
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
  EXPECT_EQ(result.tasks[2].resolved_controller_mode, "local_planner");
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

  ASSERT_EQ(result.tasks.size(), 3U);
  EXPECT_EQ(result.tasks[1].split_reasons.front(), "linear_speed_changed");
  EXPECT_EQ(result.tasks[2].split_reasons.front(), "controller_mode_changed");
  EXPECT_TRUE(result.tasks[1].requires_stop_at_end);
}

}  // namespace
