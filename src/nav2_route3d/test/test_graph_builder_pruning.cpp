#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "nav2_route3d/graph_builder_3d.hpp"

namespace
{

size_t countShortcutEdges(const nav2_route3d::Graph3D & graph, const nav2_route3d::NodeId node_id)
{
  size_t count = 0U;
  for (const auto & edge : graph.outgoingEdges(node_id)) {
    if (nav2_route3d::getString(edge.metadata, "edge_source", "") == "radius_visibility") {
      ++count;
    }
  }
  return count;
}


size_t countEdgesBySource(const nav2_route3d::Graph3D & graph, const std::string & edge_source)
{
  size_t count = 0U;
  for (const auto & [_, edge] : graph.edges()) {
    if (nav2_route3d::getString(edge.metadata, "edge_source", "") == edge_source) {
      ++count;
    }
  }
  return count;
}

bool nodeHasShortcutEdge(const nav2_route3d::Graph3D & graph, const nav2_route3d::NodeId node_id)
{
  for (const auto & edge : graph.outgoingEdges(node_id)) {
    if (nav2_route3d::getString(edge.metadata, "edge_source", "") == "radius_visibility") {
      return true;
    }
  }
  return false;
}

std::filesystem::path makeTempMapPath()
{
  const auto suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  auto path = std::filesystem::temp_directory_path() / ("nav2_route3d_pruning_test_" + suffix);
  std::filesystem::create_directories(path);
  return path;
}

}  // namespace

TEST(GraphBuilder3D, CapsFoldbackShortcutDegree)
{
  const auto map_path = makeTempMapPath();
  {
    std::ofstream poses(map_path / "poses.json");
    poses <<
      "[\n"
      "  [0, 0.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [20, 1.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [40, 2.0, 0.1, 0.0, 0, 0, 0, 1],\n"
      "  [60, 3.0, 0.1, 0.0, 0, 0, 0, 1],\n"
      "  [80, 4.0, 0.2, 0.0, 0, 0, 0, 1],\n"
      "  [100, 5.0, 0.2, 0.0, 0, 0, 0, 1]\n"
      "]\n";
  }

  nav2_route3d::GraphBuilderConfig config;
  config.map_path = map_path;
  config.radius_m = 10.0;
  config.time_threshold_s = 1.0;
  config.z_threshold_m = 1.0;
  config.min_shortcut_distance_m = 0.0;
  config.shortcut_angular_bin_deg = 360.0;
  config.max_candidates_per_angular_bin = 8U;
  config.max_raytrace_candidates_per_node = 8U;
  config.max_shortcut_edges_per_node = 1U;
  config.max_workers = 1U;

  const auto graph = nav2_route3d::buildGraphFromMap(config);

  EXPECT_LE(countShortcutEdges(graph, 0U), 1U);
  EXPECT_TRUE(graph.hasNode(0U));
  EXPECT_TRUE(graph.hasNode(5U));

  std::filesystem::remove_all(map_path);
}

TEST(GraphBuilder3D, UsesSegmentMidpointAnchorsForShortcuts)
{
  const auto map_path = makeTempMapPath();
  {
    std::ofstream poses(map_path / "poses.json");
    poses <<
      "[\n"
      "  [0, 0.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [20, 1.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [40, 2.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [60, 3.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [80, 4.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [100, 5.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [120, 6.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [140, 7.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [160, 8.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [180, 9.0, 0.0, 0.0, 0, 0, 0, 1],\n"
      "  [200, 10.0, 0.0, 0.0, 0, 0, 0, 1]\n"
      "]\n";
  }

  nav2_route3d::GraphBuilderConfig config;
  config.map_path = map_path;
  config.radius_m = 10.0;
  config.time_threshold_s = 1.0;
  config.z_threshold_m = 1.0;
  config.anchor_segment_length_m = 5.0;
  config.min_shortcut_distance_m = 0.0;
  config.shortcut_angular_bin_deg = 360.0;
  config.max_candidates_per_angular_bin = 8U;
  config.max_raytrace_candidates_per_node = 8U;
  config.max_shortcut_edges_per_node = 8U;
  config.max_workers = 1U;

  const auto graph = nav2_route3d::buildGraphFromMap(config);

  EXPECT_EQ(countEdgesBySource(graph, "sequential_odom"), 10U);
  EXPECT_EQ(graph.metadata().at("anchor_segment_length_m"), 5.0);
  EXPECT_EQ(graph.metadata().at("anchor_node_count"), 2U);
  EXPECT_TRUE(graph.node(2U).metadata.value("shortcut_anchor", false));
  EXPECT_TRUE(graph.node(7U).metadata.value("shortcut_anchor", false));
  EXPECT_FALSE(nodeHasShortcutEdge(graph, 0U));
  EXPECT_FALSE(graph.node(0U).metadata.value("shortcut_anchor", false));
  // Without PCD frames, visibility checks reject shortcut candidates.
  EXPECT_FALSE(nodeHasShortcutEdge(graph, 2U));
  EXPECT_FALSE(nodeHasShortcutEdge(graph, 7U));

  std::filesystem::remove_all(map_path);
}
