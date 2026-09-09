#include "route3d_dijkstra_planner/topology_graph.hpp"

#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace route3d_dijkstra_planner
{
namespace
{

TopologyGraph loadTestGraph()
{
  return TopologyGraph::load(std::filesystem::path(TEST_GRAPH_FILE), true);
}

TEST(SchemaV2Loader, LoadsCompleteComplexGraph)
{
  const auto graph = loadTestGraph();
  EXPECT_EQ(graph.frameId(), "camera_init");
  EXPECT_EQ(graph.sceneMode(), "normal");
  EXPECT_EQ(graph.vertices().size(), 22U);
  EXPECT_EQ(graph.edges().size(), 40U);
  EXPECT_TRUE(graph.vertex(1).align_final_yaw);
  EXPECT_EQ(graph.vertex(1).charging_mode, 0);
  EXPECT_EQ(graph.edge(1).controller_mode, "pid");
  EXPECT_EQ(graph.edge(38).travel_mode, "first_to_second");
}

TEST(Dijkstra, UsesWeightedDiagonalShortcuts)
{
  const auto graph = loadTestGraph();
  const auto result = dijkstraShortestPath(graph, 1, 20);
  EXPECT_EQ(result.vertex_ids, (std::vector<VertexId>{1, 7, 13, 19, 20}));
  EXPECT_EQ(result.edge_ids, (std::vector<EdgeId>{32, 33, 34, 16}));
  EXPECT_NEAR(result.total_cost, 9.05, 1.0e-9);
  EXPECT_GT(result.expanded_vertices, result.vertex_ids.size());
}

TEST(Dijkstra, FindsSecondComplexRoute)
{
  const auto graph = loadTestGraph();
  const auto result = dijkstraShortestPath(graph, 5, 16);
  EXPECT_EQ(result.vertex_ids, (std::vector<VertexId>{5, 9, 13, 17, 16}));
  EXPECT_NEAR(result.total_cost, 9.05, 1.0e-9);
}

TEST(Dijkstra, RespectsSchemaV2TravelDirection)
{
  const auto graph = loadTestGraph();
  const auto forward = dijkstraShortestPath(graph, 3, 13);
  const auto reverse = dijkstraShortestPath(graph, 13, 3);
  EXPECT_EQ(forward.vertex_ids, (std::vector<VertexId>{3, 13}));
  EXPECT_NEAR(forward.total_cost, 3.2, 1.0e-9);
  EXPECT_EQ(reverse.vertex_ids, (std::vector<VertexId>{13, 8, 3}));
  EXPECT_NEAR(reverse.total_cost, 4.0, 1.0e-9);
}

TEST(Dijkstra, SameVertexHasZeroCost)
{
  const auto graph = loadTestGraph();
  const auto result = dijkstraShortestPath(graph, 7, 7);
  EXPECT_EQ(result.vertex_ids, (std::vector<VertexId>{7}));
  EXPECT_TRUE(result.edge_ids.empty());
  EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
}

TEST(Dijkstra, ReportsDisconnectedAndUnknownVertices)
{
  const auto graph = loadTestGraph();
  try {
    static_cast<void>(dijkstraShortestPath(graph, 1, 22));
    FAIL() << "expected NoPathError";
  } catch (const NoPathError & error) {
    EXPECT_EQ(error.expandedVertices(), 20U);
  }
  EXPECT_THROW(dijkstraShortestPath(graph, 999, 1), std::invalid_argument);
}

TEST(NearestVertex, UsesThreeDimensionalDistance)
{
  const auto graph = loadTestGraph();
  const auto nearest = nearestVertex(graph, Point3{0.20, 0.10, 0.05});
  ASSERT_TRUE(nearest.has_value());
  EXPECT_EQ(nearest->vertex_id, 1);
  EXPECT_NEAR(nearest->distance_m, std::sqrt(0.0525), 1.0e-9);
}

TEST(NearestVertex, RejectsNonFinitePosition)
{
  const auto graph = loadTestGraph();
  EXPECT_THROW(
    nearestVertex(graph, Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}),
    std::invalid_argument);
}

}  // namespace
}  // namespace route3d_dijkstra_planner
