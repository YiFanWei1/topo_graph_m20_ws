#ifndef ROUTE3D_DIJKSTRA_PLANNER__TOPOLOGY_GRAPH_HPP_
#define ROUTE3D_DIJKSTRA_PLANNER__TOPOLOGY_GRAPH_HPP_

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace route3d_dijkstra_planner
{

using VertexId = std::int32_t;
using EdgeId = std::int32_t;
using Point3 = std::array<double, 3>;

struct TopologyVertex
{
  VertexId id{0};
  Point3 position{};
  Point3 rpy{};
  int semantic_type{0};
  int semantic_type_id{0};
  bool is_corner{false};
  // Author-provided meta.isSlope.  Unlike is_slope, this value is not
  // changed by the one-pass point/edge synchronization and can therefore be
  // used for physical-point decisions such as whether final yaw alignment is
  // safe.
  bool configured_is_slope{false};
  // Effective runtime slope annotation after one-pass synchronization.
  bool is_slope{false};
  bool is_junction{false};
  double turn_degrees{0.0};
  double goal_tolerance_m{0.5};
  double pass_radius_m{0.45};
  bool must_pass_through{false};
  bool pass_radius_explicit{false};
  bool must_pass_through_explicit{false};
  bool turnable{true};
  bool align_final_yaw{true};
  int charging_mode{0};
  std::string pcd_name;
};

struct TopologyEdge
{
  EdgeId id{0};
  VertexId first{0};
  VertexId second{0};
  double weight{0.0};
  std::string travel_mode{"bidirectional"};
  bool rotation_allowed{true};
  int locomotion_mode{0};
  double linear_speed_mps{1.0};
  double angular_speed_radps{0.0};
  double height_offset_m{0.0};
  int obstacle_mode{0};
  double heading_angle_rad{0.0};
  std::array<double, 4> obstacle_box_m{};
  std::string grid_map_name;
  std::string controller_mode{"auto"};
  // Runtime-only result of the one-pass point/edge slope synchronization.
  // The JSON representation continues to use vertex meta.isSlope and edge
  // locomotionMode=2 as its two explicit slope inputs.
  bool is_slope{false};
};

struct Arc
{
  VertexId neighbor{0};
  std::size_t neighbor_index{0};
  EdgeId edge_id{0};
  double cost{0.0};
};

struct DijkstraResult
{
  std::vector<VertexId> vertex_ids;
  std::vector<EdgeId> edge_ids;
  double total_cost{0.0};
  std::size_t expanded_vertices{0};
  std::size_t relaxed_edges{0};
};

struct NearestVertexResult
{
  VertexId vertex_id{0};
  double distance_m{0.0};
};

class NoPathError : public std::runtime_error
{
public:
  NoPathError(VertexId start_id, VertexId goal_id, std::size_t expanded_vertices);
  std::size_t expandedVertices() const noexcept {return expanded_vertices_;}

private:
  std::size_t expanded_vertices_{0};
};

class TopologyGraph
{
public:
  static TopologyGraph load(const std::filesystem::path & path, bool strict_schema = true);

  TopologyGraph(
    std::string frame_id,
    std::string scene_mode,
    std::unordered_map<VertexId, TopologyVertex> vertices,
    std::unordered_map<EdgeId, TopologyEdge> edges);

  const std::string & frameId() const noexcept {return frame_id_;}
  const std::string & sceneMode() const noexcept {return scene_mode_;}
  const std::unordered_map<VertexId, TopologyVertex> & vertices() const noexcept
  {
    return vertices_;
  }
  const std::unordered_map<EdgeId, TopologyEdge> & edges() const noexcept {return edges_;}
  const std::vector<Arc> & outgoing(VertexId vertex_id) const;
  const std::vector<Arc> & outgoingByIndex(std::size_t vertex_index) const;
  std::size_t vertexIndex(VertexId vertex_id) const;
  VertexId vertexIdAt(std::size_t vertex_index) const;
  const TopologyVertex & vertex(VertexId vertex_id) const;
  const TopologyEdge & edge(EdgeId edge_id) const;
  std::vector<VertexId> sortedVertexIds() const;
  std::vector<EdgeId> sortedEdgeIds() const;

private:
  std::string frame_id_;
  std::string scene_mode_;
  std::unordered_map<VertexId, TopologyVertex> vertices_;
  std::unordered_map<EdgeId, TopologyEdge> edges_;
  std::vector<VertexId> vertex_ids_by_index_;
  std::unordered_map<VertexId, std::size_t> vertex_index_by_id_;
  std::vector<std::vector<Arc>> adjacency_;
};

DijkstraResult dijkstraShortestPath(
  const TopologyGraph & graph, VertexId start_id, VertexId goal_id);

std::optional<NearestVertexResult> nearestVertex(
  const TopologyGraph & graph, const Point3 & position);

}  // namespace route3d_dijkstra_planner

#endif  // ROUTE3D_DIJKSTRA_PLANNER__TOPOLOGY_GRAPH_HPP_
