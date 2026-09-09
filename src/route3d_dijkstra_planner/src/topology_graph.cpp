#include "route3d_dijkstra_planner/topology_graph.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <nlohmann/json.hpp>

namespace route3d_dijkstra_planner
{
namespace
{

using json = nlohmann::json;

double finiteNumber(const json & value, const std::string & path)
{
  if (!value.is_number()) {
    throw std::runtime_error(path + " must be numeric");
  }
  const double result = value.get<double>();
  if (!std::isfinite(result)) {
    throw std::runtime_error(path + " must be finite");
  }
  return result;
}

VertexId positiveId(const json & value, const std::string & path)
{
  if (!value.is_number_integer()) {
    throw std::runtime_error(path + " must be a positive integer");
  }
  const auto wide = value.get<std::int64_t>();
  if (wide < 1 || wide > std::numeric_limits<VertexId>::max()) {
    throw std::runtime_error(path + " is outside the supported positive int32 range");
  }
  return static_cast<VertexId>(wide);
}

VertexId keyId(const std::string & key, const std::string & path)
{
  std::size_t consumed = 0;
  std::int64_t wide = 0;
  try {
    wide = std::stoll(key, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(path + " must be a canonical positive integer");
  }
  if (consumed != key.size() || wide < 1 || wide > std::numeric_limits<VertexId>::max() ||
    std::to_string(wide) != key)
  {
    throw std::runtime_error(path + " must be a canonical positive integer");
  }
  return static_cast<VertexId>(wide);
}

Point3 point3(const json & value, const std::string & path)
{
  if (!value.is_array() || value.size() != 3U) {
    throw std::runtime_error(path + " must be a three-number array");
  }
  return {
    finiteNumber(value.at(0), path + "[0]"),
    finiteNumber(value.at(1), path + "[1]"),
    finiteNumber(value.at(2), path + "[2]")};
}

const json * objectMember(const json & parent, const char * key, const std::string & path)
{
  const auto iterator = parent.find(key);
  if (iterator == parent.end() || !iterator->is_object()) {
    throw std::runtime_error(path + " must be an object");
  }
  return &iterator.value();
}

int integerMember(const json & parent, const char * key, int fallback, const std::string & path)
{
  const auto iterator = parent.find(key);
  if (iterator == parent.end()) {
    return fallback;
  }
  if (!iterator->is_number_integer()) {
    throw std::runtime_error(path + " must be integer");
  }
  return iterator->get<int>();
}

double numberMember(
  const json & parent, const char * key, double fallback, const std::string & path)
{
  const auto iterator = parent.find(key);
  return iterator == parent.end() ? fallback : finiteNumber(*iterator, path);
}

bool booleanMember(
  const json & parent, const char * key, const bool fallback, const std::string & path)
{
  const auto iterator = parent.find(key);
  if (iterator == parent.end()) {
    return fallback;
  }
  if (!iterator->is_boolean()) {
    throw std::runtime_error(path + " must be boolean");
  }
  return iterator->get<bool>();
}

std::string stringMember(
  const json & parent, const char * key, const std::string & fallback,
  const std::string & path)
{
  const auto iterator = parent.find(key);
  if (iterator == parent.end()) {
    return fallback;
  }
  if (!iterator->is_string()) {
    throw std::runtime_error(path + " must be a string");
  }
  return iterator->get<std::string>();
}

std::uint64_t endpointPairKey(VertexId first, VertexId second)
{
  const auto low = static_cast<std::uint32_t>(std::min(first, second));
  const auto high = static_cast<std::uint32_t>(std::max(first, second));
  return (static_cast<std::uint64_t>(low) << 32U) | high;
}

void synchronizeSlopeAnnotationsOnePass(
  std::unordered_map<VertexId, TopologyVertex> & vertices,
  std::unordered_map<EdgeId, TopologyEdge> & edges)
{
  constexpr int kSlopeLocomotionMode = 2;

  // Snapshot the author-provided point annotations first.  Newly promoted
  // endpoint vertices must not promote their other incident edges, otherwise
  // one slope seed would recursively spread through the whole component.
  std::unordered_set<VertexId> configured_slope_vertices;
  configured_slope_vertices.reserve(vertices.size());
  for (const auto & [vertex_id, vertex] : vertices) {
    if (vertex.is_slope) {
      configured_slope_vertices.insert(vertex_id);
    }
  }

  std::unordered_set<EdgeId> synchronized_slope_edges;
  synchronized_slope_edges.reserve(edges.size());
  for (const auto & [edge_id, edge] : edges) {
    const bool edge_requests_slope_gait =
      edge.is_slope || edge.locomotion_mode == kSlopeLocomotionMode;
    const bool touches_configured_slope_vertex =
      configured_slope_vertices.count(edge.first) != 0U ||
      configured_slope_vertices.count(edge.second) != 0U;
    if (edge_requests_slope_gait || touches_configured_slope_vertex) {
      synchronized_slope_edges.insert(edge_id);
    }
  }

  // Apply the snapshot result once.  Do not rescan the newly marked endpoint
  // vertices: they are transition boundaries, not new propagation seeds.
  for (const auto edge_id : synchronized_slope_edges) {
    auto & edge = edges.at(edge_id);
    edge.is_slope = true;
    vertices.at(edge.first).is_slope = true;
    vertices.at(edge.second).is_slope = true;
  }
}

struct QueueEntry
{
  double cost{0.0};
  VertexId vertex_id{0};
  std::size_t vertex_index{0};
};

struct QueueGreater
{
  bool operator()(const QueueEntry & left, const QueueEntry & right) const
  {
    if (left.cost != right.cost) {
      return left.cost > right.cost;
    }
    return left.vertex_id > right.vertex_id;
  }
};

struct PreviousStep
{
  VertexId vertex_id{0};
  EdgeId edge_id{0};
};

}  // namespace

NoPathError::NoPathError(
  const VertexId start_id, const VertexId goal_id, const std::size_t expanded_vertices)
: std::runtime_error(
    "no path from vertex " + std::to_string(start_id) + " to vertex " +
    std::to_string(goal_id)),
  expanded_vertices_(expanded_vertices)
{
}

TopologyGraph::TopologyGraph(
  std::string frame_id,
  std::string scene_mode,
  std::unordered_map<VertexId, TopologyVertex> vertices,
  std::unordered_map<EdgeId, TopologyEdge> edges)
: frame_id_(std::move(frame_id)),
  scene_mode_(std::move(scene_mode)),
  vertices_(std::move(vertices)),
  edges_(std::move(edges))
{
  synchronizeSlopeAnnotationsOnePass(vertices_, edges_);
  vertex_ids_by_index_.reserve(vertices_.size());
  for (const auto & entry : vertices_) {
    vertex_ids_by_index_.push_back(entry.first);
  }
  std::sort(vertex_ids_by_index_.begin(), vertex_ids_by_index_.end());
  vertex_index_by_id_.reserve(vertex_ids_by_index_.size());
  for (std::size_t index = 0; index < vertex_ids_by_index_.size(); ++index) {
    vertex_index_by_id_.emplace(vertex_ids_by_index_[index], index);
  }
  adjacency_.resize(vertex_ids_by_index_.size());
  for (const auto & entry : edges_) {
    const auto & edge = entry.second;
    if (edge.travel_mode == "bidirectional" || edge.travel_mode == "first_to_second") {
      adjacency_.at(vertexIndex(edge.first)).push_back(
        {edge.second, vertexIndex(edge.second), edge.id, edge.weight});
    }
    if (edge.travel_mode == "bidirectional" || edge.travel_mode == "second_to_first") {
      adjacency_.at(vertexIndex(edge.second)).push_back(
        {edge.first, vertexIndex(edge.first), edge.id, edge.weight});
    }
  }
  for (auto & arcs : adjacency_) {
    std::sort(
      arcs.begin(), arcs.end(), [](const Arc & left, const Arc & right) {
        return std::tie(left.neighbor, left.edge_id) < std::tie(right.neighbor, right.edge_id);
      });
  }
}

TopologyGraph TopologyGraph::load(const std::filesystem::path & path, const bool strict_schema)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open topology graph: " + path.string());
  }
  json document;
  try {
    input >> document;
  } catch (const json::exception & error) {
    throw std::runtime_error("invalid JSON in " + path.string() + ": " + error.what());
  }
  if (!document.is_object()) {
    throw std::runtime_error("root must be an object");
  }
  const auto & schema = *objectMember(document, "schema", "schema");
  if (stringMember(schema, "name", "", "schema.name") != "route3d_topology") {
    throw std::runtime_error("schema.name must be 'route3d_topology'");
  }
  if (integerMember(schema, "version", -1, "schema.version") != 2) {
    throw std::runtime_error("schema.version must be 2");
  }
  const auto frame_id = stringMember(document, "frame_id", "", "frame_id");
  const auto scene_mode = stringMember(document, "sceneMode", "", "sceneMode");
  if (frame_id.empty()) {
    throw std::runtime_error("frame_id must be a non-empty string");
  }
  if (scene_mode.empty()) {
    throw std::runtime_error("sceneMode must be a non-empty string");
  }

  const auto & raw_vertices = *objectMember(document, "vertices", "vertices");
  const auto & raw_edges = *objectMember(document, "edges", "edges");
  if (raw_vertices.empty()) {
    throw std::runtime_error("vertices must not be empty");
  }

  std::unordered_map<VertexId, TopologyVertex> vertices;
  vertices.reserve(raw_vertices.size());
  for (auto iterator = raw_vertices.begin(); iterator != raw_vertices.end(); ++iterator) {
    const auto id = keyId(iterator.key(), "vertices key " + iterator.key());
    const auto path_prefix = "vertices." + iterator.key();
    if (!iterator.value().is_object()) {
      throw std::runtime_error(path_prefix + " must be an object");
    }
    const auto & raw_vertex = iterator.value();
    const auto & meta = *objectMember(raw_vertex, "meta", path_prefix + ".meta");
    const auto align_iterator = raw_vertex.find("alignFinalYaw");
    const auto charging_iterator = meta.find("chargingMode");
    if (strict_schema && (align_iterator == raw_vertex.end() || !align_iterator->is_boolean())) {
      throw std::runtime_error(path_prefix + ".alignFinalYaw must be boolean");
    }
    if (strict_schema &&
      (charging_iterator == meta.end() || !charging_iterator->is_number_integer()))
    {
      throw std::runtime_error(path_prefix + ".meta.chargingMode must be integer");
    }
    TopologyVertex vertex;
    vertex.id = id;
    vertex.position = point3(raw_vertex.at("pos"), path_prefix + ".pos");
    vertex.rpy = point3(raw_vertex.at("rpy"), path_prefix + ".rpy");
    vertex.semantic_type = integerMember(meta, "type", 0, path_prefix + ".meta.type");
    vertex.semantic_type_id = integerMember(meta, "typeId", 0, path_prefix + ".meta.typeId");
    vertex.is_corner = booleanMember(
      meta, "isCorner", false, path_prefix + ".meta.isCorner");
    vertex.is_slope = booleanMember(meta, "isSlope", false, path_prefix + ".meta.isSlope");
    vertex.is_junction = booleanMember(
      meta, "isJunction", false, path_prefix + ".meta.isJunction");
    vertex.turn_degrees = numberMember(
      meta, "turnDeg", 0.0, path_prefix + ".meta.turnDeg");
    vertex.goal_tolerance_m = numberMember(
      raw_vertex, "acc", 0.5, path_prefix + ".acc");
    if (vertex.goal_tolerance_m < 0.0) {
      throw std::runtime_error(path_prefix + ".acc must be non-negative");
    }
    vertex.must_pass_through_explicit = raw_vertex.contains("mustPassThrough");
    vertex.must_pass_through = booleanMember(
      raw_vertex, "mustPassThrough", vertex.is_corner,
      path_prefix + ".mustPassThrough");
    vertex.pass_radius_explicit = raw_vertex.contains("passRadiusM");
    vertex.pass_radius_m = numberMember(
      raw_vertex, "passRadiusM", vertex.is_corner ? 0.20 : 0.45,
      path_prefix + ".passRadiusM");
    if (vertex.pass_radius_m < 0.0) {
      throw std::runtime_error(path_prefix + ".passRadiusM must be non-negative");
    }
    vertex.turnable = booleanMember(raw_vertex, "turnable", true, path_prefix + ".turnable");
    vertex.align_final_yaw = align_iterator == raw_vertex.end() ? true : align_iterator->get<bool>();
    vertex.charging_mode = charging_iterator == meta.end() ? 0 : charging_iterator->get<int>();
    vertex.pcd_name = stringMember(raw_vertex, "pcd", "", path_prefix + ".pcd");
    vertices.emplace(id, vertex);
  }

  static const std::array<const char *, 10> required_v2_fields = {
    "locomotionMode", "linearSpeedMps", "angularSpeedRadps", "heightOffsetM",
    "obstacleMode", "travelMode", "headingAngleRad", "obstacleBoxM", "gridMapName",
    "controllerMode"};
  std::unordered_map<EdgeId, TopologyEdge> edges;
  edges.reserve(raw_edges.size());
  std::unordered_set<std::uint64_t> endpoint_pairs;
  endpoint_pairs.reserve(raw_edges.size());
  for (auto iterator = raw_edges.begin(); iterator != raw_edges.end(); ++iterator) {
    const auto id = keyId(iterator.key(), "edges key " + iterator.key());
    const auto path_prefix = "edges." + iterator.key();
    if (!iterator.value().is_object()) {
      throw std::runtime_error(path_prefix + " must be an object");
    }
    const auto & raw_edge = iterator.value();
    const auto endpoint_iterator = raw_edge.find("v");
    if (endpoint_iterator == raw_edge.end() || !endpoint_iterator->is_array() ||
      endpoint_iterator->size() != 2U)
    {
      throw std::runtime_error(path_prefix + ".v must contain two vertex IDs");
    }
    const auto first = positiveId(endpoint_iterator->at(0), path_prefix + ".v[0]");
    const auto second = positiveId(endpoint_iterator->at(1), path_prefix + ".v[1]");
    if (first == second || vertices.count(first) == 0U || vertices.count(second) == 0U) {
      throw std::runtime_error(path_prefix + " has invalid endpoints");
    }
    if (!endpoint_pairs.insert(endpointPairKey(first, second)).second) {
      throw std::runtime_error(path_prefix + " duplicates an existing endpoint pair");
    }
    const auto weight = finiteNumber(raw_edge.at("weight"), path_prefix + ".weight");
    if (weight < 0.0) {
      throw std::runtime_error(path_prefix + ".weight must be non-negative");
    }
    const auto & meta = *objectMember(raw_edge, "meta", path_prefix + ".meta");
    if (strict_schema) {
      for (const auto * field : required_v2_fields) {
        if (meta.find(field) == meta.end()) {
          throw std::runtime_error(path_prefix + ".meta missing Schema V2 field " + field);
        }
      }
    }

    TopologyEdge edge;
    edge.id = id;
    edge.first = first;
    edge.second = second;
    edge.weight = weight;
    edge.travel_mode = stringMember(meta, "travelMode", "", path_prefix + ".meta.travelMode");
    if (edge.travel_mode != "bidirectional" && edge.travel_mode != "first_to_second" &&
      edge.travel_mode != "second_to_first")
    {
      throw std::runtime_error(path_prefix + ".meta.travelMode has an unsupported value");
    }
    const auto rotation_iterator = raw_edge.find("rotationAllowed");
    if (strict_schema &&
      (rotation_iterator == raw_edge.end() || !rotation_iterator->is_boolean()))
    {
      throw std::runtime_error(path_prefix + ".rotationAllowed must be boolean");
    }
    edge.rotation_allowed =
      rotation_iterator == raw_edge.end() ? true : rotation_iterator->get<bool>();
    edge.locomotion_mode = integerMember(
      meta, "locomotionMode", 0, path_prefix + ".meta.locomotionMode");
    edge.linear_speed_mps = numberMember(
      meta, "linearSpeedMps", 1.0, path_prefix + ".meta.linearSpeedMps");
    if (edge.linear_speed_mps < 0.0) {
      throw std::runtime_error(path_prefix + ".meta.linearSpeedMps must be non-negative");
    }
    edge.angular_speed_radps = numberMember(
      meta, "angularSpeedRadps", 0.0, path_prefix + ".meta.angularSpeedRadps");
    edge.height_offset_m = numberMember(
      meta, "heightOffsetM", 0.0, path_prefix + ".meta.heightOffsetM");
    edge.obstacle_mode = integerMember(
      meta, "obstacleMode", 0, path_prefix + ".meta.obstacleMode");
    edge.heading_angle_rad = numberMember(
      meta, "headingAngleRad", 0.0, path_prefix + ".meta.headingAngleRad");
    const auto obstacle_iterator = meta.find("obstacleBoxM");
    if (obstacle_iterator == meta.end() || !obstacle_iterator->is_array() ||
      obstacle_iterator->size() != 4U)
    {
      throw std::runtime_error(path_prefix + ".meta.obstacleBoxM must contain four numbers");
    }
    for (std::size_t index = 0; index < 4U; ++index) {
      edge.obstacle_box_m[index] = finiteNumber(
        obstacle_iterator->at(index), path_prefix + ".meta.obstacleBoxM");
    }
    edge.grid_map_name = stringMember(
      meta, "gridMapName", "", path_prefix + ".meta.gridMapName");
    edge.controller_mode = stringMember(
      meta, "controllerMode", "auto", path_prefix + ".meta.controllerMode");
    if (edge.controller_mode.empty()) {
      throw std::runtime_error(path_prefix + ".meta.controllerMode must not be empty");
    }
    const auto direction = integerMember(meta, "dir", 0, path_prefix + ".meta.dir");
    const std::string expected = direction == 0 ? "bidirectional" :
      direction == 1 ? "first_to_second" :
      direction == 2 ? "second_to_first" : "";
    if (expected.empty()) {
      throw std::runtime_error(path_prefix + ".meta.dir must be 0, 1 or 2");
    }
    if (expected != edge.travel_mode) {
      throw std::runtime_error(path_prefix + " has inconsistent dir and travelMode");
    }
    edges.emplace(id, std::move(edge));
  }
  return TopologyGraph(frame_id, scene_mode, std::move(vertices), std::move(edges));
}

const std::vector<Arc> & TopologyGraph::outgoing(const VertexId vertex_id) const
{
  return outgoingByIndex(vertexIndex(vertex_id));
}

const std::vector<Arc> & TopologyGraph::outgoingByIndex(const std::size_t vertex_index) const
{
  if (vertex_index >= adjacency_.size()) {
    throw std::out_of_range("vertex index is outside the graph");
  }
  return adjacency_[vertex_index];
}

std::size_t TopologyGraph::vertexIndex(const VertexId vertex_id) const
{
  const auto iterator = vertex_index_by_id_.find(vertex_id);
  if (iterator == vertex_index_by_id_.end()) {
    throw std::out_of_range("unknown vertex " + std::to_string(vertex_id));
  }
  return iterator->second;
}

VertexId TopologyGraph::vertexIdAt(const std::size_t vertex_index) const
{
  if (vertex_index >= vertex_ids_by_index_.size()) {
    throw std::out_of_range("vertex index is outside the graph");
  }
  return vertex_ids_by_index_[vertex_index];
}

const TopologyVertex & TopologyGraph::vertex(const VertexId vertex_id) const
{
  const auto iterator = vertices_.find(vertex_id);
  if (iterator == vertices_.end()) {
    throw std::out_of_range("unknown vertex " + std::to_string(vertex_id));
  }
  return iterator->second;
}

const TopologyEdge & TopologyGraph::edge(const EdgeId edge_id) const
{
  const auto iterator = edges_.find(edge_id);
  if (iterator == edges_.end()) {
    throw std::out_of_range("unknown edge " + std::to_string(edge_id));
  }
  return iterator->second;
}

std::vector<VertexId> TopologyGraph::sortedVertexIds() const
{
  return vertex_ids_by_index_;
}

std::vector<EdgeId> TopologyGraph::sortedEdgeIds() const
{
  std::vector<EdgeId> result;
  result.reserve(edges_.size());
  for (const auto & entry : edges_) {
    result.push_back(entry.first);
  }
  std::sort(result.begin(), result.end());
  return result;
}

DijkstraResult dijkstraShortestPath(
  const TopologyGraph & graph, const VertexId start_id, const VertexId goal_id)
{
  if (graph.vertices().count(start_id) == 0U) {
    throw std::invalid_argument("start vertex " + std::to_string(start_id) + " does not exist");
  }
  if (graph.vertices().count(goal_id) == 0U) {
    throw std::invalid_argument("goal vertex " + std::to_string(goal_id) + " does not exist");
  }
  if (start_id == goal_id) {
    return {{start_id}, {}, 0.0, 1U, 0U};
  }

  const auto vertex_count = graph.vertices().size();
  const auto start_index = graph.vertexIndex(start_id);
  const auto goal_index = graph.vertexIndex(goal_id);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueGreater> frontier;
  std::vector<double> distances(vertex_count, std::numeric_limits<double>::infinity());
  std::vector<PreviousStep> previous(vertex_count);
  std::vector<bool> has_previous(vertex_count, false);
  std::vector<bool> settled(vertex_count, false);
  distances[start_index] = 0.0;
  frontier.push({0.0, start_id, start_index});
  std::size_t relaxed_edges = 0U;
  std::size_t expanded_vertices = 0U;

  while (!frontier.empty()) {
    const auto current = frontier.top();
    frontier.pop();
    if (settled[current.vertex_index]) {
      continue;
    }
    const auto current_distance = distances[current.vertex_index];
    if (current.cost > current_distance + 1.0e-12) {
      continue;
    }
    settled[current.vertex_index] = true;
    ++expanded_vertices;
    if (current.vertex_index == goal_index) {
      break;
    }
    for (const auto & arc : graph.outgoingByIndex(current.vertex_index)) {
      const double candidate = current_distance + arc.cost;
      if (candidate + 1.0e-12 < distances[arc.neighbor_index]) {
        distances[arc.neighbor_index] = candidate;
        previous[arc.neighbor_index] = {current.vertex_id, arc.edge_id};
        has_previous[arc.neighbor_index] = true;
        frontier.push({candidate, arc.neighbor, arc.neighbor_index});
        ++relaxed_edges;
      }
    }
  }
  if (!settled[goal_index]) {
    throw NoPathError(start_id, goal_id, expanded_vertices);
  }

  DijkstraResult result;
  result.total_cost = distances[goal_index];
  result.expanded_vertices = expanded_vertices;
  result.relaxed_edges = relaxed_edges;
  auto current = goal_id;
  auto current_index = goal_index;
  result.vertex_ids.push_back(current);
  while (current != start_id) {
    if (!has_previous[current_index]) {
      throw std::logic_error("Dijkstra predecessor chain is incomplete");
    }
    const auto step = previous[current_index];
    result.edge_ids.push_back(step.edge_id);
    current = step.vertex_id;
    current_index = graph.vertexIndex(current);
    result.vertex_ids.push_back(current);
  }
  std::reverse(result.vertex_ids.begin(), result.vertex_ids.end());
  std::reverse(result.edge_ids.begin(), result.edge_ids.end());
  return result;
}

std::optional<NearestVertexResult> nearestVertex(
  const TopologyGraph & graph, const Point3 & position)
{
  if (!std::isfinite(position[0]) || !std::isfinite(position[1]) ||
    !std::isfinite(position[2]))
  {
    throw std::invalid_argument("nearest-vertex query position must be finite");
  }

  std::optional<NearestVertexResult> nearest;
  for (const auto vertex_id : graph.sortedVertexIds()) {
    const auto & vertex_position = graph.vertex(vertex_id).position;
    const double dx = vertex_position[0] - position[0];
    const double dy = vertex_position[1] - position[1];
    const double dz = vertex_position[2] - position[2];
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!nearest || distance < nearest->distance_m) {
      nearest = NearestVertexResult{vertex_id, distance};
    }
  }
  return nearest;
}

}  // namespace route3d_dijkstra_planner
