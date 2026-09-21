#include "route3d_topology_editor/topology_document.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include <QFile>
#include <QSaveFile>

namespace route3d_topology_editor
{
namespace
{

int canonicalPositiveId(const std::string & text, const std::string & field)
{
  std::size_t consumed = 0;
  long long value = 0;
  try {
    value = std::stoll(text, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(field + " must be a positive integer");
  }
  if (consumed != text.size() || value < 1 ||
    value > std::numeric_limits<int>::max() || std::to_string(value) != text)
  {
    throw std::runtime_error(field + " must be a canonical positive integer");
  }
  return static_cast<int>(value);
}

double finiteNumber(const nlohmann::json & value, const std::string & field)
{
  if (!value.is_number()) {
    throw std::runtime_error(field + " must be numeric");
  }
  const auto result = value.get<double>();
  if (!std::isfinite(result)) {
    throw std::runtime_error(field + " must be finite");
  }
  return result;
}

void validateVector3(const nlohmann::json & value, const std::string & field)
{
  if (!value.is_array() || value.size() != 3U) {
    throw std::runtime_error(field + " must contain three numbers");
  }
  for (std::size_t index = 0; index < 3U; ++index) {
    (void)finiteNumber(value.at(index), field + "[" + std::to_string(index) + "]");
  }
}

int nextId(const nlohmann::json & values, const std::string & field)
{
  int maximum = 0;
  for (const auto & [key, value] : values.items()) {
    (void)value;
    maximum = std::max(maximum, canonicalPositiveId(key, field));
  }
  if (maximum == std::numeric_limits<int>::max()) {
    throw std::runtime_error(field + " space is exhausted");
  }
  return maximum + 1;
}

}  // namespace

void TopologyDocument::load(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open topology graph: " + path.string());
  }
  nlohmann::json loaded;
  try {
    input >> loaded;
  } catch (const nlohmann::json::exception & error) {
    throw std::runtime_error("invalid JSON: " + std::string(error.what()));
  }
  data_ = std::move(loaded);
  validate();
  path_ = std::filesystem::absolute(path);
}

void TopologyDocument::validate() const
{
  if (!data_.is_object()) {
    throw std::runtime_error("topology root must be an object");
  }
  if (!data_.contains("schema") || !data_.at("schema").is_object() ||
    data_.at("schema").value("name", "") != "route3d_topology" ||
    data_.at("schema").value("version", 0) != 2)
  {
    throw std::runtime_error("graph must use route3d_topology Schema V2");
  }
  if (!data_.contains("vertices") || !data_.at("vertices").is_object() ||
    data_.at("vertices").empty())
  {
    throw std::runtime_error("vertices must be a non-empty object");
  }
  if (!data_.contains("edges") || !data_.at("edges").is_object()) {
    throw std::runtime_error("edges must be an object");
  }

  const auto & vertices = data_.at("vertices");
  for (const auto & [id_text, vertex] : vertices.items()) {
    (void)canonicalPositiveId(id_text, "vertex id");
    if (!vertex.is_object()) {
      throw std::runtime_error("vertex " + id_text + " must be an object");
    }
    validateVector3(vertex.at("pos"), "vertices." + id_text + ".pos");
    validateVector3(vertex.at("rpy"), "vertices." + id_text + ".rpy");
    if (!vertex.contains("meta") || !vertex.at("meta").is_object()) {
      throw std::runtime_error("vertices." + id_text + ".meta must be an object");
    }
  }

  for (const auto & [id_text, edge] : data_.at("edges").items()) {
    (void)canonicalPositiveId(id_text, "edge id");
    if (!edge.is_object() || !edge.contains("v") || !edge.at("v").is_array() ||
      edge.at("v").size() != 2U)
    {
      throw std::runtime_error("edges." + id_text + ".v must contain two vertices");
    }
    const int first = edge.at("v").at(0).get<int>();
    const int second = edge.at("v").at(1).get<int>();
    if (first == second || !vertices.contains(std::to_string(first)) ||
      !vertices.contains(std::to_string(second)))
    {
      throw std::runtime_error("edge " + id_text + " has invalid endpoints");
    }
    if (!edge.contains("meta") || !edge.at("meta").is_object()) {
      throw std::runtime_error("edges." + id_text + ".meta must be an object");
    }
    if (finiteNumber(edge.at("weight"), "edges." + id_text + ".weight") < 0.0) {
      throw std::runtime_error("edge " + id_text + " weight must be non-negative");
    }
  }
}

void TopologyDocument::save(const std::filesystem::path & path) const
{
  validate();
  const auto absolute = std::filesystem::absolute(path);
  std::error_code error;
  std::filesystem::create_directories(absolute.parent_path(), error);
  if (error) {
    throw std::runtime_error("cannot create output directory: " + error.message());
  }
  if (std::filesystem::exists(absolute)) {
    std::filesystem::copy_file(
      absolute, absolute.string() + ".bak",
      std::filesystem::copy_options::overwrite_existing, error);
    if (error) {
      throw std::runtime_error("cannot create backup: " + error.message());
    }
  }

  QSaveFile output(QString::fromStdString(absolute.string()));
  if (!output.open(QIODevice::WriteOnly | QIODevice::Text)) {
    throw std::runtime_error("cannot open output file for writing");
  }
  const auto serialized = data_.dump(2) + "\n";
  if (output.write(serialized.data(), static_cast<qint64>(serialized.size())) !=
    static_cast<qint64>(serialized.size()) || !output.commit())
  {
    throw std::runtime_error("atomic save failed");
  }
}

void TopologyDocument::recomputeIncidentWeights(nlohmann::json & document, int vertex_id)
{
  const auto key = std::to_string(vertex_id);
  const auto & vertices = document.at("vertices");
  for (auto & [edge_id, edge] : document.at("edges").items()) {
    (void)edge_id;
    const int first = edge.at("v").at(0).get<int>();
    const int second = edge.at("v").at(1).get<int>();
    if (first != vertex_id && second != vertex_id) {
      continue;
    }
    const auto & a = vertices.at(std::to_string(first)).at("pos");
    const auto & b = vertices.at(std::to_string(second)).at("pos");
    const double dx = a.at(0).get<double>() - b.at(0).get<double>();
    const double dy = a.at(1).get<double>() - b.at(1).get<double>();
    const double dz = a.at(2).get<double>() - b.at(2).get<double>();
    edge["weight"] = std::sqrt(dx * dx + dy * dy + dz * dz);
  }
}

int TopologyDocument::addVertex(
  nlohmann::json & document, double x, double y, double z, double source_stamp)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
    !std::isfinite(source_stamp))
  {
    throw std::runtime_error("new vertex position and timestamp must be finite");
  }
  auto & vertices = document.at("vertices");
  const int id = nextId(vertices, "vertex id");
  vertices[std::to_string(id)] = {
    {"pos", {x, y, z}},
    {"rpy", {0.0, 0.0, 0.0}},
    {"meta", {
      {"type", 0}, {"typeId", 0}, {"isCorner", false}, {"isSlope", false},
      {"isJunction", false}, {"state", "confirmed"},
      {"source", "incremental_topology"}, {"sourceStamp", source_stamp},
      {"component", 0}, {"turnDeg", 0.0}, {"chargingMode", 0},
    }},
    {"pcd", ""}, {"acc", 0.5}, {"turnable", true},
    {"alignFinalYaw", false}, {"mustPassThrough", false}, {"passRadiusM", 0.45},
  };
  return id;
}

int TopologyDocument::addEdge(
  nlohmann::json & document, int first_vertex_id, int second_vertex_id)
{
  auto & vertices = document.at("vertices");
  auto & edges = document.at("edges");
  if (first_vertex_id == second_vertex_id ||
    !vertices.contains(std::to_string(first_vertex_id)) ||
    !vertices.contains(std::to_string(second_vertex_id)))
  {
    throw std::runtime_error("新边必须连接两个不同的已有顶点");
  }
  for (const auto & [edge_id, edge] : edges.items()) {
    (void)edge_id;
    const int first = edge.at("v").at(0).get<int>();
    const int second = edge.at("v").at(1).get<int>();
    if ((first == first_vertex_id && second == second_vertex_id) ||
      (first == second_vertex_id && second == first_vertex_id))
    {
      throw std::runtime_error("这两个顶点之间已经存在边");
    }
  }
  const auto & first = vertices.at(std::to_string(first_vertex_id)).at("pos");
  const auto & second = vertices.at(std::to_string(second_vertex_id)).at("pos");
  const double dx = first.at(0).get<double>() - second.at(0).get<double>();
  const double dy = first.at(1).get<double>() - second.at(1).get<double>();
  const double dz = first.at(2).get<double>() - second.at(2).get<double>();
  const int id = nextId(edges, "edge id");
  edges[std::to_string(id)] = {
    {"v", {first_vertex_id, second_vertex_id}},
    {"weight", std::sqrt(dx * dx + dy * dy + dz * dz)},
    {"meta", {
      {"dir", 0}, {"source", "discovery"}, {"linearSpeedMps", 1.0},
      {"angularSpeedRadps", 0.0}, {"heightOffsetM", 0.0}, {"obstacleMode", 0},
      {"travelMode", "bidirectional"}, {"headingAngleRad", 0.0},
      {"obstacleBoxM", {0.0, 0.0, 0.0, 0.0}}, {"gridMapName", ""},
      {"controllerMode", "auto"},
    }},
    {"rotationAllowed", true},
  };
  return id;
}

void TopologyDocument::removeVertex(nlohmann::json & document, int vertex_id)
{
  auto & vertices = document.at("vertices");
  const auto key = std::to_string(vertex_id);
  if (!vertices.contains(key)) {
    throw std::runtime_error("要删除的顶点不存在");
  }
  if (vertices.size() <= 1U) {
    throw std::runtime_error("拓扑至少需要保留一个顶点");
  }
  vertices.erase(key);
  auto & edges = document.at("edges");
  std::vector<std::string> incident_edges;
  for (const auto & [edge_id, edge] : edges.items()) {
    if (edge.at("v").at(0).get<int>() == vertex_id ||
      edge.at("v").at(1).get<int>() == vertex_id)
    {
      incident_edges.push_back(edge_id);
    }
  }
  for (const auto & edge_id : incident_edges) {
    edges.erase(edge_id);
  }
}

void TopologyDocument::removeEdge(nlohmann::json & document, int edge_id)
{
  auto & edges = document.at("edges");
  if (edges.erase(std::to_string(edge_id)) == 0U) {
    throw std::runtime_error("要删除的边不存在");
  }
}

TopologyPath TopologyDocument::shortestPath(
  const nlohmann::json & document, int start_vertex_id, int end_vertex_id)
{
  const auto & vertices = document.at("vertices");
  const auto & edges = document.at("edges");
  const auto start_key = std::to_string(start_vertex_id);
  const auto end_key = std::to_string(end_vertex_id);
  if (!vertices.contains(start_key) || !vertices.contains(end_key)) {
    throw std::runtime_error("batch path endpoint does not exist");
  }
  if (start_vertex_id == end_vertex_id) {
    return {{start_vertex_id}, {}};
  }

  struct Neighbor
  {
    int vertex_id;
    int edge_id;
    double weight;
  };
  std::unordered_map<int, std::vector<Neighbor>> adjacency;
  for (const auto & [edge_text, edge] : edges.items()) {
    const int edge_id = canonicalPositiveId(edge_text, "edge id");
    const int first = edge.at("v").at(0).get<int>();
    const int second = edge.at("v").at(1).get<int>();
    const double weight = finiteNumber(edge.at("weight"), "edge weight");
    adjacency[first].push_back({second, edge_id, weight});
    adjacency[second].push_back({first, edge_id, weight});
  }

  using QueueEntry = std::pair<double, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  std::unordered_map<int, double> distances;
  std::unordered_map<int, std::pair<int, int>> previous;
  distances[start_vertex_id] = 0.0;
  queue.emplace(0.0, start_vertex_id);

  while (!queue.empty()) {
    const auto [distance, vertex_id] = queue.top();
    queue.pop();
    if (distance > distances.at(vertex_id)) {
      continue;
    }
    if (vertex_id == end_vertex_id) {
      break;
    }
    for (const auto & neighbor : adjacency[vertex_id]) {
      const double candidate = distance + neighbor.weight;
      const auto found = distances.find(neighbor.vertex_id);
      if (found == distances.end() || candidate < found->second) {
        distances[neighbor.vertex_id] = candidate;
        previous[neighbor.vertex_id] = {vertex_id, neighbor.edge_id};
        queue.emplace(candidate, neighbor.vertex_id);
      }
    }
  }

  if (distances.find(end_vertex_id) == distances.end()) {
    throw std::runtime_error("起点和终点之间没有连通路径");
  }

  TopologyPath result;
  int current = end_vertex_id;
  result.vertex_ids.push_back(current);
  while (current != start_vertex_id) {
    const auto step = previous.find(current);
    if (step == previous.end()) {
      throw std::runtime_error("cannot reconstruct batch path");
    }
    result.edge_ids.push_back(step->second.second);
    current = step->second.first;
    result.vertex_ids.push_back(current);
  }
  std::reverse(result.vertex_ids.begin(), result.vertex_ids.end());
  std::reverse(result.edge_ids.begin(), result.edge_ids.end());
  return result;
}

}  // namespace route3d_topology_editor
