#ifndef ROUTE3D_TOPOLOGY_EDITOR__TOPOLOGY_DOCUMENT_HPP_
#define ROUTE3D_TOPOLOGY_EDITOR__TOPOLOGY_DOCUMENT_HPP_

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace route3d_topology_editor
{

struct TopologyPath
{
  std::vector<int> vertex_ids;
  std::vector<int> edge_ids;
};

class TopologyDocument
{
public:
  void load(const std::filesystem::path & path);
  void save(const std::filesystem::path & path) const;
  void validate() const;

  nlohmann::json & data() {return data_;}
  const nlohmann::json & data() const {return data_;}
  const std::filesystem::path & path() const {return path_;}

  static void recomputeIncidentWeights(nlohmann::json & document, int vertex_id);
  static int addVertex(
    nlohmann::json & document, double x, double y, double z, double source_stamp);
  static int addEdge(nlohmann::json & document, int first_vertex_id, int second_vertex_id);
  static void removeVertex(nlohmann::json & document, int vertex_id);
  static void removeEdge(nlohmann::json & document, int edge_id);
  static TopologyPath shortestPath(
    const nlohmann::json & document, int start_vertex_id, int end_vertex_id);

private:
  std::filesystem::path path_;
  nlohmann::json data_;
};

}  // namespace route3d_topology_editor

#endif  // ROUTE3D_TOPOLOGY_EDITOR__TOPOLOGY_DOCUMENT_HPP_
