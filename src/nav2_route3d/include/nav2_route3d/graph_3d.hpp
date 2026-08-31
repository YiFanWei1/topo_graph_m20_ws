#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "nav2_route3d/types.hpp"

namespace nav2_route3d
{

class Graph3D
{
public:
  explicit Graph3D(std::string frame_id = "map");

  const std::string & frameId() const;
  void setFrameId(const std::string & frame_id);
  Metadata & metadata();
  const Metadata & metadata() const;

  void clear();
  void addNode(const Node3D & node);
  void removeNode(NodeId node_id);
  EdgeId addEdge(
    NodeId start_id,
    NodeId end_id,
    std::optional<double> cost = std::nullopt,
    Metadata metadata = Metadata::object(),
    std::vector<Metadata> operations = {},
    bool bidirectional = true,
    std::optional<EdgeId> edge_id = std::nullopt);
  void removeEdge(EdgeId edge_id);
  void markFakeNeighbor(NodeId a, NodeId b);

  bool hasNode(NodeId node_id) const;
  bool hasEdge(EdgeId edge_id) const;
  Node3D & node(NodeId node_id);
  const Node3D & node(NodeId node_id) const;
  Edge3D & edge(EdgeId edge_id);
  const Edge3D & edge(EdgeId edge_id) const;

  const std::unordered_map<NodeId, Node3D> & nodes() const;
  const std::unordered_map<EdgeId, Edge3D> & edges() const;
  std::vector<Edge3D> outgoingEdges(NodeId node_id) const;
  double distance(NodeId a, NodeId b) const;
  NodeId nearestNode(double x, double y, double z) const;

private:
  std::pair<NodeId, NodeId> edgeKey(NodeId start_id, NodeId end_id, bool bidirectional) const;

  std::string frame_id_{"map"};
  Metadata metadata_{Metadata::object()};
  std::unordered_map<NodeId, Node3D> nodes_;
  std::unordered_map<EdgeId, Edge3D> edges_;
  std::unordered_map<uint64_t, EdgeId> edge_lookup_;
  EdgeId next_edge_id_{1};
};

uint64_t makeEdgeLookupKey(NodeId a, NodeId b);

}  // namespace nav2_route3d
