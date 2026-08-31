#include "nav2_route3d/graph_3d.hpp"

// 本文件实现三维路线图的内存数据结构与基本增删查操作。
// Graph3D 本身不负责从点云生成骨架，也不执行最短路算法；它主要维护：
// - node_id -> Node3D 的节点表；
// - edge_id -> Edge3D 的边表；
// - 节点的 real_neighbors（已有真实边）与 fake_neighbors（已判定不可连接）；
// - 节点对 -> edge_id 的快速去重索引；
// - frame_id 和图级 JSON metadata。
//
// NodeId/EdgeId 均为 uint32_t，因此两个 NodeId 可以无损打包到一个 uint64_t 键中。

#include <algorithm>
#include <cmath>
// numeric_limits<double>::max() 用作线性最近节点搜索的初始距离上界。
#include <limits>
#include <stdexcept>
#include <utility>

namespace nav2_route3d
{

// 把两个 32 位节点编号打包成一个 64 位整数：高 32 位放 a，低 32 位放 b。
// 该函数只负责打包；是否先按大小排序由 Graph3D::edgeKey() 根据边方向决定。
uint64_t makeEdgeLookupKey(const NodeId a, const NodeId b)
{
  return (static_cast<uint64_t>(a) << 32U) | static_cast<uint64_t>(b);
}

// 创建空图，并保存所有节点位姿共同使用的全局坐标系名称。
Graph3D::Graph3D(std::string frame_id)
: frame_id_(std::move(frame_id))
{
}

// 下面四个短函数直接暴露图的坐标系和图级元数据。
// metadata() 的非 const 版本允许构图器写入参数、来源等 JSON 字段。
const std::string & Graph3D::frameId() const {return frame_id_;}
void Graph3D::setFrameId(const std::string & frame_id) {frame_id_ = frame_id;}
Metadata & Graph3D::metadata() {return metadata_;}
const Metadata & Graph3D::metadata() const {return metadata_;}

// 删除全部节点、边和边查找索引，使下一条自动编号边重新从 1 开始。
// 注意：clear() 有意保留 frame_id_ 和 metadata_，它清理的是拓扑内容而不是图的身份信息。
void Graph3D::clear()
{
  nodes_.clear();
  edges_.clear();
  edge_lookup_.clear();
  next_edge_id_ = 1;
}

// 插入一个完整 Node3D，节点编号必须在当前图中唯一。
void Graph3D::addNode(const Node3D & node)
{
  // 重复 ID 会使现有边和邻接关系含义不明确，因此拒绝覆盖。
  if (nodes_.count(node.node_id) != 0U) {
    throw std::runtime_error("Duplicate node id " + std::to_string(node.node_id));
  }
  // 这里复制整个 Node3D，包括 pose、metadata 以及调用方传入的邻居集合；
  // addNode() 本身不会根据邻居集合自动创建 Edge3D。
  nodes_.emplace(node.node_id, node);
}

// 删除节点以及所有与该节点关联的边和邻居引用。
void Graph3D::removeNode(const NodeId node_id)
{
  // 删除不存在的节点视为幂等操作，直接返回。
  if (nodes_.count(node_id) == 0U) {
    return;
  }

  // 不能在遍历 edges_ 时直接调用 removeEdge() 擦除同一容器；
  // 先收集所有入边/出边 ID，再在第二个循环中安全删除。
  std::vector<EdgeId> to_remove;
  for (const auto & [edge_id, edge] : edges_) {
    if (edge.start_id == node_id || edge.end_id == node_id) {
      to_remove.push_back(edge_id);
    }
  }
  for (const auto edge_id : to_remove) {
    // removeEdge() 同时维护 edge_lookup_ 和 real_neighbors。
    removeEdge(edge_id);
  }
  // 所有真实关联边已删除后，再移除节点实体。
  nodes_.erase(node_id);
  // 扫描剩余节点，清除指向被删节点的真实/假邻居残留。
  // fake_neighbors 没有对应 Edge3D，必须在这里单独清理。
  for (auto & [_, node] : nodes_) {
    node.real_neighbors.erase(node_id);
    node.fake_neighbors.erase(node_id);
  }
}

// 添加一条边，并同步更新边去重索引与节点真实邻居集合。
EdgeId Graph3D::addEdge(
  const NodeId start_id,
  const NodeId end_id,
  const std::optional<double> cost,
  Metadata metadata,
  std::vector<Metadata> operations,
  const bool bidirectional,
  const std::optional<EdgeId> edge_id)
{
  // 自环边在本图模型中没有路线意义，并可能干扰邻居与最短路逻辑。
  if (start_id == end_id) {
    throw std::runtime_error("Self edges are not allowed");
  }
  // 必须先有两个端点节点，避免产生悬空边。
  if (!hasNode(start_id) || !hasNode(end_id)) {
    throw std::runtime_error("Missing node for edge");
  }
  // 双向边先把节点对规范化为 (min,max)，因此 addEdge(a,b,true) 与
  // addEdge(b,a,true) 会命中同一个查找键；有向边则保留 (start,end) 顺序。
  const auto [a, b] = edgeKey(start_id, end_id, bidirectional);
  const auto lookup_key = makeEdgeLookupKey(a, b);
  const auto lookup_it = edge_lookup_.find(lookup_key);
  if (lookup_it != edge_lookup_.end()) {
    // 节点对已经有边时直接返回原 edge_id，不覆盖原边的代价、元数据或 operations。
    // lookup key 不额外编码边类型，因此该结构不支持同一查找键上的多条平行边。
    return lookup_it->second;
  }

  // 调用方可指定 edge_id（例如从 JSON 恢复），否则使用内部自增 ID。
  const auto resolved_edge_id = edge_id.value_or(next_edge_id_);
  // 显式 ID 可能大于当前计数器；把下一自动 ID 推进到至少 resolved+1。
  // 调用方传入的显式 edge_id 应保证唯一，本函数没有单独覆盖检查。
  next_edge_id_ = std::max(next_edge_id_, static_cast<EdgeId>(resolved_edge_id + 1U));

  // 组装边对象。start/end 保留调用 addEdge() 时给出的方向，即使它是双向边。
  Edge3D edge;
  edge.edge_id = resolved_edge_id;
  edge.start_id = start_id;
  edge.end_id = end_id;
  // 未提供 cost 时使用两个节点平移位置的三维欧氏距离；显式 cost 则原样保存。
  edge.cost = cost.value_or(distance(start_id, end_id));
  edge.bidirectional = bidirectional;
  // metadata 和 operations 按值传入后 move 到 Edge3D，避免再次复制较大的 JSON 数据。
  edge.metadata = std::move(metadata);
  edge.operations = std::move(operations);
  // 正式写入边表和“节点对->边 ID”索引。
  edges_.emplace(resolved_edge_id, edge);
  edge_lookup_[lookup_key] = resolved_edge_id;
  // 对有向边，只把 end 记入 start 的真实邻居；反向遍历不会看到这条边。
  nodes_.at(start_id).real_neighbors.insert(end_id);
  if (bidirectional) {
    // 双向边同时把 start 记入 end 的真实邻居。
    nodes_.at(end_id).real_neighbors.insert(start_id);
  }
  return resolved_edge_id;
}

// 按 edge_id 删除边，并同步清理查找索引和真实邻居集合。
void Graph3D::removeEdge(const EdgeId edge_id)
{
  const auto it = edges_.find(edge_id);
  // 删除不存在的边视为幂等操作。
  if (it == edges_.end()) {
    return;
  }
  // 先复制边，因为最后会从 edges_ 擦除迭代器指向的对象。
  const auto edge = it->second;
  // 使用添加边时相同的规范化规则重建 lookup key。
  const auto [a, b] = edgeKey(edge.start_id, edge.end_id, edge.bidirectional);
  edge_lookup_.erase(makeEdgeLookupKey(a, b));
  // 端点可能在更复杂的删除流程中已经不存在，因此更新邻居前先检查。
  if (hasNode(edge.start_id)) {
    nodes_.at(edge.start_id).real_neighbors.erase(edge.end_id);
  }
  if (edge.bidirectional && hasNode(edge.end_id)) {
    // 只有双向边曾向 end 的邻居集合写入 start，有向边无需清理这一侧。
    nodes_.at(edge.end_id).real_neighbors.erase(edge.start_id);
  }
  edges_.erase(it);
}

// 对称地记录两个节点已经被判定为“不可直接建边”的假邻居。
void Graph3D::markFakeNeighbor(const NodeId a, const NodeId b)
{
  // 两个节点都存在时才写入，避免 fake_neighbors 中出现悬空编号。
  if (hasNode(a) && hasNode(b)) {
    // unordered_set 会自动去重，多次标记同一关系不会产生重复记录。
    nodes_.at(a).fake_neighbors.insert(b);
    nodes_.at(b).fake_neighbors.insert(a);
    // 本函数不会删除已经存在的真实边；正常构图流程应只对尚未连接的候选调用它。
  }
}

// 常数平均复杂度的节点/边存在性检查。
bool Graph3D::hasNode(const NodeId node_id) const
{
  return nodes_.count(node_id) != 0U;
}

bool Graph3D::hasEdge(const EdgeId edge_id) const
{
  return edges_.count(edge_id) != 0U;
}

// 按 ID 返回图元素引用。unordered_map::at() 在 ID 不存在时会抛出 std::out_of_range，
// 因而调用方需要先确认 ID 合法，或让异常暴露数据不一致问题。
Node3D & Graph3D::node(const NodeId node_id) {return nodes_.at(node_id);}
const Node3D & Graph3D::node(const NodeId node_id) const {return nodes_.at(node_id);}
Edge3D & Graph3D::edge(const EdgeId edge_id) {return edges_.at(edge_id);}
const Edge3D & Graph3D::edge(const EdgeId edge_id) const {return edges_.at(edge_id);}
// 只读暴露完整容器，供序列化、可视化和路线算法遍历；unordered_map 不保证遍历顺序。
const std::unordered_map<NodeId, Node3D> & Graph3D::nodes() const {return nodes_;}
const std::unordered_map<EdgeId, Edge3D> & Graph3D::edges() const {return edges_;}

// 返回从 node_id 出发能够行走的所有边副本。
std::vector<Edge3D> Graph3D::outgoingEdges(const NodeId node_id) const
{
  std::vector<Edge3D> result;
  // real_neighbors 数量可作为结果容量的近似/正常值，先 reserve 减少动态扩容。
  result.reserve(nodes_.at(node_id).real_neighbors.size());
  // 当前实现直接扫描所有边，复杂度为 O(E)，没有单独维护 node->edge_id 邻接表。
  for (const auto & [_, edge] : edges_) {
    if (edge.start_id == node_id) {
      // 无论有向还是双向，只要该节点是存储方向的 start，都可以原样出发。
      result.push_back(edge);
    } else if (edge.bidirectional && edge.end_id == node_id) {
      // 若当前节点是双向边存储方向的 end，复制边并交换端点，
      // 让调用方始终看到 start_id==node_id 的统一“出边”形式。
      Edge3D reversed = edge;
      reversed.start_id = edge.end_id;
      reversed.end_id = edge.start_id;
      result.push_back(reversed);
    }
  }
  // 返回顺序继承自 unordered_map 的遍历顺序，不按 edge_id、代价或终点排序。
  return result;
}

// 计算两个图节点平移坐标之间的三维欧氏距离，姿态四元数不参与距离。
double Graph3D::distance(const NodeId a, const NodeId b) const
{
  const auto & na = nodes_.at(a);
  const auto & nb = nodes_.at(b);
  const auto dx = na.pose.tx - nb.pose.tx;
  const auto dy = na.pose.ty - nb.pose.ty;
  const auto dz = na.pose.tz - nb.pose.tz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 在线性扫描中查找距离给定 XYZ 最近的节点。
NodeId Graph3D::nearestNode(const double x, const double y, const double z) const
{
  // 空图不存在合法返回值，因此明确抛出异常。
  if (nodes_.empty()) {
    throw std::runtime_error("Cannot find nearest node in empty graph");
  }
  // 先用任意一个现有节点初始化 best_id，距离上界从 double 最大值开始。
  NodeId best_id = nodes_.begin()->first;
  double best_dist = std::numeric_limits<double>::max();
  for (const auto & [node_id, node] : nodes_) {
    const auto dx = node.pose.tx - x;
    const auto dy = node.pose.ty - y;
    const auto dz = node.pose.tz - z;
    // 和 KD-Tree 一样只比较平方距离，省去每个节点一次 sqrt。
    const auto dist = dx * dx + dy * dy + dz * dz;
    if (dist < best_dist) {
      best_dist = dist;
      best_id = node_id;
    }
  }
  // 使用严格 <；完全等距时保留 unordered_map 遍历中先遇到的节点。
  return best_id;
}

// 生成边去重所使用的有序节点对。
std::pair<NodeId, NodeId> Graph3D::edgeKey(
  const NodeId start_id,
  const NodeId end_id,
  const bool bidirectional) const
{
  if (bidirectional) {
    // 双向边与传入方向无关，统一把较小编号放前面。
    return {std::min(start_id, end_id), std::max(start_id, end_id)};
  }
  // 有向边必须保留方向，所以 (a,b) 与 (b,a) 是不同键。
  return {start_id, end_id};
}

}  // namespace nav2_route3d
