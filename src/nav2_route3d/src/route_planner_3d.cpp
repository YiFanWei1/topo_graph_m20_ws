#include "nav2_route3d/route_planner_3d.hpp"

// 本文件在 Graph3D 拓扑图上实现 A* 路线搜索。
// 搜索对象是“节点和边”，不是点云中的连续自由空间：
// - Graph3D::outgoingEdges() 决定当前节点可以走向哪些邻居；
// - EdgeScorer3D 决定边是否可用以及实际搜索代价；
// - blocked_edges 提供本次规划临时禁用的边；
// - 三维节点直线距离作为 A* 启发值。

#include <chrono>
#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace nav2_route3d
{
namespace
{

// 优先队列中的轻量条目。priority 是 A* 的 f(n)=g(n)+h(n)，node_id 是待展开节点。
struct QueueItem
{
  double priority{0.0};
  NodeId node_id{0};
  // std::priority_queue 默认是最大堆；定义 operator> 并配合 std::greater，得到最小 f 值优先。
  bool operator>(const QueueItem & other) const {return priority > other.priority;}
};

}  // namespace

// 规划器不复制图和评分器，只保存引用；二者的生命周期必须长于 RoutePlanner3D。
RoutePlanner3D::RoutePlanner3D(const Graph3D & graph, const EdgeScorer3D & scorer)
: graph_(graph), scorer_(scorer)
{
}

// 从 start_id 到 goal_id 搜索最低评分代价路线。
Route3D RoutePlanner3D::plan(
  const NodeId start_id,
  const NodeId goal_id,
  const std::unordered_set<EdgeId> & blocked_edges,
  const double max_planning_time_s) const
{
  // 起点或终点不存在时无法搜索，立即报告输入错误。
  if (!graph_.hasNode(start_id) || !graph_.hasNode(goal_id)) {
    throw std::runtime_error("Route endpoint not found in graph");
  }

  // steady_clock 不受系统时间校准影响，适合测量规划耗时。
  const auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(max_planning_time_s);
  // frontier 是按最小 priority 弹出的开放列表。
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> frontier;
  // 起点 g=0；此处 priority 也置 0，而不是额外加入起点启发值，不影响后续最优搜索。
  frontier.push({0.0, start_id});
  // cost_so_far[n] 保存当前已知从起点到 n 的最低累计评分代价 g(n)。
  std::unordered_map<NodeId, double> cost_so_far;
  // came_from[n] 保存到达 n 的最佳前驱节点及所用边，用于终点到起点反向重建。
  std::unordered_map<NodeId, std::pair<NodeId, Edge3D>> came_from;
  cost_so_far[start_id] = 0.0;

  while (!frontier.empty()) {
    // max_planning_time_s<=0 表示关闭超时限制；检查发生在每次展开节点之前。
    if (max_planning_time_s > 0.0 && std::chrono::steady_clock::now() - start_time > timeout) {
      throw std::runtime_error("3D route planning timeout");
    }
    // 取出当前估计总代价最小的节点。
    const auto current = frontier.top().node_id;
    frontier.pop();
    if (current == goal_id) {
      // 起点等于终点时也会走到这里，返回只包含一个节点、总代价为 0 的路线。
      return reconstruct(start_id, goal_id, came_from, cost_so_far.at(goal_id));
    }

    // outgoingEdges() 会把双向边在反向遍历时交换端点，因此这里统一使用 edge.end_id。
    for (const auto & edge : graph_.outgoingEdges(current)) {
      // 同一条双向边正反方向共享 edge_id，所以加入 blocked_edges 后两个方向都被禁用。
      if (blocked_edges.count(edge.edge_id) != 0U) {
        continue;
      }
      // 评分器会综合基础 cost、风险、坡度和场景属性，也可能把 closed/forbidden 边判为无效。
      const auto score = scorer_.score(edge);
      if (!score.valid) {
        continue;
      }
      // 经当前节点走这条边到达邻居时的新累计代价。
      const auto new_cost = cost_so_far.at(current) + score.cost;
      const auto old_it = cost_so_far.find(edge.end_id);
      // 第一次到达邻居，或找到了更便宜的路线时，执行“松弛”。
      if (old_it == cost_so_far.end() || new_cost < old_it->second) {
        cost_so_far[edge.end_id] = new_cost;
        came_from[edge.end_id] = {current, edge};
        // 不执行 priority_queue 的 decrease-key，而是压入一个新条目；
        // 因此 frontier 可能含同一节点的旧条目，但 cost_so_far 始终保留最新最低 g 值。
        frontier.push({new_cost + heuristic(edge.end_id, goal_id), edge.end_id});
      }
    }
  }

  // 开放列表耗尽仍未弹出终点，说明在方向、阻塞和评分约束下没有可达路线。
  throw std::runtime_error("No valid 3D route found");
}

// A* 启发函数：当前节点到目标节点的三维欧氏直线距离。
double RoutePlanner3D::heuristic(const NodeId node_id, const NodeId goal_id) const
{
  // 当所有有效边评分代价不小于几何行走距离时，该启发值是可采纳的；
  // 若通过 metadata 配置了负 penalty/risk 或低于几何距离的边代价，则最优性前提不再成立。
  return graph_.distance(node_id, goal_id);
}

// 根据 came_from 从目标反向追溯到起点，再翻转为正向 Route3D。
Route3D RoutePlanner3D::reconstruct(
  const NodeId start_id,
  const NodeId goal_id,
  const std::unordered_map<NodeId, std::pair<NodeId, Edge3D>> & came_from,
  const double cost) const
{
  Route3D route;
  // cost 已由 plan() 从 cost_so_far[goal] 传入，是 EdgeScorer3D 评分的累计值。
  route.cost = cost;
  NodeId current = goal_id;
  // 先从目标节点开始反向收集。
  route.node_ids.push_back(goal_id);
  while (current != start_id) {
    const auto it = came_from.find(current);
    // 正常搜索到达的非起点节点都应有前驱；缺失说明内部状态不一致。
    if (it == came_from.end()) {
      throw std::runtime_error("Cannot reconstruct route");
    }
    // 即使搜索沿双向边的反向副本前进，edge_id 仍是图中原边 ID。
    route.edge_ids.push_back(it->second.second.edge_id);
    current = it->second.first;
    route.node_ids.push_back(current);
  }
  // 当前两个数组都是 goal->start 顺序，分别翻转成 start->goal。
  std::reverse(route.node_ids.begin(), route.node_ids.end());
  std::reverse(route.edge_ids.begin(), route.edge_ids.end());
  return route;
}

}  // namespace nav2_route3d
