#include "nav2_route3d/graph_builder_3d.hpp"

// 本文件实现“离线三维路线图”的核心构建流程：
// 1. 从 poses.txt 读取全部里程计位姿，并把每个位姿都保留为图节点；
// 2. 沿原始轨迹建立连续边，保证图至少包含一条可通行的示教路线；
// 3. 从轨迹中稀疏选取 Anchor，在 Anchor 之间搜索空间邻近但时间相隔较远的候选点；
// 4. 合并候选点附近的 PCD，用二维栅格射线判断两个 Anchor 之间是否可直连；
// 5. 把可直连关系写成 shortcut 边，把被过滤或遮挡的关系记为 fake neighbor。
//
// 这里不负责 ROS 话题订阅、路线最短路求解、拐点检测或最终 TXT 路点导出；
// 它只消费已经落盘并处于同一全局坐标系下的 poses.txt 与 pcd/，生成 Graph3D。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
// std::future/std::async 用于并行执行多条候选边的栅格射线检测。
#include <future>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "nav2_route3d/gridmap_2d.hpp"
#include "nav2_route3d/pcd_io.hpp"

namespace nav2_route3d
{
namespace
{

// C++17 标准库没有保证提供 std::numbers::pi，因此在此固定定义圆周率。
// 它只用于把配置中的角度制 shortcut_angular_bin_deg 转成弧度。
constexpr double kPi = 3.14159265358979323846;

// 计算两个轨迹位姿平移部分的三维欧氏距离。
// 注意这里包含 z，因此 Anchor 的“沿程距离”会反映楼梯、坡道上的高度变化，
// 而不只是俯视平面中的 XY 距离。
double poseDistance(const Pose3D & lhs, const Pose3D & rhs)
{
  const auto dx = lhs.tx - rhs.tx;
  const auto dy = lhs.ty - rhs.ty;
  const auto dz = lhs.tz - rhs.tz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 把一帧 PCD 中、以雷达/机体局部坐标表示的点转换到建图全局坐标系：
//
//   P_map = R(q_map_body) * P_body + t_map_body
//
// poses.txt 中的位姿与对应编号 PCD 必须严格配对，且四元数应已经归一化。
// 这里显式展开四元数旋转矩阵，避免为这一处点变换引入额外的 Eigen 依赖。
Point3D transformPointToMap(const Pose3D & pose, const Point3D & local)
{
  // 局部点坐标。
  const double x = local[0];
  const double y = local[1];
  const double z = local[2];
  // 当前 PCD 帧在全局坐标系中的姿态四元数 (x, y, z, w)。
  const double qx = pose.qx;
  const double qy = pose.qy;
  const double qz = pose.qz;
  const double qw = pose.qw;
  // 预先计算旋转矩阵展开式中的 2*q 及其乘积，减少每个点的重复乘法。
  const double tx = 2.0 * qx;
  const double ty = 2.0 * qy;
  const double tz = 2.0 * qz;
  const double twx = tx * qw;
  const double twy = ty * qw;
  const double twz = tz * qw;
  const double txx = tx * qx;
  const double txy = ty * qx;
  const double txz = tz * qx;
  const double tyy = ty * qy;
  const double tyz = tz * qy;
  const double tzz = tz * qz;
  // 前三项是旋转 R(q)*P_body，最后分别加全局平移 pose.t{x,y,z}。
  return {
    (1.0 - (tyy + tzz)) * x + (txy - twz) * y + (txz + twy) * z + pose.tx,
    (txy + twz) * x + (1.0 - (txx + tzz)) * y + (tyz - twx) * z + pose.ty,
    (txz - twy) * x + (tyz + twx) * y + (1.0 - (txx + tyy)) * z + pose.tz
  };
}

// 沿原始轨迹按“累计三维路程”分段，并在每段中选择最靠近路程中点的位姿作为 Anchor。
// Anchor 只用于昂贵的近邻搜索和射线建边；非 Anchor 位姿仍然会进入图并组成连续路线。
std::vector<size_t> anchorNodeIndicesFromPoses(
  const std::vector<Pose3D> & poses,
  const double segment_length_m)
{
  std::vector<size_t> anchors;
  // 没有位姿时自然无法产生 Anchor。
  if (poses.empty()) {
    return anchors;
  }

  // 非正分段长度表示关闭稀疏化：每个位姿都参与 shortcut 搜索。
  if (segment_length_m <= 0.0) {
    anchors.reserve(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      anchors.push_back(i);
    }
    return anchors;
  }

  // cumulative_distance[i] 表示从第 0 帧沿原轨迹走到第 i 帧的累计三维距离。
  // 这是沿轨迹的弧长，不是第 0 帧到第 i 帧的直线距离。
  std::vector<double> cumulative_distance(poses.size(), 0.0);
  for (size_t i = 1U; i < poses.size(); ++i) {
    cumulative_distance[i] = cumulative_distance[i - 1U] + poseDistance(poses[i - 1U], poses[i]);
  }

  const double total_distance = cumulative_distance.back();
  // 所有位姿几乎重合时没有可用的沿程长度，退化为选择序列中间帧。
  if (total_distance <= 1.0e-6) {
    anchors.push_back(poses.size() / 2U);
    return anchors;
  }

  double segment_start = 0.0;
  while (segment_start < total_distance) {
    // 最后一段可能短于 segment_length_m。
    const double segment_end = std::min(segment_start + segment_length_m, total_distance);
    const double segment_midpoint = 0.5 * (segment_start + segment_end);
    // lower_bound 找到累计路程第一个不小于分段中点的位姿。
    const auto upper = std::lower_bound(
      cumulative_distance.begin(), cumulative_distance.end(), segment_midpoint);

    size_t anchor_index = 0U;
    if (upper == cumulative_distance.end()) {
      // 理论上 segment_midpoint 不会大于 total_distance；保留该分支作边界保护。
      anchor_index = cumulative_distance.size() - 1U;
    } else {
      anchor_index = static_cast<size_t>(std::distance(cumulative_distance.begin(), upper));
      if (anchor_index > 0U) {
        // 中点通常落在两帧之间，比较前后两帧到中点的沿程误差，选择较近者。
        const size_t lower_index = anchor_index - 1U;
        const double lower_error = std::abs(cumulative_distance[lower_index] - segment_midpoint);
        const double upper_error = std::abs(cumulative_distance[anchor_index] - segment_midpoint);
        if (lower_error <= upper_error) {
          anchor_index = lower_index;
        }
      }
    }

    // 当位姿很密、分段很短时，相邻分段可能选中同一帧；这里避免重复 Anchor。
    if (anchors.empty() || anchors.back() != anchor_index) {
      anchors.push_back(anchor_index);
    }
    segment_start = segment_end;
  }

  // 防御性兜底：只要 poses 非空，返回值至少包含一个合法索引。
  if (anchors.empty()) {
    anchors.push_back(0U);
  }
  return anchors;
}

// 提取 Anchor 位姿的 XYZ，作为 KD-Tree 的点集。
// 关键索引关系：KD-Tree 返回的是此 points 数组的下标，而不是原 poses_ 的帧号；
// 后续必须再通过 anchor_node_indices_ 映射回真正的图节点索引。
std::vector<Point3D> pointsFromPoseIndices(
  const std::vector<Pose3D> & poses,
  const std::vector<size_t> & indices)
{
  std::vector<Point3D> points;
  points.reserve(indices.size());
  for (const auto index : indices) {
    const auto & pose = poses[index];
    points.push_back({pose.tx, pose.ty, pose.tz});
  }
  return points;
}

}  // namespace

// 初始化列表的顺序与头文件中的成员声明顺序一致：
// 先保存配置，再由配置定位 poses.txt，随后选 Anchor、创建图和 Anchor KD-Tree。
GraphBuilder3D::GraphBuilder3D(GraphBuilderConfig config)
: config_(std::move(config)),
  poses_(loadPoseRows(config_.map_path / config_.poses_file)),
  anchor_node_indices_(anchorNodeIndicesFromPoses(poses_, config_.anchor_segment_length_m)),
  graph_(config_.frame_id),
  kdtree_(pointsFromPoseIndices(poses_, anchor_node_indices_))
{
  // 图节点不会因 Anchor 稀疏化而减少：每一行 pose 都成为一个节点，node_id 等于行索引。
  // 这样连续边可以完整保存原始轨迹的楼梯起伏和局部几何。
  for (size_t i = 0; i < poses_.size(); ++i) {
    Node3D node;
    node.node_id = static_cast<NodeId>(i);
    node.pose = poses_[i];
    node.metadata = Metadata::object({{"source", "odom_pose"}});
    // Anchor 索引按轨迹顺序产生，因此是有序的，可以用 binary_search 判定。
    // 该标记便于 JSON 可视化和调试，本身不改变节点的通行含义。
    if (std::binary_search(anchor_node_indices_.begin(), anchor_node_indices_.end(), i)) {
      node.metadata["shortcut_anchor"] = true;
    }
    graph_.addNode(node);
  }
}

// 完整构图入口。先建立不会失败的原始轨迹链，再尝试增加 shortcut。
// 即使没有任何 PCD 或所有 shortcut 都被拒绝，连续图仍然保留。
Graph3D GraphBuilder3D::build()
{
  connectSequentialChain();
  // 只对 Anchor 执行半径搜索和 PCD 可见性检测，以控制离线构图耗时。
  for (const auto anchor_index : anchor_node_indices_) {
    processPose(anchor_index);
  }
  if (!graph_.metadata().is_object()) {
    graph_.metadata() = Metadata::object();
  }
  // 把影响构图结果的主要参数随图保存，便于之后追溯 JSON 是如何生成的。
  graph_.metadata()["builder"] = "GraphBuilder3D";
  graph_.metadata()["radius_m"] = config_.radius_m;
  graph_.metadata()["time_threshold_s"] = config_.time_threshold_s;
  graph_.metadata()["z_threshold_m"] = config_.z_threshold_m;
  graph_.metadata()["grid_resolution_m"] = config_.grid_resolution_m;
  graph_.metadata()["anchor_segment_length_m"] = config_.anchor_segment_length_m;
  graph_.metadata()["anchor_node_count"] = anchor_node_indices_.size();
  graph_.metadata()["min_shortcut_distance_m"] = config_.min_shortcut_distance_m;
  graph_.metadata()["shortcut_angular_bin_deg"] = config_.shortcut_angular_bin_deg;
  graph_.metadata()["max_candidates_per_angular_bin"] = config_.max_candidates_per_angular_bin;
  graph_.metadata()["max_raytrace_candidates_per_node"] = config_.max_raytrace_candidates_per_node;
  graph_.metadata()["max_shortcut_edges_per_node"] = config_.max_shortcut_edges_per_node;
  return graph_;
}

// 把相邻的原始位姿 i 与 i+1 依次连接，构成示教/回放的基础链路。
void GraphBuilder3D::connectSequentialChain()
{
  // 0 或 1 个节点都没有可连接的相邻对。
  if (poses_.size() < 2U) {
    return;
  }
  for (size_t i = 0; i + 1U < poses_.size(); ++i) {
    // cost 传 std::nullopt：由 Graph3D 按节点三维距离自动计算代价。
    // operations 为空；最后的 true 表示添加双向边，所以路线可以正向或反向使用。
    // 连续边不检查 PCD 障碍，因为它代表数据采集时机器实际走过的轨迹。
    graph_.addEdge(
      static_cast<NodeId>(i),
      static_cast<NodeId>(i + 1U),
      std::nullopt,
      Metadata::object({{"edge_source", "sequential_odom"}, {"edge_type", "teach_repeat"}}),
      {},
      true);
  }
}

// 为一个 Anchor 搜索并验证可见的 shortcut 邻居，是构图中计算量最大的函数。
void GraphBuilder3D::processPose(const size_t current_index)
{
  const auto current_id = static_cast<NodeId>(current_index);
  const auto & current = graph_.node(current_id);
  // KD-Tree 建在 Anchor 坐标上，因此这里只会返回半径内的其他 Anchor。
  // 搜索距离采用三维欧氏距离，radius_m 同时约束 XY 和 Z。
  const auto anchor_candidates = kdtree_.radiusSearch(
    {current.pose.tx, current.pose.ty, current.pose.tz},
    config_.radius_m);

  std::vector<size_t> candidates;
  candidates.reserve(anchor_candidates.size());
  for (const auto anchor_candidate : anchor_candidates) {
    // anchor_candidate 是 KD-Tree 点集下标，先检查再映射回 poses_/Graph3D 的节点索引。
    if (anchor_candidate >= anchor_node_indices_.size()) {
      continue;
    }
    const auto idx = anchor_node_indices_[anchor_candidate];
    const auto nb_id = static_cast<NodeId>(idx);
    // 排除自己，以及此前已判定为真实邻居或假邻居的节点，避免重复检测同一关系。
    if (idx == current_index ||
      current.real_neighbors.count(nb_id) != 0U ||
      current.fake_neighbors.count(nb_id) != 0U)
    {
      continue;
    }
    candidates.push_back(idx);
  }

  if (candidates.empty()) {
    return;
  }
  // 第一层轻量剪枝：排除时间上太近（通常只是同一路段相邻帧）或高度差过大的点。
  candidates = filterTimeAndZ(current_index, candidates);
  if (candidates.empty()) {
    return;
  }
  // 第二层轻量剪枝：按最短距离、方向角分箱和数量上限减少昂贵的 PCD 射线任务。
  candidates = pruneRedundantCandidates(current_index, candidates);
  if (candidates.empty()) {
    return;
  }

  // 加载所有保留候选点附近窗口内的 PCD，并统一变换到 map 坐标系。
  auto points = loadCandidateClouds(candidates);
  if (points.empty()) {
    // “没有加载到任何原始点”通常意味着 PCD 缺失/读取失败，不能把未知空间误判成畅通。
    rejectCandidates(current_index, candidates);
    return;
  }

  // 仅保留当前 Anchor 可导航高度带内的点，作为二维投影后的障碍点。
  auto obstacle_points = filterObstaclePoints(current.pose, points);
  if (obstacle_points.empty()) {
    // 与 points.empty() 不同：这里确认读到了点云，但指定高度带内没有障碍，按可见处理。
    connectAllVisible(current_index, candidates, "empty_obstacle_cloud");
    return;
  }

  // OccupancyGrid2D 把三维障碍点投影到 XY 栅格，后续只检测线段穿过的栅格。
  const OccupancyGrid2D grid(obstacle_points, config_.grid_resolution_m);
  if (grid.empty()) {
    // 防御性处理：有 obstacle_points 却未能建立有效栅格时，选择保守拒绝而不是直连。
    rejectCandidates(current_index, candidates);
    return;
  }

  // max_workers 配成 0 时也至少保留一个工作线程。
  const size_t worker_count = std::max<size_t>(1U, config_.max_workers);
  std::vector<std::future<std::pair<size_t, bool>>> futures;
  std::vector<size_t> visible_candidates;
  visible_candidates.reserve(candidates.size());
  futures.reserve(candidates.size());
  for (const auto nb_idx : candidates) {
    // 每个异步任务只读 graph_ 和 grid；真实/假邻居的写入统一留在主线程 future.get() 后执行，
    // 避免多个工作线程同时修改 Graph3D。
    futures.push_back(
      std::async(
        std::launch::async,
        [this, current_index, nb_idx, &grid]() {
          return std::make_pair(nb_idx, isOccluded(current_index, nb_idx, grid));
        }));
    // 以 worker_count 为一批等待，限制同时进行的射线检测数量。
    if (futures.size() >= worker_count) {
      for (auto & future : futures) {
        const auto [idx, blocked] = future.get();
        if (blocked) {
          // 被占用栅格挡住的候选关系对称地登记为 fake neighbor，之后不再重复尝试。
          graph_.markFakeNeighbor(current_id, static_cast<NodeId>(idx));
        } else {
          visible_candidates.push_back(idx);
        }
      }
      futures.clear();
    }
  }

  // 处理不足一个完整批次的尾部任务。
  for (auto & future : futures) {
    const auto [idx, blocked] = future.get();
    if (blocked) {
      graph_.markFakeNeighbor(current_id, static_cast<NodeId>(idx));
    } else {
      visible_candidates.push_back(idx);
    }
  }
  // 可见候选还要经过每节点 shortcut 总数上限，最终才真正写边。
  connectAllVisible(current_index, visible_candidates, "raytrace_clear");
}

// 用时间差排除同一次局部经过，用高度差排除上下层误连。
// 两个条件是“或”关系：任意一个不满足 shortcut 要求，就不进入后续点云检测。
std::vector<size_t> GraphBuilder3D::filterTimeAndZ(
  const size_t current_index,
  const std::vector<size_t> & candidates)
{
  const auto current_id = static_cast<NodeId>(current_index);
  const auto & current = graph_.node(current_id);
  std::vector<size_t> filtered;
  filtered.reserve(candidates.size());

  for (const auto idx : candidates) {
    const auto nb_id = static_cast<NodeId>(idx);
    const auto & neighbor = graph_.node(nb_id);
    // 时间戳太接近通常说明两个 Anchor 属于同一次经过的相邻局部轨迹。
    // 这类点已有 sequential_odom 链连接，没有必要再添加 shortcut。
    const bool time_close =
      std::abs(neighbor.pose.timestamp - current.pose.timestamp) <= config_.time_threshold_s;
    // 高差太大的节点即使 XY 很近，也可能分别位于上下两层或不同楼梯段，禁止直接连边。
    const bool z_too_far = std::abs(neighbor.pose.tz - current.pose.tz) > config_.z_threshold_m;
    if (time_close || z_too_far) {
      // 被确定性规则淘汰的关系记入 fake neighbor，使反向处理到该节点时也能跳过。
      graph_.markFakeNeighbor(current_id, nb_id);
    } else {
      filtered.push_back(idx);
    }
  }
  return filtered;
}

// 在加载点云之前对候选做几何与数量剪枝，降低磁盘 IO 和射线检测次数。
std::vector<size_t> GraphBuilder3D::pruneRedundantCandidates(
  const size_t current_index,
  const std::vector<size_t> & candidates)
{
  const auto current_id = static_cast<NodeId>(current_index);
  // 正数表示启用每节点 shortcut 上限；0 表示不限制。
  // 当前节点已经达到上限时，剩余候选无需再加载 PCD。
  if (config_.max_shortcut_edges_per_node > 0U &&
    countShortcutEdges(current_id) >= config_.max_shortcut_edges_per_node)
  {
    for (const auto idx : candidates) {
      graph_.markFakeNeighbor(current_id, static_cast<NodeId>(idx));
    }
    return {};
  }

  struct Candidate
  {
    // poses_/图节点索引。
    size_t index{0};
    // 当前节点到该候选节点的三维直线距离，用于优先保留近邻。
    double distance{0.0};
  };

  const auto & current = graph_.node(current_id);
  // 配置使用角度制；atan2 返回弧度，所以先完成单位转换。
  const double bin_rad = config_.shortcut_angular_bin_deg * kPi / 180.0;
  // bins 按当前点看向候选点的 XY 方位角分组；同一方向只保留较近的少量候选。
  std::unordered_map<int, std::vector<Candidate>> bins;
  // 关闭角度分箱时，所有候选进入 unbinned，跳过每角度箱数量限制。
  std::vector<Candidate> unbinned;
  for (const auto idx : candidates) {
    const auto distance = candidateDistance(current_index, idx);
    // 太近的 shortcut 没有实际拓扑价值，原始连续链已经能连接这些节点。
    if (distance < config_.min_shortcut_distance_m) {
      graph_.markFakeNeighbor(current_id, static_cast<NodeId>(idx));
      continue;
    }

    const auto & neighbor = graph_.node(static_cast<NodeId>(idx));
    // 这里只用 XY 方位角分箱；z 不参与方向分类。
    const double angle = std::atan2(neighbor.pose.ty - current.pose.ty, neighbor.pose.tx - current.pose.tx);
    const Candidate candidate{idx, distance};
    // 角度宽度有效且箱容量为正时才启用分箱，否则视为不限制每方向候选数。
    if (bin_rad > 1.0e-6 && config_.max_candidates_per_angular_bin > 0U) {
      // atan2 范围为 [-pi, pi]，先平移到 [0, 2*pi] 再除以箱宽得到整数箱号。
      const int bin = static_cast<int>(std::floor((angle + kPi) / bin_rad));
      bins[bin].push_back(candidate);
    } else {
      unbinned.push_back(candidate);
    }
  }

  std::vector<Candidate> kept;
  for (auto & [_, items] : bins) {
    // 每个方位优先检测距离更近的候选，超过容量的直接记为假邻居。
    std::sort(
      items.begin(), items.end(),
      [](const Candidate & lhs, const Candidate & rhs) {return lhs.distance < rhs.distance;});
    const auto keep_count = std::min(config_.max_candidates_per_angular_bin, items.size());
    kept.insert(kept.end(), items.begin(), items.begin() + static_cast<std::ptrdiff_t>(keep_count));
    for (size_t i = keep_count; i < items.size(); ++i) {
      graph_.markFakeNeighbor(current_id, static_cast<NodeId>(items[i].index));
    }
  }
  // 未启用分箱时，unbinned 中的候选全部进入下一阶段。
  kept.insert(kept.end(), unbinned.begin(), unbinned.end());

  // 再做一次全局距离排序，使每个 Anchor 优先把有限射线预算用于近处候选。
  std::sort(
    kept.begin(), kept.end(),
    [](const Candidate & lhs, const Candidate & rhs) {return lhs.distance < rhs.distance;});
  if (config_.max_raytrace_candidates_per_node > 0U &&
    kept.size() > config_.max_raytrace_candidates_per_node)
  {
    // 正数上限表示每个 Anchor 最多对这么多候选执行 PCD 射线检测；0 表示不限。
    for (size_t i = config_.max_raytrace_candidates_per_node; i < kept.size(); ++i) {
      graph_.markFakeNeighbor(current_id, static_cast<NodeId>(kept[i].index));
    }
    kept.resize(config_.max_raytrace_candidates_per_node);
  }

  // 对外只返回节点索引，不暴露剪枝阶段临时计算的距离。
  std::vector<size_t> result;
  result.reserve(kept.size());
  for (const auto & candidate : kept) {
    result.push_back(candidate.index);
  }
  return result;
}

// 加载候选 Anchor 周围的多帧 PCD，并把所有点变换、合并到全局坐标系。
std::vector<Point3D> GraphBuilder3D::loadCandidateClouds(
  const std::vector<size_t> & candidate_indices) const
{
  // unordered_set 用于去重：多个候选的窗口很可能包含同一帧，避免重复读盘和重复加点。
  // 集合遍历顺序不固定，但点的合并顺序不影响后续占用栅格的几何结果。
  std::unordered_set<size_t> frame_indices;
  for (const auto idx : candidate_indices) {
    // 每个候选不仅使用自身 PCD，还使用前后 neighbor_pcd_window 帧补充局部环境。
    for (int offset = -config_.neighbor_pcd_window; offset <= config_.neighbor_pcd_window; ++offset) {
      // 先转为有符号整数再加负 offset，避免 size_t 无符号下溢成巨大索引。
      const auto frame = static_cast<int64_t>(idx) + static_cast<int64_t>(offset);
      // 轨迹开头和结尾的窗口会越界，越界帧直接跳过。
      if (frame >= 0 && frame < static_cast<int64_t>(poses_.size())) {
        frame_indices.insert(static_cast<size_t>(frame));
      }
    }
  }

  std::vector<Point3D> merged;
  const auto pcd_dir = config_.map_path / config_.pcd_dir;
  for (const auto frame_idx : frame_indices) {
    // findPcdForIndex 负责按帧号查找实际 PCD 文件，兼容项目定义的文件命名方式。
    const auto pcd_path = findPcdForIndex(pcd_dir, frame_idx);
    if (!pcd_path.has_value()) {
      // 单帧缺失时允许使用窗口内其余帧；只有最终 merged 为空才整体拒绝候选。
      continue;
    }
    try {
      // PCD 中保存的是该帧雷达/机体坐标点，不能直接与全局位姿节点做射线比较。
      auto cloud = readPcdXYZ(pcd_path.value());
      const auto & pose = poses_.at(frame_idx);
      for (const auto & point : cloud) {
        // 使用同编号优化位姿，把局部点统一投到 graph_.frameId() 对应的全局坐标系。
        merged.push_back(transformPointToMap(pose, point));
      }
    } catch (const std::exception & ex) {
      // 坏掉的单个 PCD 不终止整个离线构图；打印警告并继续利用其他帧。
      std::cerr << "[WARN] Failed to read " << pcd_path->string() << ": " << ex.what() << "\n";
    }
  }
  return merged;
}

// 从全局点云中截取相对于当前 Anchor 的导航高度带，并把它们视为二维障碍投影来源。
std::vector<Point3D> GraphBuilder3D::filterObstaclePoints(
  const Pose3D & anchor_pose,
  const std::vector<Point3D> & points) const
{
  // 下边界大致落在机器人脚下/地面附近：Anchor 高度减传感器安装高度。
  const double nav_low = anchor_pose.tz - config_.sensor_height_m;
  // 上边界由 current_up_threshold_m 控制，用于忽略过高、与通行无关的点。
  const double nav_high = anchor_pose.tz + config_.current_up_threshold_m;
  std::vector<Point3D> obstacle_points;
  obstacle_points.reserve(points.size());
  for (const auto & point : points) {
    // 这里只做全局 z 区间过滤，不拟合地面，也不根据坡度或楼梯形状分类。
    if (point[2] >= nav_low && point[2] <= nav_high) {
      obstacle_points.push_back(point);
    }
  }
  return obstacle_points;
}

// 将一组候选统一登记为不可连接。Graph3D 内部会维护对称的 fake-neighbor 关系。
void GraphBuilder3D::rejectCandidates(
  const size_t current_index,
  const std::vector<size_t> & candidates)
{
  const auto current_id = static_cast<NodeId>(current_index);
  for (const auto idx : candidates) {
    graph_.markFakeNeighbor(current_id, static_cast<NodeId>(idx));
  }
}

// 把已经通过可见性判定的候选写成 shortcut 边，同时执行最终的每节点边数限制。
void GraphBuilder3D::connectAllVisible(
  const size_t current_index,
  const std::vector<size_t> & candidates,
  const std::string & reason)
{
  const auto current_id = static_cast<NodeId>(current_index);
  std::vector<size_t> sorted = candidates;
  // 优先连接更近的可见节点，确保边数预算不足时保留更局部、通常更稳健的 shortcut。
  std::sort(
    sorted.begin(), sorted.end(),
    [this, current_index](const size_t lhs, const size_t rhs) {
      return candidateDistance(current_index, lhs) < candidateDistance(current_index, rhs);
    });

  // 统计此前已经由其他 Anchor 处理流程写入的 shortcut，防止双向构图时超过配置上限。
  size_t shortcut_count = countShortcutEdges(current_id);
  for (const auto idx : sorted) {
    if (config_.max_shortcut_edges_per_node > 0U &&
      shortcut_count >= config_.max_shortcut_edges_per_node)
    {
      // 超额候选显式记为 fake，避免稍后从另一端再次尝试。
      graph_.markFakeNeighbor(current_id, static_cast<NodeId>(idx));
      continue;
    }
    // cost 为 nullopt，仍由 Graph3D 使用节点三维距离；operations 暂为空。
    // visibility_reason 区分“高度带无障碍点”和“射线逐格检查畅通”两种通过原因。
    // true 表示一条可正反通行的双向 shortcut。
    graph_.addEdge(
      current_id,
      static_cast<NodeId>(idx),
      std::nullopt,
      Metadata::object({{"edge_source", "radius_visibility"}, {"edge_type", "shortcut"}, {"visibility_reason", reason}}),
      {},
      true);
    ++shortcut_count;
  }
}

// 判断两个节点中心在 XY 平面上的线段是否穿过占用栅格。
bool GraphBuilder3D::isOccluded(
  const size_t current_index,
  const size_t neighbor_index,
  const OccupancyGrid2D & grid) const
{
  const auto & current = graph_.node(static_cast<NodeId>(current_index));
  const auto & neighbor = graph_.node(static_cast<NodeId>(neighbor_index));
  // 当前检测是二维中心线射线，不包含机器人足迹宽度膨胀，也不沿线插值检查 z。
  // 若需要安全余量，应在 OccupancyGrid2D 生成阶段对障碍做膨胀或扩展该接口。
  return grid.occupiedOnSegment(
    current.pose.tx, current.pose.ty,
    neighbor.pose.tx, neighbor.pose.ty);
}

// 统计一个节点现有的“半径可见性 shortcut”边数。
size_t GraphBuilder3D::countShortcutEdges(const NodeId node_id) const
{
  size_t count = 0U;
  for (const auto & edge : graph_.outgoingEdges(node_id)) {
    // 依据 edge_source 元数据计数，而不是笼统计算所有出边；
    // sequential_odom 基础链边不会消耗 shortcut 数量预算。
    if (getString(edge.metadata, "edge_source", "") == "radius_visibility") {
      ++count;
    }
  }
  return count;
}

// 统一调用 Graph3D 的三维节点距离函数，供候选排序和剪枝复用。
double GraphBuilder3D::candidateDistance(const size_t current_index, const size_t neighbor_index) const
{
  return graph_.distance(static_cast<NodeId>(current_index), static_cast<NodeId>(neighbor_index));
}

// 面向命令行工具和其他调用方的便捷函数：用配置构造一次 Builder 并立即完成构图。
// GraphBuilder3D 构造函数按值接收配置，因此这里会复制一份 config，再在构造函数内部 move。
Graph3D buildGraphFromMap(const GraphBuilderConfig & config)
{
  return GraphBuilder3D(config).build();
}

}  // namespace nav2_route3d
