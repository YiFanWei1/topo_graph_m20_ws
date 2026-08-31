#pragma once

// 本头文件声明Route3D离线构图器的配置、执行阶段和内部状态。
// 这里只描述“如何构建图”的接口；各步骤的具体算法实现在graph_builder_3d.cpp。

#include <filesystem>
#include <thread>
#include <vector>

// file_loader_3d.hpp提供Pose3D读取、Graph3D以及JSON/GeoJSON文件接口。
#include "nav2_route3d/file_loader_3d.hpp"
// gridmap_2d.hpp提供二维占据栅格和Bresenham线段遮挡检测。
#include "nav2_route3d/gridmap_2d.hpp"
// kdtree_3d.hpp提供Point3D类型以及anchor的三维半径搜索。
#include "nav2_route3d/kdtree_3d.hpp"

namespace nav2_route3d
{

// 离线构图的完整配置。route_graph_builder_3d_main.cpp负责从命令行填充这些字段，
// GraphBuilder3D构造后保存一份配置副本，整个build()过程中参数保持不变。
struct GraphBuilderConfig
{
  // 数据根目录。姿态文件、PCD目录以及默认输出图文件都以此路径为基准。
  std::filesystem::path map_path;

  // 相对于map_path的姿态JSON文件名。每个姿态必须包含：
  // [timestamp, tx, ty, tz, qx, qy, qz, qw]。
  std::string poses_file{"poses.json"};

  // 相对于map_path的关键帧PCD目录。姿态索引i对应目录中的i.pcd。
  std::string pcd_dir{"pcd"};

  // KD-Tree三维半径搜索范围，单位m。只有该范围内的其他anchor才可能成为shortcut候选。
  double radius_m{5.0};

  // 候选anchor与当前anchor的最小时间间隔，单位s。
  // 时间差小于等于该值时认为仍属于邻近连续运动，拒绝额外shortcut。
  double time_threshold_s{10.0};

  // 两个anchor允许的最大机体Z差，单位m。超过后拒绝候选，避免误连上下楼层。
  double z_threshold_m{0.2};

  // 障碍高度带向anchor下方延伸的距离，单位m：nav_low=anchor_z-sensor_height_m。
  double sensor_height_m{0.7};

  // 障碍高度带向anchor上方延伸的距离，单位m：
  // nav_high=anchor_z+current_up_threshold_m。
  double current_up_threshold_m{1.2};

  // 每个shortcut候选前后额外加载的关键帧数量。
  // 值为3时会尝试读取candidate-3至candidate+3共7帧PCD，边界索引会自动忽略。
  int neighbor_pcd_window{3};

  // 把障碍点XY投影为二维占据栅格时的分辨率，单位m/格。
  double grid_resolution_m{0.2};

  // 沿原始三维累计轨迹划分anchor区段的长度，单位m。
  // 每段选择靠近区段中点的一个真实姿态；它不控制图节点或TXT目标点间距。
  double anchor_segment_length_m{5.0};

  // 允许创建shortcut的最小三维直线距离，单位m；更近的候选收益过小而被拒绝。
  double min_shortcut_distance_m{1.0};

  // 按当前anchor指向候选anchor的XY方位角分桶，单位deg。
  double shortcut_angular_bin_deg{20.0};

  // 每个方位角桶最多保留的最近候选数，用于抑制同方向重复边。
  size_t max_candidates_per_angular_bin{2};

  // 每个anchor最多进入PCD占据栅格和射线检测阶段的候选数量。
  size_t max_raytrace_candidates_per_node{24};

  // 每个图节点最多允许存在的radius_visibility shortcut边数量。
  // 达到上限后的候选会标记为fake_neighbor，不会加入Graph3D::edges()。
  size_t max_shortcut_edges_per_node{8};

  // 点云射线遮挡判断的最大并行任务数；默认采用系统报告的硬件线程数。
  // 若hardware_concurrency()返回0，processPose()内部仍会把实际worker数提升为1。
  size_t max_workers{std::thread::hardware_concurrency()};

  // 输出Graph3D和图文件声明的全局坐标系名称。
  // 注意该字段只设置frame字符串，不会数值变换输入姿态。
  std::string frame_id{"map"};
};

// 从时间排序的关键帧姿态和机体系PCD构建三维路线拓扑图。
// 构造阶段读取姿态、选择anchor、创建全部节点和KD-Tree；build()阶段再创建所有边。
class GraphBuilder3D
{
public:
  // 按值接收配置，实现在内部move到config_，确保构图器拥有独立且完整的配置生命周期。
  explicit GraphBuilder3D(GraphBuilderConfig config);

  // 执行一次完整构图并按值返回Graph3D：先建立顺序链，再为每个anchor搜索shortcut。
  // 同一个实例设计为一次性离线构建器，不提供增量追加关键帧或回环后更新接口。
  Graph3D build();

private:
  // 无条件连接时间相邻节点i和i+1，生成双向sequential_odom/teach_repeat边，
  // 保证即使没有任何shortcut，图中仍保留完整录制路线。
  void connectSequentialChain();

  // 对一个anchor执行完整shortcut流程：半径搜索、候选过滤、点云合并、
  // 障碍高度过滤、二维栅格射线检测以及真实边/fake邻居记录。
  // current_index是poses_中的真实姿态下标，同时也是当前图节点ID。
  void processPose(size_t current_index);

  // 拒绝时间差过小或Z差过大的候选，并把它们写入双方fake_neighbors。
  std::vector<size_t> filterTimeAndZ(size_t current_index, const std::vector<size_t> & candidates);

  // 依次执行最小shortcut距离、方向分桶、每桶数量和最大射线候选数量剪枝。
  std::vector<size_t> pruneRedundantCandidates(size_t current_index, const std::vector<size_t> & candidates);

  // 加载所有候选前后neighbor_pcd_window范围的PCD，去重帧索引，
  // 再使用各帧Pose3D把base_link点云转换到图的全局坐标。
  std::vector<Point3D> loadCandidateClouds(const std::vector<size_t> & candidate_indices) const;

  // 只保留anchor_z-sensor_height_m至anchor_z+current_up_threshold_m内的点，
  // 这些点将投影到XY占据栅格，作为可能阻挡shortcut的障碍。
  std::vector<Point3D> filterObstaclePoints(const Pose3D & anchor_pose, const std::vector<Point3D> & points) const;

  // 将射线检测通过的候选按距离排序，并在每节点shortcut上限内创建双向边。
  // reason会写入边metadata.visibility_reason，例如raytrace_clear。
  void connectAllVisible(size_t current_index, const std::vector<size_t> & candidates, const std::string & reason);

  // 批量把候选与当前节点标记为对称fake_neighbors，不创建Edge3D。
  void rejectCandidates(size_t current_index, const std::vector<size_t> & candidates);

  // 使用OccupancyGrid2D检查两个anchor的XY中心线是否经过任一占据单元。
  // 返回true表示被障碍阻挡，不能创建shortcut。
  bool isOccluded(size_t current_index, size_t neighbor_index, const OccupancyGrid2D & grid) const;

  // 统计指定节点已有的radius_visibility边，用于执行max_shortcut_edges_per_node限制。
  size_t countShortcutEdges(NodeId node_id) const;

  // 返回两个姿态索引所对应图节点之间的三维欧氏距离。
  double candidateDistance(size_t current_index, size_t neighbor_index) const;

  GraphBuilderConfig config_;  // 本次离线构图的不可变配置副本。
  std::vector<Pose3D> poses_;  // 从姿态JSON读取的全部关键帧姿态，数组下标即图节点ID。
  std::vector<size_t> anchor_node_indices_;  // 被选作shortcut搜索anchor的poses_下标。
  Graph3D graph_;  // 正在构建的完整节点、真实边、real/fake邻居及metadata。
  KDTree3D kdtree_;  // 只包含anchor坐标；查询结果需映射回anchor_node_indices_。
};

// 供CLI或其他代码使用的便捷一次性接口，等价于：
// GraphBuilder3D(config).build()。
Graph3D buildGraphFromMap(const GraphBuilderConfig & config);

}  // namespace nav2_route3d
