#include "efficient_3d_local_planner/bspline_path_optimizer.hpp"
#include "efficient_3d_local_planner/guided_astar.hpp"
#include "efficient_3d_local_planner/replanning_policy.hpp"

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <efficient_3d_local_planner_msgs/msg/voxel_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace efficient_3d_local_planner
{

// 走廊约束三维 A* 规划节点的数据流：
//   活动全局路径（地面高度） + 机器人里程计 + hard/soft 两层局部体素地图
//       -> 坐标变换并把地面路径抬高到机身中心
//       -> 截取有限前视距离的三维 guide
//       -> 空旷时严格直连，否则执行严格/放宽两阶段 Guided A*
//       -> 路径视线简化
//       -> 常规 B 样条优化；失败时用未简化 A* 路径进行窄通道重试
//       -> 仅把碰撞复查成功的 B 样条作为 /local_planner/local_path 发布给控制器。
//
// 地图通常以约 10 Hz 更新，而 A* 和 B 样条可能耗时数毫秒到数十毫秒。订阅回调只
// 更新输入快照并通知 worker_；规划在线程中执行。规划完成后还要在最新地图上复查，
// 防止发布“基于旧地图算出、但新障碍已经出现”的过时路径。
class CorridorAStarPlannerNode : public rclcpp::Node
{
public:
  CorridorAStarPlannerNode()
  : Node("corridor_astar_planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    // -------- 全局路径预处理与坐标系参数 --------
    // 输入路线保存的是地面坐标，path_height_ 将每个点抬升到机器人机身中心高度；
    // planning_frame_ 通常为 camera_init，所有地图、里程计和输出路径最终都在该系表达。
    planning_frame_ = declare_parameter<std::string>("frames.planning", "camera_init");
    path_topic_ = declare_parameter<std::string>("input.path_topic", "/plan");
    path_height_ = declare_parameter<double>("planner.path_height", 0.40);
    // 楼梯点云和里程计高度会有小误差。允许在 start_z_max_correction_ 内把规划起点 Z
    // 投影到当前 guide；若差值更大，则认为活动路径属于错误楼层并拒绝规划。
    project_start_z_ = declare_parameter<bool>("planner.project_start_z", true);
    start_z_max_correction_ = declare_parameter<double>(
      "planner.start_z_max_correction", 0.60);
    reject_wrong_path_level_ = declare_parameter<bool>(
      "planner.reject_wrong_path_level", true);
    // 每次只规划当前位置之后 horizon_ 米的局部段；guide_spacing_ 控制参考线重采样。
    // replacement_grace_period_ 是旧路径被新地图判无效后等待替代路径的短暂宽限时间。
    horizon_ = declare_parameter<double>("planner.horizon", 5.0);
    guide_spacing_ = declare_parameter<double>("planner.guide_spacing", 0.15);
    replacement_grace_period_ = std::max(
      0.0, declare_parameter<double>("planner.replacement_grace_period", 0.25));

    // -------- Guided A* 配置 --------
    // corridor_xy/z 限定搜索不能离开全局 guide 太远，尤其避免上下层 XY 重叠时搜索到
    // 另一层；cylinder_offset 表示机器人前后两个碰撞圆柱中心相对机体中心的偏移。
    GuidedAStar::Config config;
    config.corridor_xy = declare_parameter<double>("planner.corridor_xy", 1.50);
    config.corridor_z = declare_parameter<double>("planner.corridor_z", 0.40);
    config.cylinder_offset = declare_parameter<double>("robot.cylinder_offset", 0.18);
    // soft/path/vertical/turn 分别惩罚贴近障碍、偏离全局路径、过度改变高度和频繁转向。
    // rotation_cost 是原地改变一个离散航向的代价。
    config.soft_weight = declare_parameter<double>("planner.soft_weight", 12.0);
    config.path_weight = declare_parameter<double>("planner.path_weight", 0.35);
    config.vertical_weight = declare_parameter<double>("planner.vertical_weight", 0.60);
    config.turn_weight = declare_parameter<double>("planner.turn_weight", 0.08);
    config.rotation_cost = declare_parameter<double>("planner.rotation_cost", 0.12);
    // goal_tolerance 允许搜索进入目标邻域后尝试精确连到目标；start/goal_snap_radius
    // 用于起点或局部目标被新障碍覆盖时，在有限范围寻找可用替代姿态。
    config.goal_tolerance = declare_parameter<double>("planner.goal_tolerance", 0.30);
    config.start_snap_radius = declare_parameter<double>("planner.start_snap_radius", 0.50);
    config.goal_snap_radius = declare_parameter<double>("planner.goal_snap_radius", 0.60);
    // A* 前置严格直连：按 sample_spacing 采样整条三维线及起步旋转扫掠，要求完整
    // footprint 既不进入硬障碍也不进入任何软膨胀，并限制相对 guide 的 XY/Z 偏差，
    // 因而只消除开放直线路段的 45°离散折线，不允许在拐角处切弯。
    config.direct_shortcut_enabled = declare_parameter<bool>(
      "planner.direct_shortcut_enabled", true);
    config.direct_shortcut_sample_spacing = std::max(
      0.01, declare_parameter<double>("planner.direct_shortcut_sample_spacing", 0.05));
    config.direct_shortcut_max_guide_deviation_xy = std::max(
      0.0, declare_parameter<double>(
        "planner.direct_shortcut_max_guide_deviation_xy", 0.15));
    config.direct_shortcut_max_guide_deviation_z = std::max(
      0.0, declare_parameter<double>(
        "planner.direct_shortcut_max_guide_deviation_z", 0.15));
    // strict 阶段把软膨胀视为不可进入；若无路，再进入允许付出软代价的 relaxed 阶段。
    // 两个 expansion 上限防止复杂或无解地图长期占满 CPU。
    config.strict_max_expansions = declare_parameter<int>(
      "planner.strict_max_expansions", 8000);
    config.max_expansions = declare_parameter<int>("planner.max_expansions", 120000);
    planner_ = std::make_unique<GuidedAStar>(config);
    // 简化后的任一线段不能过长，且下限至少为两个 guide 间隔，避免无意义的极短配置。
    max_simplified_segment_ = std::max(
      2.0 * guide_spacing_,
      declare_parameter<double>("planner.max_simplified_segment", 0.80));

    // -------- 常规 B 样条优化配置 --------
    // A* 给出离散可行路径，B 样条优化同时最小化二阶差分平滑代价、障碍距离代价、
    // 偏离 A* 参考路径代价和目标点移动代价，最后必须通过连续采样碰撞复查。
    BsplinePathOptimizer::Config optimization_config;
    optimization_config.enabled = declare_parameter<bool>("optimization.enabled", true);
    optimization_config.control_point_spacing = declare_parameter<double>(
      "optimization.control_point_spacing", 0.20);
    optimization_config.sample_spacing = declare_parameter<double>(
      "optimization.sample_spacing", 0.05);
    // max_iterations 是联合优化迭代数；clearance_repair 是联合优化结束后专门修补净空
    // 不足的额外迭代；max_step 限制控制点单次移动，防止梯度过大造成跳变。
    optimization_config.max_iterations = declare_parameter<int>(
      "optimization.max_iterations", 80);
    optimization_config.max_clearance_repair_iterations = declare_parameter<int>(
      "optimization.max_clearance_repair_iterations", 25);
    optimization_config.max_step = declare_parameter<double>("optimization.max_step", 0.04);
    // lambda_* 是四类目标函数的权重，不是米制距离。clearance_distance 才是期望离开
    // hard 的额外距离；minimum_acceptable_clearance 是成功验收硬门限。
    optimization_config.lambda_smooth = declare_parameter<double>(
      "optimization.lambda_smooth", 1.0);
    optimization_config.lambda_collision = declare_parameter<double>(
      "optimization.lambda_collision", 400.0);
    optimization_config.lambda_reference = declare_parameter<double>(
      "optimization.lambda_reference", 3.0);
    optimization_config.lambda_goal = declare_parameter<double>(
      "optimization.lambda_goal", 4.0);
    optimization_config.clearance_distance = declare_parameter<double>(
      "optimization.clearance_distance", 0.30);
    optimization_config.minimum_acceptable_clearance = declare_parameter<double>(
      "optimization.minimum_acceptable_clearance", 0.25);
    // 起点由机器人当前真实位置决定，无法通过优化瞬移；因此起步一小段不参与最低
    // 净空验收。reference_deadband 内允许自由平滑，超过后才产生回拉代价。
    optimization_config.start_clearance_ignore_distance = declare_parameter<double>(
      "optimization.start_clearance_ignore_distance", 0.20);
    optimization_config.reference_deadband = declare_parameter<double>(
      "optimization.reference_deadband", 0.10);
    // max_deviation/max_z_deviation 限制控制点离开 A* 参考的范围；goal_max_deviation
    // 单独限制末端。allow_goal_adjustment 允许局部目标稍离障碍，但仍受 lambda_goal 拉回。
    optimization_config.max_deviation = declare_parameter<double>(
      "optimization.max_deviation", 0.75);
    optimization_config.max_z_deviation = declare_parameter<double>(
      "optimization.max_z_deviation", 0.75);
    optimization_config.goal_max_deviation = declare_parameter<double>(
      "optimization.goal_max_deviation", 0.55);
    optimization_config.allow_goal_adjustment = declare_parameter<bool>(
      "optimization.allow_goal_adjustment", true);
    optimization_config.cylinder_offset = config.cylinder_offset;
    optimizer_ = std::make_unique<BsplinePathOptimizer>(optimization_config);

    // -------- 窄路/楼梯 B 样条重试配置 --------
    // 常规优化失败时，从“未简化 A* 状态”重新初始化：更密控制点和采样、更强参考约束、
    // 更小 Z 偏差及较低平滑权重，优先保持 A* 的可通行几何而不是强行把急弯拉圆。
    BsplinePathOptimizer::Config narrow_config = optimization_config;
    narrow_config.enabled = optimization_config.enabled && declare_parameter<bool>(
      "optimization.narrow.enabled", true);
    narrow_config.control_point_spacing = declare_parameter<double>(
      "optimization.narrow.control_point_spacing", 0.10);
    narrow_config.sample_spacing = declare_parameter<double>(
      "optimization.narrow.sample_spacing", 0.03);
    narrow_config.max_iterations = declare_parameter<int>(
      "optimization.narrow.max_iterations", 100);
    narrow_config.max_clearance_repair_iterations = declare_parameter<int>(
      "optimization.narrow.max_clearance_repair_iterations", 40);
    narrow_config.lambda_smooth = declare_parameter<double>(
      "optimization.narrow.lambda_smooth", 0.25);
    narrow_config.lambda_collision = declare_parameter<double>(
      "optimization.narrow.lambda_collision", 600.0);
    narrow_config.lambda_reference = declare_parameter<double>(
      "optimization.narrow.lambda_reference", 8.0);
    narrow_config.minimum_acceptable_clearance = declare_parameter<double>(
      "optimization.narrow.minimum_acceptable_clearance", 0.10);
    narrow_config.reference_deadband = declare_parameter<double>(
      "optimization.narrow.reference_deadband", 0.04);
    narrow_config.max_deviation = declare_parameter<double>(
      "optimization.narrow.max_deviation", 0.35);
    narrow_config.max_z_deviation = declare_parameter<double>(
      "optimization.narrow.max_z_deviation", 0.05);
    narrow_config.goal_max_deviation = declare_parameter<double>(
      "optimization.narrow.goal_max_deviation", 0.30);
    narrow_optimizer_ = std::make_unique<BsplinePathOptimizer>(narrow_config);

    // -------- 输出与输入话题 --------
    // 路径类话题采用 Reliable + Transient Local 且只保留最后一条，控制器/RViz 后启动
    // 也能立即获得当前路径，同时不会排队执行旧路径。
    const auto path_qos = rclcpp::QoS(1).reliable().transient_local();
    // lifted_global_path：地面路线抬高后的完整路线，仅供理解高度处理和 RViz 对照。
    lifted_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/local_planner/lifted_global_path", path_qos);
    // astar_path：原始离散 A* 结果；local_path/optimized_path：验证成功的 B 样条，
    // 其中 local_path 是控制器唯一实际跟踪的话题。
    astar_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/local_planner/astar_path", path_qos);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/local_planner/local_path", path_qos);
    optimized_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/local_planner/optimized_path", path_qos);
    corridor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/local_planner/search_corridor", path_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/local_planner/diagnostics", 10);

    // 地图和里程计只保留最新数据。路径订阅请求 Volatile，使它既能接收 PCTPlanner
    // 标准 /plan 的 Reliable+Volatile，也能接收拓扑路线节点提供的
    // Reliable+TransientLocal；若订阅端请求 TransientLocal，则无法匹配 /plan。
    map_sub_ = create_subscription<efficient_3d_local_planner_msgs::msg::VoxelGrid>(
      "/local_voxel_map/grid", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&CorridorAStarPlannerNode::mapCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lio_odom_hf", rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&CorridorAStarPlannerNode::odomCallback, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&CorridorAStarPlannerNode::pathCallback, this, std::placeholders::_1));

    // 所有算法对象和 ROS 通信对象建立后再启动工作线程。
    worker_ = std::thread(&CorridorAStarPlannerNode::workerLoop, this);
    RCLCPP_DEBUG(
      get_logger(), "guided 3-D A*: path=%s horizon=%.1fm corridor=[%.1fm, +/-%.1fm] "
      "soft_weight=%.1f control_bspline=%d clearance=%.3fm narrow=[%d %.3fm]",
      path_topic_.c_str(), horizon_, config.corridor_xy, config.corridor_z,
      config.soft_weight, optimization_config.enabled,
      optimization_config.clearance_distance, narrow_config.enabled,
      narrow_config.minimum_acceptable_clearance);
  }

  ~CorridorAStarPlannerNode() override
  {
    // 设置原子退出标志并唤醒条件变量；search() 的取消回调也会看到 stop_，从而尽快
    // 中止大规模搜索。join 后再析构 planner/publisher，防止线程访问悬空对象。
    stop_.store(true);
    condition_.notify_all();
    if (worker_.joinable()) {worker_.join();}
  }

private:
  // 一次规划使用的不可变输入快照。generation 标识任意输入变化；path_generation 只在
  // 活动全局路径变化时递增，用来区分“同一路线的新地图”与“完全不同的新路线”。
  struct Inputs
  {
    std::shared_ptr<const GridSnapshot> map;
    nav_msgs::msg::Path::ConstSharedPtr path;
    Eigen::Vector3d robot{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    bool has_odometry{false};
    std::uint64_t generation{0U};
    std::uint64_t path_generation{0U};
  };

  static std::shared_ptr<GridSnapshot> snapshotFromMessage(
    const efficient_3d_local_planner_msgs::msg::VoxelGrid & message)
  {
    // 线上的 VoxelGrid 用稀疏索引节省带宽；A* 高频随机查询需要 O(1)，因此回调中把
    // hard/soft 两层索引展开为等长稠密 byte 数组。越界索引被忽略，随后 valid() 复查结构尺寸。
    auto snapshot = std::make_shared<GridSnapshot>();
    snapshot->origin = Eigen::Vector3d(message.origin.x, message.origin.y, message.origin.z);
    snapshot->resolution = message.resolution;
    snapshot->dimensions = Eigen::Vector3i(
      static_cast<int>(message.size_x), static_cast<int>(message.size_y),
      static_cast<int>(message.size_z));
    snapshot->revision = message.revision;
    const std::size_t count = snapshot->cellCount();
    snapshot->hard.assign(count, 0U);
    snapshot->soft_cost.assign(count, 0U);
    // hard 是唯一绝对禁止层；soft_cost 的 1~254 表示从膨胀外缘到 hard 逐渐增大的代价。
    for (const auto index : message.hard_occupied_indices) {
      if (index < count) {snapshot->hard[index] = 1U;}
    }
    const std::size_t soft_count = std::min(
      message.soft_indices.size(), message.soft_cost_values.size());
    for (std::size_t i = 0; i < soft_count; ++i) {
      if (message.soft_indices[i] < count) {
        snapshot->soft_cost[message.soft_indices[i]] = message.soft_cost_values[i];
      }
    }
    return snapshot;
  }

  void mapCallback(
    const efficient_3d_local_planner_msgs::msg::VoxelGrid::ConstSharedPtr message)
  {
    // 回调开始时间既用于性能诊断，也作为旧路径首次失效的 steady_clock 时间基准。
    const auto callback_begin = std::chrono::steady_clock::now();
    auto snapshot = snapshotFromMessage(*message);
    if (!snapshot->valid()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "planning_result=failure stage=map_input reason=malformed_voxel_grid");
      return;
    }

    // 每次新地图到达都要复查当前已发布路径。先在锁内复制路径、guide、机器人位置和
    // path_id，随后在锁外做几何验证，避免长时间阻塞里程计/路径回调和规划线程。
    std::vector<Eigen::Vector3d> path_to_validate;
    std::vector<Eigen::Vector3d> guide_to_validate;
    Eigen::Vector3d robot_at_validation{Eigen::Vector3d::Zero()};
    std::uint64_t path_id_to_validate = 0U;
    bool path_valid_to_validate = false;
    std::chrono::steady_clock::time_point invalid_since;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_map_ = snapshot;
      path_to_validate = current_path_;
      guide_to_validate = current_guide_;
      robot_at_validation = robot_;
      path_id_to_validate = current_path_id_;
      path_valid_to_validate = current_path_valid_;
      invalid_since = path_invalid_since_;
      work_pending_ = true;
      // 地图变化会使正在进行的规划成为潜在旧结果，同时通知 worker_基于新地图重规划。
      const std::uint64_t generation = ++generation_;
      (void)generation;
    }

    std::size_t invalid_segment = 0U;
    std::string validation_failure;
    if (!path_to_validate.empty() && path_valid_to_validate &&
      !planner_->validatePath(
        *snapshot, path_to_validate, guide_to_validate,
        &invalid_segment, &validation_failure))
    {
      // 锁外验证期间可能已经发布了新路径。只有 path_id 仍等于被检查的路径，才能把
      // “失效”状态提交回共享数据；否则丢弃这次针对旧路径的验证结果。
      bool invalidation_committed = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_path_id_ == path_id_to_validate && current_path_valid_) {
          current_path_valid_ = false;
          path_invalid_since_ = callback_begin;
          invalidation_committed = true;
        }
      }
      if (!invalidation_committed) {
        std::uint64_t current_path_id = 0U;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          current_path_id = current_path_id_;
        }
        RCLCPP_DEBUG(
          get_logger(),
          "path_validation_discarded rev=%lu checked_path_id=%lu current_path_id=%lu",
          static_cast<unsigned long>(message->revision),
          static_cast<unsigned long>(path_id_to_validate),
          static_cast<unsigned long>(current_path_id));
        condition_.notify_one();
        return;
      }
      // 首次失效不立刻发布空路径，而是给同一帧触发的新规划一个短宽限时间；机器人
      // 在此期间仍可能沿旧路径行驶，所以日志明确标记 await_replacement。
      ++path_invalidations_;
      const auto & from = path_to_validate[std::min(
          invalid_segment, path_to_validate.size() - 1U)];
      const auto & to = path_to_validate[std::min(
          invalid_segment + 1U, path_to_validate.size() - 1U)];
      RCLCPP_WARN(
        get_logger(),
        "planning_result=failure stage=path_validation reason=%s "
        "rev=%lu path_id=%lu action=await_replacement grace_ms=%.0f "
        "segment=%zu/%zu cause=%s "
        "robot=[%.2f %.2f %.2f] segment_robot_distance=[%.2f %.2f] "
        "from=[%.2f %.2f %.2f] to=[%.2f %.2f %.2f]",
        validation_failure.c_str(), static_cast<unsigned long>(message->revision),
        static_cast<unsigned long>(path_id_to_validate), replacement_grace_period_ * 1000.0,
        invalid_segment,
        path_to_validate.size() - 1U,
        validation_failure.c_str(), robot_at_validation.x(), robot_at_validation.y(),
        robot_at_validation.z(), (robot_at_validation - from).norm(),
        (robot_at_validation - to).norm(),
        from.x(), from.y(), from.z(), to.x(), to.y(), to.z());
    } else if (!path_to_validate.empty() && !path_valid_to_validate) {
      // 路径在之前地图帧已经失效。如果直到宽限期结束仍未被安全新路径替换，则清空
      // 控制输出；再次用 path_id 和状态复核，防止误删刚发布的新路径。
      const double invalid_age = std::chrono::duration<double>(
        callback_begin - invalid_since).count();
      if (invalidationAction(invalid_age, replacement_grace_period_) ==
        InvalidationAction::Stop)
      {
        bool stop_committed = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (current_path_id_ == path_id_to_validate && !current_path_valid_ &&
            !current_path_.empty())
          {
            current_path_.clear();
            current_guide_.clear();
            stop_committed = true;
          }
        }
        if (stop_committed) {
          publishEmptyPath(
            message->header.stamp, "invalid_path_replacement_timeout", message->revision);
          RCLCPP_ERROR(
            get_logger(),
            "planning_result=failure stage=path_validation "
            "reason=invalid_path_replacement_timeout rev=%lu path_id=%lu invalid_age_ms=%.1f",
            static_cast<unsigned long>(message->revision),
            static_cast<unsigned long>(path_id_to_validate), invalid_age * 1000.0);
        }
      }
    }
    const double callback_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_begin).count();
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "map_update rev=%lu generation=%lu current_path_id=%lu path_points=%zu "
      "hard=%zu soft=%zu callback_ms=%.2f invalidations=%lu",
      static_cast<unsigned long>(message->revision),
      static_cast<unsigned long>(generation_.load()),
      static_cast<unsigned long>(path_id_to_validate), path_to_validate.size(),
      message->hard_occupied_indices.size(), message->soft_indices.size(), callback_ms,
      static_cast<unsigned long>(path_invalidations_.load()));
    // 不论旧路径是否仍有效，新地图都可能改善或恶化搜索结果，因此唤醒一次规划线程。
    condition_.notify_one();
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    // 里程计回调只保存最新位置与 yaw。先检查四元数，避免坏姿态把 NaN 带入航向状态。
    const auto & pose = message->pose.pose;
    Eigen::Quaterniond quaternion(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (quaternion.norm() < 1e-6 || !quaternion.coeffs().allFinite()) {return;}
    quaternion.normalize();
    // A* 离散运动只使用水平偏航角；roll/pitch 仍由三维位置和路径 Z 表达楼梯坡度。
    const Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

    // 位姿三个分量和 has_odometry_ 在同一临界区原子更新。
    std::lock_guard<std::mutex> lock(mutex_);
    robot_ = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    robot_yaw_ = euler.z();
    has_odometry_ = true;
  }

  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr message)
  {
    // 每收到一条新的活动全局路径都递增 path_generation_，即使路径内容看起来相似。
    // 这是防止第二次发送目标后继续保留上一目标局部轨迹的核心标识。
    const bool empty = message->poses.empty();
    std::uint64_t path_generation = 0U;
    std::uint64_t map_revision = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      global_path_ = message;
      ++generation_;
      path_generation = ++path_generation_;
      map_revision = latest_map_ ? latest_map_->revision : 0U;
      if (empty) {
        // 空全局路径是路线节点明确的停止命令：立即清除旧局部路径状态，且不安排规划。
        current_path_.clear();
        current_guide_.clear();
        current_path_valid_ = false;
        current_path_generation_ = 0U;
        work_pending_ = false;
      } else {
        // 旧局部路径属于不同全局段时先标为无效。若新段规划失败，后续策略禁止保留
        // 这条“去往旧目标”的轨迹，否则机器人会在目标切换后继续向错误方向行驶。
        if (!current_path_.empty() && current_path_generation_ != path_generation) {
          current_path_valid_ = false;
          path_invalid_since_ = std::chrono::steady_clock::now();
        }
        work_pending_ = true;
      }
    }
    const std::uint64_t receipt = ++global_path_receipts_;
    RCLCPP_DEBUG(
      get_logger(),
      "global_path_received receipt=%lu generation=%lu points=%zu action=%s",
      static_cast<unsigned long>(receipt), static_cast<unsigned long>(path_generation),
      message->poses.size(), empty ? "stop" : "replan");
    if (empty) {
      // 同步发布三条空路径，使控制器和 RViz 都立即删除上一结果。
      publishEmptyPath(message->header.stamp, "empty_global_path", map_revision);
      return;
    }
    condition_.notify_one();
  }

  Inputs captureInputs()
  {
    // 在一个短临界区内抓取 map/path/odom 的一致快照，同时消费 work_pending_ 标志。
    // worker 使用 shared_ptr 保持消息和地图对象生命周期，锁释放后仍可安全访问。
    std::lock_guard<std::mutex> lock(mutex_);
    Inputs inputs;
    inputs.map = latest_map_;
    inputs.path = global_path_;
    inputs.robot = robot_;
    inputs.yaw = robot_yaw_;
    inputs.has_odometry = has_odometry_;
    inputs.generation = generation_.load();
    inputs.path_generation = path_generation_.load();
    work_pending_ = false;
    return inputs;
  }

  std::vector<Eigen::Vector3d> transformAndLiftPath(
    const nav_msgs::msg::Path & message) const
  {
    // 输入是地面路径。先统一变换到 planning_frame_，再把 Z 加 path_height_ 转成机器人
    // 机身中心路径。TF 使用最新可用变换并设置 50 ms 超时，异常交由 worker 捕获。
    std::vector<Eigen::Vector3d> path;
    path.reserve(message.poses.size());
    const std::string source_frame = message.header.frame_id.empty() ? planning_frame_ :
      message.header.frame_id;
    geometry_msgs::msg::TransformStamped transform;
    const bool needs_transform = source_frame != planning_frame_;
    if (needs_transform) {
      transform = tf_buffer_.lookupTransform(
        planning_frame_, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
    }
    for (const auto & pose : message.poses) {
      geometry_msgs::msg::PoseStamped converted = pose;
      if (needs_transform) {
        tf2::doTransform(pose, converted, transform);
      }
      // 抬升发生在坐标变换之后，意味着 path_height_ 沿 planning_frame_ 的竖直 Z 添加。
      const Eigen::Vector3d point(
        converted.pose.position.x, converted.pose.position.y,
        converted.pose.position.z + path_height_);
      if (!point.allFinite()) {continue;}
      // 删除间距小于 2 cm 的连续重复点，避免后续弧长、切线和投影出现零长度线段。
      if (path.empty() || (point - path.back()).norm() > 0.02) {path.push_back(point);}
    }
    return path;
  }

  nav_msgs::msg::Path makePath(
    const std::vector<Eigen::Vector3d> & points,
    const builtin_interfaces::msg::Time & stamp) const
  {
    // 把内部 Eigen 点序列转换回 ROS Path。每个姿态的 yaw 由前后邻点的 XY 割线估计，
    // 首尾使用单侧差分；控制器主要使用点位置，但合理 orientation 便于其他工具查看。
    nav_msgs::msg::Path message;
    message.header.frame_id = planning_frame_;
    message.header.stamp = stamp;
    message.poses.resize(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      auto & pose = message.poses[i];
      pose.header = message.header;
      pose.pose.position.x = points[i].x();
      pose.pose.position.y = points[i].y();
      pose.pose.position.z = points[i].z();
      double yaw = 0.0;
      if (points.size() > 1U) {
        const std::size_t previous = i == 0U ? 0U : i - 1U;
        const std::size_t next = i + 1U < points.size() ? i + 1U : i;
        yaw = std::atan2(
          points[next].y() - points[previous].y(),
          points[next].x() - points[previous].x());
      }
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
    }
    return message;
  }

  std::vector<Eigen::Vector3d> simplifyPath(
    const GridSnapshot & map, const std::vector<Eigen::Vector3d> & path,
    const std::vector<Eigen::Vector3d> & guide) const
  {
    // 贪心视线简化：从 anchor 开始优先尝试能安全连接的最远候选点，以减少 B 样条
    // 控制点噪声。单段长度还受 max_simplified_segment_ 限制，避免跨越急弯。
    if (path.size() <= 2U) {return path;}
    const auto guide_distance = cumulativeDistance(guide);
    std::vector<Eigen::Vector3d> result;
    result.push_back(path.front());
    std::size_t anchor = 0U;
    while (anchor + 1U < path.size()) {
      std::size_t chosen = anchor + 1U;
      // 从终点向前搜索，第一个合法候选就是当前 anchor 可连接的最远点。
      for (std::size_t candidate = path.size() - 1U; candidate > anchor + 1U; --candidate) {
        if ((path[candidate] - path[anchor]).norm() > max_simplified_segment_) {continue;}
        const double yaw = std::atan2(
          path[candidate].y() - path[anchor].y(), path[candidate].x() - path[anchor].x());
        // 简化绝不能新引入穿越软安全层的捷径。relaxed A* 在窄路中允许相邻状态位于
        // 软层内，但这些状态会被原样保留，直到路径真正离开软层后才可继续长距离简化。
        // edgeValid(..., strict_soft=true) 同时检查硬层、软层、旋转扫掠和 guide 走廊。
        if (planner_->edgeValid(
            map, path[anchor], yaw, path[candidate], yaw,
            guide, guide_distance, true))
        {
          chosen = candidate;
          break;
        }
      }
      result.push_back(path[chosen]);
      anchor = chosen;
    }
    return result;
  }

  void publishCorridor(
    const std::vector<Eigen::Vector3d> & guide,
    const builtin_interfaces::msg::Time & stamp) const
  {
    // RViz 中半透明蓝色方块表示每个 guide 点允许的三维搜索盒，青色折线表示 guide。
    // 相邻方块重叠后形成搜索管道，可直观看出 corridor_xy/z 是否太窄或跨楼层。
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker cubes;
    cubes.header.frame_id = planning_frame_;
    cubes.header.stamp = stamp;
    cubes.ns = "guided_search_corridor";
    cubes.id = 0;
    cubes.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cubes.action = visualization_msgs::msg::Marker::ADD;
    cubes.pose.orientation.w = 1.0;
    cubes.scale.x = 2.0 * planner_->config().corridor_xy;
    cubes.scale.y = 2.0 * planner_->config().corridor_xy;
    cubes.scale.z = 2.0 * planner_->config().corridor_z;
    cubes.color.r = 0.1F; cubes.color.g = 0.55F; cubes.color.b = 1.0F; cubes.color.a = 0.025F;
    for (const auto & point : guide) {
      geometry_msgs::msg::Point message;
      message.x = point.x(); message.y = point.y(); message.z = point.z();
      cubes.points.push_back(message);
    }
    array.markers.push_back(cubes);

    // 复制公共 header/ns/points 后改成 LINE_STRIP，避免再次构造同一组 guide 点。
    visualization_msgs::msg::Marker line = cubes;
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.scale.x = 0.04;
    line.color.r = 0.15F; line.color.g = 0.85F; line.color.b = 1.0F; line.color.a = 0.9F;
    array.markers.push_back(line);
    corridor_pub_->publish(array);
  }

  void publishEmptyPath(
    const builtin_interfaces::msg::Time & stamp, const char * reason,
    const std::uint64_t revision)
  {
    // 空路径是控制器停车协议的一部分。astar/local/optimized 三个输出同时清空，防止
    // RViz 仍显示旧轨迹而控制器已经停车，或反过来控制器继续使用锁存旧路径。
    const nav_msgs::msg::Path empty = makePath({}, stamp);
    astar_path_pub_->publish(empty);
    local_path_pub_->publish(empty);
    optimized_path_pub_->publish(empty);
    const std::uint64_t publish_id = ++path_publish_count_;
    RCLCPP_DEBUG(
      get_logger(),
      "path_publish id=%lu type=empty rev=%lu reason=%s successes=%lu failures=%lu "
      "invalidations=%lu",
      static_cast<unsigned long>(publish_id), static_cast<unsigned long>(revision), reason,
      static_cast<unsigned long>(planning_successes_.load()),
      static_cast<unsigned long>(planning_failures_.load()),
      static_cast<unsigned long>(path_invalidations_.load()));
  }

  void publishDiagnostics(
    const GuidedAStar::Result & result, const std::uint64_t revision,
    const double elapsed_ms,
    const BsplinePathOptimizer::Result * optimization = nullptr,
    const double optimization_ms = 0.0,
    const std::string & optimization_mode = "normal",
    const std::string & primary_optimization_reason = "not_run")
  {
    // 第一条 DiagnosticStatus 描述 A*；若执行过 B 样条，则追加第二条状态描述优化器。
    // 诊断保留详细数值，而 INFO 日志只输出 success，避免正常高频规划刷屏。
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = result.success ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "efficient_3d_local_planner/guided_astar";
    status.hardware_id = "go2";
    status.message = result.reason;
    auto add = [&](const std::string & key, const auto value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key; pair.value = std::to_string(value); status.values.push_back(pair);
      };
    add("map_revision", revision);
    add("success", result.success);
    // used_soft_stage 表示严格阶段无路后使用了可穿越软层的放宽阶段；direct_shortcut_used
    // 表示前置连续直线检查命中，此时 expansions 应为 0。
    add("used_soft_stage", result.used_soft);
    add("used_reference_start_yaw", result.used_reference_start_yaw);
    add("direct_shortcut_used", result.direct_shortcut_used);
    add("expansions", result.expansions);
    // rejected_* 是搜索邻居被拒绝的分类计数，可区分地图边界、走廊太窄、对角穿墙、
    // 硬障碍和软层约束导致的失败。
    add("rejected_outside_map", result.rejected_outside_map);
    add("rejected_corridor_xy", result.rejected_corridor_xy);
    add("rejected_corridor_z", result.rejected_corridor_z);
    add("rejected_diagonal_corner", result.rejected_diagonal_corner);
    add("rejected_hard_collision", result.rejected_hard_collision);
    add("rejected_soft_layer", result.rejected_soft_layer);
    // 前/后采样点分别报告起点和目标是否落入沿 Z 膨胀后的 hard，定位“原地不动却无路”时
    // 是机器人前半身还是后半身已经贴入硬障碍。
    add("start_front_hard", result.start_front_hard);
    add("start_rear_hard", result.start_rear_hard);
    add("goal_front_hard", result.goal_front_hard);
    add("goal_rear_hard", result.goal_rear_hard);
    add("path_points", result.path.size());
    add("maximum_soft_cost", result.maximum_soft_cost);
    // closest_goal_distance、guide_progress 和 z 范围描述无解搜索实际走到了哪里。
    add("closest_goal_distance", result.closest_goal_distance);
    add("furthest_guide_arc", result.furthest_guide_arc);
    add("reached_z_min", result.reached_z_min);
    add("reached_z_max", result.reached_z_max);
    add("start_x", result.requested_start.x());
    add("start_y", result.requested_start.y());
    add("start_z", result.requested_start.z());
    add("selected_start_x", result.selected_start.x());
    add("selected_start_y", result.selected_start.y());
    add("selected_start_z", result.selected_start.z());
    add("start_adjustment", (result.selected_start - result.requested_start).norm());
    add("goal_x", result.selected_goal.x());
    add("goal_y", result.selected_goal.y());
    add("goal_z", result.selected_goal.z());
    add("path_invalidations_total", path_invalidations_.load());
    add("planning_ms", elapsed_ms);
    array.status.push_back(std::move(status));
    if (optimization) {
      // B 样条诊断同时记录总代价及各分项、净空、偏离参考路径程度和硬碰撞采样数，
      // 可判断失败源自碰撞、净空验收还是允许偏移不足。
      diagnostic_msgs::msg::DiagnosticStatus optimized_status;
      optimized_status.level = optimization->success ?
        diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::WARN;
      optimized_status.name = "efficient_3d_local_planner/control_bspline";
      optimized_status.hardware_id = "go2";
      optimized_status.message = optimization->reason;
      auto add_optimization = [&](const std::string & key, const auto value) {
          diagnostic_msgs::msg::KeyValue pair;
          pair.key = key;
          pair.value = std::to_string(value);
          optimized_status.values.push_back(pair);
        };
      add_optimization("success", optimization->success);
      add_optimization("iterations", optimization->iterations);
      add_optimization(
        "clearance_repair_iterations", optimization->clearance_repair_iterations);
      add_optimization("path_points", optimization->path.size());
      add_optimization("control_points", optimization->control_points.size());
      add_optimization("initial_cost", optimization->initial_cost);
      add_optimization("final_cost", optimization->final_cost);
      add_optimization("smooth_cost", optimization->smooth_cost);
      add_optimization("collision_cost", optimization->collision_cost);
      add_optimization("reference_cost", optimization->reference_cost);
      add_optimization("goal_cost", optimization->goal_cost);
      add_optimization("minimum_clearance", optimization->minimum_clearance);
      add_optimization(
        "minimum_movable_clearance", optimization->minimum_movable_clearance);
      add_optimization(
        "maximum_reference_deviation", optimization->maximum_reference_deviation);
      add_optimization("hard_collision_samples", optimization->hard_collision_samples);
      add_optimization("optimization_ms", optimization_ms);
      // mode=normal 表示常规优化；narrow 表示窄路重试成功；narrow_failed 表示两次均失败。
      diagnostic_msgs::msg::KeyValue mode;
      mode.key = "mode";
      mode.value = optimization_mode;
      optimized_status.values.push_back(std::move(mode));
      diagnostic_msgs::msg::KeyValue primary_reason;
      primary_reason.key = "primary_reason";
      primary_reason.value = primary_optimization_reason;
      optimized_status.values.push_back(std::move(primary_reason));
      array.status.push_back(std::move(optimized_status));
    }
    diagnostics_pub_->publish(array);
  }

  void workerLoop()
  {
    // 工作线程采用“合并触发”策略：若计算期间连续到达多帧地图，work_pending_ 只需为
    // true 一次；当前计算结束后直接抓取最新快照，不必逐帧补算已经过时的地图。
    while (!stop_.load()) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        // 谓词式等待可处理虚假唤醒，也能在节点析构时由 stop_ 立即退出。
        condition_.wait(lock, [&]() {return stop_.load() || work_pending_;});
      }
      if (stop_.load()) {break;}

      // captureInputs() 会清除 pending 标志。输入尚不齐全时安静等待下一次回调，不把
      // “节点刚启动、还没收到地图/路径/里程计”计为规划失败。
      const Inputs inputs = captureInputs();
      if (!inputs.map || !inputs.path || !inputs.has_odometry) {continue;}
      const std::uint64_t attempt_id = ++planning_attempts_;
      const auto begin = std::chrono::steady_clock::now();
      try {
        // 第一步：全局地面路径统一坐标系并抬升到机身中心。少于两个有效点无法定义
        // 局部 guide，保持等待而不发布伪路径。
        const auto lifted_path = transformAndLiftPath(*inputs.path);
        if (lifted_path.size() < 2U) {continue;}
        bool start_z_projected = false;
        // 楼梯上 odom Z 与离散路线高度通常存在厘米级差异，只修改规划起点 Z 能让其
        // 落回 guide，同时保留真实 XY。投影函数只有在修正量不超过门限时才返回成功。
        const Eigen::Vector3d planning_start = project_start_z_ ?
          projectStartZToLiftedPath(
          inputs.robot, lifted_path, start_z_max_correction_, &start_z_projected) :
          inputs.robot;
        if (project_start_z_ && !start_z_projected) {
          // 投影失败后再求最近三维 guide 点，用它的 Z 给出可读的 wrong_path_level
          // 诊断。典型情况是机器人已经下到下一层，而路线 sequencer 仍发布上一段。
          const auto lifted_distance = cumulativeDistance(lifted_path);
          const GuideProjection projection = projectToGuide(
            inputs.robot, lifted_path, lifted_distance);
          const double z_error = inputs.robot.z() - projection.point.z();
          if (reject_wrong_path_level_) {
            // 大高度差时禁止 A* 自行跨越楼层。否则 XY 重叠的楼梯可能从错误高度开始
            // 搜索，得到穿楼板或跳层的危险路径。
            GuidedAStar::Result wrong_level;
            wrong_level.reason = "wrong_path_level";
            wrong_level.requested_start = inputs.robot;
            wrong_level.selected_start = inputs.robot;
            wrong_level.selected_goal = lifted_path.back();
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - begin).count();
            const std::uint64_t failures = ++planning_failures_;
            RCLCPP_WARN(
              get_logger(), "planning_result=failure stage=input reason=wrong_path_level "
              "attempt=%lu rev=%lu "
              "odom_z=%.3f nearest_lifted_z=%.3f z_error=%.3f max_correction=%.3f "
              "action=retain_previous_path failures=%lu",
              static_cast<unsigned long>(attempt_id),
              static_cast<unsigned long>(inputs.map->revision), inputs.robot.z(),
              projection.point.z(), z_error, start_z_max_correction_,
              static_cast<unsigned long>(failures));
            // 若错误高度来自“新全局段”，旧局部路径不能保留，因为它属于上一目标；
            // 若仍是同一全局段的瞬时高度异常，则保留已验证旧路径，避免单帧急停。
            bool cleared_previous_route = false;
            {
              std::lock_guard<std::mutex> lock(mutex_);
              if (current_path_generation_ != inputs.path_generation) {
                current_path_.clear();
                current_guide_.clear();
                current_path_valid_ = false;
                cleared_previous_route = true;
              }
            }
            if (cleared_previous_route) {
              publishEmptyPath(
                inputs.path->header.stamp, "wrong_path_level_new_route",
                inputs.map->revision);
            }
            publishDiagnostics(wrong_level, inputs.map->revision, elapsed_ms);
            continue;
          }
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "start z projection rejected: odom_z=%.3f reference_z=%.3f "
            "z_error=%.3f max_correction=%.3f",
            inputs.robot.z(), projection.point.z(), z_error, start_z_max_correction_);
        } else if (start_z_projected) {
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "start z projected onto lifted path: odom_z=%.3f reference_z=%.3f correction=%.3f",
            inputs.robot.z(), planning_start.z(), planning_start.z() - inputs.robot.z());
        }
        // 第二步：从机器人在完整路线上的三维投影处开始，按弧长截取 horizon_ 并以
        // 不小于地图分辨率的间距重采样。使用三维投影可避免上下层 XY 重合时选错段。
        const auto guide = extractLocalGuide(
          lifted_path, planning_start, horizon_,
          std::max(guide_spacing_, inputs.map->resolution));
        if (guide.size() < 2U) {continue;}
        const builtin_interfaces::msg::Time stamp = inputs.path->header.stamp;
        // 先发布参考输入和搜索走廊，哪怕后续 A* 失败，RViz 仍能观察规划器收到的路线。
        lifted_path_pub_->publish(makePath(lifted_path, stamp));
        publishCorridor(guide, stamp);

        // 第三步：执行严格直连/Guided A*。取消回调只关注 path_generation：地图更新
        // 不强行中断本次搜索，而是在结果完成后用最新地图复查；全局路线切换则必须
        // 立即取消，避免浪费时间计算旧目标。
        const auto result = planner_->search(
          *inputs.map, planning_start, inputs.yaw, guide,
          [this, path_generation = inputs.path_generation]() {
            return stop_.load() || path_generation_.load() != path_generation;
          });
        if (result.cancelled) {
          // 被新目标取消是正常并发行为，不计入 planning_failures_。
          const std::uint64_t cancellations = ++planning_cancellations_;
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "plan_cancelled attempt=%lu input_rev=%lu input_generation=%lu "
            "current_generation=%lu expansions=%d cancellations=%lu",
            static_cast<unsigned long>(attempt_id),
            static_cast<unsigned long>(inputs.map->revision),
            static_cast<unsigned long>(inputs.generation),
            static_cast<unsigned long>(generation_.load()), result.expansions,
            static_cast<unsigned long>(cancellations));
          continue;
        }
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - begin).count();

        // 第四步：取得当前最新地图复查搜索结果。计算过程中地图可能已经更新多次。
        std::shared_ptr<const GridSnapshot> newest;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          newest = latest_map_;
        }
        const bool map_changed = newest && newest->revision != inputs.map->revision;
        // 成功结果必须在最新图上仍整条有效；失败结果若地图已经变化则不应报告，因为
        // 新地图可能已清除障碍，下一轮应基于新图重新判断。
        const bool valid_on_newest = newest && result.success &&
          planner_->validatePath(*newest, result.path, guide);
        if (!newest || (result.success && !valid_on_newest) ||
          (!result.success && map_changed))
        {
          const std::uint64_t stale = ++stale_results_;
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "plan_result_discarded attempt=%lu input_rev=%lu newest_rev=%lu "
            "success=%d stale_results=%lu",
            static_cast<unsigned long>(attempt_id),
            static_cast<unsigned long>(inputs.map->revision),
            static_cast<unsigned long>(newest ? newest->revision : 0U), result.success,
            static_cast<unsigned long>(stale));
          continue;
        }
        if (!result.success) {
          // A* 真正失败时输出完整原因。失败后是否保留旧路径由三个条件共同决定：
          // 旧路径非空、仍被最新地图验证有效、且属于同一个 path_generation。
          const std::uint64_t failures = ++planning_failures_;
          RCLCPP_WARN(
            get_logger(),
            "planning_result=failure stage=astar reason=%s "
            "attempt=%lu rev=%lu elapsed_ms=%.2f expansions=%d "
            "failures=%lu reference_start_yaw=%d closest_goal=%.2f "
            "guide_progress=%.2f z_reached=[%.2f,%.2f] "
            "reject[out=%d corridor_xy=%d corridor_z=%d diagonal=%d hard=%d soft=%d] "
            "start=[%.2f %.2f %.2f front_hard=%d rear_hard=%d] "
            "selected_start=[%.2f %.2f %.2f adjust=%.2f] "
            "goal=[%.2f %.2f %.2f front_hard=%d rear_hard=%d]",
            result.reason.c_str(), static_cast<unsigned long>(attempt_id),
            static_cast<unsigned long>(inputs.map->revision), elapsed_ms,
            result.expansions, static_cast<unsigned long>(failures),
            result.used_reference_start_yaw,
            result.closest_goal_distance, result.furthest_guide_arc,
            result.reached_z_min, result.reached_z_max,
            result.rejected_outside_map, result.rejected_corridor_xy,
            result.rejected_corridor_z, result.rejected_diagonal_corner,
            result.rejected_hard_collision, result.rejected_soft_layer,
            result.requested_start.x(), result.requested_start.y(),
            result.requested_start.z(), result.start_front_hard, result.start_rear_hard,
            result.selected_start.x(), result.selected_start.y(), result.selected_start.z(),
            (result.selected_start - result.requested_start).norm(),
            result.selected_goal.x(), result.selected_goal.y(), result.selected_goal.z(),
            result.goal_front_hard, result.goal_rear_hard);
          bool retain_current_path = false;
          std::uint64_t retained_path_id = 0U;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            retain_current_path = retainCurrentPathAfterPlanningFailure(
              !current_path_.empty(), current_path_valid_,
              current_path_generation_ == inputs.path_generation);
            if (!retain_current_path) {
              current_path_.clear();
              current_guide_.clear();
            } else {
              retained_path_id = current_path_id_;
            }
          }
          if (retain_current_path) {
            // 保留安全旧路径只是不重新发布；控制器继续使用 Transient Local 中最后路径。
            RCLCPP_DEBUG(
              get_logger(),
              "plan_failure_action attempt=%lu action=retain_safe_path current_path_id=%lu",
              static_cast<unsigned long>(attempt_id),
              static_cast<unsigned long>(retained_path_id));
          } else {
            publishEmptyPath(stamp, result.reason.c_str(), inputs.map->revision);
          }
          publishDiagnostics(result, inputs.map->revision, elapsed_ms);
          continue;
        }
        // 第五步：在最新地图上简化离散 A*，然后执行常规 B 样条优化。
        const auto simplified = simplifyPath(*newest, result.path, guide);
        const auto optimization_begin = std::chrono::steady_clock::now();
        BsplinePathOptimizer::Result optimized = optimizer_->optimize(*newest, simplified);
        const std::string primary_optimization_reason = optimized.reason;
        std::string optimization_mode = "normal";
        if (!optimized.success && narrow_optimizer_->config().enabled) {
          // 常规优化失败后，从未简化且已知无硬碰撞的 A* 状态重试。密集控制点和更紧的
          // 参考管道可防止三次曲线在窄拐角切角，或在楼梯处把高度变化过度抹平。
          optimized = narrow_optimizer_->optimize(*newest, result.path);
          optimization_mode = optimized.success ? "narrow" : "narrow_failed";
          optimized.reason = optimized.success ?
            "narrow_success" : "narrow_" + optimized.reason;
        }
        const double optimization_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - optimization_begin).count();
        // 第六步：优化失败同样不能无条件退回“原始 A* 控制”，因为当前系统规定实际
        // 控制只使用连续碰撞复查成功的 B 样条。策略只能是发布新优化路径、保留同路线
        // 的安全旧优化路径，或发布空路径停车。
        OptimizedControlAction control_action;
        std::uint64_t retained_path_id = 0U;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          control_action = optimizedControlAction(
            optimized.success, !current_path_.empty(), current_path_valid_,
            current_path_generation_ == inputs.path_generation);
          if (control_action == OptimizedControlAction::RetainCurrent) {
            retained_path_id = current_path_id_;
          } else if (control_action == OptimizedControlAction::Stop) {
            current_path_.clear();
            current_guide_.clear();
            current_path_valid_ = false;
          }
        }
        if (control_action != OptimizedControlAction::PublishOptimized) {
          const std::uint64_t failures = ++planning_failures_;
          if (control_action == OptimizedControlAction::RetainCurrent) {
            RCLCPP_WARN(
              get_logger(),
              "planning_result=failure stage=bspline reason=%s attempt=%lu "
              "action=retain_safe_optimized_path current_path_id=%lu failures=%lu",
              optimized.reason.c_str(), static_cast<unsigned long>(attempt_id),
              static_cast<unsigned long>(retained_path_id),
              static_cast<unsigned long>(failures));
          } else {
            const std::string reason = "bspline_" + optimized.reason;
            RCLCPP_WARN(
              get_logger(),
              "planning_result=failure stage=bspline reason=%s attempt=%lu "
              "action=stop failures=%lu",
              optimized.reason.c_str(), static_cast<unsigned long>(attempt_id),
              static_cast<unsigned long>(failures));
            publishEmptyPath(stamp, reason.c_str(), inputs.map->revision);
          }
          publishDiagnostics(
            result, inputs.map->revision, elapsed_ms, &optimized, optimization_ms,
            optimization_mode, primary_optimization_reason);
          continue;
        }
        // 第七步：先生成新的 path_id 并在锁内原子替换“当前安全控制路径”状态，再发布
        // ROS 消息。后续地图回调会携带这个 id 验证，避免提交针对旧路径的失效结果。
        const std::uint64_t path_id = ++path_publish_count_;
        const std::uint64_t successes = ++planning_successes_;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          current_path_ = optimized.path;
          current_guide_ = guide;
          current_path_id_ = path_id;
          current_path_generation_ = inputs.path_generation;
          current_path_valid_ = true;
        }
        // astar_path 用于对照；local_path 和 optimized_path 内容相同，前者供控制器，
        // 后者在 RViz 中明确表示最终优化结果。
        astar_path_pub_->publish(makePath(result.path, stamp));
        local_path_pub_->publish(makePath(optimized.path, stamp));
        optimized_path_pub_->publish(makePath(optimized.path, stamp));
        // 正常运行时成功结果只输出一个固定关键词，便于在大量地图日志中快速筛选。
        RCLCPP_INFO(get_logger(), "planning_result=success");
        // 成功路径的完整性能和净空数据仍保留在 DEBUG 级别及 diagnostics 话题中。
        RCLCPP_DEBUG(
          get_logger(),
          "planning_success_details path_id=%lu attempt=%lu rev=%lu raw_points=%zu "
          "simplified_points=%zu optimized_points=%zu elapsed_ms=%.2f "
          "optimization_ms=%.2f optimization=%s mode=%s primary=%s "
          "min_clearance=%.3f movable_clearance=%.3f "
          "hard_samples=%d control_path=bspline "
          "expansions=%d used_soft=%d reference_start_yaw=%d successes=%lu",
          static_cast<unsigned long>(path_id), static_cast<unsigned long>(attempt_id),
          static_cast<unsigned long>(inputs.map->revision), result.path.size(), simplified.size(),
          optimized.path.size(), elapsed_ms, optimization_ms, optimized.reason.c_str(),
          optimization_mode.c_str(), primary_optimization_reason.c_str(),
          optimized.minimum_clearance, optimized.minimum_movable_clearance,
          optimized.hard_collision_samples,
          result.expansions, result.used_soft, result.used_reference_start_yaw,
          static_cast<unsigned long>(successes));
        publishDiagnostics(
          result, inputs.map->revision, elapsed_ms, &optimized, optimization_ms,
          optimization_mode, primary_optimization_reason);
      } catch (const std::exception & error) {
        // TF 查询、消息格式或算法参数异常只影响本轮，节流记录后等待下一个输入；
        // 不让工作线程退出，否则节点仍存活但再也不会产生路径，排查会更加困难。
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "planning_result=failure stage=input reason=exception detail=%s", error.what());
      }
    }
  }

  // -------- 启动后不变的路径预处理和搜索参数 --------
  std::string planning_frame_;
  std::string path_topic_;
  double path_height_{0.40};
  bool project_start_z_{true};
  double start_z_max_correction_{0.60};
  bool reject_wrong_path_level_{true};
  double horizon_{5.0};
  double guide_spacing_{0.15};
  double replacement_grace_period_{0.25};
  double max_simplified_segment_{0.80};
  // planner_ 负责离散搜索；optimizer_ 是常规 B 样条；narrow_optimizer_ 是失败重试。
  std::unique_ptr<GuidedAStar> planner_;
  std::unique_ptr<BsplinePathOptimizer> optimizer_;
  std::unique_ptr<BsplinePathOptimizer> narrow_optimizer_;
  // 输入路径 frame 与 planning_frame 不同时使用 TF；里程计和体素地图预期已在规划系。
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // -------- 订阅回调和规划线程共享的最新输入/当前控制路径 --------
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  // shared_ptr 允许 worker 在锁外长期使用某一版只读地图，同时回调替换 latest_map_。
  std::shared_ptr<const GridSnapshot> latest_map_;
  nav_msgs::msg::Path::ConstSharedPtr global_path_;
  Eigen::Vector3d robot_{Eigen::Vector3d::Zero()};
  double robot_yaw_{0.0};
  bool has_odometry_{false};
  // work_pending_ 是合并式唤醒标志；current_path_ 始终保存最后一条实际控制 B 样条，
  // current_guide_ 用于新地图到来时验证它仍处于正确参考走廊。
  bool work_pending_{false};
  std::vector<Eigen::Vector3d> current_path_, current_guide_;
  // path_id 标识每次成功发布；path_generation 标识它属于哪一条活动全局路线。
  std::uint64_t current_path_id_{0U};
  std::uint64_t current_path_generation_{0U};
  bool current_path_valid_{false};
  // 旧路径第一次被新地图判无效的墙上时间，用于 replacement_grace_period_ 超时停车。
  std::chrono::steady_clock::time_point path_invalid_since_{};

  // -------- 跨线程原子版本号和运行统计 --------
  // generation_ 随地图或全局路径变化；path_generation_ 只随全局路径变化。搜索取消
  // 读取 path_generation_，诊断与日志读取其余计数而无需占用主互斥锁。
  std::atomic<std::uint64_t> generation_{0U};
  std::atomic<std::uint64_t> path_generation_{0U};
  std::atomic<std::uint64_t> path_invalidations_{0U};
  std::atomic<std::uint64_t> planning_attempts_{0U};
  std::atomic<std::uint64_t> planning_successes_{0U};
  std::atomic<std::uint64_t> planning_failures_{0U};
  std::atomic<std::uint64_t> planning_cancellations_{0U};
  std::atomic<std::uint64_t> stale_results_{0U};
  std::atomic<std::uint64_t> path_publish_count_{0U};
  std::atomic<std::uint64_t> global_path_receipts_{0U};
  // stop_ 同时控制 worker 主循环和 GuidedAStar 的取消回调。
  std::atomic<bool> stop_{false};
  std::thread worker_;

  // -------- ROS 通信对象 --------
  rclcpp::Subscription<efficient_3d_local_planner_msgs::msg::VoxelGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lifted_path_pub_, astar_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_, optimized_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace efficient_3d_local_planner

int main(int argc, char ** argv)
{
  // ROS executor 负责三个轻量订阅回调；耗时规划始终在节点内部 worker_ 线程中执行。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<efficient_3d_local_planner::CorridorAStarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
