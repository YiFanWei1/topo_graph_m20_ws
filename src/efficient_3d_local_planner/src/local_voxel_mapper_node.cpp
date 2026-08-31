#include "efficient_3d_local_planner/rolling_voxel_map.hpp"
#include "efficient_3d_local_planner/sensor_range_box.hpp"

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <efficient_3d_local_planner_msgs/msg/voxel_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace efficient_3d_local_planner
{

// 局部体素地图节点的数据流：
//   /cloud_registered_body + /lio_odom_hf
//       -> 按消息时间戳寻找最近里程计
//       -> 将机体系点云变换到 camera_init
//       -> 射线清空 + 端点占据更新 + 时间衰减
//       -> 构建原始 hard 与按水平距离分级的单一 soft 膨胀两层地图
//       -> 发布给 A* 规划器，并按需发布 RViz 点云。
//
// 性能设计的关键是“接收和计算解耦”：DDS 订阅回调只把消息放入内存队列，真正的
// 点云转换、raycast 和 buildLayers() 在 worker_ 工作线程中执行。这样一帧地图计算
// 即使耗时几十毫秒，也不会阻塞下一帧点云和 200 Hz 里程计的接收。
class LocalVoxelMapperNode : public rclcpp::Node
{
public:
  LocalVoxelMapperNode()
  : Node("local_voxel_mapper")
  {
    // -------- 滚动体素地图的几何范围 --------
    // size 是以机器人当前位置为中心滚动的局部窗口；resolution 决定体素边长。
    // 更小 resolution 能表达更窄的通道，但体素数和膨胀计算量会立方增长。
    RollingVoxelMap::Config config;
    config.resolution = declare_parameter<double>("map.resolution", 0.15);
    config.size = Eigen::Vector3d(
      declare_parameter<double>("map.size_x", 12.0),
      declare_parameter<double>("map.size_y", 12.0),
      declare_parameter<double>("map.size_z", 4.8));
    // 每个体素用 log-odds 累积占据概率。点云端点增加 log_hit，射线穿过区域增加
    // 负的 log_miss；数值限制在 [log_min, log_max] 防止无限累积。
    config.log_hit = declare_parameter<double>("map.log_hit", 0.90);
    config.log_miss = declare_parameter<double>("map.log_miss", -0.45);
    config.log_min = declare_parameter<double>("map.log_min", -2.0);
    config.log_max = declare_parameter<double>("map.log_max", 3.5);
    // 只有 log-odds 达到 occupied_threshold 且连续命中次数足够，才成为 hard_occupied，
    // 用于过滤孤立噪点。当前实机配置可将确认次数设为 1 以获得最快动态响应。
    config.occupied_threshold = declare_parameter<double>("map.occupied_threshold", 0.80);
    config.hit_confirmation_count = declare_parameter<int>("map.hit_confirmation_count", 2);
    // raycast 根据雷达原点到每个有效端点的射线清除自由空间；decay 则处理没有被
    // 新射线穿过的旧障碍。两种清除机制互补，不是二选一。
    config.raycast_enabled = declare_parameter<bool>("map.raycast_enabled", true);
    config.decay_enabled = declare_parameter<bool>("map.decay_enabled", true);
    config.decay_start = declare_parameter<double>("map.decay_start", 0.75);
    config.decay_rate = declare_parameter<double>("map.decay_rate", 1.20);
    // front_only=true 时只对机器人朝向扇区内的旧体素做被动时间衰减，避免雷达当前
    // 看不到的后方静态墙面过快消失；角度是完整扇区角而不是左右单侧角度。
    config.decay_front_only = declare_parameter<bool>("map.decay_front_only", true);
    config.decay_front_fov_deg = declare_parameter<double>("map.decay_front_fov_deg", 180.0);
    config.ray_step_factor = declare_parameter<double>("map.ray_step_factor", 0.90);
    // hard 外只构建一个按水平距离分级的 soft_cost 层。该半径覆盖旧版两段膨胀的
    // 总范围，soft 内部不再存在额外的硬分界。
    config.soft_inflation_radius = declare_parameter<double>(
      "map.soft_inflation_radius", 0.475);
    // hard 只允许沿 Z 膨胀，向上/向下距离分别开放，默认向下覆盖机身以下 0.40m。
    config.hard_inflation_z_down = declare_parameter<double>(
      "map.hard_inflation_z_down", 0.40);
    config.hard_inflation_z_up = declare_parameter<double>(
      "map.hard_inflation_z_up", 0.00);
    // 启动阶段一次性验证参数并预计算膨胀偏移模板；运行过程中不动态重建地图配置。
    validate(config);
    map_ = std::make_unique<RollingVoxelMap>(config);

    // -------- 输入话题、规划坐标系和雷达到机体的外参 --------
    // 本节点当前接收的是 /cloud_registered_body：点已经在机体系表达。外参平移只用
    // 来确定真实射线起点；不能再次施加到点云端点，否则整片地图会平移两次。
    planning_frame_ = declare_parameter<std::string>("frames.planning", "camera_init");
    const std::string cloud_topic = declare_parameter<std::string>(
      "input.cloud_topic", "/livox/lidar");
    const std::string odometry_topic = declare_parameter<std::string>(
      "input.odometry_topic", "/lio_odom_hf");
    // 变量名 base_from_lidar 表示 T_base_lidar：把雷达坐标系点变换到机体坐标系。
    base_from_lidar_translation_ = Eigen::Vector3d(
      declare_parameter<double>("extrinsic.base_to_lidar_x", 0.15),
      declare_parameter<double>("extrinsic.base_to_lidar_y", 0.0),
      declare_parameter<double>("extrinsic.base_to_lidar_z", 0.21));
    const Eigen::Vector3d extrinsic_rpy(
      declare_parameter<double>("extrinsic.base_to_lidar_roll", 0.0),
      declare_parameter<double>("extrinsic.base_to_lidar_pitch", 0.0),
      declare_parameter<double>("extrinsic.base_to_lidar_yaw", 0.0));
    // 欧拉角按 Z-Y-X 顺序组合，即先绕 X，再绕 Y，最后绕 Z，得到统一四元数。
    base_from_lidar_rotation_ =
      Eigen::AngleAxisd(extrinsic_rpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_rpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_rpy.x(), Eigen::Vector3d::UnitX());

    // -------- 雷达坐标系内外长方体量程 --------
    // 外框允许前后、左右、上下分别配置；把上边界单独压低可以在进入世界坐标变换前
    // 直接丢弃天花板点。内框以雷达为中心，用于排除近场盲区和雷达附近的反射噪声。
    range_box_.outer_min = Eigen::Vector3d(
      declare_parameter<double>("map.range_outer_min_x", -4.0),
      declare_parameter<double>("map.range_outer_min_y", -3.0),
      declare_parameter<double>("map.range_outer_min_z", -3.0));
    range_box_.outer_max = Eigen::Vector3d(
      declare_parameter<double>("map.range_outer_max_x", 4.0),
      declare_parameter<double>("map.range_outer_max_y", 3.0),
      declare_parameter<double>("map.range_outer_max_z", 2.0));
    range_box_.inner_half = Eigen::Vector3d(
      declare_parameter<double>("map.range_inner_half_x", 0.30),
      declare_parameter<double>("map.range_inner_half_y", 0.30),
      declare_parameter<double>("map.range_inner_half_z", 0.30));
    if (!range_box_.valid()) {
      throw std::invalid_argument(
              "invalid range boxes: inner box must be non-negative and inside outer box");
    }

    // -------- 自身机身点过滤 --------
    // 对机体系轴对齐包围盒内的点直接丢弃，避免腿、线缆或外壳被写成障碍；每帧还会
    // 主动清除此盒覆盖的旧占据，防止机身运动后留下“自体拖影”。
    body_exclusion_enabled_ = declare_parameter<bool>("filter.body_exclusion_enabled", true);
    body_exclusion_half_ = Eigen::Vector3d(
      declare_parameter<double>("filter.body_half_length", 0.55),
      declare_parameter<double>("filter.body_half_width", 0.10),
      declare_parameter<double>("filter.body_half_height", 0.35));
    // -------- 点云/里程计时间同步缓存 --------
    // 点云约 10 Hz，里程计约 200 Hz。每帧点云在里程计历史中寻找时间戳差最小者；
    // tolerance 是允许的最大误差，max_wait 是等待未来里程计到达的最长墙上时间。
    sync_tolerance_ = declare_parameter<double>("sync.tolerance", 0.035);
    const int sync_queue = declare_parameter<int>("sync.queue_size", 200);
    cloud_qos_depth_ = declare_parameter<int>("sync.cloud_qos_depth", 20);
    odometry_qos_depth_ = declare_parameter<int>("sync.odometry_qos_depth", 100);
    sync_max_wait_ = declare_parameter<double>("sync.max_wait", 0.20);
    // 当即将处理的点云后面已经排着更新帧，或者它在内部队列等待超过该时间时，
    // 认为本次正在使用旧帧并输出节流警告。该时间只统计本机入队后的等待，不包含
    // 上游算法在发布 /cloud_registered_body 之前已经产生的延迟。
    stale_cloud_warn_age_ = declare_parameter<double>("sync.stale_cloud_warn_age", 0.10);
    // 强制里程计 DDS 深度至少 100，保证突发到达或地图线程繁忙时仍保存约 0.5 秒
    // 的 200 Hz 位姿，能够匹配存在传输延迟的点云时间戳。
    if (sync_tolerance_ <= 0.0 || sync_max_wait_ <= 0.0 || stale_cloud_warn_age_ <= 0.0 ||
      cloud_qos_depth_ < 1 || odometry_qos_depth_ < 100 || sync_queue < 1)
    {
      throw std::invalid_argument(
              "sync QoS depths/tolerance/max_wait must be positive and odometry depth >= 100");
    }
    // 内部点云队列容量与 DDS 点云深度一致；里程计历史取 sync_queue 与 DDS 深度较大者。
    cloud_queue_capacity_ = static_cast<std::size_t>(cloud_qos_depth_);
    odometry_history_capacity_ = static_cast<std::size_t>(
      std::max(sync_queue, odometry_qos_depth_));

    // -------- 地图发布 --------
    // /grid 是规划器实际消费的紧凑索引消息，始终发布；两个 PointCloud2 主要用于 RViz，
    // processFrame() 仅在确实有订阅者时构造它们，避免无人观察时浪费序列化 CPU。
    grid_pub_ = create_publisher<efficient_3d_local_planner_msgs::msg::VoxelGrid>(
      "/local_voxel_map/grid", rclcpp::SensorDataQoS().keep_last(1));
    // hard_occupied：原始占据仅沿 Z 膨胀后的不可通行体素，不包含 XY footprint 膨胀。
    hard_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/local_voxel_map/hard_occupied", rclcpp::SensorDataQoS().keep_last(1));
    // soft_cost：从 hard 向外一次性膨胀，值按 XY 到 hard 的距离分级，越近代价越大。
    soft_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/local_voxel_map/soft_cost", rclcpp::SensorDataQoS().keep_last(1));
    bounds_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/local_voxel_map/bounds", rclcpp::QoS(1).transient_local());
    // min/max range 是以真实雷达原点为球心的三维欧氏距离限制。使用 MarkerArray
    // 同时发布内、外两个线框球，和滚动地图长方体一起显示二者实际相交的更新区域。
    range_bounds_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/local_voxel_map/range_bounds", rclcpp::QoS(1).transient_local());
    body_exclusion_bounds_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/local_voxel_map/body_exclusion_bounds", rclcpp::QoS(1).transient_local());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/local_voxel_map/diagnostics", 10);

    // 点云和里程计分别使用独立 SensorDataQoS 深度，不使用 message_filters 的同步回调，
    // 因为同步回调内执行地图计算会反过来阻塞 DDS 消息接收。
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS().keep_last(cloud_qos_depth_),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        cloudCallback(std::move(message));
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS().keep_last(odometry_qos_depth_),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        odometryCallback(std::move(message));
      });
    // 所有订阅和发布对象建立完成后再启动工作线程，避免线程看到未初始化成员。
    worker_ = std::thread(&LocalVoxelMapperNode::workerLoop, this);

    RCLCPP_DEBUG(
      get_logger(),
      "rolling voxel mapper: %.2fm resolution, %dx%dx%d, "
      "range outer=[%.1f %.1f; %.1f %.1f; %.1f %.1f]m, "
      "raycast=%d decay=%d inputs=[%s, %s] qos=[cloud:%d odom:%d] "
      "history=%zu tolerance=%.3fs max_wait=%.3fs",
      config.resolution, map_->dimensions().x(), map_->dimensions().y(),
      map_->dimensions().z(), range_box_.outer_min.x(), range_box_.outer_max.x(),
      range_box_.outer_min.y(), range_box_.outer_max.y(), range_box_.outer_min.z(),
      range_box_.outer_max.z(), config.raycast_enabled,
      config.decay_enabled, cloud_topic.c_str(), odometry_topic.c_str(),
      cloud_qos_depth_, odometry_qos_depth_, odometry_history_capacity_,
      sync_tolerance_, sync_max_wait_);
  }

  ~LocalVoxelMapperNode() override
  {
    // 析构时在锁内设置退出标志，再唤醒可能阻塞在 condition_variable 上的工作线程，
    // 最后 join，保证 map_ 和 ROS publisher 不会在线程仍使用时被销毁。
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      stop_worker_ = true;
    }
    input_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  // 点云除了 ROS 消息本体，还记录本机接收的单调时钟时间，用于 max_wait 超时判断。
  struct QueuedCloud
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    std::chrono::steady_clock::time_point received_at;
  };

  // 一帧地图流水线各阶段耗时。raycast_ms 是 map_update_ms 的子项，不应与其重复相加。
  struct StageTimings
  {
    double transform_ms{0.0};
    double map_update_ms{0.0};
    double raycast_ms{0.0};
    double build_layers_ms{0.0};
    double publish_ms{0.0};
    double total_ms{0.0};
    double synchronization_error_ms{0.0};
  };

  static void validate(const RollingVoxelMap::Config & config)
  {
    // 在节点启动时拒绝物理意义错误的配置，避免进入工作线程后才持续抛异常刷日志。
    if (config.resolution <= 0.0 || (config.size.array() <= 0.0).any() ||
      config.log_hit <= 0.0 || config.log_miss >= 0.0 || config.log_min >= config.log_max ||
      config.hit_confirmation_count < 1 ||
      config.decay_front_fov_deg <= 0.0 || config.decay_front_fov_deg > 180.0 ||
      config.soft_inflation_radius < 0.0 ||
      config.hard_inflation_z_down < 0.0 || config.hard_inflation_z_up < 0.0)
    {
      throw std::invalid_argument("invalid voxel-map parameters");
    }
  }

  static Eigen::Isometry3d poseTransform(const geometry_msgs::msg::Pose & pose)
  {
    // 将里程计 Pose 转成 T_world_base。四元数归一化后再转旋转矩阵，兼容小的数值漂移；
    // 全零、NaN 或近零四元数直接报错，本帧由 processFrame() 捕获并计为处理失败。
    Eigen::Quaterniond quaternion(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-6) {
      throw std::runtime_error("odometry contains invalid orientation");
    }
    quaternion.normalize();
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = quaternion.toRotationMatrix();
    transform.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
    return transform;
  }

  Eigen::Isometry3d baseFromLidar() const
  {
    // 根据启动参数构造固定外参 T_base_lidar。该函数本身不读取 TF，避免每帧 TF 查询开销。
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = base_from_lidar_rotation_.toRotationMatrix();
    transform.translation() = base_from_lidar_translation_;
    return transform;
  }

  std::vector<Eigen::Vector3d> transformCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const Eigen::Isometry3d & world_from_base) const
  {
    // 输入点按 PointCloud2 的 x/y/z 字段遍历；如果驱动缺少这些 FLOAT32 字段，迭代器
    // 会抛异常并由外层捕获。reserve 使用 width*height，减少向量动态扩容。
    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      // /cloud_registered_body 已经在机体系中表达。若这里再次乘 base_from_lidar，所有
      // 端点都会错误平移一个雷达外参；外参只在 processFrame() 中用于真实射线原点。
      const Eigen::Vector3d base_point(*x, *y, *z);
      if (!base_point.allFinite()) {continue;}
      // 包围盒判断在机体系完成，所以机器人转向不会改变过滤范围的几何意义。
      if (body_exclusion_enabled_ &&
        (base_point.array().abs() <= body_exclusion_half_.array()).all())
      {
        continue;
      }
      // 输入点在 base_link 表达，先通过固定外参逆变换到雷达坐标系。内外框过滤在
      // 世界坐标变换之前执行，被拒绝的天花板/近场点不再占用后续地图计算资源。
      const Eigen::Vector3d lidar_point =
        base_from_lidar_rotation_.conjugate() * (base_point - base_from_lidar_translation_);
      if (!range_box_.accepts(lidar_point)) {
        continue;
      }
      // 只对保留下来的点乘 T_world_base，得到 planning_frame_ 中的地图端点。
      points.push_back(world_from_base * base_point);
    }
    return points;
  }

  Eigen::Vector3d pointFromIndex(
    const std::uint32_t index, const RollingVoxelMap::Layers & layers) const
  {
    // VoxelGrid 使用一维线性索引节省带宽。这里按 x 最快、y 次之、z 最慢的排列规则
    // 反解索引，并加 0.5 个体素得到体素中心而不是最小角点。
    const int x = static_cast<int>(index % layers.dimensions.x());
    const int yz = static_cast<int>(index / layers.dimensions.x());
    const int y = yz % layers.dimensions.y();
    const int z = yz / layers.dimensions.y();
    const Eigen::Vector3d origin = map_->origin(layers);
    return origin + map_->config().resolution *
      (Eigen::Vector3d(x, y, z) + Eigen::Vector3d::Constant(0.5));
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<std::uint32_t> & indices, const RollingVoxelMap::Layers & layers,
    const builtin_interfaces::msg::Time & stamp) const
  {
    // 将硬层索引转换为只含 xyz 的紧凑点云。该消息仅用于可视化，不参与规划。
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = planning_frame_;
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(indices.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(indices.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto index : indices) {
      const Eigen::Vector3d point = pointFromIndex(index, layers);
      *x = static_cast<float>(point.x());
      *y = static_cast<float>(point.y());
      *z = static_cast<float>(point.z());
      ++x; ++y; ++z;
    }
    return cloud;
  }

  sensor_msgs::msg::PointCloud2 makeSoftCloud(
    const RollingVoxelMap::Layers & layers,
    const builtin_interfaces::msg::Time & stamp) const
  {
    // 软层额外携带 intensity=cost/254，RViz 可用强度着色显示离障碍的代价梯度。
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = planning_frame_;
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(layers.soft_indices.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(layers.soft_indices.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
    for (std::size_t i = 0; i < layers.soft_indices.size(); ++i) {
      const Eigen::Vector3d point = pointFromIndex(layers.soft_indices[i], layers);
      *x = static_cast<float>(point.x());
      *y = static_cast<float>(point.y());
      *z = static_cast<float>(point.z());
      *intensity = static_cast<float>(layers.soft_costs[i]) / 254.0F;
      ++x; ++y; ++z; ++intensity;
    }
    return cloud;
  }

  void publishGrid(
    const RollingVoxelMap::Layers & layers, const builtin_interfaces::msg::Time & stamp)
  {
    // VoxelGrid 的 origin 是滚动窗口最小角，indices 均相对于该 origin；revision 每帧
    // 单调递增，规划器用它识别新地图和丢弃基于旧地图算出的过时结果。
    efficient_3d_local_planner_msgs::msg::VoxelGrid message;
    message.header.frame_id = planning_frame_;
    message.header.stamp = stamp;
    const Eigen::Vector3d origin = map_->origin(layers);
    message.origin.x = origin.x();
    message.origin.y = origin.y();
    message.origin.z = origin.z();
    message.resolution = static_cast<float>(map_->config().resolution);
    message.size_x = static_cast<std::uint32_t>(layers.dimensions.x());
    message.size_y = static_cast<std::uint32_t>(layers.dimensions.y());
    message.size_z = static_cast<std::uint32_t>(layers.dimensions.z());
    message.revision = ++revision_;
    // 两层含义：hard 是传感器占据沿 Z 膨胀后的绝对禁止层；soft 是 hard 外按水平距离分级的
    // 单一膨胀代价层，允许 A* 在放宽阶段以附加代价穿越。
    message.hard_occupied_indices = layers.hard;
    message.soft_indices = layers.soft_indices;
    message.soft_cost_values = layers.soft_costs;
    grid_pub_->publish(message);
  }

  void publishBounds(
    const RollingVoxelMap::Layers & layers, const builtin_interfaces::msg::Time & stamp)
  {
    // 用 12 条边画出当前滚动体素窗口，方便判断“目标或障碍是否已经落在地图范围外”。
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = stamp;
    marker.ns = "rolling_voxel_map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const Eigen::Vector3d origin = map_->origin(layers);
    const Eigen::Vector3d size = map_->config().resolution * layers.dimensions.cast<double>();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = 0.0F;
    marker.color.g = 0.8F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;

    const Eigen::Vector3d maximum = origin + size;
    // 先列出长方体 8 个角，再按 12 对端点构造 LINE_LIST。
    const std::array<Eigen::Vector3d, 8> corners = {{
      {origin.x(), origin.y(), origin.z()},
      {maximum.x(), origin.y(), origin.z()},
      {maximum.x(), maximum.y(), origin.z()},
      {origin.x(), maximum.y(), origin.z()},
      {origin.x(), origin.y(), maximum.z()},
      {maximum.x(), origin.y(), maximum.z()},
      {maximum.x(), maximum.y(), maximum.z()},
      {origin.x(), maximum.y(), maximum.z()},
    }};
    constexpr std::array<std::array<int, 2>, 12> edges = {{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
      {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}},
    }};
    marker.points.reserve(edges.size() * 2U);
    for (const auto & edge : edges) {
      for (const int corner_index : edge) {
        geometry_msgs::msg::Point point;
        const Eigen::Vector3d & corner = corners[static_cast<std::size_t>(corner_index)];
        point.x = corner.x();
        point.y = corner.y();
        point.z = corner.z();
        marker.points.push_back(point);
      }
    }
    bounds_pub_->publish(marker);
  }

  void publishBodyExclusionBounds(
    const Eigen::Isometry3d & world_from_base,
    const builtin_interfaces::msg::Time & stamp)
  {
    // 红色半透明盒显示自身点过滤范围。盒子的位姿跟随机器人，尺寸来自 half_* 的两倍；
    // 关闭过滤时发布 DELETE，确保 RViz 不残留上一帧标记。
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = stamp;
    marker.ns = "body_exclusion";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = body_exclusion_enabled_ ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    marker.pose.position.x = world_from_base.translation().x();
    marker.pose.position.y = world_from_base.translation().y();
    marker.pose.position.z = world_from_base.translation().z();
    const Eigen::Quaterniond orientation(world_from_base.rotation());
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker.scale.x = 2.0 * body_exclusion_half_.x();
    marker.scale.y = 2.0 * body_exclusion_half_.y();
    marker.scale.z = 2.0 * body_exclusion_half_.z();
    marker.color.r = 1.0F;
    marker.color.g = 0.18F;
    marker.color.b = 0.05F;
    marker.color.a = 0.16F;
    body_exclusion_bounds_pub_->publish(marker);
  }

  visualization_msgs::msg::Marker makeRangeBoxMarker(
    const Eigen::Isometry3d & world_from_lidar, const Eigen::Vector3d & minimum,
    const Eigen::Vector3d & maximum,
    const builtin_interfaces::msg::Time & stamp, const std::string & marker_namespace,
    const int marker_id, const std::array<float, 3> & color) const
  {
    // 线框坐标直接在雷达局部坐标系中表达，再把 Marker 位姿设置为 T_world_lidar，
    // 因此外框会随机器人和雷达一起平移、旋转，和真正的点云过滤坐标系完全一致。
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = world_from_lidar.translation().x();
    marker.pose.position.y = world_from_lidar.translation().y();
    marker.pose.position.z = world_from_lidar.translation().z();
    const Eigen::Quaterniond orientation(world_from_lidar.rotation());
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker.scale.x = 0.035;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 0.85F;

    const std::array<Eigen::Vector3d, 8> corners = {{
      {minimum.x(), minimum.y(), minimum.z()},
      {maximum.x(), minimum.y(), minimum.z()},
      {maximum.x(), maximum.y(), minimum.z()},
      {minimum.x(), maximum.y(), minimum.z()},
      {minimum.x(), minimum.y(), maximum.z()},
      {maximum.x(), minimum.y(), maximum.z()},
      {maximum.x(), maximum.y(), maximum.z()},
      {minimum.x(), maximum.y(), maximum.z()},
    }};
    constexpr std::array<std::array<int, 2>, 12> edges = {{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
      {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}},
    }};
    marker.points.reserve(edges.size() * 2U);
    for (const auto & edge : edges) {
      for (const int corner_index : edge) {
        const Eigen::Vector3d & corner = corners[static_cast<std::size_t>(corner_index)];
        geometry_msgs::msg::Point point;
        point.x = corner.x();
        point.y = corner.y();
        point.z = corner.z();
        marker.points.push_back(point);
      }
    }
    return marker;
  }

  void publishRangeBounds(
    const Eigen::Isometry3d & world_from_lidar, const builtin_interfaces::msg::Time & stamp)
  {
    // 橙色内框内的点被过滤，绿色外框外的点被过滤。真正写入地图的点还必须位于
    // /local_voxel_map/bounds 滚动窗口内，因此有效范围是“内外框壳体与滚动窗口的交集”。
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(makeRangeBoxMarker(
      world_from_lidar, -range_box_.inner_half, range_box_.inner_half, stamp,
      "lidar_inner_range", 0, {1.0F, 0.25F, 0.05F}));
    markers.markers.push_back(makeRangeBoxMarker(
      world_from_lidar, range_box_.outer_min, range_box_.outer_max, stamp,
      "lidar_outer_range", 1, {0.25F, 1.0F, 0.20F}));
    range_bounds_pub_->publish(markers);
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    // 统一转为 64 位纳秒后做整数比较，避免 sec/nanosec 分开比较和浮点精度损失。
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    // 订阅回调只入队，不做任何点云遍历。队列满时丢最旧点云而不是最新点云，因为
    // 导航更需要当前环境；queue_dropped_frames_ 会在 diagnostics 中显式报告。
    ++input_cloud_frames_;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (cloud_queue_.size() >= cloud_queue_capacity_) {
        cloud_queue_.pop_front();
        ++queue_dropped_frames_;
      }
      cloud_queue_.push_back(QueuedCloud{std::move(cloud), std::chrono::steady_clock::now()});
      current_queue_depth_.store(cloud_queue_.size());
      // 无锁更新历史最大队列深度。compare_exchange_weak 失败时会把当前值回写到
      // previous_max，循环直到无需更新或成功写入。
      std::size_t previous_max = maximum_queue_depth_.load();
      while (cloud_queue_.size() > previous_max &&
        !maximum_queue_depth_.compare_exchange_weak(previous_max, cloud_queue_.size()))
      {
      }
    }
    input_condition_.notify_all();
  }

  void odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr odometry)
  {
    // 里程计同样只缓存 shared_ptr，不复制整条消息内容；超出历史容量时从最老数据开始删。
    ++input_odometry_frames_;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      odometry_history_.push_back(std::move(odometry));
      while (odometry_history_.size() > odometry_history_capacity_) {
        odometry_history_.pop_front();
      }
    }
    input_condition_.notify_all();
  }

  void workerLoop()
  {
    // 工作线程持有 unique_lock，以便在等待消息或执行重计算前显式 unlock/lock。
    std::unique_lock<std::mutex> lock(input_mutex_);
    while (!stop_worker_) {
      if (cloud_queue_.empty()) {
        // 谓词式 wait 可处理虚假唤醒，并能在析构设置 stop_worker_ 后立即退出。
        input_condition_.wait(lock, [this]() {
          return stop_worker_ || !cloud_queue_.empty();
        });
        continue;
      }

      // 严格按点云到达顺序处理队首，防止后到帧越过前帧造成地图时间倒退。
      const QueuedCloud & queued = cloud_queue_.front();
      const std::int64_t cloud_stamp = stampNanoseconds(queued.message->header.stamp);
      nav_msgs::msg::Odometry::ConstSharedPtr nearest_odometry;
      double nearest_delta_seconds = std::numeric_limits<double>::infinity();
      std::int64_t newest_odometry_stamp = std::numeric_limits<std::int64_t>::min();
      // 遍历有限长度里程计历史，选择绝对时间差最小的一帧；同时记录最新里程计时间，
      // 用于判断是否已经不可能再等到匹配当前点云的位姿。
      for (const auto & odometry : odometry_history_) {
        const std::int64_t odometry_stamp = stampNanoseconds(odometry->header.stamp);
        newest_odometry_stamp = std::max(newest_odometry_stamp, odometry_stamp);
        const double delta_seconds =
          std::abs(static_cast<double>(odometry_stamp - cloud_stamp)) * 1e-9;
        if (delta_seconds < nearest_delta_seconds) {
          nearest_delta_seconds = delta_seconds;
          nearest_odometry = odometry;
        }
      }

      if (nearest_odometry && nearest_delta_seconds <= sync_tolerance_) {
        // 此处才是真正决定“使用这帧更新地图”的位置。若队列深度大于 1，说明当前
        // front 后面已经存在更新点云，但 FIFO 策略仍会先处理当前旧帧；即使没有
        // 更新帧，等待时间超过阈值也说明这帧在同步或计算积压中已经变旧。
        const std::size_t queue_depth = cloud_queue_.size();
        const std::size_t newer_pending = queue_depth > 0U ? queue_depth - 1U : 0U;
        const double queue_wait_ms = 1000.0 * std::chrono::duration<double>(
          std::chrono::steady_clock::now() - queued.received_at).count();
        const bool has_newer_cloud = newer_pending > 0U;
        const bool waited_too_long = queue_wait_ms >= 1000.0 * stale_cloud_warn_age_;
        if (has_newer_cloud || waited_too_long) {
          ++stale_cloud_processed_frames_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "map_input=stale_cloud action=process_fifo reason=%s queue_depth=%zu "
            "newer_pending=%zu queue_wait_ms=%.1f cloud_stamp=%d.%09u",
            has_newer_cloud ? "newer_frame_pending" : "queue_wait_exceeded",
            queue_depth, newer_pending, queue_wait_ms,
            queued.message->header.stamp.sec, queued.message->header.stamp.nanosec);
        }
        // 匹配成功后先从队列移除点云，再释放锁执行重计算。释放锁期间两个订阅回调
        // 可继续缓存新消息，这是“接收与地图计算解耦”的核心。
        auto cloud = queued.message;
        cloud_queue_.pop_front();
        current_queue_depth_.store(cloud_queue_.size());
        ++synchronized_frames_;
        lock.unlock();
        processFrame(cloud, nearest_odometry, 1000.0 * nearest_delta_seconds);
        lock.lock();
        continue;
      }

      // 若最新里程计已经比点云晚 tolerance，之后的里程计只会更晚，当前点云无需再等；
      // 若时间戳异常或里程计停更，则由 max_wait 墙上时间兜底，避免队首永久阻塞。
      const bool odometry_has_passed_cloud =
        newest_odometry_stamp != std::numeric_limits<std::int64_t>::min() &&
        newest_odometry_stamp > cloud_stamp +
        static_cast<std::int64_t>(sync_tolerance_ * 1e9);
      const bool wait_expired =
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - queued.received_at).count() >= sync_max_wait_;
      if (odometry_has_passed_cloud || wait_expired) {
        cloud_queue_.pop_front();
        current_queue_depth_.store(cloud_queue_.size());
        ++synchronization_dropped_frames_;
        continue;
      }
      // 暂无结论时短暂等待新里程计。condition_variable 会被两个订阅回调主动唤醒，
      // 5 ms 只是防止通知丢失后的最长轮询间隔。
      input_condition_.wait_for(lock, std::chrono::milliseconds(5));
    }
  }

  void publishDiagnostics(
    const RollingVoxelMap::UpdateResult & update, const RollingVoxelMap::Layers & layers,
    const StageTimings & timing)
  {
    // 每个成功处理帧先累加耗时，但诊断消息最多每秒发布一次，避免诊断本身占用带宽。
    accumulated_timings_.transform_ms += timing.transform_ms;
    accumulated_timings_.map_update_ms += timing.map_update_ms;
    accumulated_timings_.raycast_ms += timing.raycast_ms;
    accumulated_timings_.build_layers_ms += timing.build_layers_ms;
    accumulated_timings_.publish_ms += timing.publish_ms;
    accumulated_timings_.total_ms += timing.total_ms;
    accumulated_timings_.synchronization_error_ms += timing.synchronization_error_ms;
    ++diagnostic_frames_;

    const auto steady_now = std::chrono::steady_clock::now();
    const double diagnostic_interval = std::chrono::duration<double>(
      steady_now - last_diagnostic_steady_time_).count();
    if (diagnostic_interval < 1.0) {
      return;
    }

    const std::uint64_t input_clouds = input_cloud_frames_.load();
    const std::uint64_t synchronized = synchronized_frames_.load();
    const std::uint64_t processed = processed_frames_.load();
    const std::uint64_t sync_dropped = synchronization_dropped_frames_.load();
    const std::uint64_t queue_dropped = queue_dropped_frames_.load();
    // 丢帧分为两类：找不到匹配里程计的同步丢帧，以及处理跟不上导致的队列溢出丢帧。
    const std::uint64_t dropped_since_last =
      (sync_dropped + queue_dropped) - last_diagnostic_dropped_frames_;
    const double divisor = static_cast<double>(std::max<std::uint64_t>(
        1U, diagnostic_frames_));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    // 最近一个统计窗口只要发生丢帧就标 WARN；累计计数仍保留，便于观察长期丢帧率。
    status.level = dropped_since_last == 0U ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "efficient_3d_local_planner/local_voxel_map";
    status.hardware_id = "livox";
    status.message = dropped_since_last == 0U ? "active" : "active_with_dropped_frames";
    auto add = [&](const std::string & key, const auto value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = std::to_string(value);
        status.values.push_back(pair);
      };
    add("revision", revision_);
    add("input_cloud_frames", input_clouds);
    add("input_odometry_frames", input_odometry_frames_.load());
    add("synchronized_frames", synchronized);
    add("processed_frames", processed);
    add("synchronization_dropped_frames", sync_dropped);
    add("queue_dropped_frames", queue_dropped);
    // 日志按 1 秒节流，但该计数不会节流；可用它确认实际处理过多少次旧帧。
    add("stale_cloud_processed_frames", stale_cloud_processed_frames_.load());
    add("total_dropped_frames", sync_dropped + queue_dropped);
    add("processing_failures", processing_failures_.load());
    add("pending_cloud_frames", current_queue_depth_.load());
    add("maximum_pending_cloud_frames", maximum_queue_depth_.load());
    // 三种频率可定位瓶颈：input 是 DDS 到达，synchronized 是匹配成功，map_update 是
    // 真正完成地图发布。input≈10 但 map_update<10 表示同步丢帧或计算积压。
    add("input_cloud_rate_hz",
      static_cast<double>(input_clouds - last_diagnostic_input_frames_) / diagnostic_interval);
    add("synchronized_rate_hz",
      static_cast<double>(synchronized - last_diagnostic_synchronized_frames_) /
      diagnostic_interval);
    add("map_update_rate_hz",
      static_cast<double>(processed - last_diagnostic_processed_frames_) / diagnostic_interval);
    add("synchronization_success_ratio",
      input_clouds == 0U ? 0.0 : static_cast<double>(synchronized) / input_clouds);
    add("input_points", update.input_points);
    add("unique_endpoints", update.unique_endpoints);
    add("touched_voxels", update.touched_voxels);
    add("stored_voxels", update.stored_voxels);
    add("hard_voxels", layers.hard.size());
    add("soft_voxels", layers.soft_indices.size());
    // last_* 观察最新一帧尖峰，average_* 观察节点启动以来所有成功帧的累计均值。
    add("last_sync_error_ms", timing.synchronization_error_ms);
    add("last_transform_ms", timing.transform_ms);
    add("last_map_update_ms", timing.map_update_ms);
    add("last_raycast_ms", timing.raycast_ms);
    add("last_build_layers_ms", timing.build_layers_ms);
    add("last_publish_ms", timing.publish_ms);
    add("last_total_ms", timing.total_ms);
    add("average_sync_error_ms", accumulated_timings_.synchronization_error_ms / divisor);
    add("average_transform_ms", accumulated_timings_.transform_ms / divisor);
    add("average_map_update_ms", accumulated_timings_.map_update_ms / divisor);
    add("average_raycast_ms", accumulated_timings_.raycast_ms / divisor);
    add("average_build_layers_ms", accumulated_timings_.build_layers_ms / divisor);
    add("average_publish_ms", accumulated_timings_.publish_ms / divisor);
    add("average_total_ms", accumulated_timings_.total_ms / divisor);
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);

    // 发布后只更新一秒频率/丢帧窗口的基准计数；accumulated_timings_ 和
    // diagnostic_frames_ 不清零，因此 average_* 始终是从节点启动到当前的累计均值。
    last_diagnostic_steady_time_ = steady_now;
    last_diagnostic_input_frames_ = input_clouds;
    last_diagnostic_synchronized_frames_ = synchronized;
    last_diagnostic_processed_frames_ = processed;
    last_diagnostic_dropped_frames_ = sync_dropped + queue_dropped;
  }

  void processFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const double synchronization_error_ms)
  {
    // processFrame 只由 worker_ 调用，因此 RollingVoxelMap 不需要额外互斥锁。
    const auto begin = std::chrono::steady_clock::now();
    try {
      StageTimings timing;
      timing.synchronization_error_ms = synchronization_error_ms;
      // T_world_lidar = T_world_base * T_base_lidar。点云端点使用 T_world_base，因为输入
      // 已在机体系；射线原点使用 T_world_lidar.translation()，对应真实发射位置。
      const Eigen::Isometry3d world_from_base = poseTransform(odometry->pose.pose);
      const Eigen::Isometry3d world_from_lidar = world_from_base * baseFromLidar();
      const auto points = transformCloud(*cloud, world_from_base);
      const auto transform_end = std::chrono::steady_clock::now();
      timing.transform_ms =
        std::chrono::duration<double, std::milli>(transform_end - begin).count();

      // update() 内部完成滚动窗口移动、距离过滤、端点命中、raycast 空闲更新和时间衰减。
      const auto update = map_->update(
        world_from_lidar.translation(), world_from_base.translation(), points,
        rclcpp::Time(cloud->header.stamp).seconds(),
        Eigen::Quaterniond(world_from_base.rotation()));
      if (body_exclusion_enabled_) {
        // 即使本帧输入已过滤机身点，历史中仍可能有机器人移动前留下的占据；按当前
        // 世界位姿再清一次机身盒，保证机器人不会被自己的地图困住。
        map_->clearBodyExclusion(
          world_from_base.translation(), Eigen::Quaterniond(world_from_base.rotation()),
          body_exclusion_half_);
      }
      const auto update_end = std::chrono::steady_clock::now();
      timing.map_update_ms =
        std::chrono::duration<double, std::milli>(update_end - transform_end).count();
      timing.raycast_ms = update.raycast_ms;

      // 从当前占据表构建 hard + soft 两层稠密索引。膨胀偏移已在地图构造时预计算，避免每帧重复
      // hypot 和代价函数计算。
      const auto layers = map_->buildLayers();
      const auto layers_end = std::chrono::steady_clock::now();
      timing.build_layers_ms =
        std::chrono::duration<double, std::milli>(layers_end - update_end).count();

      // 规划器必需的 /grid 始终发布；大点云只在 RViz/订阅者存在时构造并发布。
      publishGrid(layers, cloud->header.stamp);
      if (hard_pub_->get_subscription_count() > 0U) {
        hard_pub_->publish(makeCloud(layers.hard, layers, cloud->header.stamp));
      }
      if (soft_pub_->get_subscription_count() > 0U) {
        soft_pub_->publish(makeSoftCloud(layers, cloud->header.stamp));
      }
      publishBounds(layers, cloud->header.stamp);
      publishRangeBounds(world_from_lidar, cloud->header.stamp);
      publishBodyExclusionBounds(world_from_base, cloud->header.stamp);
      const auto publish_end = std::chrono::steady_clock::now();
      timing.publish_ms =
        std::chrono::duration<double, std::milli>(publish_end - layers_end).count();
      timing.total_ms =
        std::chrono::duration<double, std::milli>(publish_end - begin).count();
      ++processed_frames_;
      publishDiagnostics(update, layers, timing);
    } catch (const std::exception & error) {
      // 单帧格式或姿态异常不能杀死地图节点。计数并节流记录后继续处理下一帧。
      ++processing_failures_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "planning_result=failure stage=map reason=map_update_exception detail=%s",
        error.what());
    }
  }

  // -------- 地图与固定几何配置（仅工作线程读写 map_） --------
  std::unique_ptr<RollingVoxelMap> map_;
  std::string planning_frame_;
  Eigen::Vector3d base_from_lidar_translation_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond base_from_lidar_rotation_{Eigen::Quaterniond::Identity()};
  SensorRangeBox range_box_;
  bool body_exclusion_enabled_{true};
  Eigen::Vector3d body_exclusion_half_{0.55, 0.10, 0.35};
  // -------- 时间同步和队列容量配置 --------
  double sync_tolerance_{0.035};
  double sync_max_wait_{0.20};
  double stale_cloud_warn_age_{0.10};
  int cloud_qos_depth_{20};
  int odometry_qos_depth_{100};
  std::size_t cloud_queue_capacity_{20U};
  std::size_t odometry_history_capacity_{100U};
  // revision_ 是地图消息版本；其余非原子量只在工作线程中访问，用于一秒诊断统计。
  std::uint64_t revision_{0U};
  std::uint64_t diagnostic_frames_{0U};
  StageTimings accumulated_timings_;
  std::chrono::steady_clock::time_point last_diagnostic_steady_time_{
    std::chrono::steady_clock::now()};
  std::uint64_t last_diagnostic_input_frames_{0U};
  std::uint64_t last_diagnostic_synchronized_frames_{0U};
  std::uint64_t last_diagnostic_processed_frames_{0U};
  std::uint64_t last_diagnostic_dropped_frames_{0U};

  // -------- 订阅回调与工作线程共享的输入缓存 --------
  // mutex_ 保护两个 deque 和 stop_worker_；重计算期间工作线程会释放此锁。
  std::mutex input_mutex_;
  std::condition_variable input_condition_;
  std::deque<QueuedCloud> cloud_queue_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_history_;
  bool stop_worker_{false};
  std::thread worker_;
  // 计数器跨订阅线程、工作线程和诊断发布读取，使用 atomic 避免为统计额外抢锁。
  std::atomic<std::uint64_t> input_cloud_frames_{0U};
  std::atomic<std::uint64_t> input_odometry_frames_{0U};
  std::atomic<std::uint64_t> synchronized_frames_{0U};
  std::atomic<std::uint64_t> synchronization_dropped_frames_{0U};
  std::atomic<std::uint64_t> queue_dropped_frames_{0U};
  std::atomic<std::uint64_t> stale_cloud_processed_frames_{0U};
  std::atomic<std::uint64_t> processed_frames_{0U};
  std::atomic<std::uint64_t> processing_failures_{0U};
  std::atomic<std::size_t> current_queue_depth_{0U};
  std::atomic<std::size_t> maximum_queue_depth_{0U};

  // -------- ROS 通信对象 --------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<efficient_3d_local_planner_msgs::msg::VoxelGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hard_pub_, soft_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bounds_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr range_bounds_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr body_exclusion_bounds_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace efficient_3d_local_planner

int main(int argc, char ** argv)
{
  // 普通 spin 只负责轻量订阅回调；地图重计算始终由节点内部 worker_ 独立执行。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<efficient_3d_local_planner::LocalVoxelMapperNode>());
  rclcpp::shutdown();
  return 0;
}
