#include "efficient_3d_local_planner/path_follower.hpp"

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace efficient_3d_local_planner
{

// 局部路径跟踪节点的数据流：
//   /local_planner/local_path（规划坐标系中的三维 B 样条路径）
//                         + /lio_odom_hf（机器人三维位置和姿态）
//                         -> 三维最近点投影与前视点选择
//                         -> 期望机体系速度
//                         -> 加速度限幅与禁止倒退安全约束
//                         -> /cmd_vel_smoothed。
//
// 这里不重新规划路径，也不直接查询障碍物地图。碰撞安全由上游 A* 与 B 样条负责；
// 本节点负责“沿已经验证过的路径稳定行驶”，并在路径/里程计过期、坐标系不一致或
// 偏航误差过大时及时停止。订阅回调和定时控制回调可能由不同执行线程调用，所以
// 共享的路径、位姿和时间戳统一由 mutex_ 保护。
class LocalPathFollowerNode : public rclcpp::Node
{
public:
  LocalPathFollowerNode() : Node("local_path_follower")
  {
    // -------- 控制器开关与循环频率 --------
    // control_rate_ 至少为 1 Hz，避免错误参数造成除零或无限长定时周期。
    enabled_ = declare_parameter<bool>("controller.enabled", true);
    control_rate_ = std::max(1.0, declare_parameter<double>("controller.control_rate", 50.0));

    // -------- 路径跟踪器本体参数 --------
    // 这些参数交给 path_follower.hpp 中的纯函数 computePathFollowerCommand() 使用。
    // nominal_speed 是期望巡航速度；max_vx/max_vy/max_vyaw 是最终速度硬上限。
    config_.nominal_speed = declare_parameter<double>("controller.nominal_speed", 0.45);
    config_.max_vx = declare_parameter<double>("controller.max_vx", 0.80);
    config_.max_vy = declare_parameter<double>("controller.max_vy", 0.50);
    config_.max_vyaw = declare_parameter<double>("controller.max_vyaw", 0.30);
    config_.position_gain = declare_parameter<double>("controller.position_gain", 0.80);
    config_.yaw_gain = declare_parameter<double>("controller.yaw_gain", 1.30);

    // 动态前视距离 = 基础前视 + 速度增益项，并限制在 [lookahead_min, lookahead_max]。
    // 前视越远越平滑，但在急弯处越容易切弯；前视越近则跟踪更紧但转向更敏感。
    config_.lookahead_base = declare_parameter<double>("controller.lookahead_base", 0.15);
    config_.lookahead_speed_gain = declare_parameter<double>(
      "controller.lookahead_speed_gain", 0.40);
    config_.lookahead_min = declare_parameter<double>("controller.lookahead_min", 0.15);
    config_.lookahead_max = declare_parameter<double>("controller.lookahead_max", 0.35);
    config_.tangent_window = declare_parameter<double>("controller.tangent_window", 0.30);

    // finish_distance 用三维终点距离判断是否完成当前局部路径；braking_deceleration
    // 用剩余路径长度计算终点制动速度，防止到局部目标处突然刹停。
    config_.finish_distance = declare_parameter<double>("controller.finish_distance", 0.15);
    config_.braking_deceleration = declare_parameter<double>(
      "controller.braking_deceleration", 0.60);
    config_.translation_yaw_limit = declare_parameter<double>(
      "controller.translation_yaw_limit", 1.0471975511965976);
    config_.full_speed_yaw_limit = declare_parameter<double>(
      "controller.full_speed_yaw_limit", 0.17453292519943295);

    // yaw_deadband 内不输出角速度，用来抑制机器人已经基本对准时的左右抖动。
    config_.yaw_deadband = declare_parameter<double>("controller.yaw_deadband", 0.04);

    // 加速度限制作用在相邻控制周期之间；超时参数则使用单调时钟判断数据是否新鲜，
    // 因而不受 rosbag /clock 跳变或 use_sim_time 切换影响。
    max_linear_acceleration_ = declare_parameter<double>(
      "controller.max_linear_acceleration", 0.60);
    max_yaw_acceleration_ = declare_parameter<double>(
      "controller.max_yaw_acceleration", 1.00);
    // 默认关闭，确保升级后行为与当前实机完全一致。开启时将最终非零角速度量化到
    // [minimum_yaw_speed_, max_vyaw] 或对应负区间，用于跨过底盘不响应的小角速度区。
    minimum_yaw_speed_enabled_ = declare_parameter<bool>(
      "controller.minimum_yaw_speed_enabled", false);
    minimum_yaw_speed_ = std::clamp(
      declare_parameter<double>("controller.minimum_yaw_speed", 0.25),
      0.0, std::max(0.0, config_.max_vyaw));
    odometry_timeout_ = declare_parameter<double>("controller.odometry_timeout", 0.30);
    path_timeout_ = declare_parameter<double>("controller.path_timeout", 0.50);

    // 局部路径采用 Reliable + Transient Local：新启动的控制器也能立即取得规划器
    // 最后锁存的一条路径。队列只保留 1 条，避免执行过时路径。
    const auto path_qos = rclcpp::QoS(1).reliable().transient_local();
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/local_planner/local_path", path_qos,
      std::bind(&LocalPathFollowerNode::pathCallback, this, std::placeholders::_1));
    // 高频里程计使用 SensorDataQoS，只保留最新 5 帧；控制只关心最新位姿，积压旧帧
    // 反而会增加相位延迟。速度命令使用 Reliable，降低控制命令在 DDS 层丢失的概率。
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lio_odom_hf", rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&LocalPathFollowerNode::odometryCallback, this, std::placeholders::_1));
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_smoothed", rclcpp::QoS(10).reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/local_planner/controller_lookahead", path_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/local_planner/controller_diagnostics", 10);

    // 控制循环使用墙上时间定时器。即使仿真时间暂停，也会持续执行超时检查并发布停车。
    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocalPathFollowerNode::controlLoop, this));
    last_control_steady_ = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(
      get_logger(),
      "local path follower ready: enabled=%d rate=%.1fHz speed=%.2fm/s "
      "limits=[%.2f %.2f %.2f] output=/cmd_vel_smoothed",
      enabled_, control_rate_, config_.nominal_speed,
      config_.max_vx, config_.max_vy, config_.max_vyaw);
  }

  ~LocalPathFollowerNode() override
  {
    // 节点退出前主动发送一次全零速度，避免底盘继续执行最后一帧非零命令。
    publishStop();
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  static double ageSeconds(const SteadyTime & time)
  {
    // 消息的新鲜度必须由 steady_clock 计算：系统时间校时、bag 回放跳时都不应让
    // 已经过期的路径重新变成“新路径”。
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - time).count();
  }

  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr message)
  {
    // receipt_time 是本机真正收到消息的时间，不使用消息 header.stamp 判断通信超时。
    const auto receipt_time = std::chrono::steady_clock::now();
    const double arrival_gap = have_path_message_ ?
      std::chrono::duration<double>(receipt_time - last_path_steady_).count() :
      std::numeric_limits<double>::infinity();
    // 将 ROS Pose 转成内部 Eigen 三维点，同时删除 NaN/Inf 和连续重复点。重复点会使
    // 路径切线长度为零，导致终点附近的目标航向不稳定。
    std::vector<Eigen::Vector3d> path;
    path.reserve(message->poses.size());
    for (const auto & pose : message->poses) {
      const Eigen::Vector3d point(
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      if (!point.allFinite()) {continue;}
      if (path.empty() || (point - path.back()).norm() > 1e-3) {path.push_back(point);}
    }
    {
      // 先在锁外完成解析，再以一次短临界区原子替换整条路径，避免阻塞控制循环。
      std::lock_guard<std::mutex> lock(mutex_);
      path_ = std::move(path);
      path_frame_ = message->header.frame_id;
      last_path_steady_ = receipt_time;
      have_path_message_ = true;
    }
    const std::uint64_t receipt_id = ++path_receipts_;
    if (message->poses.empty()) {
      // 空 Path 是上游明确的停车信号。立即清零而不是等下一个定时控制周期，可以
      // 缩短规划失败或路线切换时的停车延迟。
      ++empty_path_receipts_;
      RCLCPP_DEBUG(
        get_logger(),
        "path_received id=%lu type=empty gap_ms=%.1f stamp=%d.%09u "
        "previous_cmd=[%.3f %.3f %.3f] empty_total=%lu",
        static_cast<unsigned long>(receipt_id), arrival_gap * 1000.0,
        message->header.stamp.sec, message->header.stamp.nanosec,
        previous_command_.linear.x, previous_command_.linear.y,
        previous_command_.angular.z,
        static_cast<unsigned long>(empty_path_receipts_.load()));
      publishStop();
      reportState("empty_path");
    } else {
      // 注意：消息非空但清洗后少于两个有效点的情况，会在 controlLoop() 中按
      // empty_path 停车；这里仍按“收到非空消息”计数，便于区分上游发送行为。
      ++valid_path_receipts_;
      const auto & first = message->poses.front().pose.position;
      const auto & last = message->poses.back().pose.position;
      // 有效路径会随局部规划高频更新，正常日志级别不逐帧输出，避免淹没失败原因。
      RCLCPP_DEBUG(
        get_logger(),
        "path_received id=%lu type=valid points=%zu gap_ms=%.1f stamp=%d.%09u "
        "first=[%.2f %.2f %.2f] last=[%.2f %.2f %.2f] valid_total=%lu",
        static_cast<unsigned long>(receipt_id), message->poses.size(), arrival_gap * 1000.0,
        message->header.stamp.sec, message->header.stamp.nanosec,
        first.x, first.y, first.z, last.x, last.y, last.z,
        static_cast<unsigned long>(valid_path_receipts_.load()));
    }
  }

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    // tf2::getYaw 假定输入是有效四元数。先检查模长，防止全零四元数或 NaN 传播到
    // yaw_error，最终产生不可预测的角速度命令。
    const auto & orientation = message->pose.pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-6) {return;}
    const double yaw = tf2::getYaw(orientation);
    if (!std::isfinite(yaw)) {return;}
    // 位姿、坐标系和接收时间戳作为同一份状态一起更新，控制线程不会读到半新半旧值。
    std::lock_guard<std::mutex> lock(mutex_);
    robot_ = Eigen::Vector3d(
      message->pose.pose.position.x, message->pose.pose.position.y,
      message->pose.pose.position.z);
    robot_yaw_ = yaw;
    odometry_frame_ = message->header.frame_id;
    last_odometry_steady_ = std::chrono::steady_clock::now();
    have_odometry_ = true;
  }

  geometry_msgs::msg::Twist limitAcceleration(
    const PathFollowerCommand & desired, const double dt)
  {
    // 所有速度都是机器人机体系速度：x 为前进、y 为横移、z 轴角速度为偏航。
    // previous/target 的 x 先钳到非负，确保历史负速度也不能通过加速度限幅残留。
    geometry_msgs::msg::Twist command;
    const Eigen::Vector2d previous(
      std::max(0.0, previous_command_.linear.x), previous_command_.linear.y);
    const Eigen::Vector2d target(std::max(0.0, desired.vx), desired.vy);
    // 对 XY 合速度矢量统一限幅，而不是分别限制 x/y；这样斜向运动时的总加速度
    // 仍不会超过 max_linear_acceleration_。
    Eigen::Vector2d delta = target - previous;
    const double maximum_step = std::max(0.0, max_linear_acceleration_) * dt;
    if (delta.norm() > maximum_step && delta.norm() > 1e-9) {
      delta *= maximum_step / delta.norm();
    }
    const Eigen::Vector2d limited = previous + delta;
    command.linear.x = std::clamp(limited.x(), 0.0, config_.max_vx);
    command.linear.y = std::clamp(limited.y(), -config_.max_vy, config_.max_vy);
    // 角速度独立做一维加速度限制，随后再施加绝对角速度上限。
    const double yaw_step = std::max(0.0, max_yaw_acceleration_) * dt;
    command.angular.z = previous_command_.angular.z + std::clamp(
      desired.wz - previous_command_.angular.z, -yaw_step, yaw_step);
    command.angular.z = std::clamp(
      command.angular.z, -config_.max_vyaw, config_.max_vyaw);
    // 可选的底盘角速度死区补偿在加速度限幅之后执行，保证最终发布值不会落入
    // (-minimum_yaw_speed_, +minimum_yaw_speed_)。期望角速度为零时仍输出严格的零；
    // 正反换向时由辅助函数先过零，避免被最小正角速度锁死。
    command.angular.z = applyMinimumYawSpeedDeadzone(
      command.angular.z, desired.wz,
      minimum_yaw_speed_enabled_, minimum_yaw_speed_);
    return command;
  }

  void publishStop()
  {
    // 同时清空 previous_command_ 很重要：恢复控制后要从零重新加速，不能沿用停车前
    // 的速度历史跨过加速度限制。
    geometry_msgs::msg::Twist stop;
    previous_command_ = stop;
    if (command_pub_) {command_pub_->publish(stop);}
  }

  void reportState(const std::string & state)
  {
    // 只在状态变化时记录，避免 200 Hz 控制循环重复刷相同日志。
    if (state == control_state_) {return;}
    control_state_ = state;
    if (state == "active") {
      RCLCPP_DEBUG(get_logger(), "controller active");
    } else {
      RCLCPP_DEBUG(get_logger(), "controller stopped: %s", state.c_str());
    }
  }

  void publishMarkers(
    const PathFollowerCommand & command, const Eigen::Vector3d & robot, const double robot_yaw)
  {
    // RViz 标记约定：蓝色球=三维投影最近点，黄色球=前视点，绿色箭头=机器人当前朝向，
    // 粉色箭头=机器人到前视点的目标射线。对照两根箭头即可判断应顺/逆时针旋转。
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = now();
    // 小工具 lambda 统一构造球形 Marker，减少重复填写 header、颜色和尺度字段。
    auto point = [&](const int id, const Eigen::Vector3d & position,
        const float red, const float green, const float blue, const double size) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = path_frame_;
        marker.ns = "local_path_follower";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = size;
        marker.color.r = red; marker.color.g = green; marker.color.b = blue;
        marker.color.a = 1.0F;
        return marker;
      };
    // ARROW 使用两个显式端点，而不是“位姿 + 长度”，因此可直接表达任意三维射线。
    auto arrow = [&](const int id, const Eigen::Vector3d & start, const Eigen::Vector3d & end,
        const float red, const float green, const float blue) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = path_frame_;
        marker.ns = "local_path_follower";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.035;
        marker.scale.y = 0.075;
        marker.scale.z = 0.11;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 1.0F;
        for (const Eigen::Vector3d & position : {start, end}) {
          geometry_msgs::msg::Point message;
          message.x = position.x();
          message.y = position.y();
          message.z = position.z();
          marker.points.push_back(message);
        }
        return marker;
      };
    array.markers.push_back(point(0, command.sample.nearest, 0.1F, 0.6F, 1.0F, 0.10));
    array.markers.push_back(point(1, command.sample.lookahead, 1.0F, 0.9F, 0.0F, 0.16));
    // 当前朝向箭头固定为 0.65 m，仅用于观察 yaw，不代表实际速度大小。
    const Eigen::Vector3d heading_end = robot + 0.65 * Eigen::Vector3d(
      std::cos(robot_yaw), std::sin(robot_yaw), 0.0);
    Eigen::Vector3d target_end = command.sample.lookahead;
    if ((target_end - robot).head<2>().norm() < 1e-6) {
      // 当前视点与机器人 XY 重合时，直接画两点箭头没有方向；改用控制算法回退后的
      // target_yaw 构造可见箭头，与实际角速度目标保持一致。
      target_end = robot + 0.65 * Eigen::Vector3d(
        std::cos(command.target_yaw), std::sin(command.target_yaw), 0.0);
    }
    array.markers.push_back(arrow(2, robot, heading_end, 0.1F, 1.0F, 0.25F));
    array.markers.push_back(arrow(3, robot, target_end, 1.0F, 0.25F, 0.8F));
    marker_pub_->publish(array);
  }

  void publishDiagnostics(
    const PathFollowerCommand & command, const std::string & state,
    const double path_age, const double odometry_age)
  {
    // diagnostics 保留计算中间量，排查“有路径但不走/转向方向不对”时无需增加日志。
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = state == "active" ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "efficient_3d_local_planner/local_path_follower";
    status.hardware_id = "go2";
    status.message = state;
    auto add = [&](const std::string & key, const auto value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key; pair.value = std::to_string(value); status.values.push_back(pair);
      };
    auto add_bool = [&](const std::string & key, const bool value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = value ? "true" : "false";
        status.values.push_back(pair);
      };
    // age 用于判断输入是否超时；remaining/projection/endpoint 都是三维距离。
    add("path_age", path_age);
    add("odometry_age", odometry_age);
    add("lookahead_distance", command.lookahead_distance);
    add("remaining_path", command.sample.remaining_arc);
    add("projection_distance_3d", command.sample.projection_distance_3d);
    add("endpoint_distance_3d", command.endpoint_distance_3d);
    // target_ray_yaw 是当前位置指向前视点的绝对航向，yaw_error 是其与机器人航向的
    // 最短有符号角；正值应逆时针，负值应顺时针。
    add("target_ray_yaw", command.target_yaw);
    add("yaw_error", command.yaw_error);
    add("yaw_error_degrees", command.yaw_error * 180.0 / 3.14159265358979323846);
    // aligning_in_place 表示超过平移门限而只旋转；target_ray_fallback 表示射线过短，
    // 已回退到路径切线；reverse_clamped 表示禁止倒退约束曾经介入。
    add_bool("aligning_in_place", command.aligning_in_place);
    add_bool("target_ray_fallback", command.target_ray_fallback);
    add_bool("reverse_clamped", command.reverse_clamped);
    add_bool("minimum_yaw_speed_enabled", minimum_yaw_speed_enabled_);
    add("minimum_yaw_speed", minimum_yaw_speed_);
    add_bool(
      "minimum_yaw_speed_active",
      minimum_yaw_speed_enabled_ && std::abs(command.wz) > 1e-12 &&
      std::abs(previous_command_.angular.z) > 1e-12 &&
      std::abs(previous_command_.angular.z) <= minimum_yaw_speed_ + 1e-12);
    add("cmd_vx", previous_command_.linear.x);
    add("cmd_vy", previous_command_.linear.y);
    add("cmd_wz", previous_command_.angular.z);
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  void controlLoop()
  {
    // 控制循环先复制一份一致的输入快照，随后的几何计算和 DDS 发布均在锁外完成。
    // 这样高频里程计回调不会被较重的路径投影计算阻塞。
    std::vector<Eigen::Vector3d> path;
    Eigen::Vector3d robot;
    double yaw = 0.0;
    std::string path_frame, odometry_frame;
    bool have_path = false, have_odometry = false;
    double path_age = std::numeric_limits<double>::infinity();
    double odometry_age = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      path = path_;
      robot = robot_;
      yaw = robot_yaw_;
      path_frame = path_frame_;
      odometry_frame = odometry_frame_;
      have_path = have_path_message_;
      have_odometry = have_odometry_;
      if (have_path) {path_age = ageSeconds(last_path_steady_);}
      if (have_odometry) {odometry_age = ageSeconds(last_odometry_steady_);}
    }

    // 停车条件按优先级排列。任何一项失败都发布零速度并提前返回，绝不使用不完整、
    // 过期或不同坐标系的数据继续控制。
    std::string stop_reason;
    if (!enabled_) {stop_reason = "disabled";}
    else if (!have_odometry) {stop_reason = "no_odometry";}
    else if (odometry_age > odometry_timeout_) {stop_reason = "odometry_timeout";}
    else if (!have_path) {stop_reason = "no_path";}
    else if (path.size() < 2U) {stop_reason = "empty_path";}
    else if (path_age > path_timeout_) {stop_reason = "path_timeout";}
    else if (!path_frame.empty() && !odometry_frame.empty() && path_frame != odometry_frame) {
      stop_reason = "frame_mismatch";
    }
    if (!stop_reason.empty()) {
      publishStop();
      reportState(stop_reason);
      publishDiagnostics(PathFollowerCommand{}, stop_reason, path_age, odometry_age);
      return;
    }

    // dt 使用真实控制周期而不是固定理论周期，保证调度抖动时加速度约束仍按秒计算。
    const auto steady_now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(steady_now - last_control_steady_).count();
    last_control_steady_ = steady_now;
    // 节点刚启动、调试暂停或系统卡顿可能产生异常 dt；此时回退到标称周期，避免
    // 一次允许过大的速度跃变。
    if (dt <= 0.0 || dt > 0.2) {dt = 1.0 / control_rate_;}

    // 纯算法函数完成：三维路径投影、沿弧长取前视点、目标射线 yaw、终点制动、
    // 10°~60°线速度缩放以及禁止倒退的期望速度计算。
    const PathFollowerCommand desired = computePathFollowerCommand(
      path, robot, yaw, config_);
    if (!desired.valid) {
      publishStop();
      reportState("invalid_path_geometry");
      publishDiagnostics(desired, "invalid_path_geometry", path_age, odometry_age);
      return;
    }
    previous_command_ = limitAcceleration(desired, dt);
    // 原地对准是硬安全门，而不是一个可以经过多个周期缓慢衰减的速度目标。角速度
    // 仍然遵守角加速度限制，但平移必须本周期立刻归零，不能把历史前进速度带进转弯。
    if (desired.aligning_in_place) {
      previous_command_.linear.x = 0.0;
      previous_command_.linear.y = 0.0;
    }
    // 在最终发布边界再次执行“不倒退”约束。即使未来修改上游算法或限幅器，也不能
    // 从本节点发布负的 linear.x。
    previous_command_.linear.x = std::max(0.0, previous_command_.linear.x);
    command_pub_->publish(previous_command_);
    reportState("active");
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "control_active path_receipts=%lu valid=%lu empty=%lu path_age=%.3f "
      "odom_age=%.3f remaining_3d=%.2f projection_3d=%.2f yaw_error=%.2f "
      "aligning=%d reverse_clamped=%d cmd=[%.3f %.3f %.3f]",
      static_cast<unsigned long>(path_receipts_.load()),
      static_cast<unsigned long>(valid_path_receipts_.load()),
      static_cast<unsigned long>(empty_path_receipts_.load()), path_age, odometry_age,
      desired.sample.remaining_arc, desired.sample.projection_distance_3d, desired.yaw_error,
      desired.aligning_in_place, desired.reverse_clamped,
      previous_command_.linear.x, previous_command_.linear.y, previous_command_.angular.z);
    publishMarkers(desired, robot, yaw);
    publishDiagnostics(desired, "active", path_age, odometry_age);
  }

  // -------- 静态配置：节点启动后不动态改变 --------
  PathFollowerConfig config_;
  bool enabled_{true};
  double control_rate_{50.0};
  double max_linear_acceleration_{0.60};
  double max_yaw_acceleration_{1.00};
  bool minimum_yaw_speed_enabled_{false};
  double minimum_yaw_speed_{0.25};
  double odometry_timeout_{0.30};
  double path_timeout_{0.50};

  // -------- 最新输入快照：由订阅回调写、控制定时器读，必须由 mutex_ 保护 --------
  std::mutex mutex_;
  std::vector<Eigen::Vector3d> path_;
  Eigen::Vector3d robot_{Eigen::Vector3d::Zero()};
  double robot_yaw_{0.0};
  std::string path_frame_, odometry_frame_, control_state_;
  bool have_path_message_{false}, have_odometry_{false};
  SteadyTime last_path_steady_{}, last_odometry_steady_{}, last_control_steady_{};
  // 上一次实际发布的命令用于加速度限幅；三个计数器用于判断规划路径是否稳定到达。
  geometry_msgs::msg::Twist previous_command_;
  std::atomic<std::uint64_t> path_receipts_{0U};
  std::atomic<std::uint64_t> valid_path_receipts_{0U};
  std::atomic<std::uint64_t> empty_path_receipts_{0U};

  // -------- ROS 通信对象 --------
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace efficient_3d_local_planner

int main(int argc, char ** argv)
{
  // 使用普通 spin 即可；即使部署到 MultiThreadedExecutor，类内共享状态也已加锁。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<efficient_3d_local_planner::LocalPathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
