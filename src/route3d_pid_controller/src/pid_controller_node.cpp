#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "route3d_pid_controller/controller_core.hpp"
#include "route3d_pid_controller/elevation_collision_checker.hpp"
#include "route3d_pid_controller/msg/gait_transition.hpp"
#include "route3d_pid_controller/safety_clear_gate.hpp"
#include "route3d_pid_controller/srv/acknowledge_gait_transition.hpp"
#include "route3d_pid_controller/timestamped_pose_buffer.hpp"
#include "route3d_route_slicer/msg/route_task.hpp"
#include "route3d_route_slicer/msg/route_task_array.hpp"

namespace route3d_pid_controller
{
namespace
{

using route3d_route_slicer::msg::RouteTask;
using route3d_route_slicer::msg::RouteTaskArray;
using route3d_pid_controller::msg::GaitTransition;
using route3d_pid_controller::srv::AcknowledgeGaitTransition;

enum class ControllerState
{
  kIdle,
  kReady,
  kTracking,
  kPaused,
  kWaitingTransition,
  kHandoverRequired,
  kFinished,
  kCancelled,
  kError,
};

const char * stateName(const ControllerState state)
{
  switch (state) {
    case ControllerState::kIdle: return "IDLE";
    case ControllerState::kReady: return "READY";
    case ControllerState::kTracking: return "TRACKING";
    case ControllerState::kPaused: return "PAUSED";
    case ControllerState::kWaitingTransition: return "WAITING_TRANSITION";
    case ControllerState::kHandoverRequired: return "HANDOVER_REQUIRED";
    case ControllerState::kFinished: return "FINISHED";
    case ControllerState::kCancelled: return "CANCELLED";
    case ControllerState::kError: return "ERROR";
  }
  return "UNKNOWN";
}

double quaternionYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

PidAxisConfig pidConfig(
  rclcpp::Node & node, const std::string & prefix, const PidAxisConfig & defaults)
{
  PidAxisConfig config;
  config.kp = node.declare_parameter<double>(prefix + ".kp", defaults.kp);
  config.ki = node.declare_parameter<double>(prefix + ".ki", defaults.ki);
  config.kd = node.declare_parameter<double>(prefix + ".kd", defaults.kd);
  config.integral_limit = node.declare_parameter<double>(
    prefix + ".integral_limit", defaults.integral_limit);
  config.output_limit = node.declare_parameter<double>(
    prefix + ".output_limit", defaults.output_limit);
  config.integral_separation = node.declare_parameter<double>(
    prefix + ".integral_separation", defaults.integral_separation);
  config.derivative_filter_tau = node.declare_parameter<double>(
    prefix + ".derivative_filter_tau", defaults.derivative_filter_tau);
  return config;
}

}  // namespace

class PidControllerNode : public rclcpp::Node
{
public:
  PidControllerNode()
  : Node("route3d_pid_controller")
  {
    auto_start_ = declare_parameter<bool>("execution.auto_start", true);
    control_rate_hz_ = declare_parameter<double>("execution.control_rate_hz", 50.0);
    odometry_timeout_s_ = declare_parameter<double>("safety.odometry_timeout_s", 0.30);
    safety_clear_hold_s_ = declare_parameter<double>("safety.clear_hold_s", 3.0);
    emergency_collision_level_threshold_ = declare_parameter<std::int64_t>(
      "safety.emergency_collision_level_threshold", 100);
    obstacle_enabled_ = declare_parameter<bool>("obstacle.enabled", true);
    stop_on_cloud_timeout_ = declare_parameter<bool>("obstacle.stop_on_cloud_timeout", true);
    cloud_timeout_s_ = declare_parameter<double>("obstacle.cloud_timeout_s", 0.50);
    cloud_pose_sync_tolerance_s_ = declare_parameter<double>(
      "obstacle.cloud_pose_sync_tolerance_s", 0.15);
    ElevationCollisionConfig elevation_config;
    elevation_config.resolution_m = declare_parameter<double>(
      "obstacle.elevation.resolution_m", elevation_config.resolution_m);
    elevation_config.width_m = declare_parameter<double>(
      "obstacle.elevation.width_m", elevation_config.width_m);
    elevation_config.default_ground_height_m = declare_parameter<double>(
      "obstacle.elevation.default_ground_height_m",
      elevation_config.default_ground_height_m);
    elevation_config.minimum_obstacle_height_m = declare_parameter<double>(
      "obstacle.elevation.minimum_obstacle_height_m",
      elevation_config.minimum_obstacle_height_m);
    elevation_config.maximum_obstacle_height_m = declare_parameter<double>(
      "obstacle.elevation.maximum_obstacle_height_m",
      elevation_config.maximum_obstacle_height_m);
    elevation_config.roughness_threshold_m = declare_parameter<double>(
      "obstacle.elevation.roughness_threshold_m", elevation_config.roughness_threshold_m);
    elevation_config.body_min_x = declare_parameter<double>(
      "obstacle.elevation.body_exclusion.min_x", elevation_config.body_min_x);
    elevation_config.body_max_x = declare_parameter<double>(
      "obstacle.elevation.body_exclusion.max_x", elevation_config.body_max_x);
    elevation_config.body_min_y = declare_parameter<double>(
      "obstacle.elevation.body_exclusion.min_y", elevation_config.body_min_y);
    elevation_config.body_max_y = declare_parameter<double>(
      "obstacle.elevation.body_exclusion.max_y", elevation_config.body_max_y);
    elevation_config.warning_footprint.min_x = declare_parameter<double>(
      "obstacle.trajectory.footprint.min_x", elevation_config.warning_footprint.min_x);
    elevation_config.warning_footprint.max_x = declare_parameter<double>(
      "obstacle.trajectory.footprint.max_x", elevation_config.warning_footprint.max_x);
    elevation_config.warning_footprint.min_y = declare_parameter<double>(
      "obstacle.trajectory.footprint.min_y", elevation_config.warning_footprint.min_y);
    elevation_config.warning_footprint.max_y = declare_parameter<double>(
      "obstacle.trajectory.footprint.max_y", elevation_config.warning_footprint.max_y);
    elevation_config.warning_footprint.samples_x = static_cast<std::size_t>(
      std::max<std::int64_t>(2, declare_parameter<std::int64_t>(
        "obstacle.trajectory.footprint.samples_x", 5)));
    elevation_config.warning_footprint.samples_y = static_cast<std::size_t>(
      std::max<std::int64_t>(2, declare_parameter<std::int64_t>(
        "obstacle.trajectory.footprint.samples_y", 5)));
    elevation_config.warning_footprint.lateral_inset_m = declare_parameter<double>(
      "obstacle.trajectory.footprint.lateral_inset_m",
      elevation_config.warning_footprint.lateral_inset_m);
    rotation_footprint_forward_scale_ = declare_parameter<double>(
      "obstacle.rotation.footprint_forward_scale", 0.50);
    trajectory_collision_spacing_m_ = declare_parameter<double>(
      "obstacle.trajectory.spacing_m", 0.40);
    trajectory_collision_samples_ = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>(
        "obstacle.trajectory.maximum_samples", 15)));
    replan_near_spacing_m_ = declare_parameter<double>(
      "obstacle.trajectory.replan_near.spacing_m", 0.25);
    replan_near_samples_ = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>(
        "obstacle.trajectory.replan_near.maximum_samples", 25)));
    replan_far_spacing_m_ = declare_parameter<double>(
      "obstacle.trajectory.replan_far.spacing_m", 0.20);
    replan_far_samples_ = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>(
        "obstacle.trajectory.replan_far.maximum_samples", 60)));
    base_collision_hold_cycles_ = static_cast<std::size_t>(
      std::max<std::int64_t>(0, declare_parameter<std::int64_t>(
        "obstacle.elevation.base_collision_hold_cycles", 20)));
    elevation_checker_ = std::make_unique<ElevationCollisionChecker>(elevation_config);
    validateParameters();
    safety_clear_gate_ = std::make_unique<SafetyClearGate>(safety_clear_hold_s_);
    odometry_pose_buffer_ = std::make_unique<TimestampedPoseBuffer>(
      2.0, cloud_pose_sync_tolerance_s_);

    TrackerConfig tracker_config;
    tracker_config.lookahead_distance_m = declare_parameter<double>(
      "tracking.lookahead_distance_m", tracker_config.lookahead_distance_m);
    tracker_config.goal_yaw_tolerance_rad = declare_parameter<double>(
      "tracking.goal_yaw_tolerance_rad", tracker_config.goal_yaw_tolerance_rad);
    tracker_config.corner_slowdown_distance_m = declare_parameter<double>(
      "tracking.corner_slowdown_distance_m", tracker_config.corner_slowdown_distance_m);
    tracker_config.corner_speed_mps = declare_parameter<double>(
      "tracking.corner_speed_mps", tracker_config.corner_speed_mps);
    tracker_config.braking_deceleration_mps2 = declare_parameter<double>(
      "tracking.braking_deceleration_mps2", tracker_config.braking_deceleration_mps2);
    tracker_config.full_speed_yaw_error_rad = declare_parameter<double>(
      "tracking.full_speed_yaw_error_rad", tracker_config.full_speed_yaw_error_rad);
    tracker_config.stop_translation_yaw_error_rad = declare_parameter<double>(
      "tracking.stop_translation_yaw_error_rad", tracker_config.stop_translation_yaw_error_rad);
    tracker_config.maximum_vx_mps = declare_parameter<double>(
      "limits.maximum_vx_mps", tracker_config.maximum_vx_mps);
    tracker_config.maximum_vy_mps = declare_parameter<double>(
      "limits.maximum_vy_mps", tracker_config.maximum_vy_mps);
    tracker_config.maximum_wz_radps = declare_parameter<double>(
      "limits.maximum_wz_radps", tracker_config.maximum_wz_radps);
    tracker_config.maximum_linear_acceleration_mps2 = declare_parameter<double>(
      "limits.maximum_linear_acceleration_mps2",
      tracker_config.maximum_linear_acceleration_mps2);
    tracker_config.maximum_yaw_acceleration_radps2 = declare_parameter<double>(
      "limits.maximum_yaw_acceleration_radps2",
      tracker_config.maximum_yaw_acceleration_radps2);
    tracker_config.projection_backtrack_m = declare_parameter<double>(
      "tracking.projection_backtrack_m", tracker_config.projection_backtrack_m);
    tracker_config.adjustment_entry_distance_m = declare_parameter<double>(
      "adjustment.entry_distance_m", tracker_config.adjustment_entry_distance_m);
    adjustment_route_goal_position_tolerance_m_ = declare_parameter<double>(
      "adjustment.route_goal_position_tolerance_m",
      tracker_config.adjustment_route_goal_position_tolerance_m);
    tracker_config.adjustment_route_goal_position_tolerance_m =
      adjustment_route_goal_position_tolerance_m_;
    adjustment_route_goal_yaw_tolerance_rad_ = declare_parameter<double>(
      "adjustment.route_goal_yaw_tolerance_rad",
      tracker_config.adjustment_route_goal_yaw_tolerance_rad);
    tracker_config.adjustment_route_goal_yaw_tolerance_rad =
      adjustment_route_goal_yaw_tolerance_rad_;
    tracker_config.adjustment_maximum_vx_mps = declare_parameter<double>(
      "adjustment.maximum_vx_mps", tracker_config.adjustment_maximum_vx_mps);
    tracker_config.adjustment_maximum_vy_mps = declare_parameter<double>(
      "adjustment.maximum_vy_mps", tracker_config.adjustment_maximum_vy_mps);
    tracker_config.adjustment_maximum_wz_radps = declare_parameter<double>(
      "adjustment.maximum_wz_radps", tracker_config.adjustment_maximum_wz_radps);
    tracker_config.longitudinal_pid = pidConfig(
      *this, "pid.longitudinal", PidAxisConfig{1.2, 0.0, 0.05, 0.20, 0.80, 1.3, 0.04});
    tracker_config.lateral_pid = pidConfig(
      *this, "pid.lateral", PidAxisConfig{1.5, 0.0, 0.04, 0.20, 0.25, 1.3, 0.04});
    tracker_config.yaw_pid = pidConfig(
      *this, "pid.yaw", PidAxisConfig{2.0, 0.0, 0.05, 0.20, 0.80, 1.3, 0.04});
    tracker_ = std::make_unique<RouteTracker>(tracker_config);

    const auto tasks_topic = declare_parameter<std::string>(
      "topics.tasks", "/route3d_route_slicer/tasks");
    const auto odometry_topic = declare_parameter<std::string>(
      "topics.odometry", "/lio_odom_hf");
    const auto cloud_topic = declare_parameter<std::string>(
      "topics.obstacle_cloud", "/cloud_registered_body");
    const auto external_stop_topic = declare_parameter<std::string>(
      "topics.external_safety_stop", "/route3d_pid_controller/external_safety_stop");
    const auto collision_level_topic = declare_parameter<std::string>(
      "topics.collision_level", "/collision_level");
    const auto command_topic = declare_parameter<std::string>(
      "topics.command", "/cmd_vel_pid");
    const auto efficient_path_topic = declare_parameter<std::string>(
      "topics.efficient_path", "/route3d_controller/efficient_path");
    const auto active_controller_topic = declare_parameter<std::string>(
      "topics.active_controller", "/route3d_controller/active_source");
    const auto gait_transition_topic = declare_parameter<std::string>(
      "topics.gait_transition", "/route3d_pid_controller/gait_transition");
    const auto gait_acknowledged_topic = declare_parameter<std::string>(
      "topics.gait_acknowledged", "/route3d_pid_controller/gait_acknowledged");

    tasks_subscription_ = create_subscription<RouteTaskArray>(
      tasks_topic, latchedQos(),
      std::bind(&PidControllerNode::tasksCallback, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS().keep_last(50),
      std::bind(&PidControllerNode::odometryCallback, this, std::placeholders::_1));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&PidControllerNode::cloudCallback, this, std::placeholders::_1));
    external_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      external_stop_topic, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
        external_safety_stop_ = message->data;
      });
    collision_level_subscription_ = create_subscription<std_msgs::msg::Int32>(
      collision_level_topic, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Int32::ConstSharedPtr message) {
        collision_level_ = message->data;
      });
    gait_acknowledged_subscription_ = create_subscription<GaitTransition>(
      gait_acknowledged_topic, latchedQos(),
      std::bind(&PidControllerNode::gaitAcknowledgedCallback, this, std::placeholders::_1));

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      command_topic, rclcpp::QoS(10).reliable());
    efficient_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      efficient_path_topic, latchedQos());
    active_controller_publisher_ = create_publisher<std_msgs::msg::String>(
      active_controller_topic, latchedQos());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/route3d_pid_controller/status", latchedQos());
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/route3d_pid_controller/diagnostics", rclcpp::QoS(10).reliable());
    lookahead_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/route3d_pid_controller/lookahead", rclcpp::QoS(10).reliable());
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/route3d_pid_controller/active_path", latchedQos());
    obstacle_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/route3d_pid_controller/obstacle_stop", latchedQos());
    replan_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/route3d_pid_controller/replan_required", latchedQos());
    gait_transition_publisher_ = create_publisher<GaitTransition>(
      gait_transition_topic, rclcpp::QoS(10).reliable());
    elevation_debug_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/route3d_pid_controller/elevation_debug", rclcpp::QoS(1).reliable());

    start_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start", std::bind(
        &PidControllerNode::startCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    continue_service_ = create_service<std_srvs::srv::Trigger>(
      "~/continue", std::bind(
        &PidControllerNode::continueCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    acknowledge_gait_service_ = create_service<AcknowledgeGaitTransition>(
      "~/acknowledge_gait_transition", std::bind(
        &PidControllerNode::acknowledgeGaitCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    pause_service_ = create_service<std_srvs::srv::Trigger>(
      "~/pause", std::bind(
        &PidControllerNode::pauseCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    resume_service_ = create_service<std_srvs::srv::Trigger>(
      "~/resume", std::bind(
        &PidControllerNode::resumeCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel", std::bind(
        &PidControllerNode::cancelCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PidControllerNode::controlTimer, this));
    last_control_time_ = std::chrono::steady_clock::now();
    publishControllerSelection("none");
    publishEfficientStop();
    state_detail_ = "node initialized; waiting for a sliced route";
    publishStatus(state_detail_);
    RCLCPP_INFO(
      get_logger(),
      "Generic PID controller ready: tasks=%s odom=%s cloud=%s cmd=%s auto_start=%s",
      tasks_topic.c_str(), odometry_topic.c_str(), cloud_topic.c_str(), command_topic.c_str(),
      auto_start_ ? "true" : "false");
  }

  ~PidControllerNode() override
  {
    publishZero();
    publishControllerSelection("none");
    publishEfficientStop();
  }

private:
  void validateParameters() const
  {
    if (control_rate_hz_ < 5.0 || control_rate_hz_ > 200.0 || odometry_timeout_s_ <= 0.0 ||
      !std::isfinite(safety_clear_hold_s_) || safety_clear_hold_s_ < 0.0 ||
      emergency_collision_level_threshold_ <= 0 ||
      cloud_timeout_s_ <= 0.0 || !std::isfinite(cloud_pose_sync_tolerance_s_) ||
      cloud_pose_sync_tolerance_s_ < 0.0 || trajectory_collision_spacing_m_ <= 0.0 ||
      !std::isfinite(rotation_footprint_forward_scale_) ||
      rotation_footprint_forward_scale_ <= 0.0 || rotation_footprint_forward_scale_ > 1.0 ||
      trajectory_collision_samples_ == 0U || replan_near_spacing_m_ <= 0.0 ||
      replan_near_samples_ == 0U || replan_far_spacing_m_ <= 0.0 ||
      replan_far_samples_ == 0U)
    {
      throw std::invalid_argument("invalid PID controller rate, timeout, or elevation collision parameters");
    }
  }

  void tasksCallback(const RouteTaskArray::ConstSharedPtr message)
  {
    if (message->tasks.empty()) {
      setState(ControllerState::kError, "received an empty task array");
      publishZero();
      return;
    }
    publishZero();
    publishControllerSelection("none");
    publishEfficientStop();
    route_ = *message;
    has_route_ = true;
    active_task_index_ = 0U;
    active_ = false;
    task_controller_ = "none";
    paused_ = false;
    waiting_for_gait_transition_ = false;
    resume_existing_task_after_gait_ = false;
    safety_stopped_ = false;
    safety_clear_waiting_logged_ = false;
    rotation_footprint_active_ = false;
    safety_clear_gate_->reset();
    last_tracking_output_ = {};
    base_collision_hold_remaining_ = 0U;
    setState(
      ControllerState::kReady,
      auto_start_ ? "new route staged; starting automatically" :
      "new route staged; call ~/start to move");
    RCLCPP_INFO(
      get_logger(), "Staged route sequence=%lu start=%d goal=%d tasks=%zu",
      static_cast<unsigned long>(route_.route_sequence), route_.route_start_id,
      route_.route_goal_id, route_.tasks.size());
    if (auto_start_) {
      (void)activateTask(false);
    }
  }

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    robot_pose_.x = message->pose.pose.position.x;
    robot_pose_.y = message->pose.pose.position.y;
    robot_z_ = message->pose.pose.position.z;
    robot_pose_.yaw = quaternionYaw(message->pose.pose.orientation);
    frame_id_ = message->header.frame_id;
    last_odometry_time_ = now();
    has_odometry_ = true;
    const rclcpp::Time message_stamp(message->header.stamp, RCL_ROS_TIME);
    const auto stamp = message_stamp.nanoseconds() > 0 ? message_stamp : last_odometry_time_;
    odometry_pose_buffer_->add(stamp.nanoseconds(), robot_pose_);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    last_cloud_time_ = now();
    has_cloud_ = true;
    if (!message->header.frame_id.empty()) {
      cloud_frame_id_ = message->header.frame_id;
    }
    if (!obstacle_enabled_) {
      trajectory_collision_count_ = 0U;
      return;
    }

    const rclcpp::Time message_stamp(message->header.stamp, RCL_ROS_TIME);
    elevation_map_stamp_ = message_stamp.nanoseconds() > 0 ? message_stamp : last_cloud_time_;
    if (!odometry_pose_buffer_->lookup(
        elevation_map_stamp_.nanoseconds(), elevation_map_pose_))
    {
      cloud_parse_error_ = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot time-align obstacle cloud at %.6f with odometry (tolerance %.3f s)",
        elevation_map_stamp_.seconds(), cloud_pose_sync_tolerance_s_);
      return;
    }
    has_elevation_map_pose_ = true;

    try {
      std::vector<ElevationPoint> points;
      points.reserve(static_cast<std::size_t>(message->width) * message->height);
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          points.push_back({*x, *y, *z});
        }
      }
      if (!points.empty()) {
        elevation_checker_->update(points);
        ++elevation_map_generation_;
      }
      cloud_parse_error_ = false;
      if (!active_ || paused_) {
        debug_local_trajectory_ = {Pose2d{}};
        publishElevationDebug();
        debug_visualization_generation_ = elevation_map_generation_;
      }
    } catch (const std::exception & error) {
      cloud_parse_error_ = true;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot parse obstacle cloud XYZ: %s", error.what());
    }
  }

  bool activateTask(const bool transition_acknowledged)
  {
    if (!has_route_ || active_task_index_ >= route_.tasks.size()) {
      setState(ControllerState::kError, "no task is available");
      return false;
    }
    const auto & task = route_.tasks[active_task_index_];
    const bool use_pid = task.resolved_controller_mode == "pid" && task.obstacle_mode != 4;
    const bool use_efficient =
      task.resolved_controller_mode == "efficient_3d_local_planner";
    if (!use_pid && !use_efficient) {
      active_ = false;
      publishControllerSelection("none");
      setState(
        ControllerState::kHandoverRequired,
        "task requires controller '" + task.resolved_controller_mode + "'");
      return false;
    }
    if (task.requires_gait_switch_at_start && !transition_acknowledged) {
      enterGaitTransition(task);
      return false;
    }
    TrackingTask tracking_task;
    tracking_task.endpoint_tolerance_m = task.endpoint_tolerance_m;
    tracking_task.maximum_speed_mps = task.linear_speed_mps;
    tracking_task.align_goal_yaw = task.align_goal_yaw;
    tracking_task.reverse_motion = task.reverse_motion;
    tracking_task.is_route_goal = task.is_route_goal;
    tracking_task.waypoints.reserve(task.waypoints.size());
    for (const auto & input : task.waypoints) {
      tracking_task.waypoints.push_back({
        input.vertex_id, input.position.x, input.position.y, input.rpy.z,
        input.must_pass_through, input.pass_radius_m});
    }
    try {
      tracker_->setTask(std::move(tracking_task));
    } catch (const std::exception & error) {
      active_ = false;
      setState(ControllerState::kError, error.what());
      return false;
    }
    active_ = true;
    resume_existing_task_after_gait_ = false;
    task_controller_ = use_pid ? "pid" : "efficient_3d_local_planner";
    paused_ = false;
    waiting_for_gait_transition_ = false;
    safety_stopped_ = false;
    safety_clear_waiting_logged_ = false;
    rotation_footprint_active_ = false;
    safety_clear_gate_->reset();
    trajectory_collision_count_ = 0U;
    checked_trajectory_poses_ = 0U;
    elevation_replan_required_ = false;
    base_collision_hold_remaining_ = 0U;
    last_tracking_output_ = {};
    publishActivePath(task);
    if (use_efficient) {
      publishZero();
      publishEfficientPath(task);
    } else {
      publishEfficientStop();
    }
    publishControllerSelection(task_controller_);
    setState(
      ControllerState::kTracking,
      task_controller_ + " task activated");
    return true;
  }

  void advanceTask()
  {
    publishZero();
    publishControllerSelection("none");
    publishEfficientStop();
    active_ = false;
    resume_existing_task_after_gait_ = false;
    task_controller_ = "none";
    const auto & completed = route_.tasks[active_task_index_];
    if (completed.is_route_goal || active_task_index_ + 1U >= route_.tasks.size()) {
      setState(ControllerState::kFinished, "route goal reached");
      return;
    }
    ++active_task_index_;
    const auto & next = route_.tasks[active_task_index_];
    if (next.requires_gait_switch_at_start) {
      enterGaitTransition(next);
      return;
    }
    if (completed.completion_policy == RouteTask::COMPLETION_BUSINESS_STOP) {
      waiting_for_gait_transition_ = false;
      setState(
        ControllerState::kWaitingTransition,
        "stopped at task boundary; perform transition then call ~/continue");
      return;
    }
    (void)activateTask(true);
  }

  bool obstacleDetectionEnabledForTask() const
  {
    if (!active_ || !has_route_ || active_task_index_ >= route_.tasks.size()) {
      return false;
    }
    // Reference semantics: 3 means explicitly ignore ordinary obstacles.
    return route_.tasks[active_task_index_].obstacle_mode != 3;
  }

  double activeTaskEndpointDistance3d() const
  {
    if (!has_odometry_ || !has_route_ || active_task_index_ >= route_.tasks.size() ||
      route_.tasks[active_task_index_].waypoints.empty())
    {
      return std::numeric_limits<double>::infinity();
    }
    const auto & endpoint = route_.tasks[active_task_index_].waypoints.back().position;
    const double dx = endpoint.x - robot_pose_.x;
    const double dy = endpoint.y - robot_pose_.y;
    const double dz = endpoint.z - robot_z_;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  bool efficientTaskEndpointReached() const
  {
    if (!active_ || task_controller_ != "efficient_3d_local_planner" ||
      !has_route_ || active_task_index_ >= route_.tasks.size())
    {
      return false;
    }
    const auto & task = route_.tasks[active_task_index_];
    const double position_tolerance = task.is_route_goal ?
      adjustment_route_goal_position_tolerance_m_ : task.endpoint_tolerance_m;
    if (task.waypoints.empty() ||
      activeTaskEndpointDistance3d() > position_tolerance)
    {
      return false;
    }
    if (!task.align_goal_yaw) {
      return true;
    }
    const double yaw_error = normalizeAngle(
      task.waypoints.back().rpy.z - robot_pose_.yaw);
    return std::abs(yaw_error) <= adjustment_route_goal_yaw_tolerance_rad_;
  }

  FootprintGrid activeFootprint() const
  {
    FootprintGrid footprint = elevation_checker_->config().warning_footprint;
    if (!has_route_ || active_task_index_ >= route_.tasks.size()) {
      return footprint;
    }
    const auto & box = route_.tasks[active_task_index_].obstacle_box_m;
    if (box[0] < box[1] && box[2] < box[3]) {
      footprint.min_x = box[0];
      footprint.max_x = box[1];
      footprint.min_y = box[2];
      footprint.max_y = box[3];
      footprint.lateral_inset_m = 0.0;
    }
    return footprint;
  }

  FootprintGrid activeBaseFootprint() const
  {
    FootprintGrid footprint = activeFootprint();
    if (!rotation_footprint_active_) {
      return footprint;
    }
    // max_x is the warning distance measured forward from the robot origin.
    // Keep the normal route sweep unchanged and shorten only the footprint at
    // the robot pose while the tracker deliberately suppresses translation.
    const double scaled_max_x = footprint.max_x * rotation_footprint_forward_scale_;
    if (scaled_max_x > footprint.min_x) {
      footprint.max_x = scaled_max_x;
    }
    return footprint;
  }

  std::vector<Pose2d> localCollisionTrajectory(
    const double spacing_m, const std::size_t maximum_samples) const
  {
    const auto world_trajectory = tracker_->sampleRemainingPath(
      spacing_m, maximum_samples);
    std::vector<Pose2d> local_trajectory;
    local_trajectory.reserve(world_trajectory.size());
    const Pose2d & map_pose = has_elevation_map_pose_ ? elevation_map_pose_ : robot_pose_;
    const double cosine = std::cos(map_pose.yaw);
    const double sine = std::sin(map_pose.yaw);
    for (const auto & pose : world_trajectory) {
      const double dx = pose.x - map_pose.x;
      const double dy = pose.y - map_pose.y;
      local_trajectory.push_back({
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        normalizeAngle(pose.yaw - map_pose.yaw)});
    }
    return local_trajectory;
  }

  bool elevationTrajectoryBlocked()
  {
    trajectory_collision_count_ = 0U;
    checked_trajectory_poses_ = 0U;
    elevation_replan_required_ = false;
    if (!obstacleDetectionEnabledForTask() || !elevation_checker_->ready()) {
      debug_local_trajectory_.clear();
      rotation_footprint_active_ = false;
      return false;
    }
    const auto footprint = activeFootprint();
    rotation_footprint_active_ = tracker_->requiresInPlaceRotation(robot_pose_);
    const auto base_footprint = activeBaseFootprint();
    const Pose2d & map_pose = has_elevation_map_pose_ ? elevation_map_pose_ : robot_pose_;
    const double cosine = std::cos(map_pose.yaw);
    const double sine = std::sin(map_pose.yaw);
    const double robot_dx = robot_pose_.x - map_pose.x;
    const double robot_dy = robot_pose_.y - map_pose.y;
    debug_local_trajectory_ = {Pose2d{
      cosine * robot_dx + sine * robot_dy,
      -sine * robot_dx + cosine * robot_dy,
      normalizeAngle(robot_pose_.yaw - map_pose.yaw)}};
    const auto base_result = elevation_checker_->checkTrajectory(
      debug_local_trajectory_, &base_footprint);
    if (base_result.collision()) {
      base_collision_hold_remaining_ = base_collision_hold_cycles_;
    } else if (base_collision_hold_remaining_ > 0U) {
      --base_collision_hold_remaining_;
    }
    const auto obstacle_mode = route_.tasks[active_task_index_].obstacle_mode;
    if (obstacle_mode == 1) {
      const auto near_trajectory = localCollisionTrajectory(
        replan_near_spacing_m_, replan_near_samples_);
      const auto far_trajectory = localCollisionTrajectory(
        replan_far_spacing_m_, replan_far_samples_);
      const auto near_result = elevation_checker_->checkTrajectory(
        near_trajectory, &footprint);
      const auto far_result = elevation_checker_->checkTrajectory(
        far_trajectory, &footprint);
      debug_local_trajectory_.insert(
        debug_local_trajectory_.end(), far_trajectory.begin(), far_trajectory.end());
      checked_trajectory_poses_ = near_result.trajectory_poses + far_result.trajectory_poses;
      trajectory_collision_count_ = base_result.collision_samples +
        near_result.collision_samples + far_result.collision_samples;
      elevation_replan_required_ = near_result.collision() || far_result.collision();
      return base_result.collision() || base_collision_hold_remaining_ > 0U ||
             near_result.collision();
    }
    const auto path_trajectory = localCollisionTrajectory(
      trajectory_collision_spacing_m_, trajectory_collision_samples_);
    const auto path_result = elevation_checker_->checkTrajectory(path_trajectory, &footprint);
    debug_local_trajectory_.insert(
      debug_local_trajectory_.end(), path_trajectory.begin(), path_trajectory.end());
    checked_trajectory_poses_ = path_result.trajectory_poses;
    trajectory_collision_count_ = base_result.collision_samples + path_result.collision_samples;
    return base_result.collision() || base_collision_hold_remaining_ > 0U || path_result.collision();
  }

  void controlTimer()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(steady_now - last_control_time_).count();
    last_control_time_ = steady_now;
    dt = std::clamp(dt, 1.0e-4, 0.10);
    if (!active_ || paused_) {
      if ((++telemetry_tick_ % static_cast<std::size_t>(control_rate_hz_)) == 0U) {
        if (waiting_for_gait_transition_) {
          publishGaitTransition();
        }
        publishStatus(state_detail_);
      }
      return;
    }

    const bool odometry_stale = !has_odometry_ ||
      (now() - last_odometry_time_).seconds() > odometry_timeout_s_;
    const bool cloud_stale = obstacleDetectionEnabledForTask() && stop_on_cloud_timeout_ &&
      (!has_cloud_ || (now() - last_cloud_time_).seconds() > cloud_timeout_s_);
    const bool elevation_map_missing = obstacleDetectionEnabledForTask() &&
      !elevation_checker_->ready();
    const bool obstacle_stop = obstacleDetectionEnabledForTask() &&
      (cloud_parse_error_ || elevationTrajectoryBlocked());
    if (debug_visualization_generation_ != elevation_map_generation_) {
      publishElevationDebug();
      debug_visualization_generation_ = elevation_map_generation_;
    }
    const bool emergency_collision =
      collision_level_ >= emergency_collision_level_threshold_;
    const bool must_stop = external_safety_stop_ || emergency_collision || odometry_stale ||
      cloud_stale || elevation_map_missing || obstacle_stop;
    publishSafetyFlags(must_stop, elevation_replan_required_);
    if (must_stop) {
      publishZero();
      safety_clear_gate_->reset();
      safety_clear_waiting_logged_ = false;
      if (!safety_stopped_) {
        tracker_->stopAndResetControllers();
        safety_stopped_ = true;
        publishControllerSelection("none");
        std::string reason = external_safety_stop_ ? "external safety stop" :
          (emergency_collision ? "emergency collision level" :
          (odometry_stale ? "odometry timeout" :
          (cloud_stale ? "obstacle cloud timeout" :
          (elevation_map_missing ? "elevation map unavailable" :
          (cloud_parse_error_ ? "invalid obstacle cloud" :
          "elevation-map trajectory collision")))));
        publishStatus(reason);
        RCLCPP_WARN(get_logger(), "PID safety stop: %s", reason.c_str());
      }
      return;
    }
    if (safety_stopped_) {
      publishZero();
      if (!safety_clear_waiting_logged_) {
        safety_clear_waiting_logged_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Safety condition is clear; holding stop for %.2f s before restoring gait",
          safety_clear_hold_s_);
        publishStatus("safety clear hold before gait restore");
      }
      if (!safety_clear_gate_->ready(steady_now)) {
        return;
      }
      safety_stopped_ = false;
      safety_clear_waiting_logged_ = false;
      safety_clear_gate_->reset();
      // active_source=none makes the Go2 adapter call StopMove so a human can
      // take over with the remote.  StopMove also exits StaticWalk/SwitchGait
      // on the deployed firmware.  Never resume velocity directly: restore
      // the current task gait and wait for its acknowledgement first.
      const auto & task = route_.tasks[active_task_index_];
      RCLCPP_INFO(
        get_logger(), "Safety condition cleared; restoring gait '%s' before tracking resumes",
        task.gait_command.c_str());
      enterGaitTransition(task, true);
      return;
    }

    try {
      const auto output = tracker_->update(robot_pose_, dt);
      last_tracking_output_ = output;
      publishLookahead(output.lookahead);
      const bool efficient_endpoint_reached = efficientTaskEndpointReached();
      if (output.reached || efficient_endpoint_reached) {
        if (efficient_endpoint_reached && !output.reached) {
          RCLCPP_INFO(
            get_logger(),
            "Efficient-planner task endpoint reached at 3D distance %.3f m; "
            "advancing without waiting for the independent PID route gates",
            activeTaskEndpointDistance3d());
        }
        advanceTask();
        return;
      }
      if (task_controller_ == "pid") {
        publishCommand(output.command);
      }
      if ((++telemetry_tick_ % static_cast<std::size_t>(control_rate_hz_)) == 0U) {
        publishStatus("tracking");
      }
    } catch (const std::exception & error) {
      publishZero();
      active_ = false;
      setState(ControllerState::kError, error.what());
    }
  }

  void publishCommand(const VelocityCommand & command)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = command.vx;
    message.linear.y = command.vy;
    message.angular.z = command.wz;
    command_publisher_->publish(message);
  }

  void publishElevationDebug()
  {
    if (!elevation_debug_publisher_ || !elevation_checker_ || !elevation_checker_->ready()) {
      return;
    }
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray output;
    const auto stamp = elevation_map_stamp_.nanoseconds() > 0 ? elevation_map_stamp_ : now();
    const std::string frame = cloud_frame_id_.empty() ? "base_link" : cloud_frame_id_;

    Marker clear;
    clear.header.stamp = stamp;
    clear.header.frame_id = frame;
    clear.action = Marker::DELETEALL;
    output.markers.push_back(clear);

    auto marker = [&](const std::string & name, const int id, const int type) {
        Marker result;
        result.header.stamp = stamp;
        result.header.frame_id = frame;
        result.ns = name;
        result.id = id;
        result.type = type;
        result.action = Marker::ADD;
        result.pose.orientation.w = 1.0;
        // These coordinates describe the cloud-time base frame. Transform
        // them once at that stamp; following the live frame makes the sampled
        // corridor rotate away from the fixed world path between cloud frames.
        result.frame_locked = false;
        return result;
      };
    auto point = [](const double x, const double y, const double z) {
        geometry_msgs::msg::Point result;
        result.x = x;
        result.y = y;
        result.z = z;
        return result;
      };
    auto addRectangle = [&](Marker & target, const double min_x, const double max_x,
        const double min_y, const double max_y, const double z) {
        target.points = {
          point(min_x, min_y, z), point(max_x, min_y, z), point(max_x, max_y, z),
          point(min_x, max_y, z), point(min_x, min_y, z)};
      };

    const auto & config = elevation_checker_->config();
    Marker map_boundary = marker("map_boundary_4m", 0, Marker::LINE_STRIP);
    map_boundary.scale.x = 0.025;
    map_boundary.color.b = 1.0F;
    map_boundary.color.a = 1.0F;
    const double map_half = 0.5 * config.width_m;
    addRectangle(map_boundary, -map_half, map_half, -map_half, map_half, 0.03);
    output.markers.push_back(std::move(map_boundary));

    Marker accepted_cloud_range = marker("accepted_cloud_range", 1, Marker::LINE_STRIP);
    accepted_cloud_range.scale.x = 0.025;
    accepted_cloud_range.color.g = 0.9F;
    accepted_cloud_range.color.b = 0.9F;
    accepted_cloud_range.color.a = 1.0F;
    const double input_half = config.width_m / (2.01 * std::sqrt(2.0));
    addRectangle(
      accepted_cloud_range, -input_half, input_half, -input_half, input_half, 0.04);
    output.markers.push_back(std::move(accepted_cloud_range));

    Marker body_exclusion = marker("body_exclusion", 2, Marker::LINE_STRIP);
    body_exclusion.scale.x = 0.035;
    body_exclusion.color.r = 1.0F;
    body_exclusion.color.g = 0.75F;
    body_exclusion.color.a = 1.0F;
    addRectangle(
      body_exclusion, config.body_min_x, config.body_max_x,
      config.body_min_y, config.body_max_y, 0.05);
    output.markers.push_back(std::move(body_exclusion));

    Marker elevation = marker("observed_elevation", 3, Marker::CUBE_LIST);
    elevation.scale.x = config.resolution_m * 0.88;
    elevation.scale.y = config.resolution_m * 0.88;
    elevation.scale.z = 0.025;
    Marker rough = marker("rough_cells_collision_layer", 4, Marker::CUBE_LIST);
    rough.scale = elevation.scale;
    rough.scale.z = 0.05;
    rough.color.r = 1.0F;
    rough.color.a = 0.90F;
    for (const auto & cell : elevation_checker_->cellData()) {
      if (cell.observed) {
        elevation.points.push_back(point(cell.x, cell.y, cell.height));
        std_msgs::msg::ColorRGBA color;
        const double normalized = std::clamp(
          (cell.height - config.minimum_obstacle_height_m) /
          (config.maximum_obstacle_height_m - config.minimum_obstacle_height_m), 0.0, 1.0);
        color.r = static_cast<float>(normalized);
        color.g = static_cast<float>(0.85 - 0.35 * normalized);
        color.b = static_cast<float>(1.0 - normalized);
        color.a = 0.85F;
        elevation.colors.push_back(color);
      }
      if (cell.rough) {
        rough.points.push_back(point(cell.x, cell.y, cell.height + 0.04));
      }
    }
    output.markers.push_back(std::move(elevation));
    output.markers.push_back(std::move(rough));

    const auto footprint = activeFootprint();
    const auto base_footprint = activeBaseFootprint();
    const std::vector<Pose2d> trajectory = debug_local_trajectory_.empty() ?
      std::vector<Pose2d>{Pose2d{}} : debug_local_trajectory_;
    Marker path = marker("checked_trajectory", 5, Marker::LINE_STRIP);
    path.scale.x = 0.035;
    path.color.r = 1.0F;
    path.color.g = 0.85F;
    path.color.a = 1.0F;
    Marker footprint_outlines = marker("checked_footprints", 6, Marker::LINE_LIST);
    footprint_outlines.scale.x = 0.018;
    footprint_outlines.color.b = 1.0F;
    footprint_outlines.color.a = 0.75F;
    for (std::size_t pose_index = 0U; pose_index < trajectory.size(); ++pose_index) {
      const auto & pose = trajectory[pose_index];
      const auto & outline_footprint = pose_index == 0U ? base_footprint : footprint;
      path.points.push_back(point(pose.x, pose.y, 0.09));
      const double cosine = std::cos(pose.yaw);
      const double sine = std::sin(pose.yaw);
      auto transform = [&](const double x, const double y) {
          return point(
            pose.x + x * cosine - y * sine,
            pose.y + x * sine + y * cosine, 0.075);
        };
      const double outline_min_y = outline_footprint.min_y + outline_footprint.lateral_inset_m;
      const double outline_max_y = outline_footprint.max_y - outline_footprint.lateral_inset_m;
      const auto first = transform(outline_footprint.min_x, outline_min_y);
      const auto second = transform(outline_footprint.max_x, outline_min_y);
      const auto third = transform(outline_footprint.max_x, outline_max_y);
      const auto fourth = transform(outline_footprint.min_x, outline_max_y);
      footprint_outlines.points.insert(
        footprint_outlines.points.end(),
        {first, second, second, third, third, fourth, fourth, first});
    }
    output.markers.push_back(std::move(path));
    output.markers.push_back(std::move(footprint_outlines));

    Marker samples = marker("footprint_samples", 7, Marker::POINTS);
    samples.scale.x = 0.045;
    samples.scale.y = 0.045;
    Marker hits = marker("collision_hits", 8, Marker::SPHERE_LIST);
    hits.scale.x = 0.085;
    hits.scale.y = 0.085;
    hits.scale.z = 0.085;
    hits.color.r = 1.0F;
    hits.color.a = 1.0F;
    std::vector<FootprintSample> footprint_samples;
    if (!trajectory.empty()) {
      footprint_samples = elevation_checker_->sampleTrajectory(
        {trajectory.front()}, &base_footprint);
      if (trajectory.size() > 1U) {
        const std::vector<Pose2d> route_trajectory(trajectory.begin() + 1, trajectory.end());
        auto route_samples = elevation_checker_->sampleTrajectory(route_trajectory, &footprint);
        footprint_samples.insert(
          footprint_samples.end(), route_samples.begin(), route_samples.end());
      }
    }
    for (const auto & sample : footprint_samples) {
      if (sample.collision) {
        hits.points.push_back(point(sample.x, sample.y, 0.14));
      } else {
        samples.points.push_back(point(sample.x, sample.y, 0.11));
      }
    }
    samples.color.r = 0.25F;
    samples.color.g = 0.55F;
    samples.color.b = 1.0F;
    samples.color.a = 0.55F;
    output.markers.push_back(std::move(samples));
    output.markers.push_back(std::move(hits));
    elevation_debug_publisher_->publish(output);
  }

  void publishZero()
  {
    if (command_publisher_) {
      command_publisher_->publish(geometry_msgs::msg::Twist{});
    }
  }

  void publishLookahead(const Pose2d & lookahead)
  {
    geometry_msgs::msg::PointStamped message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_.empty() ? "camera_init" : frame_id_;
    message.point.x = lookahead.x;
    message.point.y = lookahead.y;
    lookahead_publisher_->publish(message);
  }

  void publishActivePath(const RouteTask & task)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = route_.header.frame_id;
    path.poses.reserve(task.waypoints.size());
    for (const auto & waypoint : task.waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = waypoint.position;
      pose.pose.orientation.z = std::sin(waypoint.rpy.z * 0.5);
      pose.pose.orientation.w = std::cos(waypoint.rpy.z * 0.5);
      path.poses.push_back(std::move(pose));
    }
    path_publisher_->publish(path);
  }

  nav_msgs::msg::Path taskPath(const RouteTask & task) const
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = route_.header.frame_id;
    path.poses.reserve(task.waypoints.size());
    for (const auto & waypoint : task.waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = waypoint.position;
      pose.pose.orientation.z = std::sin(waypoint.rpy.z * 0.5);
      pose.pose.orientation.w = std::cos(waypoint.rpy.z * 0.5);
      path.poses.push_back(std::move(pose));
    }
    return path;
  }

  void publishEfficientPath(const RouteTask & task)
  {
    efficient_path_publisher_->publish(taskPath(task));
  }

  void publishEfficientStop()
  {
    if (!efficient_path_publisher_) {
      return;
    }
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = route_.header.frame_id;
    efficient_path_publisher_->publish(path);
  }

  void publishControllerSelection(const std::string & controller)
  {
    selected_controller_ = controller;
    if (!active_controller_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = controller;
    active_controller_publisher_->publish(message);
  }

  void publishSafetyFlags(const bool blocked, const bool replan_requested)
  {
    std_msgs::msg::Bool obstacle;
    obstacle.data = blocked;
    obstacle_publisher_->publish(obstacle);
    std_msgs::msg::Bool replan;
    replan.data = replan_requested;
    replan_publisher_->publish(replan);
  }

  void enterGaitTransition(
    const RouteTask & task, const bool resume_existing_task = false)
  {
    active_ = false;
    paused_ = false;
    waiting_for_gait_transition_ = true;
    resume_existing_task_after_gait_ = resume_existing_task;
    publishZero();
    publishControllerSelection("none");
    setState(
      ControllerState::kWaitingTransition,
      "waiting for gait command '" + task.gait_command + "'");
    publishGaitTransition();
  }

  void publishGaitTransition()
  {
    if (!waiting_for_gait_transition_ || !has_route_ ||
      active_task_index_ >= route_.tasks.size())
    {
      return;
    }
    const auto & task = route_.tasks[active_task_index_];
    GaitTransition message;
    message.route_sequence = route_.route_sequence;
    message.task_index = static_cast<std::uint32_t>(active_task_index_);
    message.gait_command = task.gait_command;
    message.locomotion_mode = task.locomotion_mode;
    gait_transition_publisher_->publish(message);
  }

  void setState(const ControllerState state, const std::string & detail)
  {
    state_ = state;
    state_detail_ = detail;
    publishStatus(detail);
    RCLCPP_INFO(get_logger(), "PID state=%s detail=%s", stateName(state_), detail.c_str());
  }

  void publishStatus(const std::string & detail)
  {
    const bool has_task = has_route_ && active_task_index_ < route_.tasks.size();
    nlohmann::json status = {
      {"state", stateName(state_)},
      {"detail", detail},
      {"route_sequence", has_route_ ? route_.route_sequence : 0U},
      {"task_index", active_task_index_},
      {"active", active_},
      {"active_controller", selected_controller_},
      {"paused", paused_},
      {"resume_existing_task_after_gait", resume_existing_task_after_gait_},
      {"transition_kind", waiting_for_gait_transition_ ? "gait" :
        (state_ == ControllerState::kWaitingTransition ? "manual" : "")},
      {"safety_stopped", safety_stopped_},
      {"safety_clear_hold_s", safety_clear_hold_s_},
      {"safety_clear_waiting", safety_clear_waiting_logged_},
      {"cloud_obstacle", trajectory_collision_count_ > 0U},
      {"obstacle_point_count", trajectory_collision_count_},
      {"elevation_map_ready", elevation_checker_ && elevation_checker_->ready()},
      {"elevation_observed_cells", elevation_checker_ ? elevation_checker_->occupiedCells() : 0U},
      {"elevation_rough_cells", elevation_checker_ ? elevation_checker_->roughCells() : 0U},
      {"trajectory_poses_checked", checked_trajectory_poses_},
      {"replan_required", elevation_replan_required_},
      {"base_collision_hold_remaining", base_collision_hold_remaining_},
      {"rotation_footprint_active", rotation_footprint_active_},
      {"base_footprint_max_x_m", activeBaseFootprint().max_x},
      {"has_odometry", has_odometry_},
      {"has_cloud", has_cloud_},
      {"progress_m", last_tracking_output_.progress_m},
      {"remaining_m", last_tracking_output_.remaining_m},
      {"goal_distance_m", last_tracking_output_.goal_distance_m}};
    const double endpoint_distance_3d = activeTaskEndpointDistance3d();
    status["task_endpoint_distance_3d_m"] = std::isfinite(endpoint_distance_3d) ?
      nlohmann::json(endpoint_distance_3d) : nlohmann::json(nullptr);
    status["efficient_endpoint_reached"] = efficientTaskEndpointReached();
    status["control_phase"] = last_tracking_output_.adjusting ? "ADJUSTMENT" : "FOLLOWING";
    status["collision_level"] = collision_level_;
    if (has_task) {
      const auto & task = route_.tasks[active_task_index_];
      status["controller_mode"] = task.resolved_controller_mode;
      status["gait_command"] = task.gait_command;
      status["obstacle_mode"] = task.obstacle_mode;
    }
    std_msgs::msg::String message;
    message.data = status.dump();
    status_publisher_->publish(message);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = "route3d_pid_controller";
    diagnostic.hardware_id = "generic_twist_output";
    diagnostic.level = state_ == ControllerState::kError ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (safety_stopped_ || state_ == ControllerState::kHandoverRequired ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK);
    diagnostic.message = std::string(stateName(state_)) + ": " + detail;
    diagnostics.status.push_back(std::move(diagnostic));
    diagnostic_publisher_->publish(diagnostics);
  }

  void startCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!has_route_) {
      response->success = false;
      response->message = "no sliced route has been received";
      return;
    }
    if (!has_odometry_) {
      response->success = false;
      response->message = "no odometry has been received";
      return;
    }
    active_task_index_ = 0U;
    const bool started = activateTask(false);
    response->success = started || state_ == ControllerState::kWaitingTransition;
    response->message = state_detail_;
  }

  void continueCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (state_ != ControllerState::kWaitingTransition) {
      response->success = false;
      response->message = "controller is not waiting for a transition acknowledgement";
      return;
    }
    waiting_for_gait_transition_ = false;
    resume_existing_task_after_gait_ = false;
    response->success = activateTask(true);
    response->message = state_detail_;
  }

  void acknowledgeGaitCallback(
    const AcknowledgeGaitTransition::Request::SharedPtr request,
    AcknowledgeGaitTransition::Response::SharedPtr response)
  {
    response->success = acceptGaitAcknowledgement(
      request->route_sequence, request->task_index, request->gait_command,
      response->message);
  }

  void gaitAcknowledgedCallback(const GaitTransition::ConstSharedPtr message)
  {
    std::string detail;
    if (!acceptGaitAcknowledgement(
        message->route_sequence, message->task_index, message->gait_command, detail))
    {
      RCLCPP_DEBUG(get_logger(), "Ignored gait acknowledgement: %s", detail.c_str());
    }
  }

  bool acceptGaitAcknowledgement(
    const std::uint64_t route_sequence, const std::uint32_t task_index,
    const std::string & gait_command, std::string & detail)
  {
    if (state_ != ControllerState::kWaitingTransition || !waiting_for_gait_transition_ ||
      !has_route_ || active_task_index_ >= route_.tasks.size())
    {
      detail = "no gait transition is pending";
      return false;
    }
    const auto & task = route_.tasks[active_task_index_];
    if (route_sequence != route_.route_sequence || task_index != active_task_index_ ||
      gait_command != task.gait_command)
    {
      detail = "gait acknowledgement does not match the active route task";
      return false;
    }
    waiting_for_gait_transition_ = false;
    const bool resume_existing_task = resume_existing_task_after_gait_;
    resume_existing_task_after_gait_ = false;
    bool activated = false;
    if (resume_existing_task) {
      active_ = true;
      paused_ = false;
      safety_stopped_ = false;
      safety_clear_waiting_logged_ = false;
      rotation_footprint_active_ = false;
      safety_clear_gate_->reset();
      publishControllerSelection(task_controller_);
      setState(
        ControllerState::kTracking,
        task_controller_ + " task resumed after gait restore");
      activated = true;
    } else {
      activated = activateTask(true);
    }
    detail = state_detail_;
    return activated;
  }

  void pauseCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!active_) {
      response->success = false;
      response->message = "no active PID task";
      return;
    }
    paused_ = true;
    rotation_footprint_active_ = false;
    tracker_->stopAndResetControllers();
    publishZero();
    publishControllerSelection("none");
    setState(ControllerState::kPaused, "operator pause");
    response->success = true;
    response->message = state_detail_;
  }

  void resumeCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!active_ || !paused_) {
      response->success = false;
      response->message = "controller is not paused";
      return;
    }
    // Pausing releases SDK control through StopMove for remote operation.  As
    // with obstacle recovery, restore the task gait before autonomous output.
    const auto & task = route_.tasks[active_task_index_];
    enterGaitTransition(task, true);
    response->success = true;
    response->message = state_detail_;
  }

  void cancelCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    active_ = false;
    paused_ = false;
    waiting_for_gait_transition_ = false;
    resume_existing_task_after_gait_ = false;
    rotation_footprint_active_ = false;
    if (tracker_) {
      tracker_->stopAndResetControllers();
    }
    publishZero();
    publishControllerSelection("none");
    publishEfficientStop();
    setState(ControllerState::kCancelled, "route cancelled");
    response->success = true;
    response->message = state_detail_;
  }

  bool auto_start_{true};
  double control_rate_hz_{50.0};
  double odometry_timeout_s_{0.30};
  double safety_clear_hold_s_{3.0};
  bool obstacle_enabled_{true};
  std::int64_t emergency_collision_level_threshold_{100};
  bool stop_on_cloud_timeout_{true};
  double cloud_timeout_s_{0.50};
  double cloud_pose_sync_tolerance_s_{0.15};
  double rotation_footprint_forward_scale_{0.50};
  double trajectory_collision_spacing_m_{0.40};
  std::size_t trajectory_collision_samples_{15U};
  double replan_near_spacing_m_{0.25};
  std::size_t replan_near_samples_{25U};
  double replan_far_spacing_m_{0.20};
  std::size_t replan_far_samples_{60U};
  std::size_t base_collision_hold_cycles_{20U};
  std::size_t base_collision_hold_remaining_{0U};

  ControllerState state_{ControllerState::kIdle};
  std::string state_detail_;
  RouteTaskArray route_;
  bool has_route_{false};
  std::size_t active_task_index_{0U};
  bool active_{false};
  bool paused_{false};
  bool waiting_for_gait_transition_{false};
  bool resume_existing_task_after_gait_{false};
  std::string task_controller_{"none"};
  std::string selected_controller_{"none"};
  bool has_odometry_{false};
  bool has_cloud_{false};
  bool cloud_parse_error_{false};
  bool elevation_replan_required_{false};
  bool external_safety_stop_{false};
  std::int32_t collision_level_{0};
  bool safety_stopped_{false};
  bool safety_clear_waiting_logged_{false};
  bool rotation_footprint_active_{false};
  std::size_t trajectory_collision_count_{0U};
  std::size_t checked_trajectory_poses_{0U};
  std::size_t telemetry_tick_{0U};
  std::uint64_t elevation_map_generation_{0U};
  std::uint64_t debug_visualization_generation_{0U};
  std::string frame_id_;
  std::string cloud_frame_id_{"base_link"};
  std::vector<Pose2d> debug_local_trajectory_;
  Pose2d robot_pose_;
  double robot_z_{0.0};
  Pose2d elevation_map_pose_;
  bool has_elevation_map_pose_{false};
  TrackingOutput last_tracking_output_;
  double adjustment_route_goal_position_tolerance_m_{0.08};
  double adjustment_route_goal_yaw_tolerance_rad_{0.15};
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time elevation_map_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_control_time_;
  std::unique_ptr<SafetyClearGate> safety_clear_gate_;
  std::unique_ptr<TimestampedPoseBuffer> odometry_pose_buffer_;
  std::unique_ptr<ElevationCollisionChecker> elevation_checker_;
  std::unique_ptr<RouteTracker> tracker_;

  rclcpp::Subscription<RouteTaskArray>::SharedPtr tasks_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr external_stop_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr collision_level_subscription_;
  rclcpp::Subscription<GaitTransition>::SharedPtr gait_acknowledged_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr efficient_path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_controller_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_publisher_;
  rclcpp::Publisher<GaitTransition>::SharedPtr gait_transition_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    elevation_debug_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr continue_service_;
  rclcpp::Service<AcknowledgeGaitTransition>::SharedPtr acknowledge_gait_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace route3d_pid_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<route3d_pid_controller::PidControllerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route3d_pid_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
