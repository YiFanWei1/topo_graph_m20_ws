#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "recorded_waypoint_route/route_sequencer_core.hpp"
#include "recorded_waypoint_route/planning_profile.hpp"

namespace recorded_waypoint_route
{
namespace
{

rclcpp::QoS routeQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

std::string formatDouble(double value)
{
  if (!std::isfinite(value)) {
    return "inf";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << value;
  return stream.str();
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

class TargetRouteSequencerNode : public rclcpp::Node
{
public:
  TargetRouteSequencerNode()
  : Node("target_route_sequencer")
  {
    odom_topic_ = declare_parameter<std::string>("input.odometry_topic", "/lio_odom_hf");
    goal_topic_ = declare_parameter<std::string>(
      "input.goal_id_topic", "/recorded_waypoint_route/goal_id");
    body_height_ = declare_parameter<double>("route.body_height", 0.40);
    route_spacing_ = declare_parameter<double>("route.spacing", 0.10);
    const double legacy_arrival_tolerance =
      declare_parameter<double>("route.arrival_tolerance", 2.80);
    normal_arrival_tolerance_ = declare_parameter<double>(
      "route.normal_arrival_tolerance", legacy_arrival_tolerance);
    corner_arrival_tolerance_ = declare_parameter<double>(
      "route.corner_arrival_tolerance", 0.20);
    goal_arrival_tolerance_ = declare_parameter<double>(
      "route.goal_arrival_tolerance", 0.15);
    max_start_distance_ = declare_parameter<double>("route.max_start_distance", 3.0);
    update_rate_ = declare_parameter<double>("route.update_rate", 50.0);
    const PlanningProfile normal_profile{
      "normal",
      declare_parameter<double>("profile.normal.path_height", 0.0),
      declare_parameter<bool>("profile.normal.extension_enabled", true)};
    const PlanningProfile slope_profile{
      "slope",
      declare_parameter<double>("profile.slope.path_height", 0.40),
      declare_parameter<bool>("profile.slope.extension_enabled", false)};
    profile_resolver_ = std::make_unique<PlanningProfileResolver>(
      normal_profile, slope_profile);
    planner_node_name_ = declare_parameter<std::string>(
      "profile.planner_node", "/corridor_astar_planner");
    extension_node_name_ = declare_parameter<std::string>(
      "profile.extension_node", "/obstacle_occlusion_extension");
    if (!std::isfinite(body_height_) || body_height_ < 0.0) {
      throw std::invalid_argument("route.body_height must be finite and non-negative");
    }
    if (!std::isfinite(route_spacing_) || route_spacing_ <= 0.0) {
      throw std::invalid_argument("route.spacing must be finite and positive");
    }
    if (!validTolerance(normal_arrival_tolerance_) ||
      !validTolerance(corner_arrival_tolerance_) ||
      !validTolerance(goal_arrival_tolerance_))
    {
      throw std::invalid_argument("route arrival tolerances must be finite and positive");
    }
    if (!std::isfinite(max_start_distance_) || max_start_distance_ <= 0.0) {
      throw std::invalid_argument("route.max_start_distance must be finite and positive");
    }
    if (!std::isfinite(update_rate_) || update_rate_ <= 0.0) {
      throw std::invalid_argument("route.update_rate must be finite and positive");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    planner_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, planner_node_name_);
    extension_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, extension_node_name_);

    segment_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/recorded_waypoint_route/active_segment_ground", routeQos());
    selected_route_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/recorded_waypoint_route/selected_route_ground", routeQos());
    target_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/recorded_waypoint_route/active_target", routeQos());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/recorded_waypoint_route/diagnostics", rclcpp::QoS(10));

    targets_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/recorded_waypoint_route/targets_ground", routeQos(),
      [this](geometry_msgs::msg::PoseArray::SharedPtr message) {
        targets_message_ = std::move(message);
        tryInitializeRoute();
      });
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      "/recorded_waypoint_route/full_path_ground", routeQos(),
      [this](nav_msgs::msg::Path::SharedPtr message) {
        path_message_ = std::move(message);
        tryInitializeRoute();
      });
    types_subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/recorded_waypoint_route/target_types", routeQos(),
      [this](std_msgs::msg::UInt8MultiArray::SharedPtr message) {
        types_message_ = std::move(message);
        tryInitializeRoute();
      });
    slope_flags_subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/recorded_waypoint_route/target_slope_flags", routeQos(),
      [this](std_msgs::msg::UInt8MultiArray::SharedPtr message) {
        slope_flags_message_ = std::move(message);
        tryInitializeRoute();
      });

    // 高频里程计回调只保存最新消息，不做 TF、三维距离或路线状态计算。
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = std::move(message);
        ++odom_generation_;
      });
    goal_subscription_ = create_subscription<std_msgs::msg::Int32>(
      goal_topic_, rclcpp::QoS(10),
      std::bind(&TargetRouteSequencerNode::goalCallback, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration<double>(1.0 / update_rate_);
    update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&TargetRouteSequencerNode::updateTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "route sequencer ready: implementation=cpp odom_cache=true update_rate=%.1fHz "
      "goal_topic=%s odom=%s normal/corner/goal=%.3f/%.3f/%.3fm "
      "profile_nodes=[%s,%s]",
      update_rate_, goal_topic_.c_str(), odom_topic_.c_str(),
      normal_arrival_tolerance_, corner_arrival_tolerance_, goal_arrival_tolerance_,
      planner_node_name_.c_str(), extension_node_name_.c_str());
  }

private:
  using RouteSignature = std::tuple<int32_t, uint32_t, std::size_t, std::size_t>;

  struct ProfileTransition
  {
    std::uint64_t revision{0U};
    std::size_t target_index{0U};
    PlanningProfile profile;
    bool planner_request_sent{false};
    bool extension_request_sent{false};
    bool planner_confirmed{false};
    bool extension_confirmed{false};
  };

  static bool validTolerance(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  static std::string targetTypeName(TargetType type)
  {
    return type == TargetType::Corner ? "corner" : "normal";
  }

  static Point3 pointFromPose(const geometry_msgs::msg::Point & point)
  {
    return {point.x, point.y, point.z};
  }

  void tryInitializeRoute()
  {
    if (!targets_message_ || !path_message_ || !types_message_ || !slope_flags_message_) {
      return;
    }
    if (targets_message_->header.frame_id != path_message_->header.frame_id) {
      RCLCPP_ERROR(get_logger(), "target and interpolated path frames do not match");
      return;
    }
    const auto & target_stamp = targets_message_->header.stamp;
    const auto & path_stamp = path_message_->header.stamp;
    if (target_stamp.sec != path_stamp.sec || target_stamp.nanosec != path_stamp.nanosec) {
      return;
    }
    if (types_message_->data.size() != targets_message_->poses.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "target type count %zu does not match target count %zu",
        types_message_->data.size(), targets_message_->poses.size());
      return;
    }
    if (slope_flags_message_->data.size() != targets_message_->poses.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "target slope flag count %zu does not match target count %zu",
        slope_flags_message_->data.size(), targets_message_->poses.size());
      return;
    }
    const RouteSignature signature{
      target_stamp.sec, target_stamp.nanosec,
      targets_message_->poses.size(), path_message_->poses.size()};
    if (route_signature_ && *route_signature_ == signature) {
      return;
    }

    std::vector<Point3> targets;
    targets.reserve(targets_message_->poses.size());
    for (const auto & pose : targets_message_->poses) {
      targets.push_back(pointFromPose(pose.position));
    }
    std::vector<Point3> path;
    path.reserve(path_message_->poses.size());
    for (const auto & pose : path_message_->poses) {
      path.push_back(pointFromPose(pose.pose.position));
    }
    try {
      segments_ = splitInterpolatedPath(targets, path);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "route rejected: %s", error.what());
      return;
    }
    targets_ground_ = std::move(targets);
    target_types_.clear();
    target_types_.reserve(types_message_->data.size());
    for (const std::uint8_t value : types_message_->data) {
      if (value > static_cast<std::uint8_t>(TargetType::Corner)) {
        RCLCPP_ERROR(
          get_logger(), "route rejected: unknown target type value %u",
          static_cast<unsigned int>(value));
        return;
      }
      target_types_.push_back(static_cast<TargetType>(value));
    }
    target_is_slope_.clear();
    target_is_slope_.reserve(slope_flags_message_->data.size());
    for (const std::uint8_t value : slope_flags_message_->data) {
      if (value > 1U) {
        RCLCPP_ERROR(
          get_logger(), "route rejected: slope flag must be 0 or 1, got %u",
          static_cast<unsigned int>(value));
        return;
      }
      target_is_slope_.push_back(value != 0U);
    }
    targets_body_ = targets_ground_;
    for (Point3 & point : targets_body_) {
      point.z += body_height_;
    }
    progress_.reset();
    selected_sequence_.clear();
    planner_search_floor_position_ = 0;
    planner_segment_endpoints_.reset();
    profile_transition_.reset();
    applied_profile_.reset();
    route_signature_ = signature;
    status_state_ = latest_body_position_ ? "waiting_goal" : "waiting_localization";
    last_event_ = "route_initialized";
  }

  std::optional<Point3> odomInRouteFrame(const nav_msgs::msg::Odometry & message)
  {
    if (!targets_message_) {
      return std::nullopt;
    }
    const std::string & route_frame = targets_message_->header.frame_id;
    const std::string source_frame = message.header.frame_id.empty() ?
      route_frame : message.header.frame_id;
    geometry_msgs::msg::PoseStamped source;
    source.header = message.header;
    source.header.frame_id = source_frame;
    source.pose = message.pose.pose;
    if (source_frame != route_frame) {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          route_frame, source_frame, tf2::TimePointZero);
        geometry_msgs::msg::PoseStamped transformed;
        tf2::doTransform(source, transformed, transform);
        source = std::move(transformed);
      } catch (const tf2::TransformException & error) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "cannot transform odometry %s -> %s: %s",
          source_frame.c_str(), route_frame.c_str(), error.what());
        return std::nullopt;
      }
    }
    const Point3 position = pointFromPose(source.pose.position);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
      return std::nullopt;
    }
    return position;
  }

  nav_msgs::msg::Path makePath(const std::vector<Point3> & points)
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = targets_message_->header.frame_id;
    message.header.stamp = now();
    message.poses.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = message.header;
      stamped.pose.position.x = points[index].x;
      stamped.pose.position.y = points[index].y;
      stamped.pose.position.z = points[index].z;
      const std::size_t previous = index == 0 ? 0 : index - 1;
      const std::size_t following = std::min(points.size() - 1, index + 1);
      const double yaw = std::atan2(
        points[following].y - points[previous].y,
        points[following].x - points[previous].x);
      stamped.pose.orientation.z = std::sin(0.5 * yaw);
      stamped.pose.orientation.w = std::cos(0.5 * yaw);
      message.poses.push_back(std::move(stamped));
    }
    return message;
  }

  std::vector<Point3> recordedLegPoints(std::size_t from_index, std::size_t to_index) const
  {
    if (to_index == from_index + 1) {
      return segments_.at(from_index);
    }
    if (from_index == to_index + 1) {
      const auto & segment = segments_.at(to_index);
      return std::vector<Point3>(segment.rbegin(), segment.rend());
    }
    throw std::invalid_argument("active route leg must connect adjacent targets");
  }

  void publishEmptySegment()
  {
    planner_segment_endpoints_.reset();
    segment_publisher_->publish(makePath({}));
  }

  std::size_t plannerWindowStartPosition(const Point3 & body_position)
  {
    const std::size_t current_position = progress_->sequencePosition();
    if (current_position == 0U) {
      return 0U;
    }
    const std::size_t search_begin = std::min(
      planner_search_floor_position_, current_position);
    std::size_t nearest_position = search_begin;
    double nearest_distance = distance3d(
      body_position, targets_body_.at(selected_sequence_.at(search_begin)));
    for (std::size_t position = search_begin + 1U;
      position <= current_position; ++position)
    {
      const double candidate_distance = distance3d(
        body_position, targets_body_.at(selected_sequence_.at(position)));
      if (candidate_distance < nearest_distance) {
        nearest_position = position;
        nearest_distance = candidate_distance;
      }
    }
    planner_search_floor_position_ = nearest_position;
    return nearest_position == 0U ? 0U : nearest_position - 1U;
  }

  PlanningProfile profileForPlannerWindow(const Point3 & body_position)
  {
    const std::size_t current_position = progress_->sequencePosition();
    const std::size_t start_position = plannerWindowStartPosition(body_position);
    bool window_is_slope = false;
    for (std::size_t position = start_position; position <= current_position; ++position) {
      if (target_is_slope_.at(selected_sequence_.at(position))) {
        window_is_slope = true;
        break;
      }
    }
    return profile_resolver_->resolve({progress_->requiredTargetType(), window_is_slope});
  }

  void handleProfileResponse(
    std::uint64_t revision, bool planner, bool success, const std::string & reason)
  {
    if (!profile_transition_ || profile_transition_->revision != revision) {
      return;
    }
    bool & sent = planner ? profile_transition_->planner_request_sent :
      profile_transition_->extension_request_sent;
    bool & confirmed = planner ? profile_transition_->planner_confirmed :
      profile_transition_->extension_confirmed;
    if (!success) {
      sent = false;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "planning profile revision=%lu rejected by %s: %s; retrying",
        static_cast<unsigned long>(revision), planner ? "planner" : "extension",
        reason.c_str());
      return;
    }
    confirmed = true;
    finishProfileTransitionIfReady();
  }

  void dispatchProfileRequests()
  {
    if (!profile_transition_) {
      return;
    }
    const std::uint64_t revision = profile_transition_->revision;
    if (!profile_transition_->planner_request_sent &&
      planner_parameter_client_->service_is_ready())
    {
      profile_transition_->planner_request_sent = true;
      const double path_height = profile_transition_->profile.path_height;
      planner_parameter_client_->set_parameters(
        {rclcpp::Parameter("planner.path_height", path_height)},
        [this, revision](auto future) {
          try {
            const auto results = future.get();
            const bool success = results.size() == 1U && results.front().successful;
            handleProfileResponse(
              revision, true, success,
              results.empty() ? "empty parameter response" : results.front().reason);
          } catch (const std::exception & error) {
            handleProfileResponse(revision, true, false, error.what());
          }
        });
    }
    if (!profile_transition_->extension_request_sent &&
      extension_parameter_client_->service_is_ready())
    {
      profile_transition_->extension_request_sent = true;
      const bool enabled = profile_transition_->profile.extension_enabled;
      extension_parameter_client_->set_parameters(
        {rclcpp::Parameter("extension.enabled", enabled)},
        [this, revision](auto future) {
          try {
            const auto results = future.get();
            const bool success = results.size() == 1U && results.front().successful;
            handleProfileResponse(
              revision, false, success,
              results.empty() ? "empty parameter response" : results.front().reason);
          } catch (const std::exception & error) {
            handleProfileResponse(revision, false, false, error.what());
          }
        });
    }
  }

  void finishProfileTransitionIfReady()
  {
    if (!profile_transition_ || !profile_transition_->planner_confirmed ||
      !profile_transition_->extension_confirmed || !latest_body_position_)
    {
      return;
    }
    const PlanningProfile applied = profile_transition_->profile;
    const std::size_t target_index = profile_transition_->target_index;
    const std::uint64_t revision = profile_transition_->revision;
    applied_profile_ = applied;
    profile_transition_.reset();
    status_state_ = "tracking";
    last_event_ = "profile_applied";
    RCLCPP_INFO(
      get_logger(),
      "planning profile applied: revision=%lu target=%zu profile=%s "
      "planner.path_height=%.2f extension.enabled=%s; rechecking planner window",
      static_cast<unsigned long>(revision), target_index + 1U, applied.name.c_str(),
      applied.path_height, applied.extension_enabled ? "true" : "false");
    // Re-resolve against the latest robot projection. If the robot crossed a
    // profile boundary while services were responding, start the newer profile
    // transaction instead of publishing one segment with stale settings.
    requestPlannerWindow(*latest_body_position_);
    publishDiagnostics(
      distance3d(*latest_body_position_, targets_body_.at(progress_->requiredTarget())),
      last_event_, true);
  }

  void requestPlannerWindow(const Point3 & body_position)
  {
    const std::size_t target_index = progress_->requiredTarget();
    const PlanningProfile desired = profileForPlannerWindow(body_position);
    if (applied_profile_ && *applied_profile_ == desired) {
      publishPlannerWindow(body_position);
      return;
    }
    // Empty segment is the explicit stop/cancel command. No new segment is
    // exposed until both remote parameter services confirm this same revision.
    publishEmptySegment();
    profile_transition_ = ProfileTransition{
      ++profile_revision_, target_index, desired, false, false, false, false};
    status_state_ = "applying_profile";
    last_event_ = "profile_change_requested";
    RCLCPP_INFO(
      get_logger(),
      "planning profile requested: revision=%lu target=%zu window_has_slope=%s profile=%s "
      "planner.path_height=%.2f extension.enabled=%s",
      static_cast<unsigned long>(profile_revision_), target_index + 1U,
      desired.name == "slope" ? "true" : "false", desired.name.c_str(),
      desired.path_height, desired.extension_enabled ? "true" : "false");
    dispatchProfileRequests();
  }

  void publishEmptySelectedRoute()
  {
    selected_route_publisher_->publish(makePath({}));
  }

  void publishPlannerWindow(const Point3 & body_position)
  {
    if (!progress_ || selected_sequence_.empty()) {
      throw std::logic_error("planner window requested without an active selected route");
    }
    const std::size_t current_position = progress_->sequencePosition();
    if (current_position >= selected_sequence_.size()) {
      throw std::logic_error("planner target position is outside the selected route");
    }

    std::vector<Point3> points;
    std::size_t start_position = current_position;
    if (current_position == 0) {
      const std::size_t target_index = selected_sequence_.front();
      const Point3 start_ground{
        body_position.x, body_position.y, body_position.z - body_height_};
      if (distance3d(start_ground, targets_ground_.at(target_index)) > 1e-7) {
        points = interpolateTargets(
          {start_ground, targets_ground_.at(target_index)}, route_spacing_);
      } else {
        points.push_back(targets_ground_.at(target_index));
      }
    } else {
      // The 2.8 m NORMAL radius can advance across several 1 m targets before
      // the robot moves. Keep one target behind the robot projection so route
      // geometry and profile resolution use the exact same rolling window.
      start_position = plannerWindowStartPosition(body_position);
      for (std::size_t position = start_position; position < current_position; ++position) {
        const auto leg = recordedLegPoints(
          selected_sequence_.at(position), selected_sequence_.at(position + 1));
        if (points.empty()) {
          points = leg;
        } else {
          points.insert(points.end(), std::next(leg.begin()), leg.end());
        }
      }
    }

    const std::size_t first_target = selected_sequence_.at(start_position);
    const std::size_t last_target = selected_sequence_.at(current_position);
    planner_segment_endpoints_ = std::make_pair(first_target, last_target);
    segment_publisher_->publish(makePath(points));
    RCLCPP_INFO(
      get_logger(),
      "planner window published: targets=%zu->%zu geometry_points=%zu target_role=%s",
      first_target + 1, last_target + 1, points.size(),
      progress_->requiredTargetIsGoal() ? "final_goal" :
      (progress_->requiredTargetType() == TargetType::Corner ? "corner" : "normal"));
  }

  void publishSelectedRoute(
    const Point3 & body_position, const std::vector<std::size_t> & sequence,
    double nearest_distance)
  {
    std::vector<Point3> points;
    const std::size_t nearest_index = sequence.front();
    const double capture_tolerance = progress_ ?
      progress_->effectiveTolerance() : normal_arrival_tolerance_;
    if (nearest_distance > capture_tolerance) {
      const Point3 start_ground{
        body_position.x, body_position.y, body_position.z - body_height_};
      points = interpolateTargets(
        {start_ground, targets_ground_.at(nearest_index)}, route_spacing_);
    } else {
      points.push_back(targets_ground_.at(nearest_index));
    }
    for (std::size_t index = 0; index + 1 < sequence.size(); ++index) {
      const auto leg = recordedLegPoints(sequence[index], sequence[index + 1]);
      points.insert(points.end(), std::next(leg.begin()), leg.end());
    }
    selected_route_publisher_->publish(makePath(points));
  }

  void publishActiveTarget(std::size_t target_index)
  {
    const Point3 & point = targets_body_.at(target_index);
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = targets_message_->header.frame_id;
    message.header.stamp = now();
    message.pose.position.x = point.x;
    message.pose.position.y = point.y;
    message.pose.position.z = point.z;
    message.pose.orientation.w = 1.0;
    target_publisher_->publish(message);
  }

  void goalCallback(const std_msgs::msg::Int32::SharedPtr message)
  {
    const int goal_id = message->data;
    if (!targets_body_.empty() &&
      (goal_id < 1 || goal_id > static_cast<int>(targets_body_.size())))
    {
      RCLCPP_ERROR(
        get_logger(), "goal id %d rejected: valid target ids are 1..%zu",
        goal_id, targets_body_.size());
      last_event_ = "invalid_goal_id";
      publishDiagnostics(std::numeric_limits<double>::infinity(), last_event_, true);
      return;
    }
    pending_goal_id_ = goal_id;
    if (targets_body_.empty()) {
      status_state_ = "waiting_route";
      last_event_ = "goal_deferred_waiting_route";
    } else if (!latest_body_position_) {
      status_state_ = "waiting_localization";
      last_event_ = "goal_deferred_waiting_localization";
    }
    // 目标激活也留给 50 Hz 定时器，避免订阅回调执行路线计算。
  }

  void tryActivatePendingGoal()
  {
    if (!pending_goal_id_ || targets_body_.empty() || !latest_body_position_) {
      return;
    }
    const int goal_id = *pending_goal_id_;
    pending_goal_id_.reset();
    if (goal_id < 1 || goal_id > static_cast<int>(targets_body_.size())) {
      RCLCPP_ERROR(
        get_logger(), "goal id %d rejected: valid target ids are 1..%zu",
        goal_id, targets_body_.size());
      last_event_ = "invalid_goal_id";
      publishDiagnostics(std::numeric_limits<double>::infinity(), last_event_, true);
      return;
    }

    const auto nearest = nearestTarget(*latest_body_position_, targets_body_);
    nearest_target_id_ = nearest.first + 1;
    nearest_target_distance_ = nearest.second;
    if (nearest.second > max_start_distance_) {
      progress_.reset();
      selected_sequence_.clear();
      planner_search_floor_position_ = 0;
      selected_goal_id_.reset();
      status_state_ = "rejected_too_far";
      last_event_ = "nearest_target_too_far";
      publishEmptySegment();
      publishEmptySelectedRoute();
      publishDiagnostics(nearest.second, last_event_, true);
      return;
    }

    const auto sequence = orderedTargetIndices(
      nearest.first, static_cast<std::size_t>(goal_id - 1), targets_body_.size());
    progress_ = std::make_unique<RouteProgress>(
      sequence, target_types_, normal_arrival_tolerance_,
      corner_arrival_tolerance_, goal_arrival_tolerance_);
    selected_sequence_ = sequence;
    planner_search_floor_position_ = 0;
    selected_goal_id_ = goal_id;
    status_state_ = "tracking";
    last_event_ = "goal_accepted";
    publishActiveTarget(nearest.first);
    publishSelectedRoute(*latest_body_position_, sequence, nearest.second);
    requestPlannerWindow(*latest_body_position_);
    if (profile_transition_) {
      publishDiagnostics(nearest.second, last_event_, true);
      return;
    }
    if (nearest.second > progress_->effectiveTolerance()) {
      publishDiagnostics(nearest.second, last_event_, true);
      return;
    }
    advanceProgress(*latest_body_position_);
  }

  void advanceProgress(const Point3 & body_position)
  {
    const ProgressUpdate update = progress_->update(body_position, targets_body_);
    status_state_ = update.state == RouteState::Complete ? "complete" : "tracking";
    last_event_ = update.event;
    if (update.changed) {
      if (std::string(update.event) == "goal_reached") {
        publishEmptySegment();
      } else if (update.active_leg) {
        publishActiveTarget(update.active_leg->second);
        requestPlannerWindow(body_position);
      }
    }
    const double target_distance = distance3d(
      body_position, targets_body_.at(progress_->requiredTarget()));
    publishDiagnostics(target_distance, update.event, update.changed);
  }

  void publishDiagnostics(double distance, const std::string & event, bool force = false)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!force && steady_now - last_diagnostic_time_ < std::chrono::milliseconds(200)) {
      return;
    }
    last_diagnostic_time_ = steady_now;
    std::string active_leg = progress_ ? "approach" : "none";
    std::string planner_segment = "none";
    std::string required = "none";
    std::string required_type = "none";
    std::string required_role = "none";
    std::string required_is_slope = "none";
    double effective_tolerance = std::numeric_limits<double>::infinity();
    if (progress_) {
      required = std::to_string(progress_->requiredTarget() + 1);
      required_type = targetTypeName(progress_->requiredTargetType());
      required_role = progress_->requiredTargetIsGoal() ? "final_goal" : required_type;
      required_is_slope = target_is_slope_.at(progress_->requiredTarget()) ? "true" : "false";
      effective_tolerance = progress_->effectiveTolerance();
      if (progress_->activeLeg()) {
        active_leg = std::to_string(progress_->activeLeg()->first + 1) + "->" +
          std::to_string(progress_->activeLeg()->second + 1);
      }
      if (planner_segment_endpoints_) {
        planner_segment = std::to_string(planner_segment_endpoints_->first + 1) + "->" +
          std::to_string(planner_segment_endpoints_->second + 1);
      }
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = status_state_.rfind("rejected", 0) == 0 ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "recorded_waypoint_route/sequencer";
    status.hardware_id = "route";
    status.message = status_state_;
    status.values = {
      keyValue("state", status_state_),
      keyValue("event", event),
      keyValue("active_leg", active_leg),
      keyValue("planner_segment", planner_segment),
      keyValue("required_target_id", required),
      keyValue("required_target_type", required_type),
      keyValue("required_target_role", required_role),
      keyValue("required_target_is_slope", required_is_slope),
      keyValue("planning_profile", applied_profile_ ? applied_profile_->name : "none"),
      keyValue("profile_transition_pending", profile_transition_ ? "true" : "false"),
      keyValue("selected_goal_id", selected_goal_id_ ? std::to_string(*selected_goal_id_) : "none"),
      keyValue("nearest_target_id", nearest_target_id_ ? std::to_string(*nearest_target_id_) : "none"),
      keyValue("nearest_target_distance_3d", formatDouble(nearest_target_distance_)),
      keyValue("target_distance_3d", formatDouble(distance)),
      keyValue("target_count", std::to_string(targets_body_.size())),
      keyValue("max_start_distance", formatDouble(max_start_distance_)),
      keyValue("effective_arrival_tolerance", formatDouble(effective_tolerance)),
      keyValue("normal_arrival_tolerance", formatDouble(normal_arrival_tolerance_)),
      keyValue("corner_arrival_tolerance", formatDouble(corner_arrival_tolerance_)),
      keyValue("goal_arrival_tolerance", formatDouble(goal_arrival_tolerance_)),
      keyValue("update_rate_hz", formatDouble(update_rate_)),
      keyValue("implementation", "cpp")};
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
  }

  void updateTimer()
  {
    dispatchProfileRequests();
    nav_msgs::msg::Odometry::SharedPtr odometry;
    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odometry = latest_odom_;
      generation = odom_generation_;
    }
    if (!odometry) {
      publishDiagnostics(std::numeric_limits<double>::infinity(), "waiting_localization");
      return;
    }

    // 没有新定位且没有新目标时不重复做 TF 和三维到点判断。
    if (generation == processed_odom_generation_ && !pending_goal_id_) {
      return;
    }
    processed_odom_generation_ = generation;
    if (targets_body_.empty()) {
      publishDiagnostics(std::numeric_limits<double>::infinity(), "waiting_route");
      return;
    }
    const auto position = odomInRouteFrame(*odometry);
    if (!position) {
      return;
    }
    latest_body_position_ = *position;
    if (profile_transition_) {
      finishProfileTransitionIfReady();
      if (profile_transition_) {
        publishDiagnostics(
          distance3d(*latest_body_position_, targets_body_.at(progress_->requiredTarget())),
          "waiting_profile_confirmation");
        return;
      }
    }
    if (pending_goal_id_) {
      tryActivatePendingGoal();
      return;
    }
    if (!progress_) {
      if (status_state_ != "rejected_too_far" && status_state_ != "waiting_goal") {
        status_state_ = "waiting_goal";
      }
      publishDiagnostics(std::numeric_limits<double>::infinity(), last_event_);
      return;
    }
    advanceProgress(*latest_body_position_);
  }

  std::string odom_topic_;
  std::string goal_topic_;
  double body_height_{0.40};
  double route_spacing_{0.10};
  double normal_arrival_tolerance_{2.80};
  double corner_arrival_tolerance_{0.20};
  double goal_arrival_tolerance_{0.15};
  double max_start_distance_{3.0};
  double update_rate_{50.0};
  std::string planner_node_name_;
  std::string extension_node_name_;
  std::unique_ptr<PlanningProfileResolver> profile_resolver_;
  std::shared_ptr<rclcpp::AsyncParametersClient> planner_parameter_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> extension_parameter_client_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr segment_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_route_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr targets_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr types_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr slope_flags_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_subscription_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  geometry_msgs::msg::PoseArray::SharedPtr targets_message_;
  nav_msgs::msg::Path::SharedPtr path_message_;
  std_msgs::msg::UInt8MultiArray::SharedPtr types_message_;
  std_msgs::msg::UInt8MultiArray::SharedPtr slope_flags_message_;
  std::vector<Point3> targets_ground_;
  std::vector<Point3> targets_body_;
  std::vector<TargetType> target_types_;
  std::vector<bool> target_is_slope_;
  std::vector<std::vector<Point3>> segments_;
  std::vector<std::size_t> selected_sequence_;
  std::size_t planner_search_floor_position_{0};
  std::optional<std::pair<std::size_t, std::size_t>> planner_segment_endpoints_;
  std::optional<RouteSignature> route_signature_;
  std::unique_ptr<RouteProgress> progress_;
  std::optional<PlanningProfile> applied_profile_;
  std::optional<ProfileTransition> profile_transition_;
  std::uint64_t profile_revision_{0U};
  std::optional<Point3> latest_body_position_;
  std::optional<int> pending_goal_id_;
  std::optional<int> selected_goal_id_;
  std::optional<std::size_t> nearest_target_id_;
  double nearest_target_distance_{std::numeric_limits<double>::infinity()};
  std::string status_state_{"waiting_route"};
  std::string last_event_{"waiting_route"};
  std::chrono::steady_clock::time_point last_diagnostic_time_{};

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::uint64_t odom_generation_{0};
  std::uint64_t processed_odom_generation_{0};
};

}  // namespace recorded_waypoint_route

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<recorded_waypoint_route::TargetRouteSequencerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("target_route_sequencer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
