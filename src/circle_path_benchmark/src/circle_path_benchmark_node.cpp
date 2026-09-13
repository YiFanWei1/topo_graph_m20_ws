#include "circle_path_benchmark/circle_path.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace circle_path_benchmark
{

class CirclePathBenchmarkNode : public rclcpp::Node
{
public:
  CirclePathBenchmarkNode() : Node("circle_path_benchmark")
  {
    path_file_ = declare_parameter<std::string>("benchmark.path_file", "");
    pcd_override_ = declare_parameter<std::string>("benchmark.pcd_file", "");
    output_directory_ = declare_parameter<std::string>("benchmark.output_directory", "");
    odom_topic_ = declare_parameter<std::string>("topics.odometry", "/lio_odom_hf");
    cmd_vel_topic_ = declare_parameter<std::string>("topics.cmd_vel", "/cmd_vel_smoothed");
    control_path_topic_ = declare_parameter<std::string>(
      "topics.control_path", "/local_planner/local_path");
    initial_enable_ = declare_parameter<bool>("benchmark.enable_motion", false);
    center_from_initial_odometry_ = declare_parameter<bool>(
      "benchmark.center_from_initial_odometry", true);
    path_height_offset_ = declare_parameter<double>("benchmark.path_height_offset", 0.57);
    require_pcd_ = declare_parameter<bool>("benchmark.require_pcd", true);
    publish_rate_ = declare_parameter<double>("benchmark.publish_rate", 5.0);
    robot_radius_ = declare_parameter<double>("safety.robot_radius", 0.30);
    body_half_height_ = declare_parameter<double>("safety.body_half_height", 0.35);
    allow_unsafe_circle_ = declare_parameter<bool>("safety.allow_unsafe_circle", false);
    check_pcd_clearance_ = declare_parameter<bool>("safety.check_pcd_clearance", false);
    require_start_pose_ = declare_parameter<bool>("safety.require_start_pose", false);
    start_position_tolerance_ = declare_parameter<double>(
      "safety.start_position_tolerance", 0.20);
    start_yaw_tolerance_ = declare_parameter<double>(
      "safety.start_yaw_tolerance_degrees", 20.0) * kPi / 180.0;
    completion_angle_tolerance_ = declare_parameter<double>(
      "safety.completion_angle_tolerance_degrees", 8.0) * kPi / 180.0;
    completion_position_tolerance_ = declare_parameter<double>(
      "safety.completion_position_tolerance", 0.20);
    max_duration_ = declare_parameter<double>("safety.max_duration", 45.0);
    actual_path_spacing_ = declare_parameter<double>("record.actual_path_spacing", 0.01);

    if (path_file_.empty() || output_directory_.empty() || publish_rate_ <= 0.0 ||
      start_position_tolerance_ <= 0.0 || start_yaw_tolerance_ <= 0.0 ||
      max_duration_ <= 0.0 || actual_path_spacing_ <= 0.0 || robot_radius_ <= 0.0 ||
      body_half_height_ <= 0.0 || !std::isfinite(path_height_offset_))
    {
      throw std::invalid_argument("benchmark paths and positive safety parameters are required");
    }
    loadPath(path_file_);
    loadCloud(pcd_override_.empty() ? source_pcd_ : pcd_override_);
    openOutputs();
    path_ready_ = !center_from_initial_odometry_;
    if (path_ready_) {
      updateCircleSafety();
      writeResolvedPath();
    }

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    reference_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/circle_benchmark/reference_path", latched_qos);
    control_path_pub_ = create_publisher<nav_msgs::msg::Path>(control_path_topic_, latched_qos);
    actual_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/circle_benchmark/actual_path", latched_qos);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/circle_benchmark/map", latched_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/circle_benchmark/diagnostics", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS().keep_last(20),
      std::bind(&CirclePathBenchmarkNode::odomCallback, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(20).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {latest_command_ = *message;});
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "/circle_benchmark/enable",
      std::bind(
        &CirclePathBenchmarkNode::enableCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_)),
      std::bind(&CirclePathBenchmarkNode::timerCallback, this));

    auto_arm_pending_ = initial_enable_;
    if (path_ready_) {publishReference();}
    publishCloud();
    publishEmptyControlPath("startup_disarmed");
    if (center_from_initial_odometry_) {
      RCLCPP_WARN(
        get_logger(),
        "waiting for first odometry: its position will be locked as the %.2fm circle center; "
        "motion=%s output=%s",
        spec_.radius, initial_enable_ ? "waiting_for_start_check" : "DISARMED",
        output_directory_.c_str());
    } else {
      logCircleReady();
    }
  }

  ~CirclePathBenchmarkNode() override
  {
    publishEmptyControlPath("node_shutdown");
    writeMetrics(completed_ ? "one_lap_complete" : "node_shutdown");
  }

private:
  static diagnostic_msgs::msg::KeyValue keyValue(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = value;
    return result;
  }

  static std::string number(double value, int precision = 4)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  void loadPath(const std::string & file)
  {
    const YAML::Node document = YAML::LoadFile(file);
    frame_id_ = document["frame_id"].as<std::string>();
    source_pcd_ = document["source_pcd"].as<std::string>();
    const auto circle = document["circle"];
    const auto center = circle["center"];
    if (!center || center.size() != 3U || frame_id_.empty()) {
      throw std::runtime_error("circle path YAML has invalid frame or center");
    }
    spec_.center_x = center[0].as<double>();
    spec_.center_y = center[1].as<double>();
    spec_.path_z = center[2].as<double>();
    spec_.radius = circle["radius"].as<double>();
    spec_.sample_spacing = circle["sample_spacing"].as<double>();
    spec_.start_angle = circle["start_angle"].as<double>();
    const std::string direction = circle["direction"].as<std::string>();
    if (direction != "ccw" && direction != "cw") {
      throw std::runtime_error("circle direction must be ccw or cw");
    }
    spec_.clockwise = direction == "cw";
    validateCircleSpec(spec_);
    points_.clear();
    for (const auto & entry : document["points"]) {
      if (!entry.IsSequence() || entry.size() != 4U) {
        throw std::runtime_error("every fixed path point must be [x,y,z,yaw]");
      }
      points_.push_back(CirclePoint{
        entry[0].as<double>(), entry[1].as<double>(),
        entry[2].as<double>(), entry[3].as<double>()});
    }
    if (points_.size() < 9U ||
      std::hypot(points_.front().x - points_.back().x,
      points_.front().y - points_.back().y) > 1e-5)
    {
      throw std::runtime_error("fixed path must contain exactly one closed circle");
    }
  }

  void loadCloud(const std::string & file)
  {
    if (file.empty()) {
      if (require_pcd_) {throw std::runtime_error("PCD path is empty");}
      return;
    }
    if (pcl::io::loadPCDFile(file, cloud_) != 0 || cloud_.empty()) {
      if (require_pcd_) {throw std::runtime_error("failed to load PCD: " + file);}
      RCLCPP_WARN(get_logger(), "PCD unavailable; reference path will still be published");
    }
  }

  void openOutputs()
  {
    std::filesystem::create_directories(output_directory_);
    trajectory_.open(std::filesystem::path(output_directory_) / "trajectory.csv");
    if (!trajectory_) {throw std::runtime_error("failed to open trajectory.csv");}
    trajectory_ << "stamp_sec,x,y,z,yaw,active,progress_rad,radial_error_m,"
                   "abs_radial_error_m,cmd_vx,cmd_vy,cmd_wz\n";
    trajectory_.flush();
  }

  static std::string yamlQuote(const std::string & value)
  {
    std::string escaped;
    escaped.reserve(value.size() + 2U);
    escaped.push_back('"');
    for (const char character : value) {
      if (character == '\\' || character == '"') {escaped.push_back('\\');}
      escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
  }

  void writeResolvedPath()
  {
    const auto output = std::filesystem::path(output_directory_) / "reference_circle.yaml";
    const auto temporary = output.string() + ".tmp";
    std::ofstream stream(temporary);
    if (!stream) {throw std::runtime_error("failed to write resolved circle path");}
    stream << std::setprecision(10)
           << "version: 1\n"
           << "frame_id: " << yamlQuote(frame_id_) << "\n"
           << "source_pcd: " << yamlQuote(
      pcd_override_.empty() ? source_pcd_ : pcd_override_) << "\n"
           << "center_source: " << (
      center_from_initial_odometry_ ? "initial_odometry" : "path_file") << "\n"
           << "path_height_offset: " << path_height_offset_ << "\n"
           << "circle:\n"
           << "  center: [" << spec_.center_x << ", " << spec_.center_y << ", "
           << spec_.path_z << "]\n"
           << "  radius: " << spec_.radius << "\n"
           << "  sample_spacing: " << spec_.sample_spacing << "\n"
           << "  start_angle: " << spec_.start_angle << "\n"
           << "  direction: " << (spec_.clockwise ? "cw" : "ccw") << "\n"
           << "validation:\n"
           << "  robot_radius: " << robot_radius_ << "\n"
           << "  body_half_height: " << body_half_height_ << "\n"
           << "  pcd_slab_points: " << pcd_slab_points_ << "\n"
           << "  minimum_clearance: " << minimum_clearance_ << "\n"
           << "  inside_pcd_bounds: " << (inside_pcd_bounds_ ? "true" : "false") << "\n"
           << "  safe: " << (circle_safe_ ? "true" : "false") << "\n"
           << "points:\n";
    for (const auto & point : points_) {
      stream << "  - [" << point.x << ", " << point.y << ", " << point.z << ", "
             << point.yaw << "]\n";
    }
    stream.flush();
    if (!stream) {throw std::runtime_error("failed while writing resolved circle path");}
    std::filesystem::rename(temporary, output);
  }

  bool validateCurrentCircle(std::string & reason)
  {
    pcd_slab_points_ = 0U;
    minimum_clearance_ = std::numeric_limits<double>::infinity();
    inside_pcd_bounds_ = false;
    if (cloud_.empty()) {
      reason = require_pcd_ ? "pcd_unavailable" : "pcd_check_disabled";
      return !require_pcd_;
    }

    double minimum_x = std::numeric_limits<double>::infinity();
    double maximum_x = -minimum_x;
    double minimum_y = minimum_x;
    double maximum_y = -minimum_x;
    for (const auto & point : cloud_) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      minimum_x = std::min(minimum_x, static_cast<double>(point.x));
      maximum_x = std::max(maximum_x, static_cast<double>(point.x));
      minimum_y = std::min(minimum_y, static_cast<double>(point.y));
      maximum_y = std::max(maximum_y, static_cast<double>(point.y));
      if (point.z < spec_.path_z - body_half_height_ ||
        point.z > spec_.path_z + body_half_height_)
      {
        continue;
      }
      ++pcd_slab_points_;
      const double radial_distance = std::hypot(
        point.x - spec_.center_x, point.y - spec_.center_y);
      minimum_clearance_ = std::min(
        minimum_clearance_, std::abs(radial_distance - spec_.radius));
    }
    if (pcd_slab_points_ == 0U) {
      reason = "no_pcd_points_in_body_height_slab";
      return false;
    }
    const double required_extent = spec_.radius + robot_radius_;
    inside_pcd_bounds_ =
      spec_.center_x - required_extent >= minimum_x &&
      spec_.center_x + required_extent <= maximum_x &&
      spec_.center_y - required_extent >= minimum_y &&
      spec_.center_y + required_extent <= maximum_y;
    if (!inside_pcd_bounds_) {
      reason = "circle_outside_pcd_bounds";
      return false;
    }
    if (minimum_clearance_ < robot_radius_) {
      reason = "pcd_clearance=" + number(minimum_clearance_) +
        "_below_robot_radius=" + number(robot_radius_);
      return false;
    }
    reason = "safe";
    return true;
  }

  void updateCircleSafety()
  {
    if (!check_pcd_clearance_) {
      circle_safe_ = true;
      initialization_error_ = "pcd_clearance_check_disabled";
      return;
    }
    circle_safe_ = validateCurrentCircle(initialization_error_);
  }

  void initializeCircleFromOdometry(const nav_msgs::msg::Odometry & odometry)
  {
    if (path_ready_) {return;}
    if (odometry.header.frame_id.empty() || odometry.header.frame_id != frame_id_) {
      initialization_error_ = "odometry_frame_mismatch expected=" + frame_id_ +
        " actual=" + odometry.header.frame_id;
      return;
    }
    const auto & pose = odometry.pose.pose;
    const double initial_yaw = tf2::getYaw(pose.orientation);
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(initial_yaw))
    {
      initialization_error_ = "initial_odometry_pose_is_not_finite";
      return;
    }
    spec_.center_x = pose.position.x;
    spec_.center_y = pose.position.y;
    spec_.path_z = pose.position.z + path_height_offset_;
    spec_.start_angle = startAngleForTangentYaw(spec_, initial_yaw);
    points_ = generateCircle(spec_);
    updateCircleSafety();
    path_ready_ = true;
    writeResolvedPath();
    publishReference();
    logCircleReady();
    if (!circle_safe_ && !allow_unsafe_circle_) {
      RCLCPP_ERROR(
        get_logger(), "dynamic circle is visualization-only and cannot be armed: %s",
        initialization_error_.c_str());
    }
  }

  void logCircleReady() const
  {
    RCLCPP_WARN(
      get_logger(),
      "circle locked: center=[%.2f %.2f %.2f] radius=%.2f "
      "start=[%.2f %.2f %.2f yaw=%.1fdeg] pcd_check=%s pcd_safe=%s "
      "start_check=%s motion=%s output=%s",
      spec_.center_x, spec_.center_y, spec_.path_z, spec_.radius,
      points_.front().x, points_.front().y, points_.front().z,
      points_.front().yaw * 180.0 / kPi,
      check_pcd_clearance_ ? "enabled" : "disabled",
      circle_safe_ ? "true" : "false", require_start_pose_ ? "enabled" : "disabled",
      initial_enable_ ? "waiting_for_start_check" : "DISARMED",
      output_directory_.c_str());
  }

  nav_msgs::msg::Path makeReferencePath() const
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = frame_id_;
    message.header.stamp = now();
    message.poses.reserve(points_.size());
    for (const auto & point : points_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, point.yaw);
      pose.pose.orientation = tf2::toMsg(orientation);
      message.poses.push_back(std::move(pose));
    }
    return message;
  }

  void publishReference()
  {
    reference_pub_->publish(makeReferencePath());
  }

  void publishCloud()
  {
    if (cloud_.empty()) {return;}
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud_, message);
    message.header.frame_id = frame_id_;
    message.header.stamp = now();
    cloud_pub_->publish(message);
  }

  void publishEmptyControlPath(const std::string & reason)
  {
    if (!control_path_pub_) {return;}
    nav_msgs::msg::Path empty;
    empty.header.frame_id = frame_id_;
    empty.header.stamp = now();
    control_path_pub_->publish(empty);
    last_stop_reason_ = reason;
  }

  std::pair<bool, std::string> startCheck() const
  {
    if (!path_ready_) {return {false, "waiting_for_initial_odometry_center"};}
    if (check_pcd_clearance_ && !circle_safe_ && !allow_unsafe_circle_) {
      return {false, "unsafe_circle=" + initialization_error_};
    }
    if (!latest_odometry_) {return {false, "no_odometry"};}
    if (latest_odometry_->header.frame_id != frame_id_) {
      return {false, "odometry_frame_mismatch"};
    }
    if (!require_start_pose_) {return {true, "ready_without_start_pose_check"};}
    const auto & pose = latest_odometry_->pose.pose;
    const double position_error = std::hypot(
      pose.position.x - points_.front().x,
      pose.position.y - points_.front().y);
    if (position_error > start_position_tolerance_) {
      return {false, "start_position_error=" + number(position_error)};
    }
    const double yaw_error = std::abs(normalizeAngle(
          tf2::getYaw(pose.orientation) - points_.front().yaw));
    if (yaw_error > start_yaw_tolerance_) {
      return {false, "start_yaw_error_deg=" + number(yaw_error * 180.0 / kPi, 1)};
    }
    return {true, "ready"};
  }

  bool arm(std::string & reason)
  {
    if (active_) {reason = "already_active"; return true;}
    if (completed_) {reason = "benchmark_already_completed"; return false;}
    const auto check = startCheck();
    if (!check.first) {reason = check.second; return false;}
    active_ = true;
    auto_arm_pending_ = false;
    start_steady_ = std::chrono::steady_clock::now();
    previous_angle_ = std::atan2(
      latest_odometry_->pose.pose.position.y - spec_.center_y,
      latest_odometry_->pose.pose.position.x - spec_.center_x);
    accumulated_progress_ = 0.0;
    reference_pub_->publish(makeReferencePath());
    control_path_pub_->publish(makeReferencePath());
    reason = "armed";
    RCLCPP_WARN(get_logger(), "REAL CONTROL PATH ARMED: one %.2fm-radius lap", spec_.radius);
    return true;
  }

  void stop(const std::string & reason, bool complete)
  {
    if (!active_ && !complete) {publishEmptyControlPath(reason); return;}
    active_ = false;
    completed_ = completed_ || complete;
    publishEmptyControlPath(reason);
    writeMetrics(reason);
    RCLCPP_WARN(get_logger(), "circle benchmark stopped: %s", reason.c_str());
  }

  void enableCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (!request->data) {
      stop("service_disarm", false);
      response->success = true;
      response->message = "disarmed";
      return;
    }
    response->success = arm(response->message);
  }

  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    latest_odometry_ = message;
    if (center_from_initial_odometry_ && !path_ready_) {
      initializeCircleFromOdometry(*message);
    }
    if (auto_arm_pending_) {
      std::string reason;
      if (!arm(reason)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "motion not armed: %s; move robot to start pose first", reason.c_str());
      }
    }

    if (!path_ready_) {return;}

    const auto & pose = message->pose.pose;
    const double yaw = tf2::getYaw(pose.orientation);
    const double angle = std::atan2(
      pose.position.y - spec_.center_y, pose.position.x - spec_.center_x);
    if (active_ && previous_angle_) {
      accumulated_progress_ = std::max(
        0.0, accumulated_progress_ + directedAngularDelta(spec_, *previous_angle_, angle));
    }
    previous_angle_ = angle;
    const double radial_error = radialError(spec_, pose.position.x, pose.position.y);

    trajectory_ << std::setprecision(12)
                << rclcpp::Time(message->header.stamp).seconds() << ','
                << pose.position.x << ',' << pose.position.y << ',' << pose.position.z << ','
                << yaw << ',' << (active_ ? 1 : 0) << ',' << accumulated_progress_ << ','
                << radial_error << ',' << std::abs(radial_error) << ','
                << latest_command_.linear.x << ',' << latest_command_.linear.y << ','
                << latest_command_.angular.z << '\n';

    if (active_) {
      ++error_samples_;
      squared_error_sum_ += radial_error * radial_error;
      absolute_error_sum_ += std::abs(radial_error);
      signed_error_sum_ += radial_error;
      maximum_absolute_error_ = std::max(maximum_absolute_error_, std::abs(radial_error));
      minimum_signed_error_ = std::min(minimum_signed_error_, radial_error);
      appendActualPose(*message);
      const double endpoint_distance = std::hypot(
        pose.position.x - points_.back().x,
        pose.position.y - points_.back().y);
      if (accumulated_progress_ >= 2.0 * kPi - completion_angle_tolerance_ &&
        endpoint_distance <= completion_position_tolerance_)
      {
        stop("one_lap_complete", true);
      }
    }
  }

  void appendActualPose(const nav_msgs::msg::Odometry & odometry)
  {
    const auto & position = odometry.pose.pose.position;
    if (!actual_path_.poses.empty()) {
      const auto & previous = actual_path_.poses.back().pose.position;
      if (std::hypot(position.x - previous.x, position.y - previous.y) < actual_path_spacing_) {
        return;
      }
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odometry.header;
    pose.pose = odometry.pose.pose;
    actual_path_.header = odometry.header;
    actual_path_.poses.push_back(std::move(pose));
  }

  void timerCallback()
  {
    if (path_ready_) {publishReference();}
    publishCloud();
    if (active_) {
      control_path_pub_->publish(makeReferencePath());
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_steady_).count();
      if (elapsed >= max_duration_) {stop("max_duration_timeout", false);}
    }
    if (!actual_path_.poses.empty()) {
      actual_path_.header.stamp = now();
      actual_path_pub_->publish(actual_path_);
    }
    publishDiagnostics();
    trajectory_.flush();
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "circle_path_benchmark/tracking";
    status.hardware_id = "fixed_circle";
    if (!path_ready_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "waiting_for_initial_odometry";
    } else if (!circle_safe_ && !allow_unsafe_circle_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "unsafe_circle_visualization_only";
    } else {
      status.level = active_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = active_ ? "active" : (completed_ ? "complete" : "disarmed");
    }
    status.values = {
      keyValue("active", active_ ? "true" : "false"),
      keyValue("path_ready", path_ready_ ? "true" : "false"),
      keyValue("center_source", center_from_initial_odometry_ ?
        "initial_odometry" : "path_file"),
      keyValue("circle_safe", circle_safe_ ? "true" : "false"),
      keyValue("check_pcd_clearance", check_pcd_clearance_ ? "true" : "false"),
      keyValue("require_start_pose", require_start_pose_ ? "true" : "false"),
      keyValue("path_height_offset", number(path_height_offset_)),
      keyValue("circle_center_x", path_ready_ ? number(spec_.center_x) : "waiting"),
      keyValue("circle_center_y", path_ready_ ? number(spec_.center_y) : "waiting"),
      keyValue("circle_center_z", path_ready_ ? number(spec_.path_z) : "waiting"),
      keyValue("initialization_error", initialization_error_),
      keyValue("completed", completed_ ? "true" : "false"),
      keyValue("progress_rad", number(accumulated_progress_)),
      keyValue("progress_percent", number(100.0 * accumulated_progress_ / (2.0 * kPi), 1)),
      keyValue("samples", std::to_string(error_samples_)),
      keyValue("rms_radial_error", error_samples_ ? number(std::sqrt(
            squared_error_sum_ / static_cast<double>(error_samples_))) : "none"),
      keyValue("maximum_absolute_error", number(maximum_absolute_error_)),
      keyValue("maximum_inward_cut", number(std::max(0.0, -minimum_signed_error_))),
      keyValue("last_stop_reason", last_stop_reason_),
    };
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  void writeMetrics(const std::string & reason)
  {
    if (output_directory_.empty()) {return;}
    const auto output = std::filesystem::path(output_directory_) / "metrics.json";
    const auto temporary = output.string() + ".tmp";
    std::ofstream stream(temporary);
    if (!stream) {return;}
    const double samples = static_cast<double>(std::max<std::size_t>(1U, error_samples_));
    stream << std::fixed << std::setprecision(6)
           << "{\n"
           << "  \"status\": \"" << (completed_ ? "complete" : "stopped") << "\",\n"
           << "  \"reason\": \"" << reason << "\",\n"
           << "  \"samples\": " << error_samples_ << ",\n"
           << "  \"progress_rad\": " << accumulated_progress_ << ",\n"
           << "  \"rms_radial_error_m\": " << std::sqrt(squared_error_sum_ / samples) << ",\n"
           << "  \"mean_absolute_radial_error_m\": " << absolute_error_sum_ / samples << ",\n"
           << "  \"mean_signed_radial_error_m\": " << signed_error_sum_ / samples << ",\n"
           << "  \"maximum_absolute_radial_error_m\": " << maximum_absolute_error_ << ",\n"
           << "  \"maximum_inward_cut_m\": " << std::max(0.0, -minimum_signed_error_) << "\n"
           << "}\n";
    stream.flush();
    if (stream) {std::filesystem::rename(temporary, output);}
  }

  std::string path_file_;
  std::string pcd_override_;
  std::string source_pcd_;
  std::string output_directory_;
  std::string frame_id_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string control_path_topic_;
  std::string last_stop_reason_{"not_started"};
  std::string initialization_error_{"waiting_for_initial_odometry"};
  CircleSpec spec_;
  std::vector<CirclePoint> points_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  nav_msgs::msg::Path actual_path_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odometry_;
  geometry_msgs::msg::Twist latest_command_;
  std::ofstream trajectory_;
  bool initial_enable_{false};
  bool center_from_initial_odometry_{true};
  bool require_pcd_{true};
  bool allow_unsafe_circle_{false};
  bool check_pcd_clearance_{false};
  bool require_start_pose_{false};
  bool auto_arm_pending_{false};
  bool active_{false};
  bool completed_{false};
  bool path_ready_{false};
  bool circle_safe_{false};
  double publish_rate_{5.0};
  double start_position_tolerance_{0.20};
  double start_yaw_tolerance_{20.0 * kPi / 180.0};
  double completion_angle_tolerance_{8.0 * kPi / 180.0};
  double completion_position_tolerance_{0.20};
  double max_duration_{45.0};
  double actual_path_spacing_{0.01};
  double path_height_offset_{0.57};
  double robot_radius_{0.30};
  double body_half_height_{0.35};
  double minimum_clearance_{std::numeric_limits<double>::infinity()};
  std::size_t pcd_slab_points_{0U};
  bool inside_pcd_bounds_{false};
  std::optional<double> previous_angle_;
  double accumulated_progress_{0.0};
  std::chrono::steady_clock::time_point start_steady_;
  std::size_t error_samples_{0U};
  double squared_error_sum_{0.0};
  double absolute_error_sum_{0.0};
  double signed_error_sum_{0.0};
  double maximum_absolute_error_{0.0};
  double minimum_signed_error_{0.0};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr control_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace circle_path_benchmark

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<circle_path_benchmark::CirclePathBenchmarkNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("circle_path_benchmark"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
