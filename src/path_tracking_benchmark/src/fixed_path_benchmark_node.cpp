#include "path_tracking_benchmark/benchmark_math.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace path_tracking_benchmark
{
namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;

double steadySeconds(const SteadyTime & start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

std::string timestampForFile()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::string jsonEscape(const std::string & value)
{
  std::string result;
  result.reserve(value.size());
  for (const char character : value) {
    if (character == '"' || character == '\\') {result.push_back('\\');}
    result.push_back(character);
  }
  return result;
}

bool isZeroCommand(const geometry_msgs::msg::Twist & command)
{
  return std::abs(command.linear.x) < 1e-3 && std::abs(command.linear.y) < 1e-3 &&
         std::abs(command.angular.z) < 1e-3;
}

struct Sample
{
  double time{0.0};
  Point3 actual;
  Point3 reference;
  double arc{0.0};
  double xy_error{0.0};
  double error_3d{0.0};
  double lateral_error{0.0};
  double yaw{0.0};
  double reference_yaw{0.0};
  double yaw_error{0.0};
  geometry_msgs::msg::Twist command;
  double lookahead{0.0};
  std::string controller_state;
};

}  // namespace

class FixedPathBenchmarkNode : public rclcpp::Node
{
public:
  FixedPathBenchmarkNode()
  : Node("fixed_path_benchmark")
  {
    planning_frame_ = declare_parameter<std::string>("frames.planning", "camera_init");
    odometry_topic_ = declare_parameter<std::string>("input.odometry_topic", "/lio_odom_hf");
    path_topic_ = declare_parameter<std::string>("output.path_topic", "/local_planner/local_path");
    command_topic_ = declare_parameter<std::string>("input.command_topic", "/cmd_vel_smoothed");
    trajectory_type_ = declare_parameter<std::string>("trajectory", "straight");
    sample_spacing_ = declare_parameter<double>("sample_spacing", 0.05);
    max_tracking_error_ = declare_parameter<double>("max_tracking_error", 0.30);
    completion_distance_ = declare_parameter<double>("completion_distance", 0.15);
    settle_duration_ = declare_parameter<double>("settle_duration", 0.30);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 30.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.30);
    output_directory_ = declare_parameter<std::string>("output_directory", defaultOutputDirectory());
    run_label_ = declare_parameter<std::string>("run_label", "baseline");
    controller_node_name_ = declare_parameter<std::string>(
      "controller_node_name", "/local_path_follower");
    publish_rate_ = std::max(1.0, declare_parameter<double>("publish_rate", 50.0));

    // launch 参数为空时保留节点的可移植默认目录，而不是让 filesystem 创建空路径。
    if (output_directory_.empty()) {output_directory_ = defaultOutputDirectory();}

    if (planning_frame_.empty() || sample_spacing_ <= 0.0 || max_tracking_error_ <= 0.0 ||
      completion_distance_ <= 0.0 || settle_duration_ < 0.0 || timeout_sec_ <= 0.0 ||
      odometry_timeout_ <= 0.0 || makeLocalTrajectory(trajectory_type_, sample_spacing_).size() < 2U)
    {
      throw std::invalid_argument("invalid fixed-path benchmark parameters");
    }

    const auto path_qos = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);
    reference_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/path_tracking_benchmark/reference_path", path_qos);
    actual_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/path_tracking_benchmark/actual_path", path_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/path_tracking_benchmark/markers", path_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/path_tracking_benchmark/diagnostics", 10);

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&FixedPathBenchmarkNode::odometryCallback, this, std::placeholders::_1));
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, rclcpp::QoS(20).reliable(),
      std::bind(&FixedPathBenchmarkNode::commandCallback, this, std::placeholders::_1));
    controller_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/local_planner/controller_diagnostics", rclcpp::QoS(20),
      std::bind(&FixedPathBenchmarkNode::controllerDiagnosticsCallback, this, std::placeholders::_1));

    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/path_tracking_benchmark/start",
      std::bind(&FixedPathBenchmarkNode::startCallback, this, std::placeholders::_1,
      std::placeholders::_2));
    abort_service_ = create_service<std_srvs::srv::Trigger>(
      "/path_tracking_benchmark/abort",
      std::bind(&FixedPathBenchmarkNode::abortCallback, this, std::placeholders::_1,
      std::placeholders::_2));

    controller_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, controller_node_name_);
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FixedPathBenchmarkNode::timerCallback, this));
    RCLCPP_INFO(
      get_logger(), "fixed path benchmark ready: trajectory=%s path=%s start_service=/path_tracking_benchmark/start",
      trajectory_type_.c_str(), path_topic_.c_str());
  }

  ~FixedPathBenchmarkNode() override
  {
    publishEmptyPath();
  }

private:
  enum class State {Idle, Running, Completed, Aborted};

  static std::string defaultOutputDirectory()
  {
    const char * home = std::getenv("HOME");
    return std::string(home ? home : "/tmp") + "/.ros/path_tracking_benchmark";
  }

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const auto & orientation = message->pose.pose.orientation;
    const double norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(norm) || norm < 1e-6) {return;}
    const Point3 position{
      message->pose.pose.position.x, message->pose.pose.position.y, message->pose.pose.position.z};
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    latest_position_ = position;
    latest_yaw_ = tf2::getYaw(orientation);
    latest_odom_frame_ = message->header.frame_id;
    latest_odom_steady_ = std::chrono::steady_clock::now();
    have_odometry_ = true;
  }

  void commandCallback(const geometry_msgs::msg::Twist::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_command_ = *message;
  }

  void controllerDiagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
  {
    for (const auto & status : message->status) {
      if (status.name != "efficient_3d_local_planner/local_path_follower") {continue;}
      std::lock_guard<std::mutex> lock(mutex_);
      controller_state_ = status.message;
      for (const auto & value : status.values) {
        if (value.key == "lookahead_distance") {
          try {latest_lookahead_ = std::stod(value.value);} catch (const std::exception &) {}
        }
      }
    }
  }

  bool pathHasExternalPublisher() const
  {
    for (const auto & endpoint : get_publishers_info_by_topic(path_topic_)) {
      if (endpoint.node_name() != get_name() || endpoint.node_namespace() != get_namespace()) {
        return true;
      }
    }
    return false;
  }

  void startCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::Running) {
      response->success = false;
      response->message = "benchmark is already running";
      return;
    }
    if (!have_odometry_ || steadySeconds(latest_odom_steady_) > odometry_timeout_) {
      response->success = false;
      response->message = "fresh odometry is required before starting";
      return;
    }
    if (!latest_odom_frame_.empty() && latest_odom_frame_ != planning_frame_) {
      response->success = false;
      response->message = "odometry frame must match frames.planning for this isolated benchmark";
      return;
    }
    if (pathHasExternalPublisher()) {
      response->success = false;
      response->message = "another publisher is active on the local path topic; stop planner/route nodes first";
      return;
    }
    const auto local = makeLocalTrajectory(trajectory_type_, sample_spacing_);
    reference_ = transformTrajectory(local, latest_position_, latest_yaw_);
    actual_.clear();
    samples_.clear();
    controller_parameters_snapshot_.clear();
    run_started_ = std::chrono::steady_clock::now();
    zero_command_since_.reset();
    previous_yaw_command_sign_ = 0;
    yaw_sign_changes_ = 0;
    result_reason_ = "running";
    state_ = State::Running;
    openCsv();
    requestControllerParameters();
    const auto message = makePath(reference_);
    reference_pub_->publish(message);
    path_pub_->publish(message);
    publishVisuals(std::nullopt);
    response->success = true;
    response->message = "benchmark started: " + trajectory_type_;
  }

  void abortCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Running) {
      response->success = false;
      response->message = "benchmark is not running";
      return;
    }
    finishLocked(State::Aborted, "manual_abort");
    response->success = true;
    response->message = "benchmark aborted and local path cleared";
  }

  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Running) {return;}
    if (!have_odometry_ || steadySeconds(latest_odom_steady_) > odometry_timeout_) {
      finishLocked(State::Aborted, "odometry_timeout");
      return;
    }
    const double elapsed = steadySeconds(run_started_);
    if (elapsed > timeout_sec_) {
      finishLocked(State::Aborted, "timeout");
      return;
    }
    const Projection projection = projectToPath(latest_position_, reference_);
    if (!projection.valid) {
      finishLocked(State::Aborted, "invalid_reference_projection");
      return;
    }
    const double yaw_error = normalizeAngle(projection.yaw - latest_yaw_);
    const Sample sample{
      elapsed, latest_position_, projection.point, projection.arc, projection.distance_xy,
      projection.distance_3d, projection.signed_lateral, latest_yaw_, projection.yaw, yaw_error,
      latest_command_, latest_lookahead_, controller_state_};
    samples_.push_back(sample);
    actual_.push_back(latest_position_);
    writeCsv(sample);
    actual_pub_->publish(makePath(actual_));
    publishVisuals(projection);
    if (projection.distance_3d > max_tracking_error_) {
      finishLocked(State::Aborted, "tracking_error_limit");
      return;
    }
    const Point3 & endpoint = reference_.back();
    const bool inside_terminal = distance3d(latest_position_, endpoint) <= completion_distance_;
    if (inside_terminal && isZeroCommand(latest_command_)) {
      if (!zero_command_since_) {zero_command_since_ = std::chrono::steady_clock::now();}
      if (steadySeconds(*zero_command_since_) >= settle_duration_) {
        finishLocked(State::Completed, "completed");
      }
    } else {
      zero_command_since_.reset();
    }
  }

  nav_msgs::msg::Path makePath(const std::vector<Point3> & points) const
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = planning_frame_;
    message.header.stamp = now();
    message.poses.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = points[i].x;
      pose.pose.position.y = points[i].y;
      pose.pose.position.z = points[i].z;
      const std::size_t before = i == 0U ? 0U : i - 1U;
      const std::size_t after = std::min(points.size() - 1U, i + 1U);
      const double yaw = std::atan2(points[after].y - points[before].y,
        points[after].x - points[before].x);
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      message.poses.push_back(std::move(pose));
    }
    return message;
  }

  void publishEmptyPath()
  {
    if (!path_pub_) {return;}
    nav_msgs::msg::Path empty;
    empty.header.frame_id = planning_frame_;
    empty.header.stamp = now();
    path_pub_->publish(empty);
  }

  void publishVisuals(const std::optional<Projection> & projection)
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = now();
    auto sphere = [&](int id, const Point3 & point, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = planning_frame_; marker.header.stamp = stamp;
        marker.ns = "path_tracking_benchmark"; marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point.x; marker.pose.position.y = point.y; marker.pose.position.z = point.z;
        marker.pose.orientation.w = 1.0; marker.scale.x = marker.scale.y = marker.scale.z = 0.16;
        marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0F;
        return marker;
      };
    if (!reference_.empty()) {
      array.markers.push_back(sphere(0, reference_.front(), 0.1F, 1.0F, 0.1F));
      array.markers.push_back(sphere(1, reference_.back(), 1.0F, 0.1F, 0.1F));
    }
    if (projection && have_odometry_) {
      array.markers.push_back(sphere(2, projection->point, 1.0F, 0.9F, 0.0F));
      visualization_msgs::msg::Marker line;
      line.header.frame_id = planning_frame_; line.header.stamp = stamp;
      line.ns = "path_tracking_benchmark"; line.id = 3;
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0; line.scale.x = 0.025;
      line.color.r = 1.0F; line.color.g = 0.25F; line.color.b = 0.9F; line.color.a = 1.0F;
      geometry_msgs::msg::Point actual; actual.x = latest_position_.x; actual.y = latest_position_.y; actual.z = latest_position_.z;
      geometry_msgs::msg::Point reference; reference.x = projection->point.x; reference.y = projection->point.y; reference.z = projection->point.z;
      line.points = {actual, reference}; array.markers.push_back(std::move(line));
    }
    marker_pub_->publish(array);
  }

  void openCsv()
  {
    std::filesystem::create_directories(output_directory_);
    run_basename_ = timestampForFile() + "_" + trajectory_type_ + "_" + run_label_;
    csv_path_ = std::filesystem::path(output_directory_) / (run_basename_ + ".csv");
    csv_.open(csv_path_);
    csv_ << "time_s,actual_x,actual_y,actual_z,actual_yaw,reference_x,reference_y,reference_z,reference_yaw,"
         << "arc_m,xy_error_m,error_3d_m,lateral_error_m,yaw_error_rad,cmd_vx,cmd_vy,cmd_wz,"
         << "lookahead_m,controller_state\n";
  }

  void writeCsv(const Sample & sample)
  {
    if (!csv_) {return;}
    csv_ << std::fixed << std::setprecision(6)
         << sample.time << ',' << sample.actual.x << ',' << sample.actual.y << ',' << sample.actual.z << ','
         << sample.yaw << ',' << sample.reference.x << ',' << sample.reference.y << ',' << sample.reference.z << ','
         << sample.reference_yaw << ',' << sample.arc << ',' << sample.xy_error << ',' << sample.error_3d << ','
         << sample.lateral_error << ',' << sample.yaw_error << ',' << sample.command.linear.x << ','
         << sample.command.linear.y << ',' << sample.command.angular.z << ',' << sample.lookahead << ','
         << '"' << jsonEscape(sample.controller_state) << '"' << '\n';
    const int sign = std::abs(sample.command.angular.z) < 0.05 ? 0 :
      (sample.command.angular.z > 0.0 ? 1 : -1);
    if (sign != 0 && previous_yaw_command_sign_ != 0 && sign != previous_yaw_command_sign_) {
      ++yaw_sign_changes_;
    }
    if (sign != 0) {previous_yaw_command_sign_ = sign;}
  }

  void requestControllerParameters()
  {
    if (!controller_parameters_->service_is_ready()) {return;}
    const std::vector<std::string> names{
      "controller.nominal_speed", "controller.max_vx", "controller.max_vy", "controller.max_vyaw",
      "controller.yaw_gain", "controller.yaw_deadband", "controller.lookahead_base",
      "controller.lookahead_speed_gain", "controller.lookahead_min", "controller.lookahead_max",
      "controller.translation_yaw_limit", "controller.full_speed_yaw_limit",
      "controller.minimum_yaw_speed_enabled", "controller.minimum_yaw_speed"};
    controller_parameters_->get_parameters(names,
      [this, names](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        try {
          const auto values = future.get();
          std::lock_guard<std::mutex> lock(mutex_);
          for (std::size_t i = 0; i < values.size() && i < names.size(); ++i) {
            controller_parameters_snapshot_[names[i]] = values[i].value_to_string();
          }
        } catch (const std::exception &) {}
      });
  }

  void finishLocked(const State state, const std::string & reason)
  {
    if (state_ != State::Running) {return;}
    state_ = state;
    result_reason_ = reason;
    publishEmptyPath();
    writeSummary();
    if (csv_) {csv_.close();}
    publishDiagnostics();
    RCLCPP_INFO(get_logger(), "benchmark finished: state=%s reason=%s csv=%s",
      state == State::Completed ? "completed" : "aborted", reason.c_str(), csv_path_.c_str());
  }

  void writeSummary()
  {
    std::vector<double> xy, xyz, yaw;
    double max_vx = 0.0, max_vy = 0.0, max_wz = 0.0;
    for (const Sample & sample : samples_) {
      xy.push_back(sample.xy_error); xyz.push_back(sample.error_3d); yaw.push_back(std::abs(sample.yaw_error));
      max_vx = std::max(max_vx, std::abs(sample.command.linear.x));
      max_vy = std::max(max_vy, std::abs(sample.command.linear.y));
      max_wz = std::max(max_wz, std::abs(sample.command.angular.z));
    }
    const auto average = [](const std::vector<double> & values) {
        if (values.empty()) {return 0.0;}
        double total = 0.0; for (const double value : values) {total += value;}
        return total / values.size();
      };
    const auto rms = [](const std::vector<double> & values) {
        if (values.empty()) {return 0.0;}
        double total = 0.0; for (const double value : values) {total += value * value;}
        return std::sqrt(total / values.size());
      };
    const auto maximum = [](const std::vector<double> & values) {
        return values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
      };
    const Point3 endpoint = reference_.empty() ? Point3{} : reference_.back();
    const double terminal_error = samples_.empty() ? 0.0 : distance3d(samples_.back().actual, endpoint);
    const auto summary_path = std::filesystem::path(output_directory_) / (run_basename_ + ".json");
    std::ofstream summary(summary_path);
    summary << std::fixed << std::setprecision(6);
    summary << "{\n"
            << "  \"trajectory\": \"" << jsonEscape(trajectory_type_) << "\",\n"
            << "  \"run_label\": \"" << jsonEscape(run_label_) << "\",\n"
            << "  \"result\": \"" << jsonEscape(result_reason_) << "\",\n"
            << "  \"csv\": \"" << jsonEscape(csv_path_.string()) << "\",\n"
            << "  \"duration_s\": " << (samples_.empty() ? 0.0 : samples_.back().time) << ",\n"
            << "  \"samples\": " << samples_.size() << ",\n"
            << "  \"xy_error\": {\"mean\": " << average(xy) << ", \"rms\": " << rms(xy)
            << ", \"p95\": " << percentile(xy, 0.95) << ", \"max\": " << maximum(xy) << "},\n"
            << "  \"error_3d\": {\"mean\": " << average(xyz) << ", \"rms\": " << rms(xyz)
            << ", \"p95\": " << percentile(xyz, 0.95) << ", \"max\": " << maximum(xyz) << "},\n"
            << "  \"yaw_error_rad\": {\"mean\": " << average(yaw) << ", \"rms\": " << rms(yaw)
            << ", \"p95\": " << percentile(yaw, 0.95) << ", \"max\": " << maximum(yaw) << "},\n"
            << "  \"terminal_error_3d_m\": " << terminal_error << ",\n"
            << "  \"max_command\": {\"vx\": " << max_vx << ", \"vy\": " << max_vy << ", \"wz\": " << max_wz << "},\n"
            << "  \"yaw_sign_changes\": " << yaw_sign_changes_ << ",\n"
            << "  \"controller_parameters\": {";
    bool first = true;
    for (const auto & [name, value] : controller_parameters_snapshot_) {
      if (!first) {summary << ',';}
      summary << "\n    \"" << jsonEscape(name) << "\": \"" << jsonEscape(value) << "\"";
      first = false;
    }
    if (!controller_parameters_snapshot_.empty()) {summary << '\n';}
    summary << "  }\n}\n";
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "path_tracking_benchmark/fixed_path";
    status.hardware_id = "m20";
    status.level = state_ == State::Completed ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = result_reason_;
    auto add = [&](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair; pair.key = key; pair.value = value; status.values.push_back(pair);
      };
    add("trajectory", trajectory_type_); add("samples", std::to_string(samples_.size()));
    add("csv", csv_path_.string()); add("state", state_ == State::Completed ? "completed" : "aborted");
    array.status.push_back(std::move(status)); diagnostics_pub_->publish(array);
  }

  std::string planning_frame_, odometry_topic_, path_topic_, command_topic_, trajectory_type_;
  std::string output_directory_, run_label_, controller_node_name_, run_basename_, result_reason_{"idle"};
  double sample_spacing_{0.05}, max_tracking_error_{0.30}, completion_distance_{0.15};
  double settle_duration_{0.30}, timeout_sec_{30.0}, odometry_timeout_{0.30}, publish_rate_{50.0};
  std::mutex mutex_;
  bool have_odometry_{false};
  Point3 latest_position_;
  double latest_yaw_{0.0}, latest_lookahead_{0.0};
  std::string latest_odom_frame_, controller_state_{"unknown"};
  SteadyTime latest_odom_steady_{}, run_started_{};
  geometry_msgs::msg::Twist latest_command_;
  std::optional<SteadyTime> zero_command_since_;
  State state_{State::Idle};
  std::vector<Point3> reference_, actual_;
  std::vector<Sample> samples_;
  std::ofstream csv_;
  std::filesystem::path csv_path_;
  std::map<std::string, std::string> controller_parameters_snapshot_;
  int previous_yaw_command_sign_{0}, yaw_sign_changes_{0};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, reference_pub_, actual_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr controller_diagnostics_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_, abort_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::AsyncParametersClient> controller_parameters_;
};

}  // namespace path_tracking_benchmark

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_tracking_benchmark::FixedPathBenchmarkNode>());
  rclcpp::shutdown();
  return 0;
}
