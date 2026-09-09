#include "route3d_dijkstra_planner/topology_graph.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace route3d_dijkstra_planner
{
namespace
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Clock = std::chrono::steady_clock;

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

geometry_msgs::msg::Point pointMessage(const Point3 & position, const double z_offset = 0.0)
{
  geometry_msgs::msg::Point point;
  point.x = position[0];
  point.y = position[1];
  point.z = position[2] + z_offset;
  return point;
}

geometry_msgs::msg::Pose poseMessage(const Point3 & position, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position = pointMessage(position);
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

template<typename Id>
std::vector<Id> evenlySample(const std::vector<Id> & ids, const std::size_t maximum_count)
{
  if (maximum_count == 0U || ids.size() <= maximum_count) {
    return ids;
  }
  std::vector<Id> result;
  result.reserve(maximum_count);
  for (std::size_t index = 0; index < maximum_count; ++index) {
    result.push_back(ids[index * ids.size() / maximum_count]);
  }
  return result;
}

std::int64_t elapsedNanoseconds(const Clock::time_point begin)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - begin).count();
}

}  // namespace

class DijkstraPlannerNode : public rclcpp::Node
{
public:
  DijkstraPlannerNode()
  : Node("route3d_dijkstra_planner"),
    graph_(TopologyGraph::load(graphFile(), declare_parameter<bool>("strict_schema_v2", true)))
  {
    request_topic_ = declare_parameter<std::string>(
      "topics.request", "/route3d_dijkstra/plan_request");
    goal_request_topic_ = declare_parameter<std::string>(
      "topics.goal_request", "/route3d_dijkstra/goal_request");
    const auto odometry_topic = declare_parameter<std::string>(
      "topics.odometry", "/lio_odom_hf");
    start_snap_radius_m_ = declare_parameter<double>(
      "request.start_snap_radius_m", 1.0);
    if (!std::isfinite(start_snap_radius_m_) || start_snap_radius_m_ < 0.0) {
      throw std::runtime_error("request.start_snap_radius_m must be finite and non-negative");
    }
    odometry_body_height_m_ = declare_parameter<double>("request.odometry_body_height_m", 0.40);
    if (!std::isfinite(odometry_body_height_m_) || odometry_body_height_m_ < 0.0) {
      throw std::runtime_error(
              "request.odometry_body_height_m must be finite and non-negative");
    }
    const auto path_topic = declare_parameter<std::string>(
      "topics.path", "/route3d_dijkstra/path");
    const auto vertex_ids_topic = declare_parameter<std::string>(
      "topics.path_vertex_ids", "/route3d_dijkstra/path_vertex_ids");
    const auto edge_ids_topic = declare_parameter<std::string>(
      "topics.path_edge_ids", "/route3d_dijkstra/path_edge_ids");
    const auto marker_topic = declare_parameter<std::string>(
      "topics.markers", "/route3d_dijkstra/markers");
    const auto status_topic = declare_parameter<std::string>(
      "topics.status", "/route3d_dijkstra/status");
    marker_z_offset_ = declare_parameter<double>("visualization.z_offset", 0.12);
    show_labels_ = declare_parameter<bool>("visualization.show_vertex_labels", true);
    const auto max_vertices = declareNonNegativeLimit(
      "visualization.max_display_vertices", 20000);
    const auto max_edges = declareNonNegativeLimit(
      "visualization.max_display_edges", 50000);
    const auto max_labels = declareNonNegativeLimit(
      "visualization.max_display_labels", 2000);
    if (!std::isfinite(marker_z_offset_)) {
      throw std::runtime_error("visualization.z_offset must be finite");
    }

    display_vertex_ids_ = evenlySample(graph_.sortedVertexIds(), max_vertices);
    display_edge_ids_ = evenlySample(graph_.sortedEdgeIds(), max_edges);
    display_label_ids_ = evenlySample(display_vertex_ids_, max_labels);

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic, latchedQos());
    vertex_ids_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      vertex_ids_topic, latchedQos());
    edge_ids_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      edge_ids_topic, latchedQos());
    marker_publisher_ = create_publisher<MarkerArray>(marker_topic, latchedQos());
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, latchedQos());
    request_subscription_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      request_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&DijkstraPlannerNode::requestCallback, this, std::placeholders::_1));
    goal_request_subscription_ = create_subscription<std_msgs::msg::Int32>(
      goal_request_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&DijkstraPlannerNode::goalRequestCallback, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&DijkstraPlannerNode::odometryCallback, this, std::placeholders::_1));

    publishOutputs(std::nullopt);
    RCLCPP_INFO(
      get_logger(),
      "loaded Route3D Topology Schema V2: file=%s frame=%s sceneMode=%s "
      "vertices=%zu edges=%zu request_topic=%s goal_request_topic=%s implementation=C++",
      graph_file_.c_str(), graph_.frameId().c_str(), graph_.sceneMode().c_str(),
      graph_.vertices().size(), graph_.edges().size(), request_topic_.c_str(),
      goal_request_topic_.c_str());
    if (display_vertex_ids_.size() < graph_.vertices().size() ||
      display_edge_ids_.size() < graph_.edges().size() ||
      display_label_ids_.size() < display_vertex_ids_.size())
    {
      RCLCPP_WARN(
        get_logger(),
        "RViz background markers capped: vertices=%zu/%zu edges=%zu/%zu labels=%zu/%zu; "
        "selected shortest path remains complete",
        display_vertex_ids_.size(), graph_.vertices().size(),
        display_edge_ids_.size(), graph_.edges().size(),
        display_label_ids_.size(), graph_.vertices().size());
    }
  }

private:
  std::string graphFile()
  {
    const auto fallback =
      ament_index_cpp::get_package_share_directory("route3d_dijkstra_planner") +
      "/config/complex_topology_v2.json";
    graph_file_ = declare_parameter<std::string>("graph_file", fallback);
    return graph_file_;
  }

  std::size_t declareNonNegativeLimit(const std::string & name, const std::int64_t fallback)
  {
    const auto value = declare_parameter<std::int64_t>(name, fallback);
    if (value < 0) {
      throw std::runtime_error(name + " must be non-negative");
    }
    return static_cast<std::size_t>(value);
  }

  void requestCallback(const std_msgs::msg::Int32MultiArray::SharedPtr message)
  {
    request_mode_ = "start_goal";
    start_snap_distance_m_.reset();
    if (message->data.size() != 2U) {
      publishFailure(
        std::nullopt, std::nullopt,
        "plan_request data must contain exactly [start_id, goal_id]", 0, 0U);
      return;
    }
    plan(message->data[0], message->data[1]);
  }

  void goalRequestCallback(const std_msgs::msg::Int32::SharedPtr message)
  {
    request_mode_ = "goal_only";
    start_snap_distance_m_.reset();
    if (!has_odometry_) {
      publishFailure(
        std::nullopt, message->data,
        "goal-only request rejected: no odometry has been received", 0, 0U);
      return;
    }
    const auto nearest = nearestVertex(graph_, current_ground_position_);
    if (!nearest) {
      publishFailure(
        std::nullopt, message->data,
        "goal-only request rejected: topology has no vertices", 0, 0U);
      return;
    }
    start_snap_distance_m_ = nearest->distance_m;
    if (nearest->distance_m > start_snap_radius_m_) {
      std::ostringstream error;
      error << "goal-only request rejected: nearest vertex " << nearest->vertex_id <<
        " is " << nearest->distance_m << " m away, exceeding " <<
        start_snap_radius_m_ << " m";
      publishFailure(std::nullopt, message->data, error.str(), 0, 0U);
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "Goal-only request resolved start=%d at distance=%.3f m (limit=%.3f m), goal=%d",
      nearest->vertex_id, nearest->distance_m, start_snap_radius_m_, message->data);
    plan(nearest->vertex_id, message->data);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring non-finite odometry position");
      return;
    }
    current_ground_position_ = {
      position.x, position.y, position.z - odometry_body_height_m_};
    has_odometry_ = true;
  }

  void plan(const VertexId start_id, const VertexId goal_id)
  {
    const auto begin = Clock::now();
    try {
      const auto result = dijkstraShortestPath(graph_, start_id, goal_id);
      const auto elapsed_ns = elapsedNanoseconds(begin);
      current_result_ = result;
      publishOutputs(current_result_);
      publishSuccess(start_id, goal_id, result, elapsed_ns);
    } catch (const NoPathError & error) {
      publishFailure(
        start_id, goal_id, error.what(), elapsedNanoseconds(begin), error.expandedVertices());
    } catch (const std::exception & error) {
      publishFailure(start_id, goal_id, error.what(), elapsedNanoseconds(begin), 0U);
    }
  }

  void publishSuccess(
    const VertexId start_id, const VertexId goal_id, const DijkstraResult & result,
    const std::int64_t elapsed_ns)
  {
    std::vector<std::string> controller_modes;
    controller_modes.reserve(result.edge_ids.size());
    for (const auto edge_id : result.edge_ids) {
      controller_modes.push_back(graph_.edge(edge_id).controller_mode);
    }
    nlohmann::json status = {
      {"success", true}, {"algorithm", "dijkstra"}, {"implementation", "cpp"},
      {"request_mode", request_mode_},
      {"schema", {{"name", "route3d_topology"}, {"version", 2}}},
      {"start_id", start_id}, {"goal_id", goal_id},
      {"vertex_ids", result.vertex_ids}, {"edge_ids", result.edge_ids},
      {"controller_modes", controller_modes}, {"total_cost", result.total_cost},
      {"expanded_vertices", result.expanded_vertices},
      {"relaxed_edges", result.relaxed_edges},
      {"planning_time_ns", elapsed_ns},
      {"planning_time_ms", static_cast<double>(elapsed_ns) / 1.0e6}};
    if (start_snap_distance_m_) {
      status["start_snap_distance_m"] = *start_snap_distance_m_;
      status["start_snap_radius_m"] = start_snap_radius_m_;
    }
    publishStatus(status);
    RCLCPP_INFO(
      get_logger(),
      "Dijkstra C++ success: start=%d goal=%d cost=%.6f vertices=%zu edges=%zu "
      "expanded=%zu relaxed=%zu planning_time_ms=%.6f path=%s",
      start_id, goal_id, result.total_cost, result.vertex_ids.size(), result.edge_ids.size(),
      result.expanded_vertices, result.relaxed_edges,
      static_cast<double>(elapsed_ns) / 1.0e6, pathText(result.vertex_ids).c_str());
  }

  void publishFailure(
    const std::optional<VertexId> start_id, const std::optional<VertexId> goal_id,
    const std::string & error, const std::int64_t elapsed_ns,
    const std::size_t expanded_vertices)
  {
    current_result_.reset();
    publishOutputs(std::nullopt);
    nlohmann::json status = {
      {"success", false}, {"algorithm", "dijkstra"}, {"implementation", "cpp"},
      {"request_mode", request_mode_},
      {"schema", {{"name", "route3d_topology"}, {"version", 2}}},
      {"start_id", start_id ? nlohmann::json(*start_id) : nlohmann::json(nullptr)},
      {"goal_id", goal_id ? nlohmann::json(*goal_id) : nlohmann::json(nullptr)},
      {"error", error}, {"expanded_vertices", expanded_vertices},
      {"planning_time_ns", elapsed_ns},
      {"planning_time_ms", static_cast<double>(elapsed_ns) / 1.0e6}};
    if (start_snap_distance_m_) {
      status["start_snap_distance_m"] = *start_snap_distance_m_;
      status["start_snap_radius_m"] = start_snap_radius_m_;
    }
    publishStatus(status);
    RCLCPP_WARN(
      get_logger(),
      "Dijkstra C++ failed: start=%s goal=%s error=%s expanded=%zu planning_time_ms=%.6f",
      start_id ? std::to_string(*start_id).c_str() : "none",
      goal_id ? std::to_string(*goal_id).c_str() : "none", error.c_str(), expanded_vertices,
      static_cast<double>(elapsed_ns) / 1.0e6);
  }

  static std::string pathText(const std::vector<VertexId> & ids)
  {
    constexpr std::size_t max_log_ids = 60U;
    std::ostringstream stream;
    const auto count = std::min(ids.size(), max_log_ids);
    for (std::size_t index = 0; index < count; ++index) {
      if (index != 0U) {
        stream << "->";
      }
      stream << ids[index];
    }
    if (ids.size() > count) {
      stream << "->...(" << ids.size() << " vertices)";
    }
    return stream.str();
  }

  void publishStatus(const nlohmann::json & status)
  {
    std_msgs::msg::String message;
    message.data = status.dump();
    status_publisher_->publish(message);
  }

  double pathYaw(const std::vector<VertexId> & ids, const std::size_t index) const
  {
    if (ids.size() < 2U) {
      return graph_.vertex(ids[index]).rpy[2];
    }
    if (index + 1U < ids.size()) {
      const auto & current = graph_.vertex(ids[index]).position;
      const auto & next = graph_.vertex(ids[index + 1U]).position;
      return std::atan2(next[1] - current[1], next[0] - current[0]);
    }
    const auto & previous = graph_.vertex(ids[index - 1U]).position;
    const auto & current = graph_.vertex(ids[index]).position;
    return std::atan2(current[1] - previous[1], current[0] - previous[0]);
  }

  void publishOutputs(const std::optional<DijkstraResult> & result)
  {
    const auto stamp = now();
    nav_msgs::msg::Path path;
    path.header.frame_id = graph_.frameId();
    path.header.stamp = stamp;
    std_msgs::msg::Int32MultiArray vertex_ids;
    std_msgs::msg::Int32MultiArray edge_ids;
    if (result) {
      vertex_ids.data = result->vertex_ids;
      edge_ids.data = result->edge_ids;
      path.poses.reserve(result->vertex_ids.size());
      for (std::size_t index = 0; index < result->vertex_ids.size(); ++index) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose = poseMessage(
          graph_.vertex(result->vertex_ids[index]).position,
          pathYaw(result->vertex_ids, index));
        path.poses.push_back(std::move(pose));
      }
    }
    path_publisher_->publish(path);
    vertex_ids_publisher_->publish(vertex_ids);
    edge_ids_publisher_->publish(edge_ids);
    marker_publisher_->publish(buildMarkers(result, stamp));
  }

  Marker marker(
    const std::string & namespace_name, const int id, const int type,
    const rclcpp::Time & stamp) const
  {
    Marker marker_message;
    marker_message.header.frame_id = graph_.frameId();
    marker_message.header.stamp = stamp;
    marker_message.ns = namespace_name;
    marker_message.id = id;
    marker_message.type = type;
    marker_message.action = Marker::ADD;
    marker_message.pose.orientation.w = 1.0;
    return marker_message;
  }

  MarkerArray buildMarkers(
    const std::optional<DijkstraResult> & result, const rclcpp::Time & stamp) const
  {
    MarkerArray array;
    Marker clear;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    auto graph_edges = marker("topology_edges", 1, Marker::LINE_LIST, stamp);
    graph_edges.scale.x = 0.055;
    graph_edges.color.r = 0.48F;
    graph_edges.color.g = 0.52F;
    graph_edges.color.b = 0.58F;
    graph_edges.color.a = 0.62F;
    graph_edges.points.reserve(display_edge_ids_.size() * 2U);
    for (const auto edge_id : display_edge_ids_) {
      const auto & edge = graph_.edge(edge_id);
      graph_edges.points.push_back(pointMessage(graph_.vertex(edge.first).position, marker_z_offset_));
      graph_edges.points.push_back(pointMessage(graph_.vertex(edge.second).position, marker_z_offset_));
    }
    array.markers.push_back(std::move(graph_edges));

    auto graph_vertices = marker("topology_vertices", 2, Marker::SPHERE_LIST, stamp);
    graph_vertices.scale.x = 0.22;
    graph_vertices.scale.y = 0.22;
    graph_vertices.scale.z = 0.22;
    graph_vertices.color.r = 0.20F;
    graph_vertices.color.g = 0.48F;
    graph_vertices.color.b = 0.95F;
    graph_vertices.color.a = 0.95F;
    graph_vertices.points.reserve(display_vertex_ids_.size());
    for (const auto vertex_id : display_vertex_ids_) {
      graph_vertices.points.push_back(
        pointMessage(graph_.vertex(vertex_id).position, marker_z_offset_));
    }
    array.markers.push_back(std::move(graph_vertices));

    if (show_labels_) {
      for (const auto vertex_id : display_label_ids_) {
        auto label = marker("topology_vertex_ids", 1000 + vertex_id, Marker::TEXT_VIEW_FACING, stamp);
        label.pose.position = pointMessage(
          graph_.vertex(vertex_id).position, marker_z_offset_ + 0.36);
        label.scale.z = 0.32;
        label.color.r = 0.96F;
        label.color.g = 0.96F;
        label.color.b = 0.96F;
        label.color.a = 1.0F;
        label.text = std::to_string(vertex_id);
        array.markers.push_back(std::move(label));
      }
    }
    if (!result) {
      return array;
    }

    auto selected_path = marker("dijkstra_path", 10, Marker::LINE_STRIP, stamp);
    selected_path.scale.x = 0.18;
    selected_path.color.r = 0.12F;
    selected_path.color.g = 1.0F;
    selected_path.color.b = 0.22F;
    selected_path.color.a = 1.0F;
    for (const auto vertex_id : result->vertex_ids) {
      selected_path.points.push_back(
        pointMessage(graph_.vertex(vertex_id).position, marker_z_offset_ + 0.06));
    }
    array.markers.push_back(std::move(selected_path));

    auto selected_vertices = marker("dijkstra_path_vertices", 11, Marker::SPHERE_LIST, stamp);
    selected_vertices.scale.x = 0.31;
    selected_vertices.scale.y = 0.31;
    selected_vertices.scale.z = 0.31;
    selected_vertices.color.r = 1.0F;
    selected_vertices.color.g = 0.82F;
    selected_vertices.color.b = 0.05F;
    selected_vertices.color.a = 1.0F;
    for (const auto vertex_id : result->vertex_ids) {
      selected_vertices.points.push_back(
        pointMessage(graph_.vertex(vertex_id).position, marker_z_offset_ + 0.06));
    }
    array.markers.push_back(std::move(selected_vertices));
    array.markers.push_back(endpointMarker(
        "dijkstra_start", 12, result->vertex_ids.front(), stamp, 0.0F, 0.85F, 1.0F));
    array.markers.push_back(endpointMarker(
        "dijkstra_goal", 13, result->vertex_ids.back(), stamp, 1.0F, 0.08F, 0.12F));
    return array;
  }

  Marker endpointMarker(
    const std::string & namespace_name, const int id, const VertexId vertex_id,
    const rclcpp::Time & stamp, const float red, const float green, const float blue) const
  {
    auto result = marker(namespace_name, id, Marker::SPHERE, stamp);
    result.pose.position = pointMessage(
      graph_.vertex(vertex_id).position, marker_z_offset_ + 0.06);
    result.scale.x = 0.50;
    result.scale.y = 0.50;
    result.scale.z = 0.50;
    result.color.r = red;
    result.color.g = green;
    result.color.b = blue;
    result.color.a = 1.0F;
    return result;
  }

  std::string graph_file_;
  std::string request_topic_;
  std::string goal_request_topic_;
  TopologyGraph graph_;
  double start_snap_radius_m_{1.0};
  double odometry_body_height_m_{0.40};
  bool has_odometry_{false};
  Point3 current_ground_position_{};
  std::string request_mode_{"start_goal"};
  std::optional<double> start_snap_distance_m_;
  double marker_z_offset_{0.12};
  bool show_labels_{true};
  std::vector<VertexId> display_vertex_ids_;
  std::vector<EdgeId> display_edge_ids_;
  std::vector<VertexId> display_label_ids_;
  std::optional<DijkstraResult> current_result_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vertex_ids_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr edge_ids_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr request_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_request_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

}  // namespace route3d_dijkstra_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<route3d_dijkstra_planner::DijkstraPlannerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route3d_dijkstra_planner"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
