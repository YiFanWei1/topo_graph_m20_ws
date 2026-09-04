#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "route3d_dijkstra_planner/topology_graph.hpp"
#include "route3d_route_slicer/msg/route_task.hpp"
#include "route3d_route_slicer/msg/route_task_array.hpp"
#include "route3d_route_slicer/msg/route_waypoint.hpp"
#include "route3d_route_slicer/route_slicer.hpp"

namespace route3d_route_slicer
{
namespace
{

using json = nlohmann::json;
using TaskMessage = route3d_route_slicer::msg::RouteTask;
using TaskArrayMessage = route3d_route_slicer::msg::RouteTaskArray;
using WaypointMessage = route3d_route_slicer::msg::RouteWaypoint;

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

int checkedIntParameter(const std::int64_t value, const char * name)
{
  if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
    throw std::out_of_range(std::string("parameter '") + name + "' is outside int range");
  }
  return static_cast<int>(value);
}

std::vector<std::int32_t> integerArray(const json & document, const char * key)
{
  const auto iterator = document.find(key);
  if (iterator == document.end() || !iterator->is_array()) {
    throw std::invalid_argument(std::string("Dijkstra status missing array '") + key + "'");
  }
  std::vector<std::int32_t> result;
  result.reserve(iterator->size());
  for (const auto & value : *iterator) {
    if (!value.is_number_integer()) {
      throw std::invalid_argument(std::string("Dijkstra status '") + key + "' must contain integers");
    }
    result.push_back(value.get<std::int32_t>());
  }
  return result;
}

WaypointMessage toMessage(const WaypointConstraint & source)
{
  WaypointMessage message;
  message.vertex_id = source.vertex_id;
  message.position.x = source.position[0];
  message.position.y = source.position[1];
  message.position.z = source.position[2];
  message.rpy.x = source.rpy[0];
  message.rpy.y = source.rpy[1];
  message.rpy.z = source.rpy[2];
  message.semantic_type = source.semantic_type;
  message.semantic_type_id = source.semantic_type_id;
  message.charging_mode = source.charging_mode;
  message.is_corner = source.is_corner;
  message.is_slope = source.is_slope;
  message.is_junction = source.is_junction;
  message.turn_degrees = source.turn_degrees;
  message.must_pass_through = source.must_pass_through;
  message.pass_radius_m = source.pass_radius_m;
  message.goal_tolerance_m = source.goal_tolerance_m;
  message.align_final_yaw = source.align_final_yaw;
  message.turnable = source.turnable;
  message.pcd_name = source.pcd_name;
  return message;
}

TaskMessage toMessage(const RouteTask & source)
{
  TaskMessage message;
  message.task_index = static_cast<std::uint32_t>(source.task_index);
  message.task_mode = source.task_mode;
  message.configured_controller_mode = source.configured_controller_mode;
  message.resolved_controller_mode = source.resolved_controller_mode;
  message.gait_command = source.gait_command;
  message.completion_policy = static_cast<std::uint8_t>(source.completion_policy);
  message.is_route_goal = source.is_route_goal;
  message.requires_stop_at_end = source.requires_stop_at_end;
  message.requires_gait_switch_at_start = source.requires_gait_switch_at_start;
  message.contains_slope = source.contains_slope;
  message.reverse_motion = source.reverse_motion;
  message.align_goal_yaw = source.align_goal_yaw;
  message.endpoint_tolerance_m = source.endpoint_tolerance_m;
  message.locomotion_mode = source.locomotion_mode;
  message.linear_speed_mps = source.linear_speed_mps;
  message.angular_speed_radps = source.angular_speed_radps;
  message.height_offset_m = source.height_offset_m;
  message.obstacle_mode = source.obstacle_mode;
  message.obstacle_box_m = source.obstacle_box_m;
  message.grid_map_name = source.grid_map_name;
  message.rotation_allowed = source.rotation_allowed;
  message.edge_ids.assign(source.edge_ids.begin(), source.edge_ids.end());
  message.split_reasons = source.split_reasons;
  message.waypoints.reserve(source.waypoints.size());
  for (const auto & waypoint : source.waypoints) {
    message.waypoints.push_back(toMessage(waypoint));
  }
  return message;
}

json waypointJson(const WaypointConstraint & waypoint)
{
  return {
    {"vertex_id", waypoint.vertex_id},
    {"position", waypoint.position},
    {"rpy", waypoint.rpy},
    {"semantic_type", waypoint.semantic_type},
    {"semantic_type_id", waypoint.semantic_type_id},
    {"charging_mode", waypoint.charging_mode},
    {"is_corner", waypoint.is_corner},
    {"is_slope", waypoint.is_slope},
    {"is_junction", waypoint.is_junction},
    {"turn_degrees", waypoint.turn_degrees},
    {"must_pass_through", waypoint.must_pass_through},
    {"pass_radius_m", waypoint.pass_radius_m},
    {"goal_tolerance_m", waypoint.goal_tolerance_m},
    {"align_final_yaw", waypoint.align_final_yaw},
    {"turnable", waypoint.turnable},
    {"pcd_name", waypoint.pcd_name}};
}

json taskJson(const RouteTask & task)
{
  json waypoints = json::array();
  for (const auto & waypoint : task.waypoints) {
    waypoints.push_back(waypointJson(waypoint));
  }
  return {
    {"task_index", task.task_index},
    {"task_mode", task.task_mode},
    {"configured_controller_mode", task.configured_controller_mode},
    {"resolved_controller_mode", task.resolved_controller_mode},
    {"gait_command", task.gait_command},
    {"completion_policy", static_cast<std::uint8_t>(task.completion_policy)},
    {"is_route_goal", task.is_route_goal},
    {"requires_stop_at_end", task.requires_stop_at_end},
    {"requires_gait_switch_at_start", task.requires_gait_switch_at_start},
    {"contains_slope", task.contains_slope},
    {"reverse_motion", task.reverse_motion},
    {"align_goal_yaw", task.align_goal_yaw},
    {"endpoint_tolerance_m", task.endpoint_tolerance_m},
    {"locomotion_mode", task.locomotion_mode},
    {"linear_speed_mps", task.linear_speed_mps},
    {"angular_speed_radps", task.angular_speed_radps},
    {"height_offset_m", task.height_offset_m},
    {"obstacle_mode", task.obstacle_mode},
    {"obstacle_box_m", task.obstacle_box_m},
    {"grid_map_name", task.grid_map_name},
    {"rotation_allowed", task.rotation_allowed},
    {"edge_ids", task.edge_ids},
    {"split_reasons", task.split_reasons},
    {"waypoints", std::move(waypoints)}};
}

std_msgs::msg::ColorRGBA taskColor(const std::size_t index)
{
  static const std::array<std::array<float, 3>, 6> colors = {{
    {{0.10F, 0.90F, 0.25F}}, {{0.10F, 0.70F, 1.00F}}, {{1.00F, 0.55F, 0.05F}},
    {{0.75F, 0.25F, 1.00F}}, {{1.00F, 0.15F, 0.35F}}, {{0.10F, 1.00F, 0.90F}}}};
  std_msgs::msg::ColorRGBA color;
  color.r = colors[index % colors.size()][0];
  color.g = colors[index % colors.size()][1];
  color.b = colors[index % colors.size()][2];
  color.a = 1.0F;
  return color;
}

}  // namespace

class RouteSlicerNode : public rclcpp::Node
{
public:
  RouteSlicerNode()
  : Node("route3d_route_slicer")
  {
    graph_file_ = declare_parameter<std::string>("graph_file", "");
    if (graph_file_.empty()) {
      throw std::invalid_argument("graph_file must not be empty");
    }
    const bool strict_schema = declare_parameter<bool>("strict_schema_v2", true);
    SliceOptions options;
    options.enable_slope_gait = declare_parameter<bool>("slope.enable_gait_switch", true);
    options.slope_ignore_ordinary_obstacles = declare_parameter<bool>(
      "slope.ignore_ordinary_obstacles", true);
    options.normal_locomotion_mode = checkedIntParameter(
      declare_parameter<std::int64_t>("slope.normal_locomotion_mode", 0),
      "slope.normal_locomotion_mode");
    options.slope_locomotion_mode = checkedIntParameter(
      declare_parameter<std::int64_t>("slope.slope_locomotion_mode", 2),
      "slope.slope_locomotion_mode");
    options.default_corner_pass_radius_m =
      declare_parameter<double>("waypoint.corner_pass_radius_m", 0.20);
    options.default_normal_pass_radius_m =
      declare_parameter<double>("waypoint.normal_pass_radius_m", 0.45);
    options.attribute_epsilon = declare_parameter<double>("slicing.attribute_epsilon", 1.0e-6);
    options.auto_default_controller =
      declare_parameter<std::string>("controller.auto_default", "pid");
    options.auto_grid_controller =
      declare_parameter<std::string>("controller.auto_grid", "local_planner");
    options.auto_slope_controller = declare_parameter<std::string>(
      "slope.controller_mode", "efficient_3d_local_planner");
    options.normal_gait_command =
      declare_parameter<std::string>("gait.normal_command", "static_walk");
    options.slope_gait_command =
      declare_parameter<std::string>("gait.slope_command", "switch_gait_3");

    graph_ = std::make_unique<route3d_dijkstra_planner::TopologyGraph>(
      route3d_dijkstra_planner::TopologyGraph::load(graph_file_, strict_schema));
    slicer_ = std::make_unique<RouteSlicer>(std::move(options));

    const auto input_topic = declare_parameter<std::string>(
      "topics.dijkstra_status", "/route3d_dijkstra/status");
    const auto tasks_topic = declare_parameter<std::string>(
      "topics.tasks", "/route3d_route_slicer/tasks");
    const auto tasks_json_topic = declare_parameter<std::string>(
      "topics.tasks_json", "/route3d_route_slicer/tasks_json");
    const auto status_topic = declare_parameter<std::string>(
      "topics.status", "/route3d_route_slicer/status");
    const auto markers_topic = declare_parameter<std::string>(
      "topics.markers", "/route3d_route_slicer/markers");

    tasks_publisher_ = create_publisher<TaskArrayMessage>(tasks_topic, latchedQos());
    tasks_json_publisher_ = create_publisher<std_msgs::msg::String>(
      tasks_json_topic, latchedQos());
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, latchedQos());
    markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      markers_topic, latchedQos());
    status_subscription_ = create_subscription<std_msgs::msg::String>(
      input_topic, latchedQos(),
      std::bind(&RouteSlicerNode::statusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Loaded Schema V2 graph for semantic slicing: %zu vertices, %zu edges, %s",
      graph_->vertices().size(), graph_->edges().size(), graph_file_.c_str());
  }

private:
  void statusCallback(const std_msgs::msg::String::SharedPtr message)
  {
    const auto start = std::chrono::steady_clock::now();
    try {
      const auto status = json::parse(message->data);
      if (!status.value("success", false)) {
        publishStatus(false, "upstream Dijkstra request failed", 0.0, 0U);
        return;
      }
      const auto vertex_ids = integerArray(status, "vertex_ids");
      const auto edge_ids = integerArray(status, "edge_ids");
      const auto result = slicer_->slice(*graph_, vertex_ids, edge_ids);
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      const double total_cost = status.value("total_cost", 0.0);
      ++route_sequence_;
      publishTasks(result, total_cost, elapsed_ms);
      publishMarkers(result);
      publishStatus(true, "route sliced", elapsed_ms, result.tasks.size());
      RCLCPP_INFO(
        get_logger(), "Sliced route %d -> %d: %zu vertices, %zu edges, %zu tasks, %.3f ms",
        result.route_start_id, result.route_goal_id, vertex_ids.size(), edge_ids.size(),
        result.tasks.size(), elapsed_ms);
      for (const auto & task : result.tasks) {
        RCLCPP_INFO(
          get_logger(),
          "  task[%zu] %d->%d mode=%s controller=%s gait=%s edges=%zu slope=%s "
          "goal=%s stop=%s reasons=%s",
          task.task_index, task.waypoints.front().vertex_id, task.waypoints.back().vertex_id,
          task.task_mode.c_str(), task.resolved_controller_mode.c_str(), task.gait_command.c_str(),
          task.edge_ids.size(), task.contains_slope ? "true" : "false",
          task.is_route_goal ? "true" : "false", task.requires_stop_at_end ? "true" : "false",
          task.split_reasons.empty() ? "none" : task.split_reasons.front().c_str());
      }
    } catch (const std::exception & error) {
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      RCLCPP_ERROR(get_logger(), "Route slicing failed: %s", error.what());
      publishStatus(false, error.what(), elapsed_ms, 0U);
    }
  }

  void publishTasks(const SliceResult & result, const double total_cost, const double elapsed_ms)
  {
    TaskArrayMessage message;
    message.header.stamp = now();
    message.header.frame_id = graph_->frameId();
    message.route_sequence = route_sequence_;
    message.route_start_id = result.route_start_id;
    message.route_goal_id = result.route_goal_id;
    message.total_cost = total_cost;
    message.slicing_time_ms = elapsed_ms;
    message.source_graph = graph_file_;
    message.tasks.reserve(result.tasks.size());
    json task_array = json::array();
    for (const auto & task : result.tasks) {
      message.tasks.push_back(toMessage(task));
      task_array.push_back(taskJson(task));
    }
    tasks_publisher_->publish(message);

    std_msgs::msg::String json_message;
    json_message.data = json({
        {"success", true}, {"implementation", "cpp"},
        {"route_sequence", route_sequence_}, {"route_start_id", result.route_start_id},
        {"route_goal_id", result.route_goal_id}, {"total_cost", total_cost},
        {"slicing_time_ms", elapsed_ms}, {"source_graph", graph_file_},
        {"tasks", std::move(task_array)}}).dump();
    tasks_json_publisher_->publish(json_message);
  }

  void publishStatus(
    const bool success, const std::string & detail, const double elapsed_ms,
    const std::size_t task_count)
  {
    std_msgs::msg::String message;
    message.data = json({
        {"success", success}, {"implementation", "cpp"}, {"detail", detail},
        {"route_sequence", route_sequence_}, {"task_count", task_count},
        {"slicing_time_ms", elapsed_ms}}).dump();
    status_publisher_->publish(message);
  }

  void publishMarkers(const SliceResult & result)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = graph_->frameId();
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int marker_id = 1;
    for (const auto & task : result.tasks) {
      visualization_msgs::msg::Marker line;
      line.header = clear.header;
      line.ns = "route_tasks";
      line.id = marker_id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.10;
      line.color = taskColor(task.task_index);
      for (const auto & waypoint : task.waypoints) {
        geometry_msgs::msg::Point point;
        point.x = waypoint.position[0];
        point.y = waypoint.position[1];
        point.z = waypoint.position[2] + 0.28;
        line.points.push_back(point);
      }
      markers.markers.push_back(std::move(line));

      visualization_msgs::msg::Marker label;
      label.header = clear.header;
      label.ns = "route_task_labels";
      label.id = marker_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.orientation.w = 1.0;
      const auto & middle = task.waypoints[task.waypoints.size() / 2U];
      label.pose.position.x = middle.position[0];
      label.pose.position.y = middle.position[1];
      label.pose.position.z = middle.position[2] + 0.65;
      label.scale.z = 0.30;
      label.color = taskColor(task.task_index);
      label.text = "T" + std::to_string(task.task_index) + " " +
        task.resolved_controller_mode + " / " + task.gait_command;
      markers.markers.push_back(std::move(label));

      for (std::size_t waypoint_index = 0; waypoint_index < task.waypoints.size();
        ++waypoint_index)
      {
        const auto & waypoint = task.waypoints[waypoint_index];
        const bool route_goal = task.is_route_goal &&
          waypoint_index + 1U == task.waypoints.size();
        if (!waypoint.is_corner && !waypoint.is_slope && !route_goal) {
          continue;
        }
        visualization_msgs::msg::Marker point;
        point.header = clear.header;
        point.ns = "semantic_waypoints";
        point.id = marker_id++;
        point.type = visualization_msgs::msg::Marker::SPHERE;
        point.action = visualization_msgs::msg::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.pose.position.x = waypoint.position[0];
        point.pose.position.y = waypoint.position[1];
        point.pose.position.z = waypoint.position[2] + 0.28;
        point.scale.x = point.scale.y = point.scale.z = route_goal ? 0.30 : 0.22;
        point.color.a = 1.0F;
        if (route_goal) {
          point.color.r = 1.0F;
          point.color.g = 0.10F;
          point.color.b = 0.10F;
        } else if (waypoint.is_corner) {
          point.color.r = 1.0F;
          point.color.g = 0.65F;
          point.color.b = 0.05F;
        } else {
          point.color.r = 0.75F;
          point.color.g = 0.25F;
          point.color.b = 1.0F;
        }
        markers.markers.push_back(std::move(point));
      }
    }
    markers_publisher_->publish(markers);
  }

  std::string graph_file_;
  std::uint64_t route_sequence_{0};
  std::unique_ptr<route3d_dijkstra_planner::TopologyGraph> graph_;
  std::unique_ptr<RouteSlicer> slicer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Publisher<TaskArrayMessage>::SharedPtr tasks_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tasks_json_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
};

}  // namespace route3d_route_slicer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<route3d_route_slicer::RouteSlicerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route3d_route_slicer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
