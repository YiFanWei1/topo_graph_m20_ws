#include "nav2_route3d/route_server_3d.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <tf2/time.h>

namespace nav2_route3d
{

namespace
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

builtin_interfaces::msg::Duration durationToMsg(const rclcpp::Duration & duration)
{
  return static_cast<builtin_interfaces::msg::Duration>(duration);
}

void setColor(Marker & marker, const float red, const float green, const float blue, const float alpha)
{
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = alpha;
}

Marker baseMarker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & marker_namespace,
  const int id,
  const int type)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

Marker deleteAllMarker(const std::string & frame_id, const rclcpp::Time & stamp)
{
  auto marker = baseMarker(frame_id, stamp, "delete_all", 0, Marker::SPHERE);
  marker.action = Marker::DELETEALL;
  return marker;
}

bool isShortcutEdge(const Edge3D & edge)
{
  return getString(edge.metadata, "edge_source", "") == "radius_visibility" ||
         getString(edge.metadata, "edge_type", "") == "shortcut";
}

bool isRouteOperationEdge(const Edge3D & edge)
{
  const auto edge_type = getString(edge.metadata, "edge_type", "");
  return edge_type == "stair_like_transition" ||
         edge_type == "indoor_outdoor_transition" ||
         edge_type == "steep_slope";
}

geometry_msgs::msg::Point edgeMidpoint(const Graph3D & graph, const Edge3D & edge)
{
  const auto & start = graph.node(edge.start_id).pose;
  const auto & end = graph.node(edge.end_id).pose;
  geometry_msgs::msg::Point point;
  point.x = 0.5 * (start.tx + end.tx);
  point.y = 0.5 * (start.ty + end.ty);
  point.z = 0.5 * (start.tz + end.tz);
  return point;
}

}  // namespace


RouteServer3D::RouteServer3D(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("route_server", "", options),
  graph_("map")
{
  declare_parameter("route_frame", "map");
  declare_parameter("base_frame", "base_link");
  declare_parameter("graph_filepath", "");
  declare_parameter("path_density", 0.5);
  declare_parameter("max_planning_time", 2.0);
  declare_parameter("radius_to_achieve_node", 1.0);
  declare_parameter("tracker_update_rate", 20.0);
  declare_parameter("visualization_enabled", true);
  declare_parameter("publish_graph_markers", true);
  declare_parameter("publish_route_markers", true);
  declare_parameter("publish_route_event_markers", true);
  declare_parameter("publish_rejected_neighbor_markers", false);
  declare_parameter("graph_node_marker_scale", 0.08);
  declare_parameter("graph_edge_marker_scale", 0.025);
  declare_parameter("route_marker_scale", 0.08);
  declare_parameter("route_event_marker_scale", 0.25);
}

nav2_util::CallbackReturn RouteServer3D::on_configure(const rclcpp_lifecycle::State &)
{
  route_frame_ = get_parameter("route_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  graph_filepath_ = get_parameter("graph_filepath").as_string();
  path_density_m_ = get_parameter("path_density").as_double();
  max_planning_time_s_ = get_parameter("max_planning_time").as_double();
  radius_to_achieve_node_ = get_parameter("radius_to_achieve_node").as_double();
  tracker_update_rate_ = get_parameter("tracker_update_rate").as_double();
  visualization_enabled_ = get_parameter("visualization_enabled").as_bool();
  publish_graph_markers_ = get_parameter("publish_graph_markers").as_bool();
  publish_route_markers_ = get_parameter("publish_route_markers").as_bool();
  publish_route_event_markers_ = get_parameter("publish_route_event_markers").as_bool();
  publish_rejected_neighbor_markers_ = get_parameter("publish_rejected_neighbor_markers").as_bool();
  graph_node_marker_scale_ = get_parameter("graph_node_marker_scale").as_double();
  graph_edge_marker_scale_ = get_parameter("graph_edge_marker_scale").as_double();
  route_marker_scale_ = get_parameter("route_marker_scale").as_double();
  route_event_marker_scale_ = get_parameter("route_event_marker_scale").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!graph_filepath_.empty() && !reloadGraph(graph_filepath_)) {
    RCLCPP_ERROR(get_logger(), "Failed to load 3D route graph: %s", graph_filepath_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  operations_ = defaultOperations();
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/route_plan", rclcpp::SystemDefaultsQoS());
  route_events_pub_ = create_publisher<std_msgs::msg::String>("route_events", 10);
  if (visualization_enabled_) {
    const auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    if (publish_graph_markers_) {
      graph_markers_pub_ = create_publisher<MarkerArray>("route_graph_markers", marker_qos);
    }
    if (publish_route_markers_) {
      route_markers_pub_ = create_publisher<MarkerArray>("route_markers", marker_qos);
    }
    if (publish_route_event_markers_) {
      route_event_markers_pub_ = create_publisher<MarkerArray>("route_event_markers", marker_qos);
    }
  }

  compute_route_server_ = rclcpp_action::create_server<ComputeRoute>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "compute_route",
    std::bind(&RouteServer3D::handleComputeRouteGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RouteServer3D::handleComputeRouteCancel, this, std::placeholders::_1),
    std::bind(&RouteServer3D::handleComputeRouteAccepted, this, std::placeholders::_1));

  compute_and_track_route_server_ = rclcpp_action::create_server<ComputeAndTrackRoute>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "compute_and_track_route",
    std::bind(
      &RouteServer3D::handleComputeAndTrackRouteGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&RouteServer3D::handleComputeAndTrackRouteCancel, this, std::placeholders::_1),
    std::bind(&RouteServer3D::handleComputeAndTrackRouteAccepted, this, std::placeholders::_1));

  set_graph_service_ = create_service<nav2_msgs::srv::SetRouteGraph>(
    std::string(get_name()) + "/set_route_graph",
    std::bind(
      &RouteServer3D::setRouteGraph, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(get_logger(), "Configured nav2_route3d route server");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RouteServer3D::on_activate(const rclcpp_lifecycle::State &)
{
  path_pub_->on_activate();
  route_events_pub_->on_activate();
  if (graph_markers_pub_) {
    graph_markers_pub_->on_activate();
  }
  if (route_markers_pub_) {
    route_markers_pub_->on_activate();
  }
  if (route_event_markers_pub_) {
    route_event_markers_pub_->on_activate();
  }
  publishGraphMarkers();
  createBond();
  RCLCPP_INFO(get_logger(), "Activated nav2_route3d route server");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RouteServer3D::on_deactivate(const rclcpp_lifecycle::State &)
{
  destroyBond();
  path_pub_->on_deactivate();
  route_events_pub_->on_deactivate();
  if (graph_markers_pub_) {
    graph_markers_pub_->on_deactivate();
  }
  if (route_markers_pub_) {
    route_markers_pub_->on_deactivate();
  }
  if (route_event_markers_pub_) {
    route_event_markers_pub_->on_deactivate();
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RouteServer3D::on_cleanup(const rclcpp_lifecycle::State &)
{
  compute_route_server_.reset();
  compute_and_track_route_server_.reset();
  set_graph_service_.reset();
  path_pub_.reset();
  route_events_pub_.reset();
  graph_markers_pub_.reset();
  route_markers_pub_.reset();
  route_event_markers_pub_.reset();
  planner_.reset();
  scorer_.reset();
  goal_intent_extractor_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RouteServer3D::on_shutdown(const rclcpp_lifecycle::State &)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse RouteServer3D::handleComputeRouteGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ComputeRoute::Goal>)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  return graph_.nodes().empty() ? rclcpp_action::GoalResponse::REJECT :
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RouteServer3D::handleComputeRouteCancel(
  const std::shared_ptr<ComputeRouteGoalHandle>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RouteServer3D::handleComputeRouteAccepted(
  const std::shared_ptr<ComputeRouteGoalHandle> goal_handle)
{
  std::thread([this, goal_handle]() {executeComputeRoute(goal_handle);}).detach();
}

void RouteServer3D::executeComputeRoute(const std::shared_ptr<ComputeRouteGoalHandle> goal_handle)
{
  const auto start_time = now();
  auto result = std::make_shared<ComputeRoute::Result>();
  try {
    Route3D route;
    nav_msgs::msg::Path path;
    nav2_msgs::msg::Route route_msg;
    const auto stamp = now();
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      const auto intent = extractIntent(*goal_handle->get_goal());
      route = computeRouteForIntent(intent);
      path = routeToPath(graph_, route, route_frame_, stamp, path_density_m_);
      route_msg = routeToMsg(graph_, route, route_frame_, stamp);
      publishRouteEvents(route);
      publishRouteMarkers(route);
      publishRouteEventMarkers(route);
    }
    path_pub_->publish(path);

    result->planning_time = durationToMsg(stamp - start_time);
    result->path = path;
    result->route = route_msg;
    result->error_code = ComputeRoute::Result::NONE;
    goal_handle->succeed(result);
  } catch (const std::exception & ex) {
    result->planning_time = durationToMsg(now() - start_time);
    result->error_code = ComputeRoute::Result::NO_VALID_ROUTE;
    RCLCPP_WARN(get_logger(), "ComputeRoute failed: %s", ex.what());
    goal_handle->abort(result);
  }
}

rclcpp_action::GoalResponse RouteServer3D::handleComputeAndTrackRouteGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ComputeAndTrackRoute::Goal>)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  return graph_.nodes().empty() ? rclcpp_action::GoalResponse::REJECT :
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RouteServer3D::handleComputeAndTrackRouteCancel(
  const std::shared_ptr<ComputeAndTrackRouteGoalHandle>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RouteServer3D::handleComputeAndTrackRouteAccepted(
  const std::shared_ptr<ComputeAndTrackRouteGoalHandle> goal_handle)
{
  std::thread([this, goal_handle]() {executeComputeAndTrackRoute(goal_handle);}).detach();
}

void RouteServer3D::executeComputeAndTrackRoute(
  const std::shared_ptr<ComputeAndTrackRouteGoalHandle> goal_handle)
{
  const auto start_time = now();
  auto result = std::make_shared<ComputeAndTrackRoute::Result>();
  try {
    Route3D route;
    nav_msgs::msg::Path path;
    nav2_msgs::msg::Route route_msg;
    const auto stamp = now();
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      const auto intent = extractIntent(*goal_handle->get_goal());
      route = computeRouteForIntent(intent);
      path = routeToPath(graph_, route, route_frame_, stamp, path_density_m_);
      route_msg = routeToMsg(graph_, route, route_frame_, stamp);
      publishRouteEvents(route);
      publishRouteMarkers(route);
      publishRouteEventMarkers(route);
    }
    path_pub_->publish(path);

    auto feedback = std::make_shared<ComputeAndTrackRoute::Feedback>();
    feedback->route = route_msg;
    feedback->path = path;
    feedback->rerouted = false;
    if (!route.node_ids.empty()) {
      feedback->last_node_id = static_cast<uint16_t>(route.node_ids.front());
      feedback->next_node_id = static_cast<uint16_t>(
        route.node_ids.size() > 1U ? route.node_ids[1] : route.node_ids.front());
    }
    if (!route.edge_ids.empty()) {
      feedback->current_edge_id = static_cast<uint16_t>(route.edge_ids.front());
    }
    goal_handle->publish_feedback(feedback);

    result->execution_duration = durationToMsg(now() - start_time);
    result->error_code = ComputeAndTrackRoute::Result::NONE;
    goal_handle->succeed(result);
  } catch (const std::exception & ex) {
    result->execution_duration = durationToMsg(now() - start_time);
    result->error_code = ComputeAndTrackRoute::Result::NO_VALID_ROUTE;
    RCLCPP_WARN(get_logger(), "ComputeAndTrackRoute failed: %s", ex.what());
    goal_handle->abort(result);
  }
}

template<typename GoalT>
GoalIntent3D RouteServer3D::extractIntent(const GoalT & goal) const
{
  if (goal.use_poses) {
    const auto start = goal.use_start ? goal.start : currentRobotPose();
    return goal_intent_extractor_->fromPoses(start, goal.goal);
  }
  return goal_intent_extractor_->fromIds(goal.start_id, goal.goal_id);
}

geometry_msgs::msg::PoseStamped RouteServer3D::currentRobotPose() const
{
  if (!tf_buffer_) {
    throw std::runtime_error("TF buffer is not configured");
  }
  const auto tf = tf_buffer_->lookupTransform(
    route_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
  geometry_msgs::msg::PoseStamped pose;
  pose.header = tf.header;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;
  pose.pose.orientation = tf.transform.rotation;
  return pose;
}

Route3D RouteServer3D::computeRouteForIntent(const GoalIntent3D & intent) const
{
  return planner_->plan(intent.start_id, intent.goal_id, blocked_edges_, max_planning_time_s_);
}

void RouteServer3D::publishRouteEvents(const Route3D & route)
{
  nlohmann::json events = nlohmann::json::array();
  for (const auto edge_id : route.edge_ids) {
    const auto & edge = graph_.edge(edge_id);
    for (const auto & op : operations_) {
      const auto result = op->onEdgeEnter(edge);
      if (result.triggered) {
        events.push_back(
          {
            {"edge_id", edge_id},
            {"operation", op->name()},
            {"scenario_profile", result.scenario_profile},
            {"speed_limit_mps", result.speed_limit_mps},
            {"message", result.message}
          });
      }
    }
  }
  std_msgs::msg::String msg;
  msg.data = events.dump();
  route_events_pub_->publish(msg);
}


void RouteServer3D::publishGraphMarkers()
{
  if (!visualization_enabled_ || !publish_graph_markers_ || !graph_markers_pub_ ||
    !graph_markers_pub_->is_activated())
  {
    return;
  }

  const auto stamp = now();
  MarkerArray markers;
  markers.markers.push_back(deleteAllMarker(route_frame_, stamp));

  auto nodes = baseMarker(route_frame_, stamp, "route_graph_nodes", 1, Marker::POINTS);
  nodes.scale.x = graph_node_marker_scale_;
  nodes.scale.y = graph_node_marker_scale_;
  setColor(nodes, 0.15F, 0.75F, 1.0F, 0.65F);

  auto anchors = baseMarker(route_frame_, stamp, "route_graph_shortcut_anchors", 2, Marker::POINTS);
  anchors.scale.x = graph_node_marker_scale_ * 1.8;
  anchors.scale.y = graph_node_marker_scale_ * 1.8;
  setColor(anchors, 1.0F, 0.86F, 0.15F, 0.9F);

  auto sequential_edges = baseMarker(route_frame_, stamp, "route_graph_sequential_edges", 3, Marker::LINE_LIST);
  sequential_edges.scale.x = graph_edge_marker_scale_;
  setColor(sequential_edges, 0.55F, 0.62F, 0.68F, 0.22F);

  auto shortcut_edges = baseMarker(route_frame_, stamp, "route_graph_shortcut_edges", 4, Marker::LINE_LIST);
  shortcut_edges.scale.x = graph_edge_marker_scale_ * 1.4;
  setColor(shortcut_edges, 1.0F, 0.48F, 0.05F, 0.38F);

  auto operation_edges = baseMarker(route_frame_, stamp, "route_graph_operation_edges", 5, Marker::LINE_LIST);
  operation_edges.scale.x = graph_edge_marker_scale_ * 2.2;
  setColor(operation_edges, 1.0F, 0.1F, 0.75F, 0.85F);

  auto rejected_neighbors = baseMarker(route_frame_, stamp, "route_graph_rejected_neighbors", 6, Marker::LINE_LIST);
  rejected_neighbors.scale.x = graph_edge_marker_scale_;
  setColor(rejected_neighbors, 1.0F, 0.1F, 0.05F, 0.12F);

  nodes.points.reserve(graph_.nodes().size());
  for (const auto & [node_id, node] : graph_.nodes()) {
    (void)node_id;
    nodes.points.push_back(toPoint(node));
    if (getBool(node.metadata, "shortcut_anchor", false)) {
      anchors.points.push_back(toPoint(node));
    }
    if (publish_rejected_neighbor_markers_) {
      for (const auto neighbor_id : node.fake_neighbors) {
        if (node.node_id >= neighbor_id || !graph_.hasNode(neighbor_id)) {
          continue;
        }
        rejected_neighbors.points.push_back(toPoint(node));
        rejected_neighbors.points.push_back(toPoint(graph_.node(neighbor_id)));
      }
    }
  }

  for (const auto & [edge_id, edge] : graph_.edges()) {
    (void)edge_id;
    auto & edge_marker = isShortcutEdge(edge) ? shortcut_edges : sequential_edges;
    edge_marker.points.push_back(toPoint(graph_.node(edge.start_id)));
    edge_marker.points.push_back(toPoint(graph_.node(edge.end_id)));
    if (isRouteOperationEdge(edge)) {
      operation_edges.points.push_back(toPoint(graph_.node(edge.start_id)));
      operation_edges.points.push_back(toPoint(graph_.node(edge.end_id)));
    }
  }

  markers.markers.push_back(nodes);
  markers.markers.push_back(anchors);
  markers.markers.push_back(sequential_edges);
  markers.markers.push_back(shortcut_edges);
  markers.markers.push_back(operation_edges);
  if (publish_rejected_neighbor_markers_) {
    markers.markers.push_back(rejected_neighbors);
  }
  graph_markers_pub_->publish(markers);
}

void RouteServer3D::publishRouteMarkers(const Route3D & route)
{
  if (!visualization_enabled_ || !publish_route_markers_ || !route_markers_pub_ ||
    !route_markers_pub_->is_activated())
  {
    return;
  }

  const auto stamp = now();
  MarkerArray markers;
  markers.markers.push_back(deleteAllMarker(route_frame_, stamp));

  auto route_line = baseMarker(route_frame_, stamp, "active_route_line", 1, Marker::LINE_LIST);
  route_line.scale.x = route_marker_scale_;
  setColor(route_line, 0.0F, 1.0F, 0.35F, 1.0F);

  auto route_nodes = baseMarker(route_frame_, stamp, "active_route_nodes", 2, Marker::POINTS);
  route_nodes.scale.x = route_marker_scale_ * 1.8;
  route_nodes.scale.y = route_marker_scale_ * 1.8;
  setColor(route_nodes, 0.92F, 1.0F, 0.92F, 0.95F);

  auto route_shortcuts = baseMarker(route_frame_, stamp, "active_route_shortcuts", 3, Marker::LINE_LIST);
  route_shortcuts.scale.x = route_marker_scale_ * 1.4;
  setColor(route_shortcuts, 1.0F, 0.55F, 0.02F, 1.0F);

  auto route_operations = baseMarker(route_frame_, stamp, "active_route_operations", 4, Marker::LINE_LIST);
  route_operations.scale.x = route_marker_scale_ * 1.7;
  setColor(route_operations, 1.0F, 0.05F, 0.68F, 1.0F);

  for (size_t i = 0; i < route.node_ids.size(); ++i) {
    route_nodes.points.push_back(toPoint(graph_.node(route.node_ids[i])));
    if (i + 1U < route.node_ids.size()) {
      route_line.points.push_back(toPoint(graph_.node(route.node_ids[i])));
      route_line.points.push_back(toPoint(graph_.node(route.node_ids[i + 1U])));
    }
  }

  for (const auto edge_id : route.edge_ids) {
    const auto & edge = graph_.edge(edge_id);
    if (isShortcutEdge(edge)) {
      route_shortcuts.points.push_back(toPoint(graph_.node(edge.start_id)));
      route_shortcuts.points.push_back(toPoint(graph_.node(edge.end_id)));
    }
    if (isRouteOperationEdge(edge)) {
      route_operations.points.push_back(toPoint(graph_.node(edge.start_id)));
      route_operations.points.push_back(toPoint(graph_.node(edge.end_id)));
    }
  }

  if (!route.node_ids.empty()) {
    auto start = baseMarker(route_frame_, stamp, "active_route_start", 5, Marker::SPHERE);
    start.pose.position = toPoint(graph_.node(route.node_ids.front()));
    start.scale.x = route_marker_scale_ * 3.0;
    start.scale.y = route_marker_scale_ * 3.0;
    start.scale.z = route_marker_scale_ * 3.0;
    setColor(start, 0.1F, 1.0F, 0.1F, 1.0F);
    markers.markers.push_back(start);

    auto goal = baseMarker(route_frame_, stamp, "active_route_goal", 6, Marker::SPHERE);
    goal.pose.position = toPoint(graph_.node(route.node_ids.back()));
    goal.scale.x = route_marker_scale_ * 3.0;
    goal.scale.y = route_marker_scale_ * 3.0;
    goal.scale.z = route_marker_scale_ * 3.0;
    setColor(goal, 1.0F, 0.1F, 0.1F, 1.0F);
    markers.markers.push_back(goal);
  }

  markers.markers.push_back(route_line);
  markers.markers.push_back(route_nodes);
  markers.markers.push_back(route_shortcuts);
  markers.markers.push_back(route_operations);
  route_markers_pub_->publish(markers);
}

void RouteServer3D::publishRouteEventMarkers(const Route3D & route)
{
  if (!visualization_enabled_ || !publish_route_event_markers_ || !route_event_markers_pub_ ||
    !route_event_markers_pub_->is_activated())
  {
    return;
  }

  const auto stamp = now();
  MarkerArray markers;
  markers.markers.push_back(deleteAllMarker(route_frame_, stamp));

  auto event_edges = baseMarker(route_frame_, stamp, "route_event_edges", 1, Marker::LINE_LIST);
  event_edges.scale.x = route_marker_scale_ * 1.9;
  setColor(event_edges, 1.0F, 0.9F, 0.05F, 1.0F);

  int text_id = 2;
  for (const auto edge_id : route.edge_ids) {
    const auto & edge = graph_.edge(edge_id);
    for (const auto & op : operations_) {
      const auto result = op->onEdgeEnter(edge);
      if (!result.triggered) {
        continue;
      }
      event_edges.points.push_back(toPoint(graph_.node(edge.start_id)));
      event_edges.points.push_back(toPoint(graph_.node(edge.end_id)));

      auto text = baseMarker(route_frame_, stamp, "route_event_labels", text_id++, Marker::TEXT_VIEW_FACING);
      text.pose.position = edgeMidpoint(graph_, edge);
      text.pose.position.z += route_event_marker_scale_;
      text.scale.z = route_event_marker_scale_;
      text.text = op->name();
      if (result.speed_limit_mps >= 0.0) {
        text.text += "\n" + std::to_string(result.speed_limit_mps) + " m/s";
      }
      setColor(text, 1.0F, 1.0F, 0.65F, 1.0F);
      markers.markers.push_back(text);
    }
  }

  markers.markers.push_back(event_edges);
  route_event_markers_pub_->publish(markers);
}

bool RouteServer3D::reloadGraph(const std::string & graph_filepath)
{
  try {
    graph_ = loadGraph3D(graph_filepath);
    route_frame_ = graph_.frameId().empty() ? route_frame_ : graph_.frameId();
    scorer_ = std::make_unique<EdgeScorer3D>(graph_);
    planner_ = std::make_unique<RoutePlanner3D>(graph_, *scorer_);
    goal_intent_extractor_ = std::make_unique<GoalIntentExtractor3D>(graph_);
    graph_filepath_ = graph_filepath;
    RCLCPP_INFO(
      get_logger(), "Loaded 3D route graph: %zu nodes, %zu edges",
      graph_.nodes().size(), graph_.edges().size());
    publishGraphMarkers();
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to reload 3D route graph: %s", ex.what());
    return false;
  }
}

void RouteServer3D::setRouteGraph(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  response->success = reloadGraph(request->graph_filepath);
}

}  // namespace nav2_route3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_route3d::RouteServer3D>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
