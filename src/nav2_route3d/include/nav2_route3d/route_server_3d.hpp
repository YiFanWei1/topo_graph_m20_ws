#pragma once

#include <memory>
#include <mutex>
#include <unordered_set>

#include <nav2_msgs/action/compute_and_track_route.hpp>
#include <nav2_msgs/action/compute_route.hpp>
#include <nav2_msgs/srv/set_route_graph.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmw/types.h>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_route3d/edge_scorer_3d.hpp"
#include "nav2_route3d/file_loader_3d.hpp"
#include "nav2_route3d/goal_intent_extractor_3d.hpp"
#include "nav2_route3d/path_converter_3d.hpp"
#include "nav2_route3d/route_operations_3d.hpp"
#include "nav2_route3d/route_planner_3d.hpp"
#include "nav2_route3d/route_tracker_3d.hpp"

namespace nav2_route3d
{

class RouteServer3D : public nav2_util::LifecycleNode
{
public:
  using ComputeRoute = nav2_msgs::action::ComputeRoute;
  using ComputeAndTrackRoute = nav2_msgs::action::ComputeAndTrackRoute;

  explicit RouteServer3D(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  using ComputeRouteGoalHandle = rclcpp_action::ServerGoalHandle<ComputeRoute>;
  using ComputeAndTrackRouteGoalHandle = rclcpp_action::ServerGoalHandle<ComputeAndTrackRoute>;

  rclcpp_action::GoalResponse handleComputeRouteGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputeRoute::Goal> goal);
  rclcpp_action::CancelResponse handleComputeRouteCancel(
    const std::shared_ptr<ComputeRouteGoalHandle> goal_handle);
  void handleComputeRouteAccepted(const std::shared_ptr<ComputeRouteGoalHandle> goal_handle);
  void executeComputeRoute(const std::shared_ptr<ComputeRouteGoalHandle> goal_handle);

  rclcpp_action::GoalResponse handleComputeAndTrackRouteGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputeAndTrackRoute::Goal> goal);
  rclcpp_action::CancelResponse handleComputeAndTrackRouteCancel(
    const std::shared_ptr<ComputeAndTrackRouteGoalHandle> goal_handle);
  void handleComputeAndTrackRouteAccepted(
    const std::shared_ptr<ComputeAndTrackRouteGoalHandle> goal_handle);
  void executeComputeAndTrackRoute(const std::shared_ptr<ComputeAndTrackRouteGoalHandle> goal_handle);

  template<typename GoalT>
  GoalIntent3D extractIntent(const GoalT & goal) const;
  geometry_msgs::msg::PoseStamped currentRobotPose() const;
  Route3D computeRouteForIntent(const GoalIntent3D & intent) const;
  void publishRouteEvents(const Route3D & route);
  void publishGraphMarkers();
  void publishRouteMarkers(const Route3D & route);
  void publishRouteEventMarkers(const Route3D & route);
  bool reloadGraph(const std::string & graph_filepath);
  void setRouteGraph(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Response> response);

  mutable std::mutex graph_mutex_;
  Graph3D graph_;
  std::string route_frame_{"map"};
  std::string base_frame_{"base_link"};
  std::string graph_filepath_;
  double path_density_m_{0.5};
  double max_planning_time_s_{2.0};
  double radius_to_achieve_node_{1.0};
  double tracker_update_rate_{20.0};
  bool visualization_enabled_{true};
  bool publish_graph_markers_{true};
  bool publish_route_markers_{true};
  bool publish_route_event_markers_{true};
  bool publish_rejected_neighbor_markers_{false};
  double graph_node_marker_scale_{0.08};
  double graph_edge_marker_scale_{0.025};
  double route_marker_scale_{0.08};
  double route_event_marker_scale_{0.25};
  std::unordered_set<EdgeId> blocked_edges_;

  std::unique_ptr<EdgeScorer3D> scorer_;
  std::unique_ptr<RoutePlanner3D> planner_;
  std::unique_ptr<GoalIntentExtractor3D> goal_intent_extractor_;
  std::vector<std::shared_ptr<RouteOperation3D>> operations_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::Server<ComputeRoute>::SharedPtr compute_route_server_;
  rclcpp_action::Server<ComputeAndTrackRoute>::SharedPtr compute_and_track_route_server_;
  rclcpp::Service<nav2_msgs::srv::SetRouteGraph>::SharedPtr set_graph_service_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr route_events_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    graph_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    route_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    route_event_markers_pub_;
};

}  // namespace nav2_route3d
