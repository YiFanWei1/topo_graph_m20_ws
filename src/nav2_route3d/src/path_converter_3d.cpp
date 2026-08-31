#include "nav2_route3d/path_converter_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <nav2_msgs/msg/route_edge.hpp>
#include <nav2_msgs/msg/route_node.hpp>

namespace nav2_route3d
{
namespace
{

uint16_t toRouteMsgId(const uint32_t value)
{
  return static_cast<uint16_t>(std::min<uint32_t>(value, std::numeric_limits<uint16_t>::max()));
}

}  // namespace

std::vector<geometry_msgs::msg::PoseStamped> densifyRoutePoses(
  const Graph3D & graph,
  const Route3D & route,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const double density_m)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  if (route.node_ids.empty()) {
    return poses;
  }
  const double safe_density = std::max(0.05, density_m);
  for (size_t i = 0; i + 1U < route.node_ids.size(); ++i) {
    const auto & start = graph.node(route.node_ids[i]);
    const auto & end = graph.node(route.node_ids[i + 1U]);
    const auto distance = std::max(1.0e-6, graph.distance(start.node_id, end.node_id));
    const auto steps = std::max<size_t>(1U, static_cast<size_t>(std::ceil(distance / safe_density)));
    for (size_t s = 0; s < steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.stamp = stamp;
      pose.pose.position.x = start.pose.tx + (end.pose.tx - start.pose.tx) * t;
      pose.pose.position.y = start.pose.ty + (end.pose.ty - start.pose.ty) * t;
      pose.pose.position.z = start.pose.tz + (end.pose.tz - start.pose.tz) * t;
      pose.pose.orientation.w = 1.0;
      poses.push_back(pose);
    }
  }
  const auto & last = graph.node(route.node_ids.back());
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;
  pose.pose.position.x = last.pose.tx;
  pose.pose.position.y = last.pose.ty;
  pose.pose.position.z = last.pose.tz;
  pose.pose.orientation.w = 1.0;
  poses.push_back(pose);
  return poses;
}

nav_msgs::msg::Path routeToPath(
  const Graph3D & graph,
  const Route3D & route,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const double density_m)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses = densifyRoutePoses(graph, route, frame_id, stamp, density_m);
  return path;
}

nav2_msgs::msg::Route routeToMsg(
  const Graph3D & graph,
  const Route3D & route,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav2_msgs::msg::Route msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.route_cost = static_cast<float>(route.cost);
  for (const auto node_id : route.node_ids) {
    nav2_msgs::msg::RouteNode node_msg;
    const auto & node = graph.node(node_id);
    node_msg.nodeid = toRouteMsgId(node.node_id);
    node_msg.position = toPoint(node);
    msg.nodes.push_back(node_msg);
  }
  for (const auto edge_id : route.edge_ids) {
    nav2_msgs::msg::RouteEdge edge_msg;
    const auto & edge = graph.edge(edge_id);
    edge_msg.edgeid = toRouteMsgId(edge.edge_id);
    edge_msg.start = toPoint(graph.node(edge.start_id));
    edge_msg.end = toPoint(graph.node(edge.end_id));
    msg.edges.push_back(edge_msg);
  }
  return msg;
}

}  // namespace nav2_route3d
