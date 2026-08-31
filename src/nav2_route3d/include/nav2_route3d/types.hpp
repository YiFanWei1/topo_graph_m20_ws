#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>

namespace nav2_route3d
{

using NodeId = uint32_t;
using EdgeId = uint32_t;
using Metadata = nlohmann::json;

struct Pose3D
{
  double timestamp{0.0};
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct Coordinates3D
{
  std::string frame_id{"map"};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Edge3D
{
  EdgeId edge_id{0};
  NodeId start_id{0};
  NodeId end_id{0};
  double cost{0.0};
  bool bidirectional{true};
  Metadata metadata{Metadata::object()};
  std::vector<Metadata> operations;
};

struct Node3D
{
  NodeId node_id{0};
  Pose3D pose;
  Metadata metadata{Metadata::object()};
  std::unordered_set<NodeId> real_neighbors;
  std::unordered_set<NodeId> fake_neighbors;
};

struct Route3D
{
  std::vector<NodeId> node_ids;
  std::vector<EdgeId> edge_ids;
  double cost{0.0};
};

struct ScoreResult
{
  bool valid{true};
  double cost{0.0};
  std::string reason;
};

struct GoalIntent3D
{
  NodeId start_id{0};
  NodeId goal_id{0};
};

struct OperationResult
{
  bool triggered{false};
  bool reroute{false};
  std::vector<EdgeId> blocked_edge_ids;
  std::string scenario_profile;
  double speed_limit_mps{-1.0};
  std::string message;
};

inline geometry_msgs::msg::Point toPoint(const Pose3D & pose)
{
  geometry_msgs::msg::Point point;
  point.x = pose.tx;
  point.y = pose.ty;
  point.z = pose.tz;
  return point;
}

inline geometry_msgs::msg::Point toPoint(const Node3D & node)
{
  return toPoint(node.pose);
}

inline double getNumber(
  const Metadata & metadata,
  const std::string & key,
  const double default_value)
{
  if (!metadata.is_object() || !metadata.contains(key) || !metadata.at(key).is_number()) {
    return default_value;
  }
  return metadata.at(key).get<double>();
}

inline bool getBool(
  const Metadata & metadata,
  const std::string & key,
  const bool default_value)
{
  if (!metadata.is_object() || !metadata.contains(key) || !metadata.at(key).is_boolean()) {
    return default_value;
  }
  return metadata.at(key).get<bool>();
}

inline std::string getString(
  const Metadata & metadata,
  const std::string & key,
  const std::string & default_value)
{
  if (!metadata.is_object() || !metadata.contains(key) || !metadata.at(key).is_string()) {
    return default_value;
  }
  return metadata.at(key).get<std::string>();
}

}  // namespace nav2_route3d
