#include "obstacle_occlusion_extension/occlusion_extension.hpp"

#include <efficient_3d_local_planner_msgs/msg/voxel_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/time.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace obstacle_occlusion_extension
{

class ObstacleOcclusionExtensionNode : public rclcpp::Node
{
public:
  ObstacleOcclusionExtensionNode()
  : Node("obstacle_occlusion_extension"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    enabled_ = declare_parameter<bool>("extension.enabled", true);
    extension_config_.distance = declare_parameter<double>("extension.distance", 0.60);
    extension_config_.minimum_obstacle_range = declare_parameter<double>(
      "extension.minimum_obstacle_range", 0.20);
    extension_config_.maximum_obstacle_range = declare_parameter<double>(
      "extension.maximum_obstacle_range", 8.0);
    tf_timeout_ = declare_parameter<double>("tf.timeout", 0.05);
    const std::string grid_topic = declare_parameter<std::string>(
      "input.voxel_grid_topic", "/local_voxel_map/grid");
    const std::string odometry_topic = declare_parameter<std::string>(
      "input.odometry_topic", "/lio_odom_hf");
    const std::string output_topic = declare_parameter<std::string>(
      "output.extension_topic", "/local_voxel_map/occlusion_extension");
    if (!extension_config_.valid() || !std::isfinite(tf_timeout_) || tf_timeout_ <= 0.0) {
      throw std::invalid_argument("occlusion extension distances and TF timeout are invalid");
    }

    // 这是低频、深度 1 的可视化输出。使用 Reliable 可同时兼容 RViz 在配置加载阶段
    // 创建的 Reliable/Best-Effort 订阅，避免显示项因 QoS 不匹配而永远收不到点云。
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(1).reliable().durability_volatile());
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS().keep_last(100),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_odometry_ = std::move(message);
      });
    grid_subscription_ =
      create_subscription<efficient_3d_local_planner_msgs::msg::VoxelGrid>(
      grid_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ObstacleOcclusionExtensionNode::gridCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "occlusion extension ready: enabled=%s distance=%.2fm range=[%.2f,%.2f]m "
      "grid=%s odom=%s output=%s visualization_only=true",
      enabled_ ? "true" : "false", extension_config_.distance,
      extension_config_.minimum_obstacle_range, extension_config_.maximum_obstacle_range,
      grid_topic.c_str(), odometry_topic.c_str(), output_topic.c_str());
  }

private:
  using VoxelGrid = efficient_3d_local_planner_msgs::msg::VoxelGrid;

  sensor_msgs::msg::PointCloud2 makeCloud(
    const VoxelGrid & grid, const GridGeometry & geometry,
    const std::vector<std::uint32_t> & indices) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = grid.header;
    cloud.height = 1U;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(indices.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const std::uint32_t index : indices) {
      const std::uint32_t local_x = index % geometry.size_x;
      const std::uint32_t yz = index / geometry.size_x;
      const std::uint32_t local_y = yz % geometry.size_y;
      const std::uint32_t local_z = yz / geometry.size_y;
      *x = static_cast<float>(
        geometry.origin_x + (static_cast<double>(local_x) + 0.5) * geometry.resolution);
      *y = static_cast<float>(
        geometry.origin_y + (static_cast<double>(local_y) + 0.5) * geometry.resolution);
      *z = static_cast<float>(
        geometry.origin_z + (static_cast<double>(local_z) + 0.5) * geometry.resolution);
      ++x;
      ++y;
      ++z;
    }
    return cloud;
  }

  bool observerInGridFrame(const std::string & grid_frame, double & x, double & y)
  {
    nav_msgs::msg::Odometry::SharedPtr odometry;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odometry = latest_odometry_;
    }
    if (!odometry) {return false;}
    const std::string odometry_frame = odometry->header.frame_id.empty() ?
      grid_frame : odometry->header.frame_id;
    geometry_msgs::msg::PointStamped source;
    source.header = odometry->header;
    source.header.frame_id = odometry_frame;
    source.point = odometry->pose.pose.position;
    if (odometry_frame == grid_frame) {
      x = source.point.x;
      y = source.point.y;
      return std::isfinite(x) && std::isfinite(y);
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        grid_frame, odometry_frame, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
      geometry_msgs::msg::PointStamped transformed;
      tf2::doTransform(source, transformed, transform);
      x = transformed.point.x;
      y = transformed.point.y;
      return std::isfinite(x) && std::isfinite(y);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot transform observer %s -> %s: %s",
        odometry_frame.c_str(), grid_frame.c_str(), error.what());
      return false;
    }
  }

  void gridCallback(const VoxelGrid::SharedPtr grid)
  {
    const GridGeometry geometry{
      grid->origin.x, grid->origin.y, grid->origin.z,
      static_cast<double>(grid->resolution), grid->size_x, grid->size_y, grid->size_z};
    if (!geometry.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "received invalid voxel grid");
      return;
    }
    if (!enabled_ || extension_config_.distance <= 0.0) {
      publisher_->publish(makeCloud(*grid, geometry, {}));
      return;
    }
    double observer_x = 0.0;
    double observer_y = 0.0;
    if (!observerInGridFrame(grid->header.frame_id, observer_x, observer_y)) {
      publisher_->publish(makeCloud(*grid, geometry, {}));
      return;
    }
    const auto added = computeOcclusionExtension(
      geometry, grid->hard_occupied_indices, observer_x, observer_y, extension_config_);
    publisher_->publish(makeCloud(*grid, geometry, added));
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "occlusion extension: revision=%lu hard=%zu added=%zu observer=[%.2f %.2f]",
      grid->revision, grid->hard_occupied_indices.size(), added.size(), observer_x, observer_y);
  }

  bool enabled_{true};
  ExtensionConfig extension_config_;
  double tf_timeout_{0.05};
  std::mutex mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<VoxelGrid>::SharedPtr grid_subscription_;
};

}  // namespace obstacle_occlusion_extension

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<
      obstacle_occlusion_extension::ObstacleOcclusionExtensionNode>());
  rclcpp::shutdown();
  return 0;
}
