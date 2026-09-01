#include "obstacle_occlusion_extension/occlusion_extension.hpp"

#include <efficient_3d_local_planner_msgs/msg/voxel_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/time.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <atomic>
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
    inflation_config_.horizontal_fill_cells = declare_parameter<int>(
      "inflation.horizontal_fill_cells", 1);
    inflation_config_.hard_z_down = declare_parameter<double>("inflation.hard_z_down", 0.40);
    inflation_config_.hard_z_up = declare_parameter<double>("inflation.hard_z_up", 0.0);
    inflation_config_.soft_radius = declare_parameter<double>("inflation.soft_radius", 0.60);
    tf_timeout_ = declare_parameter<double>("tf.timeout", 0.05);
    const std::string grid_topic = declare_parameter<std::string>(
      "input.voxel_grid_topic", "/local_voxel_map/grid");
    const std::string odometry_topic = declare_parameter<std::string>(
      "input.odometry_topic", "/lio_odom_hf");
    const std::string output_topic = declare_parameter<std::string>(
      "output.extension_topic", "/local_voxel_map/occlusion_extension");
    const std::string augmented_grid_topic = declare_parameter<std::string>(
      "output.augmented_grid_topic", "/local_voxel_map/grid_with_occlusion");
    const std::string augmented_hard_topic = declare_parameter<std::string>(
      "output.augmented_hard_topic", "/local_voxel_map/hard_with_occlusion");
    const std::string augmented_soft_topic = declare_parameter<std::string>(
      "output.augmented_soft_topic", "/local_voxel_map/soft_cost_with_occlusion");
    if (!extension_config_.valid() || !inflation_config_.valid() ||
      !std::isfinite(tf_timeout_) || tf_timeout_ <= 0.0)
    {
      throw std::invalid_argument("occlusion extension, inflation, or TF parameters are invalid");
    }

    // 这是低频、深度 1 的可视化输出。使用 Reliable 可同时兼容 RViz 在配置加载阶段
    // 创建的 Reliable/Best-Effort 订阅，避免显示项因 QoS 不匹配而永远收不到点云。
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(1).reliable().durability_volatile());
    augmented_hard_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      augmented_hard_topic, rclcpp::QoS(1).reliable().durability_volatile());
    augmented_soft_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      augmented_soft_topic, rclcpp::QoS(1).reliable().durability_volatile());
    // 规划地图保持与原 mapper 相同的 SensorDataQoS。无论扩展开关或 TF 是否可用，
    // 每一帧输入 grid 都会在该话题发布，确保下游规划器不会因本节点等待位姿而断图。
    grid_publisher_ = create_publisher<VoxelGrid>(
      augmented_grid_topic, rclcpp::SensorDataQoS().keep_last(1));
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
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &ObstacleOcclusionExtensionNode::parametersCallback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "occlusion extension ready: enabled=%s distance=%.2fm range=[%.2f,%.2f]m "
      "inflation=[fill=%d z_down=%.2f z_up=%.2f soft=%.2f] "
      "grid=%s odom=%s cloud=%s augmented_grid=%s hard_injection=%s",
      enabled_.load() ? "true" : "false", extension_config_.distance,
      extension_config_.minimum_obstacle_range, extension_config_.maximum_obstacle_range,
      inflation_config_.horizontal_fill_cells, inflation_config_.hard_z_down,
      inflation_config_.hard_z_up, inflation_config_.soft_radius,
      grid_topic.c_str(), odometry_topic.c_str(), output_topic.c_str(),
      augmented_grid_topic.c_str(), enabled_.load() ? "enabled" : "disabled");
  }

private:
  using VoxelGrid = efficient_3d_local_planner_msgs::msg::VoxelGrid;

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool requested_enabled = enabled_.load();
    InflationConfig requested_inflation = inflation_config_;
    bool changed = false;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "extension.enabled") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "extension.enabled must be boolean";
          return result;
        }
        requested_enabled = parameter.as_bool();
      } else if (parameter.get_name() == "inflation.soft_radius") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason = "inflation.soft_radius must be a double";
          return result;
        }
        requested_inflation.soft_radius = parameter.as_double();
      }
    }
    if (!requested_inflation.valid()) {
      result.successful = false;
      result.reason = "inflation.soft_radius must be finite and non-negative";
      return result;
    }
    changed = requested_enabled != enabled_.load() ||
      std::abs(requested_inflation.soft_radius - inflation_config_.soft_radius) > 1e-9;
    if (!changed) {
      return result;
    }
    enabled_.store(requested_enabled);
    inflation_config_ = requested_inflation;
    VoxelGrid::SharedPtr latest_grid;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_grid = latest_grid_;
    }
    // Rebuild/pass through the latest grid before acknowledging the parameter
    // service, so the planner does not receive a new route with an old map mode.
    if (latest_grid) {
      gridCallback(latest_grid);
    }
    RCLCPP_INFO(
      get_logger(), "runtime profile applied: extension.enabled=%s inflation.soft_radius=%.3f",
      requested_enabled ? "true" : "false", requested_inflation.soft_radius);
    return result;
  }

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

  sensor_msgs::msg::PointCloud2 makeSoftCloud(
    const VoxelGrid & grid, const GridGeometry & geometry,
    const std::vector<std::uint32_t> & indices,
    const std::vector<std::uint8_t> & costs) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = grid.header;
    cloud.height = 1U;
    cloud.is_dense = true;
    const std::size_t count = std::min(indices.size(), costs.size());
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(count);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
    for (std::size_t i = 0; i < count; ++i) {
      const std::uint32_t index = indices[i];
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
      *intensity = static_cast<float>(costs[i]) / 254.0F;
      ++x;
      ++y;
      ++z;
      ++intensity;
    }
    return cloud;
  }

  void publishLayerClouds(
    const VoxelGrid & grid, const GridGeometry & geometry,
    const std::vector<std::uint32_t> & hard_indices,
    const std::vector<std::uint32_t> & soft_indices,
    const std::vector<std::uint8_t> & soft_costs)
  {
    if (augmented_hard_publisher_->get_subscription_count() > 0U) {
      augmented_hard_publisher_->publish(makeCloud(grid, geometry, hard_indices));
    }
    if (augmented_soft_publisher_->get_subscription_count() > 0U) {
      augmented_soft_publisher_->publish(makeSoftCloud(grid, geometry, soft_indices, soft_costs));
    }
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_grid_ = grid;
    }
    const GridGeometry geometry{
      grid->origin.x, grid->origin.y, grid->origin.z,
      static_cast<double>(grid->resolution), grid->size_x, grid->size_y, grid->size_z};
    if (!geometry.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "received invalid voxel grid");
      grid_publisher_->publish(*grid);
      return;
    }
    std::vector<std::uint32_t> added;
    if (!enabled_.load() || extension_config_.distance <= 0.0) {
      cloud_publisher_->publish(makeCloud(*grid, geometry, added));
      publishLayerClouds(
        *grid, geometry, grid->hard_occupied_indices, grid->soft_indices,
        grid->soft_cost_values);
      grid_publisher_->publish(*grid);
      return;
    }
    double observer_x = 0.0;
    double observer_y = 0.0;
    if (!observerInGridFrame(grid->header.frame_id, observer_x, observer_y)) {
      cloud_publisher_->publish(makeCloud(*grid, geometry, added));
      publishLayerClouds(
        *grid, geometry, grid->hard_occupied_indices, grid->soft_indices,
        grid->soft_cost_values);
      grid_publisher_->publish(*grid);
      return;
    }
    if (grid->raw_occupied_indices.empty() && !grid->hard_occupied_indices.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "voxel grid has hard cells but no raw occupancy; passing through without extension");
      cloud_publisher_->publish(makeCloud(*grid, geometry, added));
      publishLayerClouds(
        *grid, geometry, grid->hard_occupied_indices, grid->soft_indices,
        grid->soft_cost_values);
      grid_publisher_->publish(*grid);
      return;
    }
    added = computeOcclusionExtension(
      geometry, grid->raw_occupied_indices, observer_x, observer_y, extension_config_);
    const InflatedLayers extension_layers = inflateObstacleSeeds(
      geometry, added, inflation_config_);
    const InflatedLayers merged_layers = mergeInflatedLayers(
      grid->hard_occupied_indices, grid->soft_indices, grid->soft_cost_values,
      extension_layers, geometry.cellCount());
    VoxelGrid augmented = *grid;
    augmented.raw_occupied_indices = mergeHardIndices(
      grid->raw_occupied_indices, added, geometry.cellCount());
    augmented.hard_occupied_indices = merged_layers.hard_indices;
    augmented.soft_indices = merged_layers.soft_indices;
    augmented.soft_cost_values = merged_layers.soft_costs;
    cloud_publisher_->publish(makeCloud(*grid, geometry, extension_layers.hard_indices));
    publishLayerClouds(
      augmented, geometry, augmented.hard_occupied_indices, augmented.soft_indices,
      augmented.soft_cost_values);
    grid_publisher_->publish(augmented);
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "occlusion extension: revision=%lu raw=%zu added_raw=%zu added_hard=%zu "
      "hard=%zu->%zu soft=%zu->%zu observer=[%.2f %.2f]",
      grid->revision, grid->raw_occupied_indices.size(), added.size(),
      extension_layers.hard_indices.size(), grid->hard_occupied_indices.size(),
      augmented.hard_occupied_indices.size(), grid->soft_indices.size(),
      augmented.soft_indices.size(), observer_x, observer_y);
  }

  std::atomic<bool> enabled_{true};
  ExtensionConfig extension_config_;
  InflationConfig inflation_config_;
  double tf_timeout_{0.05};
  std::mutex mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
  VoxelGrid::SharedPtr latest_grid_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr augmented_hard_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr augmented_soft_publisher_;
  rclcpp::Publisher<VoxelGrid>::SharedPtr grid_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<VoxelGrid>::SharedPtr grid_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;
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
