#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace route3d_web_console
{

enum class CloudStream : std::uint8_t {kStaticMap = 1, kLiveRegistered = 2, kRawLidar = 3};

struct CloudPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct CloudFrame
{
  std::string frame_id;
  std::uint64_t stamp_ns{0};
  std::vector<CloudPoint> points;
  bool has_intensity{false};
};

class PointCloudCodec
{
public:
  static CloudFrame fromRos(
    const sensor_msgs::msg::PointCloud2 & message, float voxel_size_m,
    float maximum_range_m, std::size_t maximum_points);
  static CloudFrame loadPcd(
    const std::filesystem::path & path, const std::string & frame_id,
    float voxel_size_m, std::size_t maximum_points = 0);
  static std::vector<std::vector<std::uint8_t>> encode(
    const CloudFrame & cloud, CloudStream stream, std::uint32_t sequence,
    std::size_t chunk_points);
};

}  // namespace route3d_web_console
