#include "route3d_web_console/point_cloud_codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace route3d_web_console
{
namespace
{

template<typename T>
void append(std::vector<std::uint8_t> & output, const T value)
{
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(&value);
  output.insert(output.end(), bytes, bytes + sizeof(T));
}

const sensor_msgs::msg::PointField * field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & candidate : cloud.fields) {
    if (candidate.name == name) {
      return &candidate;
    }
  }
  return nullptr;
}

float readFloat(const std::uint8_t * data, const sensor_msgs::msg::PointField * item)
{
  if (item == nullptr || item->datatype != sensor_msgs::msg::PointField::FLOAT32) {
    return 0.0F;
  }
  float value = 0.0F;
  std::memcpy(&value, data + item->offset, sizeof(float));
  return value;
}

struct Key
{
  int x;
  int y;
  int z;
  bool operator==(const Key & other) const {return x == other.x && y == other.y && z == other.z;}
};

struct KeyHash
{
  std::size_t operator()(const Key & value) const
  {
    std::size_t seed = std::hash<int>{}(value.x);
    seed ^= std::hash<int>{}(value.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(value.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

CloudFrame downsample(
  const CloudFrame & input, const float voxel, const float maximum_range,
  const std::size_t maximum_points)
{
  CloudFrame result;
  result.frame_id = input.frame_id;
  result.stamp_ns = input.stamp_ns;
  result.has_intensity = input.has_intensity;
  std::unordered_map<Key, CloudPoint, KeyHash> cells;
  cells.reserve(std::min<std::size_t>(input.points.size(), maximum_points * 2U + 1U));
  const float range_squared = maximum_range > 0.0F ? maximum_range * maximum_range :
    std::numeric_limits<float>::infinity();
  for (const auto & point : input.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      point.x * point.x + point.y * point.y + point.z * point.z > range_squared)
    {
      continue;
    }
    const Key key{static_cast<int>(std::floor(point.x / voxel)),
      static_cast<int>(std::floor(point.y / voxel)),
      static_cast<int>(std::floor(point.z / voxel))};
    cells.emplace(key, point);
  }
  result.points.reserve(cells.size());
  for (const auto & entry : cells) {
    result.points.push_back(entry.second);
  }
  if (maximum_points > 0 && result.points.size() > maximum_points) {
    std::vector<CloudPoint> limited;
    limited.reserve(maximum_points);
    for (std::size_t index = 0; index < maximum_points; ++index) {
      limited.push_back(result.points[index * result.points.size() / maximum_points]);
    }
    result.points = std::move(limited);
  }
  return result;
}

}  // namespace

CloudFrame PointCloudCodec::fromRos(
  const sensor_msgs::msg::PointCloud2 & message, const float voxel_size_m,
  const float maximum_range_m, const std::size_t maximum_points)
{
  const auto * x = field(message, "x");
  const auto * y = field(message, "y");
  const auto * z = field(message, "z");
  const auto * intensity = field(message, "intensity");
  if (x == nullptr || y == nullptr || z == nullptr || message.point_step == 0U) {
    throw std::runtime_error("PointCloud2 is missing float32 x/y/z fields");
  }
  CloudFrame raw;
  raw.frame_id = message.header.frame_id;
  raw.stamp_ns = static_cast<std::uint64_t>(message.header.stamp.sec) * 1000000000ULL +
    message.header.stamp.nanosec;
  raw.has_intensity = intensity != nullptr;
  const std::size_t count = static_cast<std::size_t>(message.width) * message.height;
  raw.points.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const std::size_t offset = index * message.point_step;
    if (offset + message.point_step > message.data.size()) {
      break;
    }
    const auto * data = message.data.data() + offset;
    raw.points.push_back({readFloat(data, x), readFloat(data, y), readFloat(data, z),
      readFloat(data, intensity)});
  }
  return downsample(raw, std::max(voxel_size_m, 0.001F), maximum_range_m, maximum_points);
}

CloudFrame PointCloudCodec::loadPcd(
  const std::filesystem::path & path, const std::string & frame_id,
  const float voxel_size_m, const std::size_t maximum_points)
{
  pcl::PCLPointCloud2 raw;
  if (pcl::io::loadPCDFile(path.string(), raw) < 0) {
    throw std::runtime_error("failed to load PCD: " + path.string());
  }
  pcl::PCLPointCloud2 filtered;
  pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
  filter.setInputCloud(std::make_shared<pcl::PCLPointCloud2>(raw));
  filter.setLeafSize(voxel_size_m, voxel_size_m, voxel_size_m);
  filter.filter(filtered);

  auto locate = [&filtered](const std::string & name) -> const pcl::PCLPointField * {
      for (const auto & item : filtered.fields) {
        if (item.name == name) {return &item;}
      }
      return nullptr;
    };
  const auto * x = locate("x");
  const auto * y = locate("y");
  const auto * z = locate("z");
  const auto * intensity = locate("intensity");
  if (x == nullptr || y == nullptr || z == nullptr) {
    throw std::runtime_error("PCD is missing x/y/z fields: " + path.string());
  }
  CloudFrame output;
  output.frame_id = frame_id;
  output.has_intensity = intensity != nullptr;
  const std::size_t count = static_cast<std::size_t>(filtered.width) * filtered.height;
  const std::size_t wanted = maximum_points == 0 ? count : std::min(count, maximum_points);
  output.points.reserve(wanted);
  for (std::size_t out_index = 0; out_index < wanted; ++out_index) {
    const std::size_t index = maximum_points == 0 ? out_index : out_index * count / wanted;
    const auto * data = filtered.data.data() + index * filtered.point_step;
    auto read = [data](const pcl::PCLPointField * item) {
        float value = 0.0F;
        if (item != nullptr && item->datatype == pcl::PCLPointField::FLOAT32) {
          std::memcpy(&value, data + item->offset, sizeof(float));
        }
        return value;
      };
    const CloudPoint point{read(x), read(y), read(z), read(intensity)};
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      output.points.push_back(point);
    }
  }
  return output;
}

std::vector<std::vector<std::uint8_t>> PointCloudCodec::encode(
  const CloudFrame & cloud, const CloudStream stream, const std::uint32_t sequence,
  const std::size_t chunk_points)
{
  const std::size_t safe_chunk = std::max<std::size_t>(1U, chunk_points);
  const std::uint32_t chunk_count = static_cast<std::uint32_t>(
    std::max<std::size_t>(1U, (cloud.points.size() + safe_chunk - 1U) / safe_chunk));
  std::vector<std::vector<std::uint8_t>> packets;
  packets.reserve(chunk_count);
  for (std::uint32_t chunk = 0; chunk < chunk_count; ++chunk) {
    const std::size_t begin = static_cast<std::size_t>(chunk) * safe_chunk;
    const std::size_t end = std::min(cloud.points.size(), begin + safe_chunk);
    const std::uint32_t count = static_cast<std::uint32_t>(end - begin);
    std::array<float, 6> bounds{{0, 0, 0, 0, 0, 0}};
    if (count > 0) {
      bounds = {{cloud.points[begin].x, cloud.points[begin].y, cloud.points[begin].z,
        cloud.points[begin].x, cloud.points[begin].y, cloud.points[begin].z}};
      for (std::size_t index = begin + 1; index < end; ++index) {
        bounds[0] = std::min(bounds[0], cloud.points[index].x);
        bounds[1] = std::min(bounds[1], cloud.points[index].y);
        bounds[2] = std::min(bounds[2], cloud.points[index].z);
        bounds[3] = std::max(bounds[3], cloud.points[index].x);
        bounds[4] = std::max(bounds[4], cloud.points[index].y);
        bounds[5] = std::max(bounds[5], cloud.points[index].z);
      }
    }
    std::vector<std::uint8_t> packet;
    const std::uint16_t stride = cloud.has_intensity ? 16U : 12U;
    packet.reserve(60U + cloud.frame_id.size() + static_cast<std::size_t>(count) * stride);
    packet.insert(packet.end(), {'R', '3', 'P', 'C'});
    append<std::uint8_t>(packet, 1U);
    append<std::uint8_t>(packet, static_cast<std::uint8_t>(stream));
    append<std::uint16_t>(packet, cloud.has_intensity ? 1U : 0U);
    append<std::uint32_t>(packet, sequence);
    append<std::uint64_t>(packet, cloud.stamp_ns);
    append<std::uint32_t>(packet, count);
    append<std::uint16_t>(packet, stride);
    append<std::uint16_t>(packet, static_cast<std::uint16_t>(cloud.frame_id.size()));
    append<std::uint32_t>(packet, chunk);
    append<std::uint32_t>(packet, chunk_count);
    for (const auto value : bounds) {append<float>(packet, value);}
    packet.insert(packet.end(), cloud.frame_id.begin(), cloud.frame_id.end());
    for (std::size_t index = begin; index < end; ++index) {
      append<float>(packet, cloud.points[index].x);
      append<float>(packet, cloud.points[index].y);
      append<float>(packet, cloud.points[index].z);
      if (cloud.has_intensity) {append<float>(packet, cloud.points[index].intensity);}
    }
    packets.push_back(std::move(packet));
  }
  return packets;
}

}  // namespace route3d_web_console
