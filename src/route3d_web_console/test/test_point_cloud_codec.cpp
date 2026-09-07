#include <cstring>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "route3d_web_console/point_cloud_codec.hpp"

using route3d_web_console::CloudStream;
using route3d_web_console::PointCloudCodec;

TEST(PointCloudCodec, DownsamplesAndEncodesProtocolHeader)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "camera_init";
  cloud.height = 1;
  cloud.width = 3;
  cloud.point_step = 12;
  cloud.row_step = 36;
  cloud.fields.resize(3);
  const char * names[] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    cloud.fields[i].name = names[i];
    cloud.fields[i].offset = i * 4;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[i].count = 1;
  }
  cloud.data.resize(36);
  const float values[] = {0, 0, 0, 0.01F, 0.01F, 0, 1, 0, 0};
  std::memcpy(cloud.data.data(), values, sizeof(values));
  const auto converted = PointCloudCodec::fromRos(cloud, 0.1F, 20.0F, 100);
  EXPECT_EQ(converted.points.size(), 2U);
  const auto packets = PointCloudCodec::encode(converted, CloudStream::kLiveRegistered, 7, 1);
  ASSERT_EQ(packets.size(), 2U);
  EXPECT_EQ(std::string(packets[0].begin(), packets[0].begin() + 4), "R3PC");
  EXPECT_EQ(packets[0][4], 1U);
  EXPECT_EQ(packets[0][5], 2U);
}
