#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "route3d_pid_controller/timestamped_pose_buffer.hpp"

namespace
{

using route3d_pid_controller::Pose2d;
using route3d_pid_controller::TimestampedPoseBuffer;

TEST(TimestampedPoseBuffer, InterpolatesPositionAndWrappedYawAtCloudStamp)
{
  TimestampedPoseBuffer buffer(2.0, 0.15);
  buffer.add(1000000000LL, Pose2d{0.0, 0.0, 3.10});
  buffer.add(1100000000LL, Pose2d{1.0, 2.0, -3.10});

  Pose2d result;
  ASSERT_TRUE(buffer.lookup(1050000000LL, result));
  EXPECT_NEAR(result.x, 0.5, 1.0e-9);
  EXPECT_NEAR(result.y, 1.0, 1.0e-9);
  EXPECT_NEAR(std::abs(result.yaw), 3.14159265358979323846, 0.05);
}

TEST(TimestampedPoseBuffer, RejectsCloudStampOutsideTolerance)
{
  TimestampedPoseBuffer buffer(2.0, 0.15);
  buffer.add(1000000000LL, Pose2d{1.0, 2.0, 0.3});

  Pose2d result;
  EXPECT_TRUE(buffer.lookup(1149000000LL, result));
  EXPECT_FALSE(buffer.lookup(1151000000LL, result));
}

}  // namespace
