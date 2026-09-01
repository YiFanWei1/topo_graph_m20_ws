#include "circle_path_benchmark/circle_path.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace benchmark = circle_path_benchmark;

TEST(CirclePath, GeneratesExactlyOneClosedCircle)
{
  benchmark::CircleSpec spec;
  spec.center_x = 1.0;
  spec.center_y = -2.0;
  spec.path_z = 0.4;
  spec.radius = 0.8;
  spec.sample_spacing = 0.05;
  const auto path = benchmark::generateCircle(spec);
  ASSERT_GT(path.size(), 100U);
  EXPECT_NEAR(path.front().x, path.back().x, 1e-9);
  EXPECT_NEAR(path.front().y, path.back().y, 1e-9);
  EXPECT_NEAR(path.front().yaw, 0.5 * benchmark::kPi, 1e-9);
  for (const auto & point : path) {
    EXPECT_NEAR(std::hypot(point.x - spec.center_x, point.y - spec.center_y), 0.8, 1e-9);
    EXPECT_DOUBLE_EQ(point.z, 0.4);
  }
}

TEST(CirclePath, DirectionAndRadialErrorAreSigned)
{
  benchmark::CircleSpec ccw;
  ccw.radius = 0.8;
  EXPECT_NEAR(benchmark::directedAngularDelta(ccw, 3.1, -3.1), 0.083185307, 1e-6);
  EXPECT_NEAR(benchmark::radialError(ccw, 0.6, 0.0), -0.2, 1e-9);
  ccw.clockwise = true;
  EXPECT_NEAR(benchmark::directedAngularDelta(ccw, 3.1, -3.1), -0.083185307, 1e-6);
}

TEST(CirclePath, InitialTangentYawDefinesDynamicCircleStart)
{
  benchmark::CircleSpec ccw;
  ccw.center_x = 3.0;
  ccw.center_y = -4.0;
  ccw.start_angle = benchmark::startAngleForTangentYaw(ccw, 0.0);
  const auto ccw_path = benchmark::generateCircle(ccw);
  EXPECT_NEAR(ccw_path.front().x, 3.0, 1e-9);
  EXPECT_NEAR(ccw_path.front().y, -4.8, 1e-9);
  EXPECT_NEAR(ccw_path.front().yaw, 0.0, 1e-9);

  benchmark::CircleSpec clockwise = ccw;
  clockwise.clockwise = true;
  clockwise.start_angle = benchmark::startAngleForTangentYaw(clockwise, 0.0);
  const auto clockwise_path = benchmark::generateCircle(clockwise);
  EXPECT_NEAR(clockwise_path.front().x, 3.0, 1e-9);
  EXPECT_NEAR(clockwise_path.front().y, -3.2, 1e-9);
  EXPECT_NEAR(clockwise_path.front().yaw, 0.0, 1e-9);
}
