#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "route3d_pid_controller/safety_clear_gate.hpp"

namespace
{

using route3d_pid_controller::ConsecutiveFrameGate;
using route3d_pid_controller::SafetyClearGate;

TEST(SafetyClearGate, RequiresContinuouslyClearHoldTime)
{
  SafetyClearGate gate(3.0);
  const auto start = SafetyClearGate::Clock::time_point{};

  EXPECT_FALSE(gate.ready(start));
  EXPECT_FALSE(gate.ready(start + std::chrono::milliseconds(2999)));
  EXPECT_TRUE(gate.ready(start + std::chrono::seconds(3)));
}

TEST(SafetyClearGate, UnsafeObservationRestartsHoldTime)
{
  SafetyClearGate gate(3.0);
  const auto start = SafetyClearGate::Clock::time_point{};

  EXPECT_FALSE(gate.ready(start));
  gate.reset();
  EXPECT_FALSE(gate.ready(start + std::chrono::seconds(2)));
  EXPECT_FALSE(gate.ready(start + std::chrono::seconds(4)));
  EXPECT_TRUE(gate.ready(start + std::chrono::seconds(5)));
}

TEST(SafetyClearGate, ZeroHoldTimePreservesImmediateRecovery)
{
  SafetyClearGate gate(0.0);
  EXPECT_TRUE(gate.ready(SafetyClearGate::Clock::time_point{}));
}

TEST(ConsecutiveFrameGate, RequiresIndependentEnterAndExitFrameCounts)
{
  ConsecutiveFrameGate gate(2U, 3U);

  EXPECT_FALSE(gate.update(true));
  EXPECT_EQ(gate.obstacleFrames(), 1U);
  EXPECT_FALSE(gate.update(false));
  EXPECT_EQ(gate.obstacleFrames(), 0U);

  EXPECT_FALSE(gate.update(true));
  EXPECT_TRUE(gate.update(true));
  EXPECT_TRUE(gate.blocked());

  EXPECT_TRUE(gate.update(false));
  EXPECT_EQ(gate.clearFrames(), 1U);
  EXPECT_TRUE(gate.update(true));
  EXPECT_EQ(gate.clearFrames(), 0U);

  EXPECT_TRUE(gate.update(false));
  EXPECT_TRUE(gate.update(false));
  EXPECT_FALSE(gate.update(false));
  EXPECT_FALSE(gate.blocked());
  EXPECT_EQ(gate.clearFrames(), 3U);
}

TEST(ConsecutiveFrameGate, ResetClearsAllHysteresisState)
{
  ConsecutiveFrameGate gate(1U, 1U);
  EXPECT_TRUE(gate.update(true));
  gate.reset();
  EXPECT_FALSE(gate.blocked());
  EXPECT_EQ(gate.obstacleFrames(), 0U);
  EXPECT_EQ(gate.clearFrames(), 0U);
}

TEST(ConsecutiveFrameGate, RejectsZeroThresholds)
{
  EXPECT_THROW(ConsecutiveFrameGate(0U, 1U), std::invalid_argument);
  EXPECT_THROW(ConsecutiveFrameGate(1U, 0U), std::invalid_argument);
}

}  // namespace
