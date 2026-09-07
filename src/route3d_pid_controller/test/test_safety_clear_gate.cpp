#include <gtest/gtest.h>

#include <chrono>

#include "route3d_pid_controller/safety_clear_gate.hpp"

namespace
{

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

}  // namespace
