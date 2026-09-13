#include <gtest/gtest.h>

#include <limits>

#include "m20_velocity_control/velocity_gate.hpp"

using m20_velocity_control::VelocityGate;
using m20_velocity_control::VelocityGateConfig;
using m20_velocity_control::VelocityGateInput;

namespace
{

VelocityGateInput validInput()
{
  VelocityGateInput input;
  input.enable_motion = true;
  input.ready = true;
  input.state_received = true;
  input.motion_state = VelocityGate::kRlMotionState;
  input.state_age_s = 0.1;
  input.command_received = true;
  input.command_age_s = 0.05;
  input.command = {0.2, 0.0, -0.1};
  return input;
}

}  // namespace

TEST(VelocityGate, PassesFreshReadyRlCommand)
{
  const VelocityGate gate;
  const auto result = gate.evaluate(validInput());
  EXPECT_FALSE(result.blocked);
  EXPECT_TRUE(result.block_reason.empty());
  EXPECT_DOUBLE_EQ(result.command.vx, 0.2);
  EXPECT_DOUBLE_EQ(result.command.vy, 0.0);
  EXPECT_DOUBLE_EQ(result.command.wz, -0.1);
}

TEST(VelocityGate, FailsClosedWhenDisabledOrEmergencyStopped)
{
  const VelocityGate gate;
  auto input = validInput();
  input.enable_motion = false;
  EXPECT_EQ(gate.evaluate(input).block_reason, "motion_disabled");
  input.enable_motion = true;
  input.emergency_stop = true;
  EXPECT_EQ(gate.evaluate(input).block_reason, "emergency_stop");
}

TEST(VelocityGate, RequiresReadyAndFreshRlState)
{
  const VelocityGate gate;
  auto input = validInput();
  input.ready = false;
  EXPECT_EQ(gate.evaluate(input).block_reason, "m20_not_ready");
  input.ready = true;
  input.motion_state = 1;
  EXPECT_EQ(gate.evaluate(input).block_reason, "m20_state_not_fresh_rl");
  input.motion_state = VelocityGate::kRlMotionState;
  input.state_age_s = 1.01;
  EXPECT_EQ(gate.evaluate(input).block_reason, "m20_state_not_fresh_rl");
}

TEST(VelocityGate, StopsWhenCommandTimesOut)
{
  const VelocityGate gate;
  auto input = validInput();
  input.command_age_s = 0.31;
  const auto result = gate.evaluate(input);
  EXPECT_TRUE(result.blocked);
  EXPECT_EQ(result.block_reason, "command_stale");
  EXPECT_DOUBLE_EQ(result.command.vx, 0.0);
}

TEST(VelocityGate, ClampsAndDisablesLateralMotionByDefault)
{
  const VelocityGate gate;
  auto input = validInput();
  input.command = {2.0, 0.2, -1.0};
  const auto result = gate.evaluate(input);
  EXPECT_FALSE(result.blocked);
  EXPECT_TRUE(result.clamped);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.8);
  EXPECT_DOUBLE_EQ(result.command.vy, 0.0);
  EXPECT_DOUBLE_EQ(result.command.wz, -0.5);
}

TEST(VelocityGate, AllowsBoundedLateralMotionWhenExplicitlyConfigured)
{
  VelocityGateConfig config;
  config.allow_lateral_motion = true;
  const VelocityGate gate(config);
  auto input = validInput();
  input.command.vy = -0.7;
  const auto result = gate.evaluate(input);
  EXPECT_FALSE(result.blocked);
  EXPECT_DOUBLE_EQ(result.command.vy, -0.3);
}

TEST(VelocityGate, RejectsNonFiniteCommand)
{
  const VelocityGate gate;
  auto input = validInput();
  input.command.vx = std::numeric_limits<double>::quiet_NaN();
  const auto result = gate.evaluate(input);
  EXPECT_TRUE(result.blocked);
  EXPECT_EQ(result.block_reason, "invalid_command");
}
