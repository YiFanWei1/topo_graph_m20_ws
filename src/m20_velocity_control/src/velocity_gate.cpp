#include "m20_velocity_control/velocity_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace m20_velocity_control
{

VelocityGate::VelocityGate(VelocityGateConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.maximum_vx_mps) || config_.maximum_vx_mps <= 0.0 ||
    !std::isfinite(config_.maximum_vy_mps) || config_.maximum_vy_mps < 0.0 ||
    !std::isfinite(config_.maximum_wz_radps) || config_.maximum_wz_radps <= 0.0 ||
    !std::isfinite(config_.command_timeout_s) || config_.command_timeout_s <= 0.0 ||
    !std::isfinite(config_.state_timeout_s) || config_.state_timeout_s <= 0.0)
  {
    throw std::invalid_argument("invalid M20 manual velocity gate configuration");
  }
}

VelocityGateResult VelocityGate::evaluate(const VelocityGateInput & input) const
{
  VelocityGateResult result;
  if (!input.enable_motion) {
    result.block_reason = "motion_disabled";
    return result;
  }
  if (input.emergency_stop) {
    result.block_reason = "emergency_stop";
    return result;
  }
  if (!input.ready) {
    result.block_reason = "m20_not_ready";
    return result;
  }
  if (!input.state_received || input.motion_state != kRlMotionState ||
    !std::isfinite(input.state_age_s) || input.state_age_s < 0.0 ||
    input.state_age_s > config_.state_timeout_s)
  {
    result.block_reason = "m20_state_not_fresh_rl";
    return result;
  }
  if (!input.command_received || !std::isfinite(input.command_age_s) ||
    input.command_age_s < 0.0 || input.command_age_s > config_.command_timeout_s)
  {
    result.block_reason = "command_stale";
    return result;
  }
  if (!std::isfinite(input.command.vx) || !std::isfinite(input.command.vy) ||
    !std::isfinite(input.command.wz))
  {
    result.block_reason = "invalid_command";
    return result;
  }

  result.command.vx = std::clamp(
    input.command.vx, -config_.maximum_vx_mps, config_.maximum_vx_mps);
  result.command.vy = config_.allow_lateral_motion ? std::clamp(
    input.command.vy, -config_.maximum_vy_mps, config_.maximum_vy_mps) : 0.0;
  result.command.wz = std::clamp(
    input.command.wz, -config_.maximum_wz_radps, config_.maximum_wz_radps);
  result.clamped = result.command.vx != input.command.vx ||
    result.command.vy != input.command.vy || result.command.wz != input.command.wz;
  result.blocked = false;
  result.block_reason.clear();
  return result;
}

const VelocityGateConfig & VelocityGate::config() const noexcept
{
  return config_;
}

}  // namespace m20_velocity_control
