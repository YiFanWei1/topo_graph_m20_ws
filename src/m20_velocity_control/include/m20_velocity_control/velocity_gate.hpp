#ifndef M20_VELOCITY_CONTROL__VELOCITY_GATE_HPP_
#define M20_VELOCITY_CONTROL__VELOCITY_GATE_HPP_

#include <cstdint>
#include <string>

namespace m20_velocity_control
{

struct VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct VelocityGateConfig
{
  double maximum_vx_mps{0.8};
  double maximum_vy_mps{0.3};
  double maximum_wz_radps{0.5};
  double command_timeout_s{0.3};
  double state_timeout_s{1.0};
  bool allow_lateral_motion{false};
};

struct VelocityGateInput
{
  bool enable_motion{false};
  bool emergency_stop{false};
  bool ready{false};
  bool state_received{false};
  std::int32_t motion_state{-1};
  double state_age_s{-1.0};
  bool command_received{false};
  double command_age_s{-1.0};
  VelocityCommand command;
};

struct VelocityGateResult
{
  VelocityCommand command;
  bool blocked{true};
  bool clamped{false};
  std::string block_reason{"motion_disabled"};
};

class VelocityGate
{
public:
  static constexpr std::int32_t kRlMotionState = 17;

  explicit VelocityGate(VelocityGateConfig config = {});

  VelocityGateResult evaluate(const VelocityGateInput & input) const;
  const VelocityGateConfig & config() const noexcept;

private:
  VelocityGateConfig config_;
};

}  // namespace m20_velocity_control

#endif  // M20_VELOCITY_CONTROL__VELOCITY_GATE_HPP_
