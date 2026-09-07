#include "route3d_pid_controller/safety_clear_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace route3d_pid_controller
{

SafetyClearGate::SafetyClearGate(const double hold_seconds)
: hold_seconds_(hold_seconds)
{
  if (!std::isfinite(hold_seconds_) || hold_seconds_ < 0.0) {
    throw std::invalid_argument("safety clear hold time must be finite and non-negative");
  }
}

void SafetyClearGate::reset()
{
  clear_since_.reset();
}

bool SafetyClearGate::ready(const Clock::time_point now)
{
  if (!clear_since_) {
    clear_since_ = now;
  }
  return elapsedSeconds(now) >= hold_seconds_;
}

double SafetyClearGate::elapsedSeconds(const Clock::time_point now) const
{
  if (!clear_since_) {
    return 0.0;
  }
  return std::max(0.0, std::chrono::duration<double>(now - *clear_since_).count());
}

double SafetyClearGate::holdSeconds() const noexcept
{
  return hold_seconds_;
}

}  // namespace route3d_pid_controller
