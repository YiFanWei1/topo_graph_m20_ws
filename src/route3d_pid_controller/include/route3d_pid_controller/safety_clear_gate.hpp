#pragma once

#include <chrono>
#include <optional>

namespace route3d_pid_controller
{

class SafetyClearGate
{
public:
  using Clock = std::chrono::steady_clock;

  explicit SafetyClearGate(double hold_seconds);

  void reset();
  bool ready(Clock::time_point now);
  double elapsedSeconds(Clock::time_point now) const;
  double holdSeconds() const noexcept;

private:
  double hold_seconds_;
  std::optional<Clock::time_point> clear_since_;
};

}  // namespace route3d_pid_controller
