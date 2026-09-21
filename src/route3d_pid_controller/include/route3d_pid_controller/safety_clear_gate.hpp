#pragma once

#include <chrono>
#include <cstddef>
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

// 对相互独立的新传感器帧做进入/退出迟滞，不能按控制循环次数调用 update()。
class ConsecutiveFrameGate
{
public:
  ConsecutiveFrameGate(std::size_t enter_frames, std::size_t exit_frames);

  bool update(bool obstacle_observed);
  void reset();
  bool blocked() const noexcept;
  std::size_t obstacleFrames() const noexcept;
  std::size_t clearFrames() const noexcept;
  std::size_t enterFrames() const noexcept;
  std::size_t exitFrames() const noexcept;

private:
  std::size_t enter_frames_;
  std::size_t exit_frames_;
  std::size_t obstacle_frames_{0U};
  std::size_t clear_frames_{0U};
  bool blocked_{false};
};

}  // namespace route3d_pid_controller
