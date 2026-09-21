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

ConsecutiveFrameGate::ConsecutiveFrameGate(
  const std::size_t enter_frames, const std::size_t exit_frames)
: enter_frames_(enter_frames), exit_frames_(exit_frames)
{
  if (enter_frames_ == 0U || exit_frames_ == 0U) {
    throw std::invalid_argument("consecutive frame thresholds must be positive");
  }
}

bool ConsecutiveFrameGate::update(const bool obstacle_observed)
{
  if (obstacle_observed) {
    clear_frames_ = 0U;
    if (obstacle_frames_ < enter_frames_) {
      ++obstacle_frames_;
    }
    if (!blocked_ && obstacle_frames_ >= enter_frames_) {
      blocked_ = true;
    }
    return blocked_;
  }

  obstacle_frames_ = 0U;
  if (!blocked_) {
    clear_frames_ = 0U;
    return false;
  }
  if (clear_frames_ < exit_frames_) {
    ++clear_frames_;
  }
  if (clear_frames_ >= exit_frames_) {
    blocked_ = false;
  }
  return blocked_;
}

void ConsecutiveFrameGate::reset()
{
  obstacle_frames_ = 0U;
  clear_frames_ = 0U;
  blocked_ = false;
}

bool ConsecutiveFrameGate::blocked() const noexcept
{
  return blocked_;
}

std::size_t ConsecutiveFrameGate::obstacleFrames() const noexcept
{
  return obstacle_frames_;
}

std::size_t ConsecutiveFrameGate::clearFrames() const noexcept
{
  return clear_frames_;
}

std::size_t ConsecutiveFrameGate::enterFrames() const noexcept
{
  return enter_frames_;
}

std::size_t ConsecutiveFrameGate::exitFrames() const noexcept
{
  return exit_frames_;
}

}  // namespace route3d_pid_controller
