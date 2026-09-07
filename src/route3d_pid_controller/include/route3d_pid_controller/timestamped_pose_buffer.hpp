#pragma once

#include <cstdint>
#include <deque>

#include "route3d_pid_controller/controller_core.hpp"

namespace route3d_pid_controller
{

class TimestampedPoseBuffer
{
public:
  TimestampedPoseBuffer(double history_seconds, double extrapolation_tolerance_seconds);

  void add(std::int64_t stamp_nanoseconds, const Pose2d & pose);
  bool lookup(std::int64_t stamp_nanoseconds, Pose2d & pose) const;
  void clear();

private:
  struct Sample
  {
    std::int64_t stamp_nanoseconds;
    Pose2d pose;
  };

  std::int64_t history_nanoseconds_;
  std::int64_t extrapolation_tolerance_nanoseconds_;
  std::deque<Sample> samples_;
};

}  // namespace route3d_pid_controller
