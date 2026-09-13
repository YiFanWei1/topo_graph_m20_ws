#pragma once

#include <cstdint>
#include <deque>

#include "route3d_pid_controller/m20_swept_volume_checker.hpp"

namespace route3d_pid_controller
{

class TimestampedPose3dBuffer
{
public:
  TimestampedPose3dBuffer(double history_seconds, double extrapolation_tolerance_seconds);
  void add(std::int64_t stamp_nanoseconds, const Pose3d & pose);
  bool lookup(std::int64_t stamp_nanoseconds, Pose3d & pose) const;
  void clear();

private:
  struct Sample {std::int64_t stamp_nanoseconds; Pose3d pose;};
  std::int64_t history_nanoseconds_;
  std::int64_t extrapolation_tolerance_nanoseconds_;
  std::deque<Sample> samples_;
};

}  // namespace route3d_pid_controller
