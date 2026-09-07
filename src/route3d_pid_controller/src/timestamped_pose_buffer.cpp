#include "route3d_pid_controller/timestamped_pose_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace route3d_pid_controller
{

TimestampedPoseBuffer::TimestampedPoseBuffer(
  const double history_seconds, const double extrapolation_tolerance_seconds)
{
  if (!std::isfinite(history_seconds) || history_seconds <= 0.0 ||
    !std::isfinite(extrapolation_tolerance_seconds) || extrapolation_tolerance_seconds < 0.0)
  {
    throw std::invalid_argument("invalid timestamped pose buffer duration");
  }
  history_nanoseconds_ = static_cast<std::int64_t>(history_seconds * 1.0e9);
  extrapolation_tolerance_nanoseconds_ = static_cast<std::int64_t>(
    extrapolation_tolerance_seconds * 1.0e9);
}

void TimestampedPoseBuffer::add(const std::int64_t stamp_nanoseconds, const Pose2d & pose)
{
  if (stamp_nanoseconds <= 0 || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
    !std::isfinite(pose.yaw))
  {
    return;
  }
  const auto insertion = std::upper_bound(
    samples_.begin(), samples_.end(), stamp_nanoseconds,
    [](const std::int64_t stamp, const Sample & sample) {
      return stamp < sample.stamp_nanoseconds;
    });
  samples_.insert(insertion, Sample{stamp_nanoseconds, pose});
  const std::int64_t newest_stamp = samples_.back().stamp_nanoseconds;
  while (!samples_.empty() &&
    newest_stamp - samples_.front().stamp_nanoseconds > history_nanoseconds_)
  {
    samples_.pop_front();
  }
}

bool TimestampedPoseBuffer::lookup(
  const std::int64_t stamp_nanoseconds, Pose2d & pose) const
{
  if (samples_.empty() || stamp_nanoseconds <= 0) {
    return false;
  }
  if (stamp_nanoseconds <= samples_.front().stamp_nanoseconds) {
    if (samples_.front().stamp_nanoseconds - stamp_nanoseconds >
      extrapolation_tolerance_nanoseconds_)
    {
      return false;
    }
    pose = samples_.front().pose;
    return true;
  }
  if (stamp_nanoseconds >= samples_.back().stamp_nanoseconds) {
    if (stamp_nanoseconds - samples_.back().stamp_nanoseconds >
      extrapolation_tolerance_nanoseconds_)
    {
      return false;
    }
    pose = samples_.back().pose;
    return true;
  }

  const auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), stamp_nanoseconds,
    [](const Sample & sample, const std::int64_t stamp) {
      return sample.stamp_nanoseconds < stamp;
    });
  const auto lower = std::prev(upper);
  const auto duration = upper->stamp_nanoseconds - lower->stamp_nanoseconds;
  if (duration <= 0) {
    pose = upper->pose;
    return true;
  }
  const double ratio = static_cast<double>(stamp_nanoseconds - lower->stamp_nanoseconds) /
    static_cast<double>(duration);
  pose.x = lower->pose.x + ratio * (upper->pose.x - lower->pose.x);
  pose.y = lower->pose.y + ratio * (upper->pose.y - lower->pose.y);
  pose.yaw = normalizeAngle(
    lower->pose.yaw + ratio * normalizeAngle(upper->pose.yaw - lower->pose.yaw));
  return true;
}

void TimestampedPoseBuffer::clear()
{
  samples_.clear();
}

}  // namespace route3d_pid_controller
