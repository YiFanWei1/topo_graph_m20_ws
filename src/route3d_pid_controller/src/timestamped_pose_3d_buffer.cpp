#include "route3d_pid_controller/timestamped_pose_3d_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace route3d_pid_controller
{
namespace
{

void normalize(Pose3d & pose)
{
  const double norm = std::sqrt(
    pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
  if (norm <= 1.0e-12) {
    pose.qx = pose.qy = pose.qz = 0.0;
    pose.qw = 1.0;
    return;
  }
  pose.qx /= norm;
  pose.qy /= norm;
  pose.qz /= norm;
  pose.qw /= norm;
}

Pose3d interpolate(const Pose3d & first, Pose3d second, const double ratio)
{
  Pose3d result;
  result.position.x = first.position.x + ratio * (second.position.x - first.position.x);
  result.position.y = first.position.y + ratio * (second.position.y - first.position.y);
  result.position.z = first.position.z + ratio * (second.position.z - first.position.z);
  const double dot = first.qx * second.qx + first.qy * second.qy +
    first.qz * second.qz + first.qw * second.qw;
  if (dot < 0.0) {
    second.qx = -second.qx;
    second.qy = -second.qy;
    second.qz = -second.qz;
    second.qw = -second.qw;
  }
  result.qx = first.qx + ratio * (second.qx - first.qx);
  result.qy = first.qy + ratio * (second.qy - first.qy);
  result.qz = first.qz + ratio * (second.qz - first.qz);
  result.qw = first.qw + ratio * (second.qw - first.qw);
  normalize(result);
  return result;
}

}  // namespace

TimestampedPose3dBuffer::TimestampedPose3dBuffer(
  const double history_seconds, const double tolerance_seconds)
{
  if (!std::isfinite(history_seconds) || history_seconds <= 0.0 ||
    !std::isfinite(tolerance_seconds) || tolerance_seconds < 0.0)
  {
    throw std::invalid_argument("invalid timestamped 3D pose buffer duration");
  }
  history_nanoseconds_ = static_cast<std::int64_t>(history_seconds * 1.0e9);
  extrapolation_tolerance_nanoseconds_ = static_cast<std::int64_t>(tolerance_seconds * 1.0e9);
}

void TimestampedPose3dBuffer::add(const std::int64_t stamp, const Pose3d & raw_pose)
{
  if (stamp <= 0 || !std::isfinite(raw_pose.position.x) ||
    !std::isfinite(raw_pose.position.y) || !std::isfinite(raw_pose.position.z))
  {
    return;
  }
  Pose3d pose = raw_pose;
  normalize(pose);
  const auto insertion = std::upper_bound(
    samples_.begin(), samples_.end(), stamp,
    [](const std::int64_t value, const Sample & sample) {
      return value < sample.stamp_nanoseconds;
    });
  samples_.insert(insertion, Sample{stamp, pose});
  const auto newest = samples_.back().stamp_nanoseconds;
  while (!samples_.empty() && newest - samples_.front().stamp_nanoseconds > history_nanoseconds_) {
    samples_.pop_front();
  }
}

bool TimestampedPose3dBuffer::lookup(const std::int64_t stamp, Pose3d & pose) const
{
  if (samples_.empty() || stamp <= 0) {
    return false;
  }
  if (stamp <= samples_.front().stamp_nanoseconds) {
    if (samples_.front().stamp_nanoseconds - stamp > extrapolation_tolerance_nanoseconds_) {
      return false;
    }
    pose = samples_.front().pose;
    return true;
  }
  if (stamp >= samples_.back().stamp_nanoseconds) {
    if (stamp - samples_.back().stamp_nanoseconds > extrapolation_tolerance_nanoseconds_) {
      return false;
    }
    pose = samples_.back().pose;
    return true;
  }
  const auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), stamp,
    [](const Sample & sample, const std::int64_t value) {
      return sample.stamp_nanoseconds < value;
    });
  const auto lower = std::prev(upper);
  const auto duration = upper->stamp_nanoseconds - lower->stamp_nanoseconds;
  const double ratio = static_cast<double>(stamp - lower->stamp_nanoseconds) /
    static_cast<double>(duration);
  pose = interpolate(lower->pose, upper->pose, ratio);
  return true;
}

void TimestampedPose3dBuffer::clear()
{
  samples_.clear();
}

}  // namespace route3d_pid_controller
