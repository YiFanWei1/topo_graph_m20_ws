#pragma once

#include <cstddef>
#include <limits>
#include <vector>

namespace route3d_pid_controller
{

struct Point3d
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pose3d
{
  Point3d position;
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct SweptVolumePose
{
  Point3d center;
  double yaw{0.0};
  double pitch{0.0};
  double distance_m{0.0};
};

struct M20SweptVolumeConfig
{
  double body_half_length_m{0.41};
  double body_half_width_m{0.215};
  double body_min_z_m{-0.285};
  double body_max_z_m{0.285};
  double detection_half_length_m{0.46};
  double detection_half_width_m{0.25};
  double detection_min_z_m{-0.285};
  double detection_max_z_m{0.285};
  double path_center_height_m{0.57};
  double surface_exclusion_height_m{0.10};
  double step_surface_max_deviation_m{0.45};
  double step_surface_support_radius_m{0.18};
  double step_surface_height_tolerance_m{0.025};
  double step_surface_min_planar_spread_m{0.02};
  std::size_t step_surface_min_support_points{3U};
};

struct M20SweptVolumeResult
{
  std::size_t points_examined{0U};
  std::size_t poses_checked{0U};
  std::size_t collision_points{0U};
  std::size_t surface_points_excluded{0U};
  double nearest_hit_distance_m{std::numeric_limits<double>::infinity()};
  std::vector<Point3d> hits;

  bool collision() const noexcept {return collision_points > 0U;}
};

class M20SweptVolumeChecker
{
public:
  explicit M20SweptVolumeChecker(M20SweptVolumeConfig config = {});

  bool isSelfPoint(const Point3d & point_in_base) const noexcept;
  M20SweptVolumeResult check(
    const std::vector<Point3d> & points_world,
    const std::vector<SweptVolumePose> & trajectory) const;
  const M20SweptVolumeConfig & config() const noexcept {return config_;}

  static Point3d transformPoint(const Pose3d & world_from_base, const Point3d & point_in_base);

private:
  M20SweptVolumeConfig config_;
};

}  // namespace route3d_pid_controller
