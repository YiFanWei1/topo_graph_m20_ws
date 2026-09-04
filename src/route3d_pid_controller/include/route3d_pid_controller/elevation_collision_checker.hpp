#ifndef ROUTE3D_PID_CONTROLLER__ELEVATION_COLLISION_CHECKER_HPP_
#define ROUTE3D_PID_CONTROLLER__ELEVATION_COLLISION_CHECKER_HPP_

#include <cstddef>
#include <vector>

#include "route3d_pid_controller/controller_core.hpp"

namespace route3d_pid_controller
{

struct ElevationPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ElevationCell
{
  double x{0.0};
  double y{0.0};
  double height{0.0};
  bool observed{false};
  bool rough{false};
};

struct FootprintSample
{
  double x{0.0};
  double y{0.0};
  bool collision{false};
};

struct FootprintGrid
{
  double min_x{0.05};
  double max_x{0.60};
  double min_y{-0.25};
  double max_y{0.24};
  std::size_t samples_x{5U};
  std::size_t samples_y{5U};
  double lateral_inset_m{0.13};
};

struct ElevationCollisionConfig
{
  double resolution_m{0.10};
  double width_m{4.0};
  double default_ground_height_m{-0.40};
  double minimum_obstacle_height_m{-0.20};
  double maximum_obstacle_height_m{0.40};
  double roughness_threshold_m{0.125};
  double body_min_x{-0.50};
  double body_max_x{0.30};
  double body_min_y{-0.25};
  double body_max_y{0.25};
  FootprintGrid warning_footprint{};
};

struct ElevationCollisionResult
{
  std::size_t trajectory_poses{0U};
  std::size_t footprint_samples{0U};
  std::size_t collision_samples{0U};

  bool collision() const noexcept {return collision_samples > 0U;}
};

// Reproduces the reference controller's local elevation-map collision test:
// a point cloud is rasterized into an elevation grid, a 3x3 height standard
// deviation produces the binary "standard" layer, and a sampled robot
// footprint is swept over the remaining route.
class ElevationCollisionChecker
{
public:
  explicit ElevationCollisionChecker(ElevationCollisionConfig config = {});

  void update(const std::vector<ElevationPoint> & points);
  ElevationCollisionResult checkTrajectory(
    const std::vector<Pose2d> & trajectory,
    const FootprintGrid * footprint_override = nullptr) const;
  void clear();

  bool ready() const noexcept {return ready_;}
  std::size_t occupiedCells() const noexcept {return occupied_cells_;}
  std::size_t roughCells() const noexcept {return rough_cells_;}
  const ElevationCollisionConfig & config() const noexcept {return config_;}
  std::vector<ElevationCell> cellData() const;
  std::vector<FootprintSample> sampleTrajectory(
    const std::vector<Pose2d> & trajectory,
    const FootprintGrid * footprint_override = nullptr) const;

private:
  bool inside(double x, double y) const noexcept;
  std::size_t index(int x, int y) const noexcept;
  bool roughAt(double x, double y) const noexcept;
  double heightAt(double x, double y) const noexcept;

  ElevationCollisionConfig config_;
  std::size_t cells_per_side_{0U};
  double half_width_m_{0.0};
  std::vector<double> elevation_;
  std::vector<bool> observed_;
  std::vector<bool> rough_;
  bool ready_{false};
  std::size_t occupied_cells_{0U};
  std::size_t rough_cells_{0U};
};

}  // namespace route3d_pid_controller

#endif  // ROUTE3D_PID_CONTROLLER__ELEVATION_COLLISION_CHECKER_HPP_
