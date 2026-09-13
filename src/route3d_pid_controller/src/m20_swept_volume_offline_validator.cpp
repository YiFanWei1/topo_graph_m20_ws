#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "route3d_pid_controller/m20_swept_volume_checker.hpp"

namespace fs = std::filesystem;
using route3d_pid_controller::M20SweptVolumeChecker;
using route3d_pid_controller::M20SweptVolumeConfig;
using route3d_pid_controller::Point3d;
using route3d_pid_controller::Pose3d;
using route3d_pid_controller::SweptVolumePose;

namespace
{

struct TimedPose
{
  double stamp{0.0};
  Pose3d pose;
};

std::string argument(int argc, char ** argv, const std::string & name)
{
  for (int index = 1; index + 1 < argc; ++index) {
    if (argv[index] == name) {
      return argv[index + 1];
    }
  }
  throw std::invalid_argument("missing required argument " + name);
}

double optionalDoubleArgument(
  int argc, char ** argv, const std::string & name, const double fallback)
{
  for (int index = 1; index + 1 < argc; ++index) {
    if (argv[index] == name) {
      const double value = std::stod(argv[index + 1]);
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(name + " must be finite and non-negative");
      }
      return value;
    }
  }
  return fallback;
}

std::vector<TimedPose> loadPoses(const fs::path & path)
{
  std::ifstream input(path);
  nlohmann::json document;
  input >> document;
  if (!document.is_array()) {
    throw std::runtime_error("pose.json must be an array");
  }
  std::vector<TimedPose> result;
  result.reserve(document.size());
  for (const auto & row : document) {
    if (!row.is_array() || row.size() != 8U) {
      throw std::runtime_error("pose.json row must contain 8 numbers");
    }
    result.push_back({row[0].get<double>(), {
      {row[1].get<double>(), row[2].get<double>(), row[3].get<double>()},
      row[4].get<double>(), row[5].get<double>(), row[6].get<double>(), row[7].get<double>()}});
  }
  return result;
}

std::vector<Point3d> loadPcd(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open " + path.string());
  }
  std::size_t point_count = 0U;
  std::string line;
  bool binary = false;
  while (std::getline(input, line)) {
    if (line.rfind("POINTS ", 0U) == 0U) {
      point_count = static_cast<std::size_t>(std::stoull(line.substr(7U)));
    }
    if (line == "DATA binary") {
      binary = true;
      break;
    }
  }
  if (!binary) {
    throw std::runtime_error("validator requires binary XYZ PCD: " + path.string());
  }
  std::vector<std::array<float, 3>> raw(point_count);
  input.read(reinterpret_cast<char *>(raw.data()),
    static_cast<std::streamsize>(raw.size() * sizeof(raw.front())));
  if (!input && !raw.empty()) {
    throw std::runtime_error("truncated PCD payload: " + path.string());
  }
  std::vector<Point3d> result;
  result.reserve(raw.size());
  for (const auto & point : raw) {
    result.push_back({point[0], point[1], point[2]});
  }
  return result;
}

double distance(const Point3d & first, const Point3d & second)
{
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  const double dz = second.z - first.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

SweptVolumePose pathPose(
  const Point3d & center, const Point3d & direction_to, const double distance_m)
{
  const double dx = direction_to.x - center.x;
  const double dy = direction_to.y - center.y;
  const double dz = direction_to.z - center.z;
  return {center, std::atan2(dy, dx), std::atan2(dz, std::hypot(dx, dy)), distance_m};
}

std::vector<SweptVolumePose> samplePath(
  const std::vector<TimedPose> & poses, const std::size_t start,
  const double replay_center_z_offset_m)
{
  constexpr double spacing = 0.40;
  constexpr std::size_t maximum_samples = 15U;
  std::vector<std::pair<Point3d, double>> samples;
  Point3d first_center = poses[start].pose.position;
  first_center.z += replay_center_z_offset_m;
  samples.push_back({first_center, 0.0});
  double accumulated = 0.0;
  double emitted_distance = 0.0;
  for (std::size_t index = start + 1U;
    index < poses.size() && samples.size() < maximum_samples; ++index)
  {
    accumulated += distance(poses[index - 1U].pose.position, poses[index].pose.position);
    if (accumulated - emitted_distance < spacing && index + 1U < poses.size()) {
      continue;
    }
    Point3d center = poses[index].pose.position;
    center.z += replay_center_z_offset_m;
    samples.push_back({center, accumulated});
    emitted_distance = accumulated;
  }
  std::vector<SweptVolumePose> result;
  result.reserve(samples.size());
  for (std::size_t index = 0U; index + 1U < samples.size(); ++index) {
    result.push_back(pathPose(
      samples[index].first, samples[index + 1U].first, samples[index].second));
  }
  if (result.empty()) {
    result.push_back({samples.front().first, 0.0, 0.0, 0.0});
  } else {
    result.push_back({
      samples.back().first, result.back().yaw, result.back().pitch, samples.back().second});
  }
  return result;
}

double minimumHeightAboveExpectedSurface(
  const Point3d & point, const std::vector<SweptVolumePose> & path,
  const M20SweptVolumeConfig & config)
{
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto & pose : path) {
    const double cosine_yaw = std::cos(pose.yaw);
    const double sine_yaw = std::sin(pose.yaw);
    const double cosine_pitch = std::cos(pose.pitch);
    const double sine_pitch = std::sin(pose.pitch);
    const Point3d forward{
      cosine_pitch * cosine_yaw, cosine_pitch * sine_yaw, sine_pitch};
    const Point3d left{-sine_yaw, cosine_yaw, 0.0};
    const Point3d up{
      -sine_pitch * cosine_yaw, -sine_pitch * sine_yaw, cosine_pitch};
    const Point3d delta{
      point.x - pose.center.x, point.y - pose.center.y, point.z - pose.center.z};
    const auto projection = [&delta](const Point3d & axis) {
        return delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
      };
    const double local_x = projection(forward);
    const double local_y = projection(left);
    const double local_z = projection(up);
    if (std::abs(local_x) <= config.detection_half_length_m &&
      std::abs(local_y) <= config.detection_half_width_m &&
      local_z >= config.detection_min_z_m && local_z <= config.detection_max_z_m)
    {
      minimum = std::min(minimum, local_z + config.path_center_height_m);
    }
  }
  return minimum;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const fs::path input_root = argument(argc, argv, "--input");
    const fs::path jsonl_path = argument(argc, argv, "--jsonl");
    const fs::path summary_path = argument(argc, argv, "--summary");
    const double recording_body_height_m = optionalDoubleArgument(
      argc, argv, "--recording-body-height", 0.40);
    const double m20_body_height_m = optionalDoubleArgument(
      argc, argv, "--m20-body-height", 0.57);
    const double replay_center_z_offset_m =
      m20_body_height_m - recording_body_height_m;
    const auto poses = loadPoses(input_root / "pose.json");
    M20SweptVolumeChecker checker;
    fs::create_directories(jsonl_path.parent_path());
    fs::create_directories(summary_path.parent_path());
    std::ofstream jsonl(jsonl_path);
    std::size_t collision_frames = 0U;
    std::size_t descending_frames = 0U;
    std::size_t descending_collision_frames = 0U;
    std::size_t total_input_points = 0U;
    std::size_t total_self_points = 0U;
    std::size_t total_surface_points = 0U;
    std::size_t terrain_band_collision_points = 0U;
    double nearest_hit = std::numeric_limits<double>::infinity();

    for (std::size_t index = 0U; index < poses.size(); ++index) {
      const auto input_points = loadPcd(input_root / "key_frames" /
        (std::to_string(index) + ".pcd"));
      total_input_points += input_points.size();
      std::vector<Point3d> world_points;
      world_points.reserve(input_points.size());
      for (const auto & point : input_points) {
        if (checker.isSelfPoint(point)) {
          ++total_self_points;
          continue;
        }
        world_points.push_back(M20SweptVolumeChecker::transformPoint(poses[index].pose, point));
      }
      const auto path = samplePath(poses, index, replay_center_z_offset_m);
      const auto check = checker.check(world_points, path);
      const bool descending = !path.empty() && path.front().pitch < -0.05235987755982988;
      collision_frames += check.collision() ? 1U : 0U;
      descending_frames += descending ? 1U : 0U;
      descending_collision_frames += descending && check.collision() ? 1U : 0U;
      nearest_hit = std::min(nearest_hit, check.nearest_hit_distance_m);
      total_surface_points += check.surface_points_excluded;
      std::size_t frame_terrain_band_collisions = 0U;
      for (const auto & hit : check.hits) {
        const double clearance = minimumHeightAboveExpectedSurface(
          hit, path, checker.config());
        if (clearance <= checker.config().surface_exclusion_height_m + 1.0e-9) {
          ++frame_terrain_band_collisions;
        }
      }
      terrain_band_collision_points += frame_terrain_band_collisions;
      jsonl << nlohmann::json{
        {"frame", index}, {"stamp", poses[index].stamp},
        {"input_points", input_points.size()}, {"used_points", world_points.size()},
        {"trajectory_poses", path.size()},
        {"path_pitch_rad", path.empty() ? 0.0 : path.front().pitch},
        {"descending", descending}, {"collision", check.collision()},
        {"collision_points", check.collision_points},
        {"surface_points_excluded", check.surface_points_excluded},
        {"terrain_band_collision_points", frame_terrain_band_collisions},
        {"nearest_hit_distance_m", std::isfinite(check.nearest_hit_distance_m) ?
          nlohmann::json(check.nearest_hit_distance_m) : nlohmann::json(nullptr)}}.dump() << '\n';
    }

    const auto synthetic_path = samplePath(poses, 0U, replay_center_z_offset_m);
    const auto synthetic = checker.check({synthetic_path.front().center}, synthetic_path);
    nlohmann::json summary = {
      {"input", input_root.string()}, {"frames", poses.size()},
      {"input_points", total_input_points}, {"self_points_filtered", total_self_points},
      {"surface_points_excluded", total_surface_points},
      {"terrain_band_collision_points", terrain_band_collision_points},
      {"collision_frames", collision_frames}, {"descending_frames", descending_frames},
      {"descending_collision_frames", descending_collision_frames},
      {"nearest_hit_distance_m", std::isfinite(nearest_hit) ?
        nlohmann::json(nearest_hit) : nlohmann::json(nullptr)},
      {"synthetic_obstacle_detected", synthetic.collision()},
      {"body_dimensions_m", {0.82, 0.43, 0.57}},
      {"detection_dimensions_m", {0.92, 0.50, 0.57}},
      {"surface_exclusion_height_m", 0.10}};
    summary["recording_body_height_m"] = recording_body_height_m;
    summary["m20_body_height_m"] = m20_body_height_m;
    summary["replay_center_z_offset_m"] = replay_center_z_offset_m;
    std::ofstream(summary_path) << std::setw(2) << summary << '\n';
    std::cout << summary.dump() << std::endl;
    return synthetic.collision() && terrain_band_collision_points == 0U ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "m20_swept_volume_offline_validator: " << error.what() << std::endl;
    return 1;
  }
}
