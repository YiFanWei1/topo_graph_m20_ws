#include <pcl/common/point_tests.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct Config {
  float crop_radius = 6.0F;
  float voxel = 0.10F;
  int normal_k = 24;
  float max_slope_deg = 30.0F;
  float max_curvature = 0.16F;
  float seed_radius = 0.45F;
  float connection_radius = 0.24F;
  float max_neighbor_dz = 0.22F;
  float clearance_radius = 0.32F;
  float clearance_min_z = 0.14F;
  float clearance_height = 0.85F;
  float traversed_override_radius = 0.45F;
  float corridor_radius = 0.65F;
  float route_sample_spacing = 0.04F;
  float route_half_width = 0.25F;
  float route_lateral_spacing = 0.05F;
};

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

static void saveCloud(const fs::path & path, const Cloud & cloud) {
  if (pcl::io::savePCDFileBinaryCompressed(path.string(), cloud) < 0) {
    throw std::runtime_error("failed to save " + path.string());
  }
}

static std::vector<char> growRegion(
  const Cloud::Ptr & candidate,
  const pcl::KdTreeFLANN<Point>::Ptr & tree,
  const Cloud::Ptr & seeds,
  const std::vector<char> * allowed,
  const Config & cfg,
  std::size_t & matched_seeds)
{
  std::vector<char> visited(candidate->size(), 0);
  std::queue<int> frontier;
  matched_seeds = 0;

  for (const auto & seed : *seeds) {
    std::vector<int> index(1);
    std::vector<float> squared_distance(1);
    if (tree->nearestKSearch(seed, 1, index, squared_distance) == 0) {
      continue;
    }
    if (squared_distance[0] > cfg.seed_radius * cfg.seed_radius) {
      continue;
    }
    const int id = index[0];
    if (allowed != nullptr && !(*allowed)[id]) {
      continue;
    }
    ++matched_seeds;
    if (!visited[id]) {
      visited[id] = 1;
      frontier.push(id);
    }
  }

  std::vector<int> neighbors;
  std::vector<float> squared_distance;
  while (!frontier.empty()) {
    const int current = frontier.front();
    frontier.pop();
    neighbors.clear();
    squared_distance.clear();
    tree->radiusSearch(candidate->points[current], cfg.connection_radius, neighbors, squared_distance);
    for (const int next : neighbors) {
      if (visited[next] || (allowed != nullptr && !(*allowed)[next])) {
        continue;
      }
      const auto & a = candidate->points[current];
      const auto & b = candidate->points[next];
      const float dx = b.x - a.x;
      const float dy = b.y - a.y;
      const float dz = std::abs(b.z - a.z);
      const float dxy = std::hypot(dx, dy);
      if (dxy < cfg.voxel * 0.35F || dz > cfg.max_neighbor_dz) {
        continue;
      }
      visited[next] = 1;
      frontier.push(next);
    }
  }
  return visited;
}

template<typename Predicate>
static Cloud::Ptr selectCloud(const Cloud::Ptr & source, Predicate predicate) {
  Cloud::Ptr selected(new Cloud);
  selected->reserve(source->size());
  for (std::size_t i = 0; i < source->size(); ++i) {
    if (predicate(i)) {
      selected->push_back(source->points[i]);
    }
  }
  selected->width = selected->size();
  selected->height = 1;
  selected->is_dense = true;
  return selected;
}

static void appendPreview(
  std::ofstream & stream, const Cloud & cloud, const std::string & category,
  std::size_t limit)
{
  if (cloud.empty()) {
    return;
  }
  const std::size_t stride = std::max<std::size_t>(1, cloud.size() / limit);
  std::size_t written = 0;
  for (std::size_t i = 0; i < cloud.size() && written < limit; i += stride, ++written) {
    const auto & p = cloud.points[i];
    stream << p.x << ',' << p.y << ',' << p.z << ',' << p.intensity << ',' << category << '\n';
  }
}

static Cloud::Ptr buildRouteSupportedRibbon(const Cloud::Ptr & trajectory, const Config & cfg) {
  Cloud::Ptr raw(new Cloud);
  if (trajectory->size() < 2) {
    return raw;
  }
  float last_lateral_x = 0.0F;
  float last_lateral_y = 1.0F;
  const float pi = 3.14159265358979323846F;
  const int lateral_steps = static_cast<int>(
    std::ceil(cfg.route_half_width / cfg.route_lateral_spacing));

  for (std::size_t i = 0; i + 1 < trajectory->size(); ++i) {
    const auto & a = trajectory->points[i];
    const auto & b = trajectory->points[i + 1];
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float dz = b.z - a.z;
    const float dxy = std::hypot(dx, dy);
    const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < 1.0e-4F) {
      continue;
    }
    if (dxy > 1.0e-3F) {
      last_lateral_x = -dy / dxy;
      last_lateral_y = dx / dxy;
    }
    const float slope_deg = std::atan2(std::abs(dz), std::max(dxy, 1.0e-4F)) * 180.0F / pi;
    const int along_steps = std::max(1, static_cast<int>(std::ceil(distance / cfg.route_sample_spacing)));
    for (int along = 0; along <= along_steps; ++along) {
      const float t = static_cast<float>(along) / static_cast<float>(along_steps);
      const float center_x = a.x + t * dx;
      const float center_y = a.y + t * dy;
      const float center_z = a.z + t * dz;
      for (int lateral = -lateral_steps; lateral <= lateral_steps; ++lateral) {
        const float offset = std::clamp(
          lateral * cfg.route_lateral_spacing, -cfg.route_half_width, cfg.route_half_width);
        Point point;
        point.x = center_x + last_lateral_x * offset;
        point.y = center_y + last_lateral_y * offset;
        point.z = center_z;
        point.intensity = slope_deg;
        raw->push_back(point);
      }
    }
  }

  Cloud::Ptr ribbon(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(raw);
  voxel.setLeafSize(
    cfg.route_sample_spacing, cfg.route_sample_spacing, cfg.route_sample_spacing);
  voxel.filter(*ribbon);
  return ribbon;
}

int main(int argc, char ** argv) {
  if (argc != 4) {
    std::cerr << "usage: " << argv[0] << " MAP.pcd TRAJECTORY_GROUND.pcd OUTPUT_DIR\n";
    return 2;
  }
  const fs::path map_path(argv[1]);
  const fs::path trajectory_path(argv[2]);
  const fs::path output_dir(argv[3]);
  fs::create_directories(output_dir);
  const Config cfg;

  Cloud::Ptr full(new Cloud);
  Cloud::Ptr trajectory(new Cloud);
  if (pcl::io::loadPCDFile<Point>(map_path.string(), *full) < 0 ||
      pcl::io::loadPCDFile<Point>(trajectory_path.string(), *trajectory) < 0) {
    return 3;
  }

  Cloud::Ptr finite(new Cloud);
  finite->reserve(full->size());
  for (const auto & p : *full) {
    if (pcl::isFinite(p)) {
      finite->push_back(p);
    }
  }

  pcl::KdTreeFLANN<Point> trajectory_tree;
  trajectory_tree.setInputCloud(trajectory);
  Cloud::Ptr cropped(new Cloud);
  std::vector<int> one_index(1);
  std::vector<float> one_distance(1);
  if (cfg.crop_radius <= 0.0F || trajectory->empty()) {
    *cropped = *finite;
  } else {
    cropped->reserve(finite->size() / 4);
    for (const auto & p : *finite) {
      if (trajectory_tree.nearestKSearch(p, 1, one_index, one_distance) > 0 &&
          one_distance[0] <= cfg.crop_radius * cfg.crop_radius) {
        cropped->push_back(p);
      }
    }
  }

  Cloud::Ptr downsampled(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(cropped);
  voxel.setLeafSize(cfg.voxel, cfg.voxel, cfg.voxel);
  voxel.filter(*downsampled);
  saveCloud(output_dir / "map_cropped_downsampled.pcd", *downsampled);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<Point, pcl::Normal> normal_estimator;
  normal_estimator.setNumberOfThreads(0);
  normal_estimator.setInputCloud(downsampled);
  pcl::search::KdTree<Point>::Ptr normal_tree(new pcl::search::KdTree<Point>);
  normal_estimator.setSearchMethod(normal_tree);
  normal_estimator.setKSearch(cfg.normal_k);
  normal_estimator.compute(*normals);

  const float pi = 3.14159265358979323846F;
  const float min_abs_nz = std::cos(cfg.max_slope_deg * pi / 180.0F);
  Cloud::Ptr candidates(new Cloud);
  std::vector<int> candidate_to_downsampled;
  std::vector<char> downsampled_is_candidate(downsampled->size(), 0);
  candidates->reserve(downsampled->size() / 2);
  candidate_to_downsampled.reserve(downsampled->size() / 2);
  for (std::size_t i = 0; i < downsampled->size(); ++i) {
    const auto & n = normals->points[i];
    if (!std::isfinite(n.normal_z) || !std::isfinite(n.curvature)) {
      continue;
    }
    const float abs_nz = std::abs(n.normal_z);
    if (abs_nz < min_abs_nz || n.curvature > cfg.max_curvature) {
      continue;
    }
    Point point = downsampled->points[i];
    point.intensity = std::acos(std::clamp(abs_nz, 0.0F, 1.0F)) * 180.0F / pi;
    candidates->push_back(point);
    candidate_to_downsampled.push_back(static_cast<int>(i));
    downsampled_is_candidate[i] = 1;
  }
  saveCloud(output_dir / "candidate_surface.pcd", *candidates);

  pcl::KdTreeFLANN<Point>::Ptr candidate_tree(new pcl::KdTreeFLANN<Point>);
  candidate_tree->setInputCloud(candidates);
  std::size_t initial_matched_seeds = 0;
  const auto initially_reachable = growRegion(
    candidates, candidate_tree, trajectory, nullptr, cfg, initial_matched_seeds);

  pcl::KdTreeFLANN<Point> map_tree;
  map_tree.setInputCloud(downsampled);
  // Build the former and stricter results from the same candidate surface so
  // RViz can compare them point-for-point.
  std::vector<char> original_clearance_ok(candidates->size(), 0);
  std::vector<char> strict_clearance_ok(candidates->size(), 0);
  std::size_t blocked_count = 0;
  std::size_t route_override_count = 0;
  const float clearance_query = std::hypot(cfg.clearance_radius, cfg.clearance_height);
  const float max_grade = std::tan(cfg.max_slope_deg * pi / 180.0F);
  std::vector<int> nearby;
  std::vector<float> nearby_distance;

  const auto has_clearance = [&](const Point & ground) {
    nearby.clear();
    nearby_distance.clear();
    map_tree.radiusSearch(ground, clearance_query, nearby, nearby_distance);
    for (const int map_id : nearby) {
      const auto & obstacle = downsampled->points[map_id];
      const float dx = obstacle.x - ground.x;
      const float dy = obstacle.y - ground.y;
      const float dxy = std::hypot(dx, dy);
      const float dz = obstacle.z - ground.z;
      if (dxy > cfg.clearance_radius || dz < cfg.clearance_min_z || dz > cfg.clearance_height) {
        continue;
      }
      const bool same_surface = downsampled_is_candidate[map_id] &&
        std::abs(dz) <= (0.08F + max_grade * dxy);
      if (!same_surface) {
        return false;
      }
    }
    return true;
  };

  for (std::size_t candidate_id = 0; candidate_id < candidates->size(); ++candidate_id) {
    if (!initially_reachable[candidate_id]) {
      continue;
    }
    const auto & ground = candidates->points[candidate_id];
    if (has_clearance(ground)) {
      original_clearance_ok[candidate_id] = 1;
      strict_clearance_ok[candidate_id] = 1;
    } else {
      ++blocked_count;
      // Preserve the old route exception only for the comparison cloud.
      if (trajectory_tree.nearestKSearch(ground, 1, one_index, one_distance) > 0 &&
          one_distance[0] <= cfg.traversed_override_radius * cfg.traversed_override_radius) {
        original_clearance_ok[candidate_id] = 1;
        ++route_override_count;
      }
    }
  }

  std::size_t original_matched_seeds = 0;
  const auto original_reachable = growRegion(
    candidates, candidate_tree, trajectory, &original_clearance_ok, cfg, original_matched_seeds);
  std::size_t strict_matched_seeds = 0;
  const auto strict_reachable = growRegion(
    candidates, candidate_tree, trajectory, &strict_clearance_ok, cfg, strict_matched_seeds);

  auto initial_region = selectCloud(candidates, [&](std::size_t i) {return initially_reachable[i];});
  auto original_geometric = selectCloud(
    candidates, [&](std::size_t i) {return original_reachable[i];});
  auto strict_geometric = selectCloud(
    candidates, [&](std::size_t i) {return strict_reachable[i];});
  auto rejected_geometric = selectCloud(
    candidates, [&](std::size_t i) {return original_reachable[i] && !strict_reachable[i];});
  saveCloud(output_dir / "reachable_before_clearance.pcd", *initial_region);
  saveCloud(output_dir / "traversable_surface_geometric_original.pcd", *original_geometric);
  saveCloud(output_dir / "traversable_surface_geometric.pcd", *strict_geometric);

  auto original_route_ribbon = buildRouteSupportedRibbon(trajectory, cfg);
  saveCloud(output_dir / "route_supported_surface_original.pcd", *original_route_ribbon);

  Cloud::Ptr strict_route_ribbon(new Cloud);
  Cloud::Ptr rejected_route_ribbon(new Cloud);
  strict_route_ribbon->reserve(original_route_ribbon->size());
  rejected_route_ribbon->reserve(original_route_ribbon->size());
  for (const auto & point : *original_route_ribbon) {
    if (has_clearance(point)) {
      strict_route_ribbon->push_back(point);
    } else {
      rejected_route_ribbon->push_back(point);
    }
  }
  strict_route_ribbon->width = strict_route_ribbon->size();
  strict_route_ribbon->height = 1;
  strict_route_ribbon->is_dense = true;
  rejected_route_ribbon->width = rejected_route_ribbon->size();
  rejected_route_ribbon->height = 1;
  rejected_route_ribbon->is_dense = true;
  saveCloud(output_dir / "route_supported_surface.pcd", *strict_route_ribbon);

  pcl::VoxelGrid<Point> continuous_voxel;
  Cloud::Ptr original_raw(new Cloud);
  *original_raw = *original_geometric;
  *original_raw += *original_route_ribbon;
  Cloud::Ptr original_traversable(new Cloud);
  continuous_voxel.setInputCloud(original_raw);
  continuous_voxel.setLeafSize(0.05F, 0.05F, 0.05F);
  continuous_voxel.filter(*original_traversable);
  saveCloud(output_dir / "traversable_surface_original.pcd", *original_traversable);

  Cloud::Ptr continuous_raw(new Cloud);
  *continuous_raw = *strict_geometric;
  *continuous_raw += *strict_route_ribbon;
  Cloud::Ptr traversable(new Cloud);
  continuous_voxel.setInputCloud(continuous_raw);
  continuous_voxel.setLeafSize(0.05F, 0.05F, 0.05F);
  continuous_voxel.filter(*traversable);
  saveCloud(output_dir / "traversable_surface.pcd", *traversable);

  Cloud::Ptr rejected_raw(new Cloud);
  *rejected_raw = *rejected_geometric;
  *rejected_raw += *rejected_route_ribbon;
  Cloud::Ptr rejected_by_clearance(new Cloud);
  continuous_voxel.setInputCloud(rejected_raw);
  continuous_voxel.filter(*rejected_by_clearance);
  saveCloud(output_dir / "traversable_rejected_by_clearance.pcd", *rejected_by_clearance);

  Cloud::Ptr corridor_geometric(new Cloud);
  corridor_geometric->reserve(strict_geometric->size() / 4);
  for (const auto & point : *strict_geometric) {
    if (trajectory_tree.nearestKSearch(point, 1, one_index, one_distance) > 0 &&
        one_distance[0] <= cfg.corridor_radius * cfg.corridor_radius) {
      corridor_geometric->push_back(point);
    }
  }
  Cloud::Ptr corridor_raw(new Cloud);
  *corridor_raw = *corridor_geometric;
  *corridor_raw += *strict_route_ribbon;
  Cloud::Ptr corridor(new Cloud);
  continuous_voxel.setInputCloud(corridor_raw);
  continuous_voxel.filter(*corridor);
  saveCloud(output_dir / "traversed_corridor.pcd", *corridor);

  auto obstacles = selectCloud(downsampled, [&](std::size_t i) {return !downsampled_is_candidate[i];});
  saveCloud(output_dir / "obstacle_points.pcd", *obstacles);

  std::ofstream preview(output_dir / "preview_points.csv");
  preview << "x,y,z,value,category\n";
  appendPreview(preview, *obstacles, "obstacle", 7000);
  appendPreview(preview, *candidates, "candidate", 9000);
  appendPreview(preview, *traversable, "traversable", 14000);
  appendPreview(preview, *corridor, "corridor", 7000);
  appendPreview(preview, *trajectory, "trajectory", 5000);

  std::ofstream summary(output_dir / "extraction_summary.json");
  summary << std::fixed << std::setprecision(3);
  summary << "{\n"
          << "  \"map\": \"" << map_path.string() << "\",\n"
          << "  \"trajectory\": \"" << trajectory_path.string() << "\",\n"
          << "  \"input_points\": " << full->size() << ",\n"
          << "  \"trajectory_seed_points\": " << trajectory->size() << ",\n"
          << "  \"cropped_points\": " << cropped->size() << ",\n"
          << "  \"downsampled_points\": " << downsampled->size() << ",\n"
          << "  \"candidate_surface_points\": " << candidates->size() << ",\n"
          << "  \"initial_seed_matches\": " << initial_matched_seeds << ",\n"
          << "  \"reachable_before_clearance_points\": " << initial_region->size() << ",\n"
          << "  \"clearance_blocked_points\": " << blocked_count << ",\n"
          << "  \"route_override_points_in_original\": " << route_override_count << ",\n"
          << "  \"original_seed_matches\": " << original_matched_seeds << ",\n"
          << "  \"strict_seed_matches\": " << strict_matched_seeds << ",\n"
          << "  \"original_geometric_traversable_points\": " << original_geometric->size() << ",\n"
          << "  \"strict_geometric_traversable_points\": " << strict_geometric->size() << ",\n"
          << "  \"original_route_surface_points\": " << original_route_ribbon->size() << ",\n"
          << "  \"strict_route_surface_points\": " << strict_route_ribbon->size() << ",\n"
          << "  \"original_traversable_surface_points\": " << original_traversable->size() << ",\n"
          << "  \"traversable_surface_points\": " << traversable->size() << ",\n"
          << "  \"rejected_by_clearance_points\": " << rejected_by_clearance->size() << ",\n"
          << "  \"traversed_corridor_points\": " << corridor->size() << ",\n"
          << "  \"parameters\": {\n"
          << "    \"crop_radius_m\": " << cfg.crop_radius << ",\n"
          << "    \"voxel_m\": " << cfg.voxel << ",\n"
          << "    \"max_slope_deg\": " << cfg.max_slope_deg << ",\n"
          << "    \"clearance_radius_m\": " << cfg.clearance_radius << ",\n"
          << "    \"clearance_height_m\": " << cfg.clearance_height << ",\n"
          << "    \"route_half_width_m\": " << cfg.route_half_width << ",\n"
          << "    \"route_sample_spacing_m\": " << cfg.route_sample_spacing << "\n"
          << "  }\n"
          << "}\n";

  std::cout << "input=" << full->size()
            << " cropped=" << cropped->size()
            << " downsampled=" << downsampled->size()
            << " candidates=" << candidates->size()
            << " initial_region=" << initial_region->size()
            << " blocked=" << blocked_count
            << " original=" << original_traversable->size()
            << " strict=" << traversable->size()
            << " rejected=" << rejected_by_clearance->size()
            << " corridor=" << corridor->size()
            << " seed_matches=" << strict_matched_seeds << '/' << trajectory->size()
            << '\n';
  return 0;
}
