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
  // A non-positive value disables route-centered cropping.  The route-supported
  // branch remains available, while the route-independent branch can recover
  // geometrically traversable surfaces throughout the registered map.
  float crop_radius = 0.0F;
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
  std::size_t min_unseeded_component_points = 80;
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

static std::vector<char> keepLargeAllowedComponents(
  const Cloud::Ptr & candidate,
  const pcl::KdTreeFLANN<Point>::Ptr & tree,
  const std::vector<char> & allowed,
  const Config & cfg,
  const std::size_t minimum_points,
  std::size_t & component_count,
  std::size_t & kept_component_count)
{
  std::vector<char> examined(candidate->size(), 0);
  std::vector<char> kept(candidate->size(), 0);
  component_count = 0;
  kept_component_count = 0;

  std::vector<int> neighbors;
  std::vector<float> squared_distance;
  for (std::size_t start = 0; start < candidate->size(); ++start) {
    if (examined[start] || !allowed[start]) {
      continue;
    }

    ++component_count;
    std::queue<int> frontier;
    std::vector<int> component;
    examined[start] = 1;
    frontier.push(static_cast<int>(start));

    while (!frontier.empty()) {
      const int current = frontier.front();
      frontier.pop();
      component.push_back(current);

      neighbors.clear();
      squared_distance.clear();
      tree->radiusSearch(
        candidate->points[static_cast<std::size_t>(current)],
        cfg.connection_radius, neighbors, squared_distance);
      for (const int next : neighbors) {
        const auto next_index = static_cast<std::size_t>(next);
        if (examined[next_index] || !allowed[next_index]) {
          continue;
        }
        const auto & a = candidate->points[static_cast<std::size_t>(current)];
        const auto & b = candidate->points[next_index];
        const float dxy = std::hypot(b.x - a.x, b.y - a.y);
        const float dz = std::abs(b.z - a.z);
        if (dxy < cfg.voxel * 0.35F || dz > cfg.max_neighbor_dz) {
          continue;
        }
        examined[next_index] = 1;
        frontier.push(next);
      }
    }

    if (component.size() < minimum_points) {
      continue;
    }
    ++kept_component_count;
    for (const int id : component) {
      kept[static_cast<std::size_t>(id)] = 1;
    }
  }
  return kept;
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
  std::vector<char> geometric_clearance_ok(candidates->size(), 0);
  std::vector<char> route_clearance_ok(candidates->size(), 0);
  std::size_t blocked_count = 0;
  std::size_t route_override_count = 0;
  const float clearance_query = std::hypot(cfg.clearance_radius, cfg.clearance_height);
  const float max_grade = std::tan(cfg.max_slope_deg * pi / 180.0F);
  std::vector<int> nearby;
  std::vector<float> nearby_distance;
  for (std::size_t candidate_id = 0; candidate_id < candidates->size(); ++candidate_id) {
    const auto & ground = candidates->points[candidate_id];
    nearby.clear();
    nearby_distance.clear();
    map_tree.radiusSearch(ground, clearance_query, nearby, nearby_distance);
    bool blocked = false;
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
        blocked = true;
        break;
      }
    }
    if (blocked) {
      ++blocked_count;
    } else {
      geometric_clearance_ok[candidate_id] = 1;
      route_clearance_ok[candidate_id] = 1;
    }

    // Keep the original route-supported behavior as a second, high-confidence
    // branch.  The robot's recorded passage can override conservative clearance
    // failures locally, but it no longer gates the global geometric branch.
    if (!route_clearance_ok[candidate_id] &&
        trajectory_tree.nearestKSearch(ground, 1, one_index, one_distance) > 0 &&
        one_distance[0] <= cfg.traversed_override_radius * cfg.traversed_override_radius) {
      route_clearance_ok[candidate_id] = 1;
      ++route_override_count;
    }
  }

  std::size_t safe_matched_seeds = 0;
  const auto safely_reachable = growRegion(
    candidates, candidate_tree, trajectory, &route_clearance_ok, cfg, safe_matched_seeds);

  std::size_t unseeded_component_count = 0;
  std::size_t kept_unseeded_component_count = 0;
  const auto unseeded_geometric_mask = keepLargeAllowedComponents(
    candidates, candidate_tree, geometric_clearance_ok, cfg,
    cfg.min_unseeded_component_points,
    unseeded_component_count, kept_unseeded_component_count);

  auto initial_region = selectCloud(candidates, [&](std::size_t i) {return initially_reachable[i];});
  auto route_seeded_geometric = selectCloud(
    candidates, [&](std::size_t i) {return safely_reachable[i];});
  auto unseeded_geometric = selectCloud(
    candidates, [&](std::size_t i) {return unseeded_geometric_mask[i];});
  saveCloud(output_dir / "reachable_before_clearance.pcd", *initial_region);
  saveCloud(output_dir / "traversable_surface_route_seeded.pcd", *route_seeded_geometric);
  saveCloud(output_dir / "traversable_surface_unseeded.pcd", *unseeded_geometric);

  Cloud::Ptr combined_geometric_raw(new Cloud);
  *combined_geometric_raw = *unseeded_geometric;
  *combined_geometric_raw += *route_seeded_geometric;
  Cloud::Ptr traversable_geometric(new Cloud);
  pcl::VoxelGrid<Point> geometric_voxel;
  geometric_voxel.setInputCloud(combined_geometric_raw);
  geometric_voxel.setLeafSize(cfg.voxel, cfg.voxel, cfg.voxel);
  geometric_voxel.filter(*traversable_geometric);
  saveCloud(output_dir / "traversable_surface_geometric.pcd", *traversable_geometric);

  auto route_ribbon = buildRouteSupportedRibbon(trajectory, cfg);
  saveCloud(output_dir / "route_supported_surface.pcd", *route_ribbon);

  Cloud::Ptr continuous_raw(new Cloud);
  *continuous_raw = *traversable_geometric;
  *continuous_raw += *route_ribbon;
  Cloud::Ptr traversable(new Cloud);
  pcl::VoxelGrid<Point> continuous_voxel;
  continuous_voxel.setInputCloud(continuous_raw);
  continuous_voxel.setLeafSize(0.05F, 0.05F, 0.05F);
  continuous_voxel.filter(*traversable);
  saveCloud(output_dir / "traversable_surface.pcd", *traversable);

  Cloud::Ptr corridor_geometric(new Cloud);
  corridor_geometric->reserve(route_seeded_geometric->size() / 4);
  for (const auto & point : *route_seeded_geometric) {
    if (trajectory_tree.nearestKSearch(point, 1, one_index, one_distance) > 0 &&
        one_distance[0] <= cfg.corridor_radius * cfg.corridor_radius) {
      corridor_geometric->push_back(point);
    }
  }
  Cloud::Ptr corridor_raw(new Cloud);
  *corridor_raw = *corridor_geometric;
  *corridor_raw += *route_ribbon;
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
          << "  \"route_clearance_override_points\": " << route_override_count << ",\n"
          << "  \"safe_seed_matches\": " << safe_matched_seeds << ",\n"
          << "  \"route_seeded_geometric_points\": " << route_seeded_geometric->size() << ",\n"
          << "  \"unseeded_component_count\": " << unseeded_component_count << ",\n"
          << "  \"kept_unseeded_component_count\": " << kept_unseeded_component_count << ",\n"
          << "  \"unseeded_geometric_points\": " << unseeded_geometric->size() << ",\n"
          << "  \"geometric_traversable_points\": " << traversable_geometric->size() << ",\n"
          << "  \"route_supported_surface_points\": " << route_ribbon->size() << ",\n"
          << "  \"traversable_surface_points\": " << traversable->size() << ",\n"
          << "  \"traversed_corridor_points\": " << corridor->size() << ",\n"
          << "  \"parameters\": {\n"
          << "    \"crop_radius_m\": " << cfg.crop_radius << ",\n"
          << "    \"voxel_m\": " << cfg.voxel << ",\n"
          << "    \"max_slope_deg\": " << cfg.max_slope_deg << ",\n"
          << "    \"clearance_radius_m\": " << cfg.clearance_radius << ",\n"
          << "    \"clearance_height_m\": " << cfg.clearance_height << ",\n"
          << "    \"min_unseeded_component_points\": "
          << cfg.min_unseeded_component_points << ",\n"
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
            << " route_seeded=" << route_seeded_geometric->size()
            << " unseeded=" << unseeded_geometric->size()
            << " geometric=" << traversable_geometric->size()
            << " traversable=" << traversable->size()
            << " corridor=" << corridor->size()
            << " seed_matches=" << safe_matched_seeds << '/' << trajectory->size()
            << '\n';
  return 0;
}
