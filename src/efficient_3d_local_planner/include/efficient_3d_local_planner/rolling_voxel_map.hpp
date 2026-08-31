#pragma once

#include "efficient_3d_local_planner/voxel_types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace efficient_3d_local_planner
{

class RollingVoxelMap
{
public:
  struct Config
  {
    double resolution{0.15};
    Eigen::Vector3d size{12.0, 12.0, 4.8};
    double log_hit{0.90};
    double log_miss{-0.45};
    double log_min{-2.0};
    double log_max{3.5};
    double occupied_threshold{0.80};
    int hit_confirmation_count{2};
    bool raycast_enabled{true};
    bool decay_enabled{true};
    double decay_start{0.75};
    double decay_rate{1.20};
    bool decay_front_only{true};
    double decay_front_fov_deg{180.0};
    double ray_step_factor{0.90};
    double cylinder_radius{0.25};
    double obstacles_inflation_z_down{0.10};
    double obstacles_inflation_z_up{0.00};
    double soft_clearance{0.225};
  };

  struct Layers
  {
    Eigen::Vector3i min_key{Eigen::Vector3i::Zero()};
    Eigen::Vector3i dimensions{Eigen::Vector3i::Zero()};
    std::vector<std::uint32_t> hard;
    std::vector<std::uint32_t> footprint_hard;
    std::vector<std::uint32_t> soft_indices;
    std::vector<std::uint8_t> soft_costs;
  };

  struct UpdateResult
  {
    std::size_t input_points{0};
    std::size_t unique_endpoints{0};
    std::size_t touched_voxels{0};
    std::size_t stored_voxels{0};
    double raycast_ms{0.0};
  };

  RollingVoxelMap() : RollingVoxelMap(Config{}) {}

  explicit RollingVoxelMap(Config config)
  : config_(std::move(config))
  {
    dimensions_ = (config_.size / config_.resolution).array().round().cast<int>();
    dimensions_ = dimensions_.cwiseMax(Eigen::Vector3i::Ones());
    buildInflationOffsets();
  }

  const Config & config() const noexcept {return config_;}
  const Eigen::Vector3i & dimensions() const noexcept {return dimensions_;}
  std::size_t storedCellCount() const noexcept {return cells_.size();}

  std::size_t clearBodyExclusion(
    const Eigen::Vector3d & body_center, const Eigen::Quaterniond & body_orientation,
    const Eigen::Vector3d & half_extents)
  {
    if (!body_center.allFinite() || !body_orientation.coeffs().allFinite() ||
      body_orientation.norm() < 1e-6 || !half_extents.allFinite() ||
      (half_extents.array() < 0.0).any())
    {
      return 0U;
    }

    const Eigen::Quaterniond world_from_body = body_orientation.normalized();
    std::size_t cleared = 0U;
    for (auto iterator = cells_.begin(); iterator != cells_.end();) {
      const Eigen::Vector3d cell_center =
        keyToCenter(iterator->first, config_.resolution);
      const Eigen::Vector3d body_point =
        world_from_body.conjugate() * (cell_center - body_center);
      const bool occupied = iterator->second.hard_confirmed &&
        iterator->second.log_odds >= config_.occupied_threshold;
      if (occupied && (body_point.array().abs() <= half_extents.array()).all()) {
        iterator = cells_.erase(iterator);
        ++cleared;
      } else {
        ++iterator;
      }
    }
    return cleared;
  }

  Eigen::Vector3i minimumKey(const Eigen::Vector3d & center) const
  {
    const VoxelKey center_key = pointToKey(center, config_.resolution);
    return Eigen::Vector3i(center_key.x, center_key.y, center_key.z) - dimensions_ / 2;
  }

  bool insideWindow(
    const VoxelKey & key, const Eigen::Vector3i & minimum) const noexcept
  {
    return key.x >= minimum.x() && key.y >= minimum.y() && key.z >= minimum.z() &&
           key.x < minimum.x() + dimensions_.x() &&
           key.y < minimum.y() + dimensions_.y() &&
           key.z < minimum.z() + dimensions_.z();
  }

  UpdateResult update(
    const Eigen::Vector3d & sensor_origin,
    const Eigen::Vector3d & window_center,
    const std::vector<Eigen::Vector3d> & world_points,
    const double stamp_seconds,
    const Eigen::Quaterniond & body_orientation = Eigen::Quaterniond::Identity())
  {
    UpdateResult result;
    result.input_points = world_points.size();
    const Eigen::Vector3i minimum = minimumKey(window_center);
    last_minimum_ = minimum;

    std::unordered_set<VoxelKey, VoxelKeyHash> endpoints;
    endpoints.reserve(world_points.size());
    for (const auto & point : world_points) {
      if (!point.allFinite()) {
        continue;
      }
      const VoxelKey key = pointToKey(point, config_.resolution);
      if (insideWindow(key, minimum)) {
        endpoints.insert(key);
      }
    }
    result.unique_endpoints = endpoints.size();

    // A hit wins over all free-ray observations in the same scan.
    std::unordered_map<VoxelKey, std::int8_t, VoxelKeyHash> observations;
    observations.reserve(endpoints.size() * 8U);
    const double step = std::max(1e-3, config_.resolution * config_.ray_step_factor);
    const auto raycast_begin = std::chrono::steady_clock::now();
    for (const auto & endpoint_key : endpoints) {
      const Eigen::Vector3d endpoint = keyToCenter(endpoint_key, config_.resolution);
      if (config_.raycast_enabled) {
        const Eigen::Vector3d delta = endpoint - sensor_origin;
        const int steps = std::max(1, static_cast<int>(std::ceil(delta.norm() / step)));
        for (int i = 1; i < steps; ++i) {
          const VoxelKey free_key = pointToKey(
            sensor_origin + delta * (static_cast<double>(i) / steps), config_.resolution);
          if (insideWindow(free_key, minimum)) {
            observations.try_emplace(free_key, static_cast<std::int8_t>(-1));
          }
        }
      }
      observations[endpoint_key] = 1;
    }
    result.raycast_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - raycast_begin).count();
    result.touched_voxels = observations.size();

    Eigen::Vector2d body_forward_xy = Eigen::Vector2d::Zero();
    bool have_body_forward = false;
    constexpr double kPi = 3.14159265358979323846;
    const double decay_half_angle = 0.5 * std::clamp(
      config_.decay_front_fov_deg, 0.0, 180.0) * kPi / 180.0;
    const double decay_cosine_threshold = std::cos(decay_half_angle);
    if (body_orientation.coeffs().allFinite() && body_orientation.norm() >= 1e-6) {
      Eigen::Vector3d body_forward =
        body_orientation.normalized() * Eigen::Vector3d::UnitX();
      body_forward.z() = 0.0;
      const double horizontal_norm = body_forward.head<2>().norm();
      if (std::isfinite(horizontal_norm) && horizontal_norm >= 1e-6) {
        body_forward_xy = body_forward.head<2>() / horizontal_norm;
        have_body_forward = true;
      }
    }

    for (const auto & observation : observations) {
      Cell & cell = cells_[observation.first];
      cell.log_odds = std::clamp(
        cell.log_odds + (observation.second > 0 ? config_.log_hit : config_.log_miss),
        config_.log_min, config_.log_max);
      cell.last_update = stamp_seconds;
      if (observation.second > 0) {
        cell.last_hit = stamp_seconds;
        cell.consecutive_hits = std::min(
          config_.hit_confirmation_count, cell.consecutive_hits + 1);
        if (cell.consecutive_hits >= config_.hit_confirmation_count &&
          cell.log_odds >= config_.occupied_threshold)
        {
          cell.hard_confirmed = true;
        }
      } else {
        cell.consecutive_hits = 0;
        if (cell.log_odds < config_.occupied_threshold) {
          cell.hard_confirmed = false;
        }
      }
    }

    for (auto iterator = cells_.begin(); iterator != cells_.end();) {
      if (!insideWindow(iterator->first, minimum)) {
        iterator = cells_.erase(iterator);
        continue;
      }
      Cell & cell = iterator->second;
      const Eigen::Vector3d cell_center = keyToCenter(iterator->first, config_.resolution);
      const Eigen::Vector2d displacement_xy =
        (cell_center - window_center).head<2>();
      const double displacement_norm = displacement_xy.norm();
      const bool in_decay_sector = have_body_forward && displacement_norm >= 1e-6 &&
        displacement_xy.dot(body_forward_xy) >
        displacement_norm * decay_cosine_threshold;
      const bool preserve_behind = config_.decay_enabled && config_.decay_front_only &&
        cell.hard_confirmed && cell.last_hit > 0.0 &&
        cell.log_odds >= config_.occupied_threshold &&
        !in_decay_sector;
      if (preserve_behind) {
        // Match SCAN-Planner's front-only decay policy. Free-ray observations
        // have already been applied above; only passive time decay is paused.
        // Advancing the decay clock avoids deleting the voxel in one large
        // catch-up step if a later body rotation places it in front.
        cell.last_update = stamp_seconds;
        ++iterator;
        continue;
      }
      const double update_age = std::max(0.0, stamp_seconds - cell.last_update);
      const double hit_age = std::max(0.0, stamp_seconds - cell.last_hit);
      if (config_.decay_enabled && cell.last_hit > 0.0 &&
        hit_age > config_.decay_start && update_age > 0.0)
      {
        cell.log_odds = std::max(
          config_.log_min, cell.log_odds - config_.decay_rate * update_age);
        cell.last_update = stamp_seconds;
        if (cell.log_odds < config_.occupied_threshold) {
          cell.hard_confirmed = false;
        }
      }
      if (cell.log_odds <= config_.log_min + 1e-6 && update_age > 0.5) {
        iterator = cells_.erase(iterator);
      } else {
        ++iterator;
      }
    }
    result.stored_voxels = cells_.size();
    return result;
  }

  Layers buildLayers() const
  {
    Layers layers;
    layers.min_key = last_minimum_;
    layers.dimensions = dimensions_;

    std::vector<VoxelKey> occupied;
    occupied.reserve(cells_.size() / 4U);
    layers.hard.reserve(cells_.size() / 4U);
    for (const auto & item : cells_) {
      if (item.second.hard_confirmed && item.second.log_odds >= config_.occupied_threshold) {
        occupied.push_back(item.first);
        // cells_ 中的体素键天然唯一，并且 update() 已经清除了滚动窗口外的体素，
        // 因此这里不需要再通过 unordered_set 去重。
        layers.hard.push_back(linearIndex(item.first, last_minimum_));
      }
    }

    // 每个索引只需一个字节：0 表示未触及，1~254 表示软代价，255 表示硬膨胀。
    // 相比每帧创建三个哈希容器，这种连续内存访问更适合数万个障碍体素的大地图。
    constexpr std::uint8_t kFootprintHard = 255U;
    const std::size_t cell_count =
      static_cast<std::size_t>(dimensions_.x()) *
      static_cast<std::size_t>(dimensions_.y()) *
      static_cast<std::size_t>(dimensions_.z());
    std::vector<std::uint8_t> layer_state(cell_count, 0U);
    std::vector<std::uint32_t> touched_indices;
    const std::size_t maximum_touched = std::min(
      cell_count, occupied.size() * inflation_offsets_.size());
    touched_indices.reserve(maximum_touched);

    for (const auto & obstacle : occupied) {
      for (const auto & offset : inflation_offsets_) {
        const VoxelKey candidate{
          obstacle.x + offset.dx, obstacle.y + offset.dy, obstacle.z + offset.dz};
        if (!insideWindow(candidate, last_minimum_)) {
          continue;
        }
        const std::uint32_t index = linearIndex(candidate, last_minimum_);
        std::uint8_t & state = layer_state[index];
        if (state == 0U) {
          touched_indices.push_back(index);
        }
        if (offset.cost == kFootprintHard) {
          // 硬膨胀始终覆盖软代价，与旧实现中的 soft_map.erase() 语义一致。
          state = kFootprintHard;
        } else if (state != kFootprintHard) {
          // 多个障碍的软膨胀重叠时保留最大代价。
          state = std::max(state, offset.cost);
        }
      }
    }

    layers.footprint_hard.reserve(touched_indices.size());
    layers.soft_indices.reserve(touched_indices.size());
    layers.soft_costs.reserve(touched_indices.size());
    for (const std::uint32_t index : touched_indices) {
      const std::uint8_t state = layer_state[index];
      if (state == kFootprintHard) {
        layers.footprint_hard.push_back(index);
      } else {
        layers.soft_indices.push_back(index);
        layers.soft_costs.push_back(state);
      }
    }
    return layers;
  }

  Eigen::Vector3d origin(const Layers & layers) const
  {
    return config_.resolution * layers.min_key.cast<double>();
  }

private:
  struct InflationOffset
  {
    int dx{0};
    int dy{0};
    int dz{0};
    // 255 表示硬膨胀；1~254 表示软膨胀代价。
    std::uint8_t cost{0U};
  };

  struct Cell
  {
    double log_odds{0.0};
    double last_update{0.0};
    double last_hit{0.0};
    int consecutive_hits{0};
    bool hard_confirmed{false};
  };

  std::uint32_t linearIndex(
    const VoxelKey & key, const Eigen::Vector3i & minimum) const noexcept
  {
    const int x = key.x - minimum.x();
    const int y = key.y - minimum.y();
    const int z = key.z - minimum.z();
    return static_cast<std::uint32_t>((z * dimensions_.y() + y) * dimensions_.x() + x);
  }

  void buildInflationOffsets()
  {
    constexpr std::uint8_t kFootprintHard = 255U;
    const int maximum_xy = static_cast<int>(std::ceil(
        (config_.cylinder_radius + config_.soft_clearance) / config_.resolution));
    const int down = static_cast<int>(std::ceil(
        config_.obstacles_inflation_z_down / config_.resolution));
    const int up = static_cast<int>(std::ceil(
        config_.obstacles_inflation_z_up / config_.resolution));
    inflation_offsets_.reserve(
      static_cast<std::size_t>(2 * maximum_xy + 1) *
      static_cast<std::size_t>(2 * maximum_xy + 1) *
      static_cast<std::size_t>(down + up + 1));

    // 膨胀参数在地图对象生命周期内不变，把几何距离、垂直范围和软代价
    // 预先算成偏移模板，避免 buildLayers() 在每一帧为每个障碍重复计算。
    for (int dx = -maximum_xy; dx <= maximum_xy; ++dx) {
      for (int dy = -maximum_xy; dy <= maximum_xy; ++dy) {
        const double radial = config_.resolution * std::hypot(dx, dy);
        const bool footprint = radial <= config_.cylinder_radius + 0.5 * config_.resolution;
        const double clearance = radial - config_.cylinder_radius;
        const bool soft = !footprint && clearance < config_.soft_clearance;
        if (!footprint && !soft) {
          continue;
        }

        std::uint8_t cost = kFootprintHard;
        if (soft && config_.soft_clearance > 1e-9) {
          const double normalized = std::clamp(
            (config_.soft_clearance - clearance) / config_.soft_clearance, 0.0, 1.0);
          cost = static_cast<std::uint8_t>(
            std::clamp(std::lround(254.0 * normalized * normalized), 1L, 254L));
        }

        for (int dz = -down; dz <= up; ++dz) {
          const double vertical = dz * config_.resolution;
          const double half_voxel = 0.5 * config_.resolution;
          if (vertical < -config_.obstacles_inflation_z_down - half_voxel ||
            vertical > config_.obstacles_inflation_z_up + half_voxel)
          {
            continue;
          }
          inflation_offsets_.push_back(InflationOffset{dx, dy, dz, cost});
        }
      }
    }
  }

  Config config_;
  Eigen::Vector3i dimensions_{Eigen::Vector3i::Ones()};
  Eigen::Vector3i last_minimum_{Eigen::Vector3i::Zero()};
  std::vector<InflationOffset> inflation_offsets_;
  std::unordered_map<VoxelKey, Cell, VoxelKeyHash> cells_;
};

}  // namespace efficient_3d_local_planner
