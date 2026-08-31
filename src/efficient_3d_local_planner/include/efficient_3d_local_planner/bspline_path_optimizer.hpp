#pragma once

#include "efficient_3d_local_planner/guided_astar.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace efficient_3d_local_planner
{

// A deliberately small, dependency-free cubic B-spline optimizer.  A result is
// eligible for control only after the sampled curve passes the hard-collision
// and minimum movable-clearance checks below.
class BsplinePathOptimizer
{
public:
  struct Config
  {
    bool enabled{true};
    double control_point_spacing{0.20};
    double sample_spacing{0.05};
    int max_iterations{80};
    int max_clearance_repair_iterations{25};
    double max_step{0.04};
    double lambda_smooth{1.0};
    double lambda_collision{400.0};
    double lambda_reference{3.0};
    double lambda_goal{4.0};
    double clearance_distance{0.30};
    double minimum_acceptable_clearance{0.25};
    double start_clearance_ignore_distance{0.20};
    double reference_deadband{0.10};
    double max_deviation{0.75};
    double max_z_deviation{0.75};
    double goal_max_deviation{0.55};
    bool allow_goal_adjustment{true};
    double cylinder_offset{0.18};
  };

  struct Result
  {
    bool success{false};
    std::string reason{"not_run"};
    int iterations{0};
    int clearance_repair_iterations{0};
    int hard_collision_samples{0};
    double initial_cost{0.0};
    double final_cost{0.0};
    double smooth_cost{0.0};
    double collision_cost{0.0};
    double reference_cost{0.0};
    double goal_cost{0.0};
    double minimum_clearance{0.0};
    double minimum_movable_clearance{0.0};
    double maximum_reference_deviation{0.0};
    std::vector<Eigen::Vector3d> control_points;
    std::vector<Eigen::Vector3d> path;
  };

  BsplinePathOptimizer() : BsplinePathOptimizer(Config{}) {}

  explicit BsplinePathOptimizer(Config config) : config_(std::move(config)) {}

  const Config & config() const {return config_;}

  Result optimize(
    const GridSnapshot & map, const std::vector<Eigen::Vector3d> & reference) const
  {
    Result result;
    if (!config_.enabled) {
      result.reason = "disabled";
      return result;
    }
    if (!map.valid() || reference.size() < 2U || !validConfig()) {
      result.reason = "invalid_input";
      return result;
    }

    std::vector<Eigen::Vector3d> controls = resamplePath(
      reference, config_.control_point_spacing);
    if (controls.size() < 3U) {
      result.reason = "path_too_short";
      result.path = reference;
      result.control_points = controls;
      return result;
    }

    Objective current = objective(map, reference, controls, true);
    result.initial_cost = current.total;
    constexpr double kGradientTolerance = 1e-5;
    constexpr double kCostTolerance = 1e-7;

    for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
      result.iterations = iteration + 1;
      current.gradient.front().setZero();
      if (!config_.allow_goal_adjustment) {current.gradient.back().setZero();}
      double maximum_gradient = 0.0;
      const std::size_t movable_end = config_.allow_goal_adjustment ?
        current.gradient.size() : current.gradient.size() - 1U;
      for (std::size_t i = 1U; i < movable_end; ++i) {
        maximum_gradient = std::max(maximum_gradient, current.gradient[i].norm());
      }
      if (!std::isfinite(maximum_gradient) || maximum_gradient < kGradientTolerance) {
        break;
      }

      const double direction_scale = config_.max_step / maximum_gradient;
      bool accepted = false;
      double line_scale = 1.0;
      std::vector<Eigen::Vector3d> candidate = controls;
      Objective candidate_objective;
      for (int line_search = 0; line_search < 10; ++line_search) {
        candidate = controls;
        for (std::size_t i = 1U; i < movable_end; ++i) {
          candidate[i] -= line_scale * direction_scale * current.gradient[i];
        }
        constrainToReference(candidate, reference);
        candidate.front() = controls.front();
        if (!config_.allow_goal_adjustment) {candidate.back() = controls.back();}
        candidate_objective = objective(map, reference, candidate, false);
        if (std::isfinite(candidate_objective.total) &&
          candidate_objective.total + kCostTolerance < current.total)
        {
          accepted = true;
          break;
        }
        line_scale *= 0.5;
      }
      if (!accepted) {break;}

      const double improvement = current.total - candidate_objective.total;
      controls = std::move(candidate);
      current = objective(map, reference, controls, true);
      if (improvement < kCostTolerance) {break;}
    }

    // A second feasibility pass deliberately ignores smooth/reference/goal
    // costs.  The first pass finds a pleasant curve; this pass prevents those
    // soft objectives from winning over the requested obstacle clearance.
    for (int repair = 0; repair < config_.max_clearance_repair_iterations; ++repair) {
      ClearanceAssessment clearance = assessClearance(map, controls, true);
      clearance.gradient.front().setZero();
      if (!config_.allow_goal_adjustment) {clearance.gradient.back().setZero();}
      const std::size_t movable_end = config_.allow_goal_adjustment ?
        controls.size() : controls.size() - 1U;
      double maximum_gradient = 0.0;
      for (std::size_t i = 1U; i < movable_end; ++i) {
        maximum_gradient = std::max(maximum_gradient, clearance.gradient[i].norm());
      }
      if (!std::isfinite(maximum_gradient) || maximum_gradient < kGradientTolerance ||
        clearance.minimum + 1e-6 >= config_.minimum_acceptable_clearance)
      {
        break;
      }
      const double direction_scale = config_.max_step / maximum_gradient;
      bool accepted = false;
      double line_scale = 1.0;
      std::vector<Eigen::Vector3d> candidate = controls;
      for (int line_search = 0; line_search < 12; ++line_search) {
        candidate = controls;
        for (std::size_t i = 1U; i < movable_end; ++i) {
          candidate[i] -= line_scale * direction_scale * clearance.gradient[i];
        }
        constrainToReference(candidate, reference);
        candidate.front() = controls.front();
        if (!config_.allow_goal_adjustment) {candidate.back() = controls.back();}
        const ClearanceAssessment candidate_clearance = assessClearance(
          map, candidate, false);
        const bool improves_minimum =
          candidate_clearance.minimum > clearance.minimum + 1e-5;
        const bool improves_inside_escape =
          std::abs(candidate_clearance.minimum - clearance.minimum) <= 1e-5 &&
          candidate_clearance.worst_cost + kCostTolerance < clearance.worst_cost;
        if (improves_minimum || improves_inside_escape)
        {
          accepted = true;
          break;
        }
        line_scale *= 0.5;
      }
      if (!accepted) {break;}
      controls = std::move(candidate);
      result.clearance_repair_iterations = repair + 1;
    }

    const Objective final_objective = objective(map, reference, controls, false);
    result.control_points = controls;
    result.path = sampleCurve(controls);
    result.final_cost = final_objective.total;
    result.smooth_cost = final_objective.smooth;
    result.collision_cost = final_objective.collision;
    result.reference_cost = final_objective.reference;
    result.goal_cost = final_objective.goal;
    assessPath(map, reference, result);
    result.success = !result.path.empty() && result.hard_collision_samples == 0 &&
      result.minimum_movable_clearance + 1e-6 >= config_.minimum_acceptable_clearance;
    if (result.success) {
      result.reason = "success";
    } else if (result.hard_collision_samples > 0) {
      result.reason = "hard_collision";
    } else {
      result.reason = "insufficient_clearance";
    }
    return result;
  }

  std::vector<Eigen::Vector3d> sampleCurve(
    const std::vector<Eigen::Vector3d> & controls) const
  {
    std::vector<Eigen::Vector3d> path;
    if (controls.size() < 2U) {return controls;}
    const auto samples = curveSamples(controls);
    path.reserve(samples.size());
    for (const auto & sample : samples) {path.push_back(sample.position);}
    if (!path.empty()) {
      path.front() = controls.front();
      path.back() = controls.back();
    }
    return path;
  }

private:
  struct CurveSample
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d derivative{Eigen::Vector3d::Zero()};
    std::array<std::size_t, 4> indices{{0U, 0U, 0U, 0U}};
    std::array<double, 4> weights{{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> derivative_weights{{0.0, 0.0, 0.0, 0.0}};
  };

  struct Repulsion
  {
    double cost{0.0};
    Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
    double clearance{std::numeric_limits<double>::infinity()};
  };

  struct Objective
  {
    double total{0.0};
    double smooth{0.0};
    double collision{0.0};
    double reference{0.0};
    double goal{0.0};
    std::vector<Eigen::Vector3d> gradient;
  };

  struct ClearanceAssessment
  {
    double minimum{std::numeric_limits<double>::infinity()};
    double worst_cost{0.0};
    std::vector<Eigen::Vector3d> gradient;
  };

  bool validConfig() const
  {
    return config_.control_point_spacing > 0.0 && config_.sample_spacing > 0.0 &&
           config_.max_iterations >= 0 && config_.max_clearance_repair_iterations >= 0 &&
           config_.max_step > 0.0 &&
           config_.lambda_smooth >= 0.0 && config_.lambda_collision >= 0.0 &&
           config_.lambda_reference >= 0.0 && config_.lambda_goal >= 0.0 &&
           config_.clearance_distance >= 0.0 &&
           config_.minimum_acceptable_clearance >= 0.0 &&
           config_.minimum_acceptable_clearance <= config_.clearance_distance &&
           config_.start_clearance_ignore_distance >= 0.0 &&
           config_.reference_deadband >= 0.0 && config_.max_deviation > 0.0 &&
           config_.max_z_deviation >= 0.0 &&
           config_.goal_max_deviation > 0.0 &&
           config_.cylinder_offset >= 0.0;
  }

  static std::vector<Eigen::Vector3d> resamplePath(
    const std::vector<Eigen::Vector3d> & path, const double spacing)
  {
    if (path.size() < 2U) {return path;}
    const auto distance = cumulativeDistance(path);
    if (distance.back() < 1e-6) {return {path.front(), path.back()};}
    const int segments = std::max(2, static_cast<int>(std::ceil(distance.back() / spacing)));
    std::vector<Eigen::Vector3d> result;
    result.reserve(static_cast<std::size_t>(segments) + 1U);
    for (int i = 0; i <= segments; ++i) {
      result.push_back(interpolatePath(
          path, distance, distance.back() * static_cast<double>(i) / segments));
    }
    return result;
  }

  static CurveSample evaluate(
    const std::vector<Eigen::Vector3d> & controls, const std::size_t segment,
    const double unclamped_t)
  {
    CurveSample sample;
    const double t = std::clamp(unclamped_t, 0.0, 1.0);
    const double one_minus_t = 1.0 - t;
    sample.weights = {{
      one_minus_t * one_minus_t * one_minus_t / 6.0,
      (3.0 * t * t * t - 6.0 * t * t + 4.0) / 6.0,
      (-3.0 * t * t * t + 3.0 * t * t + 3.0 * t + 1.0) / 6.0,
      t * t * t / 6.0}};
    sample.derivative_weights = {{
      -0.5 * one_minus_t * one_minus_t,
      1.5 * t * t - 2.0 * t,
      -1.5 * t * t + t + 0.5,
      0.5 * t * t}};
    for (std::size_t k = 0U; k < 4U; ++k) {
      const long padded = static_cast<long>(segment + k);
      const long core = std::clamp(
        padded - 2L, 0L, static_cast<long>(controls.size() - 1U));
      sample.indices[k] = static_cast<std::size_t>(core);
      sample.position += sample.weights[k] * controls[sample.indices[k]];
      sample.derivative += sample.derivative_weights[k] * controls[sample.indices[k]];
    }
    return sample;
  }

  std::vector<CurveSample> curveSamples(
    const std::vector<Eigen::Vector3d> & controls) const
  {
    std::vector<CurveSample> samples;
    if (controls.size() < 2U) {return samples;}
    const int subdivisions = std::max(
      2, static_cast<int>(std::ceil(
          config_.control_point_spacing / std::max(1e-3, config_.sample_spacing))));
    samples.reserve((controls.size() + 1U) * static_cast<std::size_t>(subdivisions));
    for (std::size_t segment = 0U; segment <= controls.size(); ++segment) {
      for (int i = 0; i <= subdivisions; ++i) {
        if (segment > 0U && i == 0) {continue;}
        CurveSample sample = evaluate(
          controls, segment, static_cast<double>(i) / subdivisions);
        if (sample.position.allFinite() &&
          (samples.empty() || (sample.position - samples.back().position).norm() > 1e-6))
        {
          samples.push_back(std::move(sample));
        }
      }
    }
    return samples;
  }

  static Eigen::Vector2d sampleHeading(
    const std::vector<CurveSample> & samples, const std::size_t index)
  {
    const std::size_t previous = index == 0U ? 0U : index - 1U;
    const std::size_t next = index + 1U < samples.size() ? index + 1U : index;
    Eigen::Vector2d heading =
      (samples[next].position - samples[previous].position).head<2>();
    if (heading.norm() < 1e-6) {heading = Eigen::Vector2d::UnitX();}
    return heading.normalized();
  }

  static Eigen::Vector2d collisionHeading(
    const std::vector<CurveSample> & samples, const std::size_t index)
  {
    Eigen::Vector2d heading = samples[index].derivative.head<2>();
    if (heading.norm() < 1e-6) {return sampleHeading(samples, index);}
    return heading.normalized();
  }

  Repulsion repulsionAt(
    const GridSnapshot & map, const Eigen::Vector3d & point,
    const double influence_distance) const
  {
    Repulsion result;
    result.clearance = influence_distance;
    if (influence_distance <= 0.0) {return result;}
    const Eigen::Vector3i centre = map.pointToCell(point);
    const int z = centre.z();
    const int point_index = map.linear(centre);
    const bool inside = point_index < 0 ||
      map.hard[static_cast<std::size_t>(point_index)] != 0U;
    const double search_distance = inside ?
      influence_distance + config_.max_deviation + config_.goal_max_deviation :
      influence_distance;
    const int radius = static_cast<int>(std::ceil(search_distance / map.resolution)) + 2;
    double nearest = std::numeric_limits<double>::infinity();
    Eigen::Vector2d escape = Eigen::Vector2d::Zero();
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        const Eigen::Vector3i cell(centre.x() + dx, centre.y() + dy, z);
        const int index = map.linear(cell);
        if (index < 0) {continue;}
        const bool hard = map.hard[static_cast<std::size_t>(index)] != 0U;
        if (hard == inside) {continue;}
        const Eigen::Vector2d delta = map.cellCenter(cell).head<2>() - point.head<2>();
        const double distance = delta.norm();
        if (distance < nearest && distance > 1e-9) {
          nearest = distance;
          escape = delta / distance;
          if (!inside) {escape = -escape;}
        }
      }
    }
    if (inside) {
      result.clearance = 0.0;
      const double penetration = std::isfinite(nearest) ? nearest : map.resolution;
      const double deficit = influence_distance + penetration;
      result.cost = 0.5 * deficit * deficit;
      if (escape.squaredNorm() > 0.0) {
        result.gradient.head<2>() = -deficit * escape;
      }
    } else if (std::isfinite(nearest) && nearest < influence_distance) {
      result.clearance = nearest;
      const double deficit = influence_distance - nearest;
      result.cost = 0.5 * deficit * deficit;
      result.gradient.head<2>() = -deficit * escape;
    }
    return result;
  }

  Objective objective(
    const GridSnapshot & map, const std::vector<Eigen::Vector3d> & reference,
    const std::vector<Eigen::Vector3d> & controls, const bool with_gradient,
    const bool collision_only = false) const
  {
    Objective output;
    output.gradient.assign(controls.size(), Eigen::Vector3d::Zero());
    if (controls.size() < 3U) {return output;}

    if (!collision_only) {
      const double smooth_normalizer = 1.0 / static_cast<double>(controls.size() - 2U);
      for (std::size_t i = 1U; i + 1U < controls.size(); ++i) {
        const Eigen::Vector3d second =
          controls[i - 1U] - 2.0 * controls[i] + controls[i + 1U];
        output.smooth += smooth_normalizer * second.squaredNorm();
        if (with_gradient) {
          const Eigen::Vector3d gradient = 2.0 * smooth_normalizer * second;
          output.gradient[i - 1U] += config_.lambda_smooth * gradient;
          output.gradient[i] -= 2.0 * config_.lambda_smooth * gradient;
          output.gradient[i + 1U] += config_.lambda_smooth * gradient;
        }
      }
    }

    const auto reference_distance = cumulativeDistance(reference);
    const auto samples = curveSamples(controls);
    const double sample_normalizer = samples.empty() ? 1.0 :
      1.0 / static_cast<double>(samples.size());
    for (std::size_t sample_index = 0U; sample_index < samples.size(); ++sample_index) {
      const auto & sample = samples[sample_index];
      if (!collision_only) {
        const GuideProjection projection = projectToGuide(
          sample.position, reference, reference_distance);
        const Eigen::Vector3d reference_error = sample.position - projection.point;
        const double reference_distance_3d = reference_error.norm();
        if (reference_distance_3d > config_.reference_deadband) {
          const double excess = reference_distance_3d - config_.reference_deadband;
          output.reference += sample_normalizer * excess * excess;
          if (with_gradient && reference_distance_3d > 1e-9) {
            const Eigen::Vector3d point_gradient =
              config_.lambda_reference * sample_normalizer * 2.0 * excess *
              reference_error / reference_distance_3d;
            distribute(sample, point_gradient, output.gradient);
          }
        }
      }

      const Eigen::Vector2d heading = collisionHeading(samples, sample_index);
      for (const double sign : {-1.0, 1.0}) {
        Eigen::Vector3d centre = sample.position;
        centre.head<2>() += sign * config_.cylinder_offset * heading;
        const Repulsion repulsion = repulsionAt(map, centre, config_.clearance_distance);
        output.collision += sample_normalizer * repulsion.cost;
        if (with_gradient) {
          const Eigen::Vector3d point_gradient =
            config_.lambda_collision * sample_normalizer * repulsion.gradient;
          distribute(sample, point_gradient, output.gradient);
          const Eigen::Vector2d derivative = sample.derivative.head<2>();
          const double derivative_norm = derivative.norm();
          if (derivative_norm > 1e-6) {
            const Eigen::Matrix2d heading_jacobian =
              (Eigen::Matrix2d::Identity() - heading * heading.transpose()) /
              derivative_norm;
            const Eigen::Vector2d derivative_gradient =
              sign * config_.cylinder_offset * heading_jacobian * point_gradient.head<2>();
            for (std::size_t k = 0U; k < 4U; ++k) {
              output.gradient[sample.indices[k]].head<2>() +=
                sample.derivative_weights[k] * derivative_gradient;
            }
          }
        }
      }
    }
    if (!collision_only && config_.allow_goal_adjustment) {
      const Eigen::Vector3d goal_error = controls.back() - reference.back();
      output.goal = goal_error.squaredNorm();
      if (with_gradient) {
        output.gradient.back() += 2.0 * config_.lambda_goal * goal_error;
      }
    }
    output.total = config_.lambda_collision * output.collision;
    if (!collision_only) {
      output.total += config_.lambda_smooth * output.smooth +
        config_.lambda_reference * output.reference + config_.lambda_goal * output.goal;
    }
    return output;
  }

  static void distribute(
    const CurveSample & sample, const Eigen::Vector3d & point_gradient,
    std::vector<Eigen::Vector3d> & gradient)
  {
    for (std::size_t k = 0U; k < 4U; ++k) {
      gradient[sample.indices[k]] += sample.weights[k] * point_gradient;
    }
  }

  void distributeCollisionGradient(
    const CurveSample & sample, const double sign, const Eigen::Vector2d & heading,
    const Eigen::Vector3d & point_gradient,
    std::vector<Eigen::Vector3d> & gradient) const
  {
    distribute(sample, point_gradient, gradient);
    const Eigen::Vector2d derivative = sample.derivative.head<2>();
    const double derivative_norm = derivative.norm();
    if (derivative_norm <= 1e-6) {return;}
    const Eigen::Matrix2d heading_jacobian =
      (Eigen::Matrix2d::Identity() - heading * heading.transpose()) / derivative_norm;
    const Eigen::Vector2d derivative_gradient =
      sign * config_.cylinder_offset * heading_jacobian * point_gradient.head<2>();
    for (std::size_t k = 0U; k < 4U; ++k) {
      gradient[sample.indices[k]].head<2>() +=
        sample.derivative_weights[k] * derivative_gradient;
    }
  }

  void constrainToReference(
    std::vector<Eigen::Vector3d> & controls,
    const std::vector<Eigen::Vector3d> & reference) const
  {
    const auto distance = cumulativeDistance(reference);
    const std::size_t movable_end = config_.allow_goal_adjustment ?
      controls.size() : controls.size() - 1U;
    for (std::size_t i = 1U; i < movable_end; ++i) {
      const GuideProjection projection = projectToGuide(controls[i], reference, distance);
      Eigen::Vector3d error = controls[i] - projection.point;
      if (error.norm() > config_.max_deviation) {
        controls[i] = projection.point + config_.max_deviation * error.normalized();
      }
      controls[i].z() = std::clamp(
        controls[i].z(), projection.point.z() - config_.max_z_deviation,
        projection.point.z() + config_.max_z_deviation);
    }
    if (config_.allow_goal_adjustment) {
      Eigen::Vector3d goal_error = controls.back() - reference.back();
      if (goal_error.norm() > config_.goal_max_deviation) {
        controls.back() = reference.back() +
          config_.goal_max_deviation * goal_error.normalized();
      }
      controls.back().z() = std::clamp(
        controls.back().z(), reference.back().z() - config_.max_z_deviation,
        reference.back().z() + config_.max_z_deviation);
    }
  }

  ClearanceAssessment assessClearance(
    const GridSnapshot & map, const std::vector<Eigen::Vector3d> & controls,
    const bool with_gradient) const
  {
    ClearanceAssessment result;
    result.minimum = config_.clearance_distance;
    result.gradient.assign(controls.size(), Eigen::Vector3d::Zero());
    const auto samples = curveSamples(controls);
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(samples.size());
    for (const auto & sample : samples) {positions.push_back(sample.position);}
    const auto distances = cumulativeDistance(positions);
    for (std::size_t i = 0U; i < samples.size(); ++i) {
      if (distances[i] < config_.start_clearance_ignore_distance) {continue;}
      const Eigen::Vector2d heading = collisionHeading(samples, i);
      for (const double sign : {-1.0, 1.0}) {
        Eigen::Vector3d centre = samples[i].position;
        centre.head<2>() += sign * config_.cylinder_offset * heading;
        const Repulsion repulsion = repulsionAt(
          map, centre, config_.clearance_distance);
        const bool lower_clearance = repulsion.clearance < result.minimum - 1e-9;
        const bool deeper_at_same_clearance =
          std::abs(repulsion.clearance - result.minimum) <= 1e-9 &&
          repulsion.cost > result.worst_cost;
        if (!lower_clearance && !deeper_at_same_clearance) {continue;}
        result.minimum = repulsion.clearance;
        result.worst_cost = repulsion.cost;
        if (with_gradient) {
          std::fill(result.gradient.begin(), result.gradient.end(), Eigen::Vector3d::Zero());
          distributeCollisionGradient(
            samples[i], sign, heading, repulsion.gradient, result.gradient);
        }
      }
    }
    return result;
  }

  void assessPath(
    const GridSnapshot & map, const std::vector<Eigen::Vector3d> & reference,
    Result & result) const
  {
    result.minimum_clearance = config_.clearance_distance;
    result.minimum_movable_clearance = config_.clearance_distance;
    const auto reference_distance = cumulativeDistance(reference);
    const auto samples = curveSamples(result.control_points);
    std::vector<Eigen::Vector3d> sample_positions;
    sample_positions.reserve(samples.size());
    for (const auto & sample : samples) {sample_positions.push_back(sample.position);}
    const auto path_distance = cumulativeDistance(sample_positions);
    for (std::size_t i = 0U; i < samples.size(); ++i) {
      const Eigen::Vector2d heading = collisionHeading(samples, i);
      bool hard = false;
      for (const double sign : {-1.0, 1.0}) {
        Eigen::Vector3d centre = samples[i].position;
        centre.head<2>() += sign * config_.cylinder_offset * heading;
        hard = hard || map.hardAt(centre);
        result.minimum_clearance = std::min(
          result.minimum_clearance,
          repulsionAt(map, centre, config_.clearance_distance).clearance);
        if (path_distance[i] >= config_.start_clearance_ignore_distance) {
          result.minimum_movable_clearance = std::min(
            result.minimum_movable_clearance,
            repulsionAt(map, centre, config_.clearance_distance).clearance);
        }
      }
      if (hard) {++result.hard_collision_samples;}
      const GuideProjection projection = projectToGuide(
        samples[i].position, reference, reference_distance);
      result.maximum_reference_deviation = std::max(
        result.maximum_reference_deviation,
        (samples[i].position - projection.point).norm());
    }
  }

  Config config_;
};

}  // namespace efficient_3d_local_planner
