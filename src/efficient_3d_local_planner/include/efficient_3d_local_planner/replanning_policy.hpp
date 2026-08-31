#pragma once

namespace efficient_3d_local_planner
{

enum class InvalidationAction
{
  AwaitReplacement,
  Stop
};

enum class OptimizedControlAction
{
  PublishOptimized,
  RetainCurrent,
  Stop
};

inline InvalidationAction invalidationAction(
  const double invalid_age, const double replacement_grace_period)
{
  return invalid_age < replacement_grace_period ?
         InvalidationAction::AwaitReplacement : InvalidationAction::Stop;
}

inline bool retainCurrentPathAfterPlanningFailure(
  const bool has_current_path, const bool current_path_valid,
  const bool same_global_path_generation)
{
  return has_current_path && current_path_valid && same_global_path_generation;
}

inline OptimizedControlAction optimizedControlAction(
  const bool optimization_success, const bool has_current_path,
  const bool current_path_valid, const bool same_global_path_generation)
{
  if (optimization_success) {
    return OptimizedControlAction::PublishOptimized;
  }
  return retainCurrentPathAfterPlanningFailure(
    has_current_path, current_path_valid, same_global_path_generation) ?
    OptimizedControlAction::RetainCurrent : OptimizedControlAction::Stop;
}

}  // namespace efficient_3d_local_planner
