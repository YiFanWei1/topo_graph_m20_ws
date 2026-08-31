#include "efficient_3d_local_planner/replanning_policy.hpp"

#include <gtest/gtest.h>

using efficient_3d_local_planner::InvalidationAction;
using efficient_3d_local_planner::OptimizedControlAction;
using efficient_3d_local_planner::invalidationAction;
using efficient_3d_local_planner::optimizedControlAction;
using efficient_3d_local_planner::retainCurrentPathAfterPlanningFailure;

TEST(ReplanningPolicy, WaitsBrieflyForAtomicPathReplacement)
{
  EXPECT_EQ(invalidationAction(0.0, 0.25), InvalidationAction::AwaitReplacement);
  EXPECT_EQ(invalidationAction(0.24, 0.25), InvalidationAction::AwaitReplacement);
  EXPECT_EQ(invalidationAction(0.25, 0.25), InvalidationAction::Stop);
}

TEST(ReplanningPolicy, FailedRefreshRetainsOnlyAStillValidPath)
{
  EXPECT_TRUE(retainCurrentPathAfterPlanningFailure(true, true, true));
  EXPECT_FALSE(retainCurrentPathAfterPlanningFailure(true, false, true));
  EXPECT_FALSE(retainCurrentPathAfterPlanningFailure(false, false, true));
}

TEST(ReplanningPolicy, NeverRetainsPathFromPreviousGlobalRoute)
{
  EXPECT_FALSE(retainCurrentPathAfterPlanningFailure(true, true, false));
}

TEST(ReplanningPolicy, PublishesOnlySuccessfulOptimizedPathForControl)
{
  EXPECT_EQ(
    optimizedControlAction(true, false, false, false),
    OptimizedControlAction::PublishOptimized);
}

TEST(ReplanningPolicy, FailedOptimizationRetainsOnlySafeCurrentOptimizedPath)
{
  EXPECT_EQ(
    optimizedControlAction(false, true, true, true),
    OptimizedControlAction::RetainCurrent);
  EXPECT_EQ(
    optimizedControlAction(false, true, false, true), OptimizedControlAction::Stop);
  EXPECT_EQ(
    optimizedControlAction(false, true, true, false), OptimizedControlAction::Stop);
  EXPECT_EQ(
    optimizedControlAction(false, false, false, true), OptimizedControlAction::Stop);
}
