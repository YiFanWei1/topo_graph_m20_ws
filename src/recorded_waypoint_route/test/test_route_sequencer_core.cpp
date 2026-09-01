#include <gtest/gtest.h>

#include <vector>

#include "recorded_waypoint_route/route_sequencer_core.hpp"
#include "recorded_waypoint_route/planning_profile.hpp"

namespace route = recorded_waypoint_route;

TEST(RoutePlanningProfile, ResolvesSlopeAndNormalParametersInOnePlace)
{
  const route::PlanningProfileResolver resolver(
    {"normal", 0.0, true}, {"slope", 0.4, false});
  const auto normal = resolver.resolve({route::TargetType::Normal, false});
  const auto slope = resolver.resolve({route::TargetType::Corner, true});
  EXPECT_EQ(normal.name, "normal");
  EXPECT_DOUBLE_EQ(normal.path_height, 0.0);
  EXPECT_TRUE(normal.extension_enabled);
  EXPECT_EQ(slope.name, "slope");
  EXPECT_DOUBLE_EQ(slope.path_height, 0.4);
  EXPECT_FALSE(slope.extension_enabled);
}

TEST(RouteSequencerCore, NearestTargetUsesThreeDimensionalDistance)
{
  const std::vector<route::Point3> targets{
    {0.0, 0.0, 0.4}, {1.0, 0.0, 0.4}, {1.0, 1.0, 1.4}, {2.0, 1.0, 1.4}};
  const auto nearest = route::nearestTarget({1.0, 1.0, 0.3}, targets);
  EXPECT_EQ(nearest.first, 1U);
  EXPECT_NEAR(nearest.second, std::hypot(1.0, 0.1), 1e-12);
}

TEST(RouteSequencerCore, OrderedTargetsSupportForwardAndReverseTravel)
{
  EXPECT_EQ(route::orderedTargetIndices(1, 3, 4), (std::vector<std::size_t>{1, 2, 3}));
  EXPECT_EQ(route::orderedTargetIndices(3, 0, 4), (std::vector<std::size_t>{3, 2, 1, 0}));
  EXPECT_EQ(route::orderedTargetIndices(2, 2, 4), (std::vector<std::size_t>{2}));
}

TEST(RouteSequencerCore, ProgressAdvancesOnlyOneRequiredTargetAtATime)
{
  const std::vector<route::Point3> targets{
    {0.0, 0.0, 0.4}, {1.0, 0.0, 0.4}, {1.0, 1.0, 1.4}, {2.0, 1.0, 1.4}};
  route::RouteProgress progress({1, 2, 3}, 0.15);
  EXPECT_FALSE(progress.update({1.151, 0.0, 0.4}, targets).changed);

  const auto first = progress.update(targets[1], targets);
  ASSERT_TRUE(first.active_leg.has_value());
  const auto expected_first_leg = std::make_pair<std::size_t, std::size_t>(1, 2);
  EXPECT_EQ(*first.active_leg, expected_first_leg);
  EXPECT_EQ(first.required_target, 2U);

  // 即使机器人已经位于更后面的目标，也不能绕过当前必须到达的三维点。
  EXPECT_FALSE(progress.update(targets[3], targets).changed);
  EXPECT_EQ(progress.requiredTarget(), 2U);
  EXPECT_TRUE(progress.update(targets[2], targets).changed);
  const auto finish = progress.update(targets[3], targets);
  EXPECT_EQ(finish.state, route::RouteState::Complete);
  EXPECT_STREQ(finish.event, "goal_reached");
}

TEST(RouteSequencerCore, MatchingXYAtWrongHeightDoesNotAdvance)
{
  const std::vector<route::Point3> targets{
    {0.0, 0.0, 0.4}, {1.0, 0.0, 0.4}, {1.0, 1.0, 1.4}, {2.0, 1.0, 1.4}};
  route::RouteProgress progress({2, 3}, 0.15);
  const auto update = progress.update({1.0, 1.0, 1.249}, targets);
  EXPECT_FALSE(update.changed);
  EXPECT_EQ(update.required_target, 2U);
}

TEST(RouteSequencerCore, ArrivalBoundaryIsInclusive)
{
  const std::vector<route::Point3> targets{{1.0, 0.0, 0.4}};
  route::RouteProgress progress({0}, 0.15);
  EXPECT_FALSE(progress.update({1.151, 0.0, 0.4}, targets).changed);
  const auto boundary = progress.update({1.15, 0.0, 0.4}, targets);
  EXPECT_TRUE(boundary.changed);
  EXPECT_EQ(boundary.state, route::RouteState::Complete);
}

TEST(RouteSequencerCore, UsesNormalCornerAndFinalGoalTolerances)
{
  const std::vector<route::Point3> targets{
    {0.0, 0.0, 0.4}, {1.0, 0.0, 0.4}, {1.0, 1.0, 0.4}};
  const std::vector<route::TargetType> types{
    route::TargetType::Normal, route::TargetType::Corner, route::TargetType::Normal};
  route::RouteProgress progress({0, 1, 2}, types, 0.50, 0.20, 0.15);

  EXPECT_TRUE(progress.update({-0.50, 0.0, 0.4}, targets).changed);
  EXPECT_EQ(progress.requiredTargetType(), route::TargetType::Corner);
  EXPECT_DOUBLE_EQ(progress.effectiveTolerance(), 0.20);
  EXPECT_FALSE(progress.update({1.0, -0.201, 0.4}, targets).changed);
  EXPECT_TRUE(progress.update({1.0, -0.20, 0.4}, targets).changed);

  // The selected final target overrides its stored NORMAL type.
  EXPECT_TRUE(progress.requiredTargetIsGoal());
  EXPECT_DOUBLE_EQ(progress.effectiveTolerance(), 0.15);
  EXPECT_FALSE(progress.update({1.0, 0.849, 0.4}, targets).changed);
  EXPECT_EQ(
    progress.update({1.0, 0.851, 0.4}, targets).state,
    route::RouteState::Complete);
}

TEST(RouteSequencerCore, ReverseRouteUsesCornerThenFinalGoalTolerance)
{
  const std::vector<route::Point3> targets{
    {0.0, 0.0, 0.4}, {1.0, 0.0, 0.4}, {1.0, 1.0, 0.4}};
  const std::vector<route::TargetType> types{
    route::TargetType::Normal, route::TargetType::Corner, route::TargetType::Normal};
  route::RouteProgress progress({2, 1, 0}, types, 2.80, 0.20, 0.15);

  EXPECT_TRUE(progress.update(targets[2], targets).changed);
  EXPECT_EQ(progress.requiredTargetType(), route::TargetType::Corner);
  EXPECT_FALSE(progress.update({1.0, -0.201, 0.4}, targets).changed);
  EXPECT_TRUE(progress.update({1.0, -0.20, 0.4}, targets).changed);
  EXPECT_TRUE(progress.requiredTargetIsGoal());
  EXPECT_FALSE(progress.update({0.151, 0.0, 0.4}, targets).changed);
  EXPECT_EQ(
    progress.update({0.15, 0.0, 0.4}, targets).state,
    route::RouteState::Complete);
}

TEST(RouteSequencerCore, InterpolationPreservesTargetsAndSpacing)
{
  const std::vector<route::Point3> targets{
    {0.0, 0.0, 0.0}, {0.24, 0.0, 0.18}, {0.24, 0.4, 0.18}};
  const auto path = route::interpolateTargets(targets, 0.10);
  EXPECT_NEAR(route::distance3d(path.front(), targets.front()), 0.0, 1e-12);
  EXPECT_NEAR(route::distance3d(path.back(), targets.back()), 0.0, 1e-12);
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    EXPECT_LE(route::distance3d(path[index], path[index + 1]), 0.10 + 1e-12);
  }
  const auto segments = route::splitInterpolatedPath(targets, path);
  ASSERT_EQ(segments.size(), 2U);
  EXPECT_NEAR(route::distance3d(segments[0].back(), targets[1]), 0.0, 1e-12);
}
