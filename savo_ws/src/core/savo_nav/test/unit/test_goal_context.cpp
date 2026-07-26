// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "savo_nav/goal_context.hpp"

namespace
{

using Source = savo_nav::GoalSource;

savo_nav::GoalContext MakeValidGoal()
{
  savo_nav::GoalContext context;

  context.goal_id = "frontier-42";
  context.source = Source::kExploration;
  context.target_frame = "map";
  context.map_id = "campus-main";
  context.sequence = 42;

  return context;
}

TEST(GoalContextTest, AcceptsValidGoal)
{
  const auto validation =
    savo_nav::GoalContextContract::Validate(
    MakeValidGoal());

  EXPECT_TRUE(validation.IsValid());
}

TEST(GoalContextTest, RejectsEmptyGoalId)
{
  auto context = MakeValidGoal();
  context.goal_id.clear();

  const auto validation =
    savo_nav::GoalContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kEmptyIdentifier);
}

TEST(GoalContextTest, RejectsWhitespaceInGoalId)
{
  auto context = MakeValidGoal();
  context.goal_id = "frontier 42";

  const auto validation =
    savo_nav::GoalContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidIdentifier);
}

TEST(GoalContextTest, RejectsUnknownSource)
{
  auto context = MakeValidGoal();
  context.source = Source::kUnknown;

  const auto validation =
    savo_nav::GoalContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidState);
}

TEST(GoalContextTest, RejectsNonMapTargetFrame)
{
  auto context = MakeValidGoal();
  context.target_frame = "odom";

  const auto validation =
    savo_nav::GoalContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidFrame);
}

TEST(GoalContextTest, RejectsZeroSequence)
{
  auto context = MakeValidGoal();
  context.sequence = 0;

  const auto validation =
    savo_nav::GoalContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidCombination);
}

TEST(GoalContextTest, ConvertsEveryGoalSource)
{
  const std::vector<std::pair<Source, std::string_view>> expected{
    {Source::kUnknown, "unknown"},
    {Source::kNavigation, "navigation"},
    {Source::kExploration, "exploration"},
    {Source::kWaypoint, "waypoint"},
    {Source::kArea, "area"},
    {Source::kSemantic, "semantic"},
    {Source::kOperator, "operator"},
    {Source::kSupervisor, "supervisor"}
  };

  for (const auto & item : expected) {
    EXPECT_EQ(
      savo_nav::GoalContextContract::ToString(item.first),
      item.second);
  }
}

}  // namespace
