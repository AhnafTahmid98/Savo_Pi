// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "gtest/gtest.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "savo_nav/coverage_action_adapter.hpp"

namespace
{

using Action = savo_msgs::action::ExecuteCoveragePath;

savo_nav::MapContext MapContext()
{
  savo_nav::MapContext context;

  context.mode =
    savo_nav::NavigationMapMode::kSavedMap;

  context.authority =
    savo_nav::MapToOdomAuthority::kAmcl;

  context.map_id = "campus-main";
  context.frame_id = "map";
  context.revision = 1;
  context.available = true;
  context.localization_ready = true;

  return context;
}

savo_nav::NavigationReadinessResult Readiness()
{
  savo_nav::NavigationReadinessResult readiness;

  readiness.state =
    savo_nav::NavigationReadinessState::kReady;

  readiness.goal_acceptance_allowed = true;
  readiness.reason = "ready";

  return readiness;
}

Action::Goal ValidGoal()
{
  Action::Goal goal;

  goal.contract_version =
    Action::Goal::CONTRACT_VERSION;

  goal.mission_id = "coverage-1";
  goal.path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped first;
  first.header.frame_id = "map";
  first.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped second = first;
  second.pose.position.x = 3.0;
  second.pose.position.y = 4.0;

  goal.path.poses = {first, second};

  return goal;
}

TEST(CoverageActionAdapterTest, AdaptsValidGoal)
{
  const auto adapted =
    savo_nav::CoverageActionAdapter::AdaptGoal(
    ValidGoal(),
    MapContext(),
    Readiness(),
    1);

  EXPECT_TRUE(adapted.validation.IsValid());
  EXPECT_EQ(
    adapted.context.source,
    savo_nav::GoalSource::kCoverage);

  EXPECT_EQ(
    adapted.validation_request.points.size(),
    2U);
}

TEST(CoverageActionAdapterTest, UsesDefaultTimeout)
{
  const auto adapted =
    savo_nav::CoverageActionAdapter::AdaptGoal(
    ValidGoal(),
    MapContext(),
    Readiness(),
    1);

  EXPECT_DOUBLE_EQ(
    adapted.resolved_execution_timeout_seconds,
    300.0);
}

TEST(CoverageActionAdapterTest, CapsTimeout)
{
  auto goal = ValidGoal();
  goal.execution_timeout.sec = 5000;

  const auto adapted =
    savo_nav::CoverageActionAdapter::AdaptGoal(
    goal,
    MapContext(),
    Readiness(),
    1);

  EXPECT_TRUE(adapted.validation.IsValid());

  EXPECT_DOUBLE_EQ(
    adapted.resolved_execution_timeout_seconds,
    3600.0);
}

TEST(CoverageActionAdapterTest, RejectsNegativeTimeout)
{
  auto goal = ValidGoal();
  goal.execution_timeout.sec = -1;

  const auto adapted =
    savo_nav::CoverageActionAdapter::AdaptGoal(
    goal,
    MapContext(),
    Readiness(),
    1);

  EXPECT_FALSE(adapted.validation.IsValid());
}

TEST(CoverageActionAdapterTest, RejectsInvalidPath)
{
  auto goal = ValidGoal();
  goal.path.header.frame_id = "odom";

  const auto adapted =
    savo_nav::CoverageActionAdapter::AdaptGoal(
    goal,
    MapContext(),
    Readiness(),
    1);

  EXPECT_FALSE(adapted.validation.IsValid());
}

TEST(CoverageActionAdapterTest, MakesExecutingFeedback)
{
  savo_nav::CoverageExecutionSnapshot execution;
  execution.state =
    savo_nav::CoverageExecutionState::kExecuting;

  execution.reason = "executing";

  savo_nav::CoverageProgressSnapshot progress;
  progress.current_waypoint = 2;
  progress.completed_waypoints = 2;
  progress.total_waypoints = 5;
  progress.completion_ratio = 0.4;
  progress.remaining_distance_m = 6.0;
  progress.elapsed_seconds = 4.0;
  progress.estimated_remaining_seconds = 6.0;

  const auto feedback =
    savo_nav::CoverageActionAdapter::MakeFeedback(
    execution,
    progress);

  EXPECT_EQ(
    feedback.state,
    Action::Feedback::STATE_EXECUTING);

  EXPECT_DOUBLE_EQ(feedback.completion_ratio, 0.4);
}

TEST(CoverageActionAdapterTest, MakesSuccessResult)
{
  savo_nav::CoverageExecutionSnapshot execution;

  execution.state =
    savo_nav::CoverageExecutionState::kSucceeded;

  execution.terminal_code =
    savo_nav::CoverageTerminalCode::kSucceeded;

  execution.reason = "coverage_succeeded";

  savo_nav::CoverageProgressSnapshot progress;
  progress.completed_waypoints = 5;
  progress.total_waypoints = 5;
  progress.completion_ratio = 1.0;

  const auto result =
    savo_nav::CoverageActionAdapter::MakeResult(
    execution,
    progress);

  EXPECT_TRUE(result.success);

  EXPECT_EQ(
    result.result_code,
    Action::Result::RESULT_SUCCEEDED);
}

TEST(CoverageActionAdapterTest, MapsTimeoutResult)
{
  savo_nav::CoverageExecutionSnapshot execution;

  execution.state =
    savo_nav::CoverageExecutionState::kTimedOut;

  execution.terminal_code =
    savo_nav::CoverageTerminalCode::kTimedOut;

  execution.reason = "execution_timeout";

  const auto result =
    savo_nav::CoverageActionAdapter::MakeResult(
    execution,
    {});

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.result_code,
    Action::Result::RESULT_TIMED_OUT);
}

}  // namespace
