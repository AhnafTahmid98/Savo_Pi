// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "savo_nav/navigation_readiness.hpp"

namespace
{

using State = savo_nav::NavigationReadinessState;

TEST(NavigationReadinessTest, StartsInStartingState)
{
  const savo_nav::NavigationReadiness readiness;
  const auto & result = readiness.GetResult();

  EXPECT_EQ(result.state, State::kStarting);
  EXPECT_FALSE(result.goal_acceptance_allowed);
  EXPECT_EQ(result.reason, "initializing");
  EXPECT_TRUE(result.failed_dependencies.empty());
}

TEST(NavigationReadinessTest, ConvertsEveryStateToStableString)
{
  const std::vector<std::pair<State, std::string>> expected{
    {State::kOffline, "offline"},
    {State::kStarting, "starting"},
    {State::kWaitingForMap, "waiting_for_map"},
    {State::kWaitingForTf, "waiting_for_tf"},
    {
      State::kWaitingForLocalization,
      "waiting_for_localization"
    },
    {State::kWaitingForLidar, "waiting_for_lidar"},
    {
      State::kWaitingForPointCloud,
      "waiting_for_pointcloud"
    },
    {State::kWaitingForNav2, "waiting_for_nav2"},
    {
      State::kWaitingForCostmaps,
      "waiting_for_costmaps"
    },
    {State::kReady, "ready"},
    {State::kDegraded, "degraded"},
    {State::kBlocked, "blocked"},
    {State::kFault, "fault"}
  };

  for (const auto & item : expected) {
    EXPECT_EQ(
      savo_nav::NavigationReadiness::ToString(item.first),
      item.second);
  }
}

TEST(NavigationReadinessTest, AcceptsValidReadyState)
{
  savo_nav::NavigationReadiness readiness;

  ASSERT_TRUE(
    readiness.Update(
      State::kReady,
      true,
      "all_required_dependencies_ready",
      {}));

  const auto & result = readiness.GetResult();

  EXPECT_EQ(result.state, State::kReady);
  EXPECT_TRUE(result.goal_acceptance_allowed);
  EXPECT_EQ(
    result.reason,
    "all_required_dependencies_ready");
  EXPECT_TRUE(result.failed_dependencies.empty());
}

TEST(NavigationReadinessTest, RejectsReadyStateWithFailures)
{
  savo_nav::NavigationReadiness readiness;

  ASSERT_FALSE(
    readiness.Update(
      State::kReady,
      true,
      "localization_failed",
      {"localization"}));

  EXPECT_EQ(
    readiness.GetResult().state,
    State::kStarting);
}

TEST(NavigationReadinessTest, RejectsBlockedStateThatAllowsGoals)
{
  savo_nav::NavigationReadiness readiness;

  ASSERT_FALSE(
    readiness.Update(
      State::kBlocked,
      true,
      "safety_stop_active",
      {"safety"}));

  EXPECT_EQ(
    readiness.GetResult().state,
    State::kStarting);
}

TEST(NavigationReadinessTest, AcceptsExplicitDegradedPolicy)
{
  savo_nav::NavigationReadiness readiness;

  ASSERT_TRUE(
    readiness.Update(
      State::kDegraded,
      true,
      "optional_pointcloud_unavailable",
      {"pointcloud"}));

  const auto & result = readiness.GetResult();

  EXPECT_EQ(result.state, State::kDegraded);
  EXPECT_TRUE(result.goal_acceptance_allowed);
  EXPECT_EQ(result.failed_dependencies.size(), 1U);
}

TEST(NavigationReadinessTest, RejectsEmptyReason)
{
  savo_nav::NavigationReadiness readiness;

  ASSERT_FALSE(
    readiness.Update(
      State::kWaitingForMap,
      false,
      "",
      {"map"}));

  EXPECT_EQ(
    readiness.GetResult().state,
    State::kStarting);
}

}  // namespace
