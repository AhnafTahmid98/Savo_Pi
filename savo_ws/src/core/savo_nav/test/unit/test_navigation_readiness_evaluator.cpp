// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "gtest/gtest.h"

#include "savo_nav/navigation_readiness.hpp"

namespace
{

using State = savo_nav::NavigationReadinessState;

savo_nav::NavigationDependencySnapshot MakeReadySnapshot()
{
  savo_nav::NavigationDependencySnapshot snapshot;

  snapshot.map_available = true;

  snapshot.map_to_odom_fresh = true;
  snapshot.odom_to_base_fresh = true;

  snapshot.base_footprint_to_base_link_available =
    true;

  snapshot.localization_fresh = true;
  snapshot.lidar_fresh = true;
  snapshot.pointcloud_fresh = true;

  snapshot.nav2_action_server_available = true;
  snapshot.global_costmap_fresh = true;
  snapshot.local_costmap_fresh = true;

  snapshot.control_state_fresh = true;
  snapshot.control_allows_navigation = true;

  snapshot.safety_state_fresh = true;
  snapshot.safety_stop_active = false;
  snapshot.slowdown_valid = true;
  snapshot.slowdown_factor = 1.0;

  return snapshot;
}

TEST(NavigationReadinessEvaluatorTest, AcceptsReadySnapshot)
{
  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    MakeReadySnapshot(),
    {});

  EXPECT_EQ(result.state, State::kReady);
  EXPECT_TRUE(result.goal_acceptance_allowed);
  EXPECT_TRUE(result.failed_dependencies.empty());
}

TEST(NavigationReadinessEvaluatorTest, BlocksMissingSafetyState)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.safety_state_fresh = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(result.state, State::kBlocked);
  EXPECT_EQ(result.reason, "safety_state_unavailable");
}

TEST(NavigationReadinessEvaluatorTest, BlocksSafetyStop)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.safety_stop_active = true;
  snapshot.slowdown_factor = 0.0;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(result.state, State::kBlocked);
  EXPECT_EQ(result.reason, "safety_stop_active");
}

TEST(NavigationReadinessEvaluatorTest, BlocksInvalidSlowdown)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.slowdown_valid = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(result.state, State::kBlocked);
  EXPECT_EQ(result.reason, "safety_slowdown_invalid");
}

TEST(NavigationReadinessEvaluatorTest, BlocksControlMode)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.control_allows_navigation = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(result.state, State::kBlocked);
  EXPECT_EQ(result.reason, "control_mode_not_navigation");
}

TEST(NavigationReadinessEvaluatorTest, BlocksMissingMapContextHeartbeat)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.map_context_synchronized = true;

  savo_nav::NavigationReadinessPolicy policy;
  policy.require_map_context_sync = true;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    policy);

  EXPECT_EQ(result.state, State::kBlocked);
  EXPECT_EQ(result.reason, "map_context_sync_unavailable");
}

TEST(NavigationReadinessEvaluatorTest, BlocksMismatchedMapContext)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.map_context_heartbeat_fresh = true;
  snapshot.map_context_synchronized = false;

  savo_nav::NavigationReadinessPolicy policy;
  policy.require_map_context_sync = true;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    policy);

  EXPECT_EQ(result.state, State::kBlocked);
  EXPECT_EQ(result.reason, "map_context_not_synchronized");
}

TEST(NavigationReadinessEvaluatorTest, AcceptsSynchronizedMapContext)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.map_context_heartbeat_fresh = true;
  snapshot.map_context_synchronized = true;

  savo_nav::NavigationReadinessPolicy policy;
  policy.require_map_context_sync = true;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    policy);

  EXPECT_EQ(result.state, State::kReady);
  EXPECT_TRUE(result.goal_acceptance_allowed);
}

TEST(NavigationReadinessEvaluatorTest, WaitsForMap)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.map_available = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(result.state, State::kWaitingForMap);
}

TEST(NavigationReadinessEvaluatorTest, WaitsForTfChain)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.map_to_odom_fresh = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(result.state, State::kWaitingForTf);
}

TEST(NavigationReadinessEvaluatorTest, WaitsForLocalization)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.localization_fresh = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(
    result.state,
    State::kWaitingForLocalization);
}

TEST(NavigationReadinessEvaluatorTest, WaitsForLidar)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.lidar_fresh = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(result.state, State::kWaitingForLidar);
}

TEST(NavigationReadinessEvaluatorTest, WaitsForPointCloud)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.pointcloud_fresh = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(
    result.state,
    State::kWaitingForPointCloud);
}

TEST(NavigationReadinessEvaluatorTest, AllowsOptionalPointCloud)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.pointcloud_fresh = false;

  savo_nav::NavigationReadinessPolicy policy;
  policy.require_pointcloud = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    policy);

  EXPECT_EQ(result.state, State::kReady);
  EXPECT_TRUE(result.goal_acceptance_allowed);
}

TEST(NavigationReadinessEvaluatorTest, WaitsForNav2)
{
  auto snapshot = MakeReadySnapshot();

  snapshot.nav2_action_server_available = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(result.state, State::kWaitingForNav2);
}

TEST(NavigationReadinessEvaluatorTest, WaitsForCostmaps)
{
  auto snapshot = MakeReadySnapshot();
  snapshot.local_costmap_fresh = false;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(
    result.state,
    State::kWaitingForCostmaps);
}

TEST(NavigationReadinessEvaluatorTest, ReportsAllFailures)
{
  savo_nav::NavigationDependencySnapshot snapshot;

  const auto result =
    savo_nav::NavigationReadiness::Evaluate(
    snapshot,
    {});

  EXPECT_EQ(result.state, State::kBlocked);

  EXPECT_GT(
    result.failed_dependencies.size(),
    5U);
}

}  // namespace
