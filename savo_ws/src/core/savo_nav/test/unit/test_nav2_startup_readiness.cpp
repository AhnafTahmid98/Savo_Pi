// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_nav/nav2_startup_readiness.hpp"

TEST(Nav2StartupReadiness, MapIsNotAStartupDependency)
{
  savo_nav::Nav2StartupEvidence evidence;
  evidence.navigate_to_pose_action_ready = true;
  evidence.follow_waypoints_action_ready = true;
  evidence.odom_to_base_footprint_tf_ready = true;

  const auto decision = savo_nav::EvaluateNav2Startup(evidence);

  EXPECT_TRUE(decision.process_started);
  EXPECT_TRUE(decision.startup_ready);
  EXPECT_EQ(decision.quality, savo_nav::StartupQuality::kMinimum);
}

TEST(Nav2StartupReadiness, LifecycleActionAndTfEvidenceFailClosed)
{
  savo_nav::Nav2StartupEvidence evidence;
  evidence.inactive_lifecycle_nodes = {"planner_server"};
  auto decision = savo_nav::EvaluateNav2Startup(evidence);
  EXPECT_FALSE(decision.startup_ready);
  EXPECT_EQ(decision.reason, "nav2_lifecycle_nodes_not_active");

  evidence.inactive_lifecycle_nodes.clear();
  evidence.navigate_to_pose_action_ready = true;
  decision = savo_nav::EvaluateNav2Startup(evidence);
  EXPECT_FALSE(decision.startup_ready);
  EXPECT_EQ(decision.reason, "follow_waypoints_action_unavailable");

  evidence.follow_waypoints_action_ready = true;
  decision = savo_nav::EvaluateNav2Startup(evidence);
  EXPECT_FALSE(decision.startup_ready);
  EXPECT_EQ(decision.reason, "odom_to_base_footprint_tf_unavailable");
}
