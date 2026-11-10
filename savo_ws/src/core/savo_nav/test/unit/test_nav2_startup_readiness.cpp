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

TEST(Nav2StartupReadiness, UnconfirmedLifecycleFailsClosed)
{
  savo_nav::Nav2StartupEvidence evidence;
  evidence.unconfirmed_lifecycle_nodes = {"bt_navigator"};
  evidence.navigate_to_pose_action_ready = true;
  evidence.follow_waypoints_action_ready = true;
  evidence.odom_to_base_footprint_tf_ready = true;
  const auto decision = savo_nav::EvaluateNav2Startup(evidence);
  EXPECT_FALSE(decision.startup_ready);
  EXPECT_EQ(decision.reason, "nav2_lifecycle_states_unconfirmed");
}

TEST(Nav2StartupReadiness, PendingRequestPreservesRecentActiveObservation)
{
  using Clock = savo_nav::LifecycleObservation::Clock;
  using namespace std::chrono_literals;
  const auto start = Clock::time_point{} + 10s;
  savo_nav::LifecycleObservation observation;
  observation.RequestStarted(start);
  observation.ResponseReceived(true, start + 100ms);
  observation.RequestStarted(start + 500ms);
  EXPECT_TRUE(observation.request_pending());
  EXPECT_EQ(observation.StateAt(start + 800ms, 2s),
    savo_nav::LifecycleObservationState::kActive);
  EXPECT_FALSE(observation.RequestTimedOut(start + 800ms, 1s));
}

TEST(Nav2StartupReadiness, TimedOutRequestCanBeCancelledAndRetried)
{
  using Clock = savo_nav::LifecycleObservation::Clock;
  using namespace std::chrono_literals;
  const auto start = Clock::time_point{} + 10s;
  savo_nav::LifecycleObservation observation;
  observation.RequestStarted(start);
  EXPECT_TRUE(observation.RequestTimedOut(start + 1100ms, 1s));
  observation.RequestCancelled();
  EXPECT_FALSE(observation.request_pending());
  observation.RequestStarted(start + 1200ms);
  observation.ResponseReceived(true, start + 1250ms);
  EXPECT_EQ(observation.StateAt(start + 1300ms, 2s),
    savo_nav::LifecycleObservationState::kActive);
}

TEST(Nav2StartupReadiness, StaleActiveConfirmationFailsClosed)
{
  using Clock = savo_nav::LifecycleObservation::Clock;
  using namespace std::chrono_literals;
  const auto start = Clock::time_point{} + 10s;
  savo_nav::LifecycleObservation observation;
  observation.RequestStarted(start);
  observation.ResponseReceived(true, start + 50ms);
  observation.RequestStarted(start + 100ms);
  EXPECT_EQ(observation.StateAt(start + 2100ms, 2s),
    savo_nav::LifecycleObservationState::kStale);
}

TEST(Nav2StartupReadiness, ExplicitInactiveOverridesPreviousActive)
{
  using Clock = savo_nav::LifecycleObservation::Clock;
  using namespace std::chrono_literals;
  const auto start = Clock::time_point{} + 10s;
  savo_nav::LifecycleObservation observation;
  observation.RequestStarted(start);
  observation.ResponseReceived(true, start + 50ms);
  observation.RequestStarted(start + 100ms);
  observation.ResponseReceived(false, start + 150ms);
  EXPECT_EQ(observation.StateAt(start + 200ms, 2s),
    savo_nav::LifecycleObservationState::kInactive);
}
