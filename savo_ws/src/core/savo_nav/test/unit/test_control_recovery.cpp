// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <limits>

#include "gtest/gtest.h"

#include "savo_nav/control_mode_client.hpp"
#include "savo_nav/control_recovery_guard.hpp"
#include "savo_nav/recovery_bridge.hpp"

namespace
{

TEST(ControlModeClientTest, ParsesSupportedModes)
{
  using Mode = savo_nav::ObservedControlMode;

  EXPECT_EQ(
    savo_nav::ControlModeClient::Parse("STOP"),
    Mode::kStop);

  EXPECT_EQ(
    savo_nav::ControlModeClient::Parse("teleop"),
    Mode::kManual);

  EXPECT_EQ(
    savo_nav::ControlModeClient::Parse("AUTO"),
    Mode::kAuto);

  EXPECT_EQ(
    savo_nav::ControlModeClient::Parse("navigation"),
    Mode::kNav);

  EXPECT_EQ(
    savo_nav::ControlModeClient::Parse("RECOVERY"),
    Mode::kRecovery);

  EXPECT_EQ(
    savo_nav::ControlModeClient::Parse("bad-mode"),
    Mode::kUnknown);
}

TEST(ControlModeClientTest, StartsBlocked)
{
  const savo_nav::ControlModeClient client;

  const auto observation =
    client.Evaluate(1.0, 1.0);

  EXPECT_FALSE(observation.observed);
  EXPECT_FALSE(observation.navigation_allowed);

  EXPECT_EQ(
    observation.reason_code,
    "control_mode_unobserved");
}

TEST(ControlModeClientTest, NavModeAllowsNavigation)
{
  savo_nav::ControlModeClient client;

  ASSERT_TRUE(client.UpdateMode("NAV", 10.0));

  const auto observation =
    client.Evaluate(10.5, 1.0);

  EXPECT_TRUE(observation.fresh);
  EXPECT_TRUE(observation.navigation_allowed);
  EXPECT_FALSE(observation.cancel_active_goal);

  EXPECT_EQ(
    observation.reason_code,
    "control_mode_nav");
}

TEST(ControlModeClientTest, StopModeBlocksNavigation)
{
  savo_nav::ControlModeClient client;

  ASSERT_TRUE(client.UpdateMode("STOP", 10.0));

  const auto observation =
    client.Evaluate(10.1, 1.0);

  EXPECT_FALSE(observation.navigation_allowed);
  EXPECT_TRUE(observation.cancel_active_goal);

  EXPECT_EQ(
    observation.reason_code,
    "control_mode_not_nav");
}

TEST(ControlModeClientTest, StaleModeBlocksNavigation)
{
  savo_nav::ControlModeClient client;

  ASSERT_TRUE(client.UpdateMode("NAV", 10.0));

  const auto observation =
    client.Evaluate(12.0, 1.0);

  EXPECT_FALSE(observation.fresh);
  EXPECT_FALSE(observation.navigation_allowed);

  EXPECT_EQ(
    observation.reason_code,
    "control_mode_stale");
}

TEST(ControlModeClientTest, RejectsInvalidTimestamp)
{
  savo_nav::ControlModeClient client;

  EXPECT_FALSE(
    client.UpdateMode(
      "NAV",
      std::numeric_limits<double>::infinity()));
}

TEST(RecoveryBridgeTest, StartsBlocked)
{
  const savo_nav::RecoveryBridge bridge;

  const auto observation =
    bridge.Evaluate(1.0, 1.0);

  EXPECT_FALSE(observation.observed);
  EXPECT_FALSE(observation.navigation_allowed);

  EXPECT_EQ(
    observation.reason_code,
    "recovery_state_unobserved");
}

TEST(RecoveryBridgeTest, FreshInactiveRecoveryAllowsNavigation)
{
  savo_nav::RecoveryBridge bridge;

  ASSERT_TRUE(
    bridge.UpdateActive(false, 10.0));

  const auto observation =
    bridge.Evaluate(10.5, 1.0);

  EXPECT_TRUE(observation.fresh);
  EXPECT_FALSE(observation.active);
  EXPECT_TRUE(observation.navigation_allowed);
  EXPECT_FALSE(observation.cancel_active_goal);

  EXPECT_EQ(
    observation.reason_code,
    "recovery_clear");
}

TEST(RecoveryBridgeTest, ActiveRecoveryBlocksNavigation)
{
  savo_nav::RecoveryBridge bridge;

  ASSERT_TRUE(
    bridge.UpdateActive(true, 10.0));

  const auto observation =
    bridge.Evaluate(10.1, 1.0);

  EXPECT_TRUE(observation.active);
  EXPECT_FALSE(observation.navigation_allowed);
  EXPECT_TRUE(observation.cancel_active_goal);

  EXPECT_EQ(
    observation.reason_code,
    "recovery_active");
}

TEST(RecoveryBridgeTest, StaleRecoveryBlocksNavigation)
{
  savo_nav::RecoveryBridge bridge;

  ASSERT_TRUE(
    bridge.UpdateActive(false, 10.0));

  const auto observation =
    bridge.Evaluate(12.0, 1.0);

  EXPECT_FALSE(observation.fresh);
  EXPECT_FALSE(observation.navigation_allowed);

  EXPECT_EQ(
    observation.reason_code,
    "recovery_state_stale");
}

TEST(ControlRecoveryGuardTest, RequiresBothInputs)
{
  savo_nav::ControlModeClient control;
  savo_nav::RecoveryBridge recovery;

  ASSERT_TRUE(control.UpdateMode("NAV", 10.0));
  ASSERT_TRUE(recovery.UpdateActive(false, 10.0));

  const auto decision =
    savo_nav::ControlRecoveryGuard::Evaluate(
    control.Evaluate(10.1, 1.0),
    recovery.Evaluate(10.1, 1.0));

  EXPECT_TRUE(decision.navigation_allowed);
  EXPECT_FALSE(decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "control_recovery_ready");
}

TEST(ControlRecoveryGuardTest, ControlBlockHasPriority)
{
  savo_nav::ControlModeClient control;
  savo_nav::RecoveryBridge recovery;

  ASSERT_TRUE(control.UpdateMode("STOP", 10.0));
  ASSERT_TRUE(recovery.UpdateActive(true, 10.0));

  const auto decision =
    savo_nav::ControlRecoveryGuard::Evaluate(
    control.Evaluate(10.1, 1.0),
    recovery.Evaluate(10.1, 1.0));

  EXPECT_FALSE(decision.navigation_allowed);
  EXPECT_TRUE(decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "control_mode_not_nav");
}

TEST(ControlRecoveryGuardTest, RecoveryBlocksNavMode)
{
  savo_nav::ControlModeClient control;
  savo_nav::RecoveryBridge recovery;

  ASSERT_TRUE(control.UpdateMode("NAV", 10.0));
  ASSERT_TRUE(recovery.UpdateActive(true, 10.0));

  const auto decision =
    savo_nav::ControlRecoveryGuard::Evaluate(
    control.Evaluate(10.1, 1.0),
    recovery.Evaluate(10.1, 1.0));

  EXPECT_FALSE(decision.navigation_allowed);
  EXPECT_TRUE(decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "recovery_active");
}

}  // namespace
