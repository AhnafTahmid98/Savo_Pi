// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "gtest/gtest.h"

#include "savo_nav/goal_admission_policy.hpp"

namespace
{

savo_nav::GoalAdmissionInput ReadyInput()
{
  savo_nav::GoalAdmissionInput input;
  input.guard_observed = true;
  input.guard_fresh = true;
  input.guard_allowed = true;
  input.guard_reason = "control_recovery_ready";
  return input;
}

TEST(GoalAdmissionPolicyTest, UnknownGuardRejectsNewGoal)
{
  const auto decision =
    savo_nav::GoalAdmissionPolicy::Evaluate({});

  EXPECT_FALSE(decision.accept_new_goal);
  EXPECT_FALSE(decision.request_active_cancel);
  EXPECT_EQ(
    decision.reason,
    "control_recovery_guard_unobserved");
}

TEST(GoalAdmissionPolicyTest, StaleGuardRejectsNewGoal)
{
  auto input = ReadyInput();
  input.guard_fresh = false;

  const auto decision =
    savo_nav::GoalAdmissionPolicy::Evaluate(input);

  EXPECT_FALSE(decision.accept_new_goal);
  EXPECT_EQ(
    decision.reason,
    "control_recovery_guard_stale");
}

TEST(GoalAdmissionPolicyTest, BlockedGuardRejectsNewGoal)
{
  auto input = ReadyInput();
  input.guard_allowed = false;
  input.guard_reason = "recovery_active";

  const auto decision =
    savo_nav::GoalAdmissionPolicy::Evaluate(input);

  EXPECT_FALSE(decision.accept_new_goal);
  EXPECT_EQ(decision.reason, "recovery_active");
}

TEST(GoalAdmissionPolicyTest, ReadyGuardAcceptsIdleGoal)
{
  const auto decision =
    savo_nav::GoalAdmissionPolicy::Evaluate(
    ReadyInput());

  EXPECT_TRUE(decision.accept_new_goal);
  EXPECT_FALSE(decision.request_active_cancel);
  EXPECT_EQ(
    decision.reason,
    "goal_admission_allowed");
}

TEST(GoalAdmissionPolicyTest, BusyGatewayRejectsReplacement)
{
  auto input = ReadyInput();
  input.active_goal = true;

  const auto decision =
    savo_nav::GoalAdmissionPolicy::Evaluate(input);

  EXPECT_FALSE(decision.accept_new_goal);
  EXPECT_FALSE(decision.request_active_cancel);
  EXPECT_EQ(decision.reason, "goal_gateway_busy");
}

TEST(GoalAdmissionPolicyTest, GuardLossCancelsActiveGoal)
{
  auto input = ReadyInput();
  input.guard_allowed = false;
  input.guard_reason = "control_mode_not_nav";
  input.active_goal = true;

  const auto decision =
    savo_nav::GoalAdmissionPolicy::Evaluate(input);

  EXPECT_FALSE(decision.accept_new_goal);
  EXPECT_TRUE(decision.request_active_cancel);
  EXPECT_EQ(decision.reason, "control_mode_not_nav");
}

TEST(GoalAdmissionPolicyTest, CancellationIsOneShot)
{
  auto input = ReadyInput();
  input.guard_allowed = false;
  input.guard_reason = "recovery_active";
  input.active_goal = true;
  input.cancellation_requested = true;

  const auto decision =
    savo_nav::GoalAdmissionPolicy::Evaluate(input);

  EXPECT_FALSE(decision.request_active_cancel);
  EXPECT_EQ(decision.reason, "recovery_active");
}

}  // namespace
