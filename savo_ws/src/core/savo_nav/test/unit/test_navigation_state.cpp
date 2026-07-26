// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "savo_nav/navigation_state.hpp"

namespace
{

using State = savo_nav::NavigationState;

TEST(NavigationStateTest, OnlyIdleAcceptsNewGoals)
{
  EXPECT_TRUE(
    savo_nav::NavigationStateContract::AcceptsNewGoal(
      State::kIdle));

  EXPECT_FALSE(
    savo_nav::NavigationStateContract::AcceptsNewGoal(
      State::kNavigating));

  EXPECT_FALSE(
    savo_nav::NavigationStateContract::AcceptsNewGoal(
      State::kCanceling));
}

TEST(NavigationStateTest, ActiveGoalStatesAreExplicit)
{
  EXPECT_TRUE(
    savo_nav::NavigationStateContract::HasActiveGoal(
      State::kValidatingGoal));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::HasActiveGoal(
      State::kNavigating));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::HasActiveGoal(
      State::kCanceling));

  EXPECT_FALSE(
    savo_nav::NavigationStateContract::HasActiveGoal(
      State::kIdle));

  EXPECT_FALSE(
    savo_nav::NavigationStateContract::HasActiveGoal(
      State::kSucceeded));
}

TEST(NavigationStateTest, TerminalStatesAreExplicit)
{
  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTerminal(
      State::kSucceeded));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTerminal(
      State::kCanceled));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTerminal(
      State::kRejected));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTerminal(
      State::kFailed));

  EXPECT_FALSE(
    savo_nav::NavigationStateContract::IsTerminal(
      State::kNavigating));
}

TEST(NavigationStateTest, AllowsNormalNavigationSequence)
{
  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kOffline,
      State::kStarting));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kStarting,
      State::kIdle));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kIdle,
      State::kValidatingGoal));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kValidatingGoal,
      State::kWaitingForControl));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kWaitingForControl,
      State::kPlanning));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kPlanning,
      State::kNavigating));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kNavigating,
      State::kSucceeded));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kSucceeded,
      State::kIdle));
}

TEST(NavigationStateTest, AllowsRecoveryLoop)
{
  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kNavigating,
      State::kRecovering));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kRecovering,
      State::kPlanning));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kRecovering,
      State::kNavigating));
}

TEST(NavigationStateTest, AllowsCancelAcknowledgementOutcomes)
{
  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kNavigating,
      State::kCanceling));

  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kCanceling,
      State::kCanceled));

  // A rejected or late cancellation may still finish successfully.
  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kCanceling,
      State::kSucceeded));
}

TEST(NavigationStateTest, RejectsUnsafeGoalReplacement)
{
  EXPECT_FALSE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kNavigating,
      State::kValidatingGoal));

  EXPECT_FALSE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kPlanning,
      State::kValidatingGoal));
}

TEST(NavigationStateTest, AllowsIdempotentPublication)
{
  EXPECT_TRUE(
    savo_nav::NavigationStateContract::IsTransitionAllowed(
      State::kNavigating,
      State::kNavigating));
}

TEST(NavigationStateTest, ConvertsEveryState)
{
  const std::vector<std::pair<State, std::string_view>> expected{
    {State::kOffline, "offline"},
    {State::kStarting, "starting"},
    {State::kIdle, "idle"},
    {State::kValidatingGoal, "validating_goal"},
    {State::kWaitingForControl, "waiting_for_control"},
    {State::kPlanning, "planning"},
    {State::kNavigating, "navigating"},
    {State::kRecovering, "recovering"},
    {State::kCanceling, "canceling"},
    {State::kSucceeded, "succeeded"},
    {State::kCanceled, "canceled"},
    {State::kRejected, "rejected"},
    {State::kFailed, "failed"}
  };

  for (const auto & item : expected) {
    EXPECT_EQ(
      savo_nav::NavigationStateContract::ToString(
        item.first),
      item.second);
  }
}

}  // namespace
