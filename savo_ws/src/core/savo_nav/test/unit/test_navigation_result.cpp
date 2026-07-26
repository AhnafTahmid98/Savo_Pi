// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <stdexcept>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "savo_nav/navigation_result.hpp"

namespace
{

using Code = savo_nav::NavigationResultCode;

TEST(NavigationResultTest, DefaultResultIsValidPendingState)
{
  const savo_nav::NavigationResult result;

  const auto validation =
    savo_nav::NavigationResultContract::Validate(result);

  EXPECT_TRUE(validation.IsValid());
  EXPECT_FALSE(result.terminal);
  EXPECT_FALSE(result.success);
}

TEST(NavigationResultTest, CreatesValidPendingResult)
{
  const auto result =
    savo_nav::NavigationResultContract::MakePending(
    "goal-1",
    "navigation_in_progress");

  const auto validation =
    savo_nav::NavigationResultContract::Validate(result);

  EXPECT_TRUE(validation.IsValid());
  EXPECT_EQ(result.code, Code::kNone);
  EXPECT_EQ(result.goal_id, "goal-1");
  EXPECT_FALSE(result.terminal);
  EXPECT_FALSE(result.success);
}

TEST(NavigationResultTest, CreatesValidSuccessResult)
{
  const auto result =
    savo_nav::NavigationResultContract::MakeTerminal(
    "goal-1",
    Code::kSucceeded,
    "goal_succeeded");

  const auto validation =
    savo_nav::NavigationResultContract::Validate(result);

  EXPECT_TRUE(validation.IsValid());
  EXPECT_TRUE(result.terminal);
  EXPECT_TRUE(result.success);
}

TEST(NavigationResultTest, CreatesValidFailureResult)
{
  const auto result =
    savo_nav::NavigationResultContract::MakeTerminal(
    "goal-1",
    Code::kNav2Aborted,
    "nav2_aborted_goal");

  const auto validation =
    savo_nav::NavigationResultContract::Validate(result);

  EXPECT_TRUE(validation.IsValid());
  EXPECT_TRUE(result.terminal);
  EXPECT_FALSE(result.success);
}

TEST(NavigationResultTest, RejectsSuccessFlagOnFailure)
{
  savo_nav::NavigationResult result;

  result.code = Code::kNav2Aborted;
  result.goal_id = "goal-1";
  result.reason = "nav2_aborted_goal";
  result.terminal = true;
  result.success = true;

  const auto validation =
    savo_nav::NavigationResultContract::Validate(result);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidResult);
}

TEST(NavigationResultTest, RejectsEmptyTerminalGoalId)
{
  auto result =
    savo_nav::NavigationResultContract::MakeTerminal(
    "",
    Code::kRejectedBusy,
    "another_goal_is_active");

  const auto validation =
    savo_nav::NavigationResultContract::Validate(result);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kEmptyIdentifier);
}

TEST(NavigationResultTest, RejectsNoneAsTerminalCode)
{
  EXPECT_THROW(
    savo_nav::NavigationResultContract::MakeTerminal(
      "goal-1",
      Code::kNone,
      "invalid"),
    std::invalid_argument);
}

TEST(NavigationResultTest, ConvertsEveryResultCode)
{
  const std::vector<std::pair<Code, std::string_view>> expected{
    {Code::kNone, "none"},
    {Code::kSucceeded, "succeeded"},
    {Code::kRejectedNotReady, "rejected_not_ready"},
    {Code::kRejectedInvalidGoal, "rejected_invalid_goal"},
    {Code::kRejectedBusy, "rejected_busy"},
    {Code::kCanceledByClient, "canceled_by_client"},
    {Code::kCanceledBySafety, "canceled_by_safety"},
    {Code::kNav2Rejected, "nav2_rejected"},
    {Code::kNav2Aborted, "nav2_aborted"},
    {Code::kNav2TimedOut, "nav2_timed_out"},
    {Code::kControlUnavailable, "control_unavailable"},
    {
      Code::kLocalizationUnavailable,
      "localization_unavailable"
    },
    {Code::kMapUnavailable, "map_unavailable"},
    {Code::kInternalError, "internal_error"}
  };

  for (const auto & item : expected) {
    EXPECT_EQ(
      savo_nav::NavigationResultContract::ToString(
        item.first),
      item.second);
  }
}

}  // namespace
