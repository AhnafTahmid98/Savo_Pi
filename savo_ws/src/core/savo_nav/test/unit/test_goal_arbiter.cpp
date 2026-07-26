// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <stdexcept>

#include "gtest/gtest.h"

#include "savo_nav/goal_arbiter.hpp"

namespace
{

using Code = savo_nav::GoalArbitrationCode;
using Source = savo_nav::GoalSource;

savo_nav::GoalContext MakeGoal(
  const std::string & goal_id,
  const Source source,
  const std::uint64_t sequence)
{
  savo_nav::GoalContext context;

  context.goal_id = goal_id;
  context.source = source;
  context.target_frame = "map";
  context.map_id = "campus-main";
  context.sequence = sequence;

  return context;
}

savo_nav::ValidationResult Valid()
{
  return {};
}

TEST(GoalArbiterTest, RejectsZeroHistoryCapacity)
{
  EXPECT_THROW(
    savo_nav::GoalArbiter(0),
    std::invalid_argument);
}

TEST(GoalArbiterTest, AcceptsFirstValidatedGoal)
{
  savo_nav::GoalArbiter arbiter;

  const auto decision = arbiter.TryAcquire(
    MakeGoal(
      "goal-1",
      Source::kNavigation,
      1),
    Valid());

  EXPECT_EQ(decision.code, Code::kAccepted);
  EXPECT_TRUE(decision.accepted);
  EXPECT_TRUE(decision.state_changed);
  EXPECT_TRUE(arbiter.HasActiveGoal());

  ASSERT_TRUE(arbiter.ActiveGoal().has_value());

  EXPECT_EQ(
    arbiter.ActiveGoal()->goal_id,
    "goal-1");
}

TEST(GoalArbiterTest, RejectsInvalidGoal)
{
  savo_nav::GoalArbiter arbiter;

  const savo_nav::ValidationResult invalid{
    savo_nav::ValidationCode::kNotReady,
    "navigation_not_ready"
  };

  const auto decision = arbiter.TryAcquire(
    MakeGoal(
      "goal-1",
      Source::kNavigation,
      1),
    invalid);

  EXPECT_EQ(decision.code, Code::kInvalidGoal);
  EXPECT_FALSE(decision.accepted);
  EXPECT_FALSE(arbiter.HasActiveGoal());
}

TEST(GoalArbiterTest, RejectsDuplicateActiveGoal)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  const auto duplicate = arbiter.TryAcquire(
    MakeGoal(
      "goal-1",
      Source::kNavigation,
      2),
    Valid());

  EXPECT_EQ(
    duplicate.code,
    Code::kDuplicateGoal);

  EXPECT_EQ(
    arbiter.ActiveGoal()->sequence,
    1U);
}

TEST(GoalArbiterTest, RejectsBusyWithoutReplacement)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "exploration-1",
        Source::kExploration,
        1),
      Valid()).accepted);

  const auto busy = arbiter.TryAcquire(
    MakeGoal(
      "navigation-1",
      Source::kNavigation,
      1),
    Valid());

  EXPECT_EQ(busy.code, Code::kBusy);

  EXPECT_EQ(
    busy.reason,
    "active_goal_preemption_forbidden");

  ASSERT_TRUE(arbiter.ActiveGoal().has_value());

  EXPECT_EQ(
    arbiter.ActiveGoal()->goal_id,
    "exploration-1");
}

TEST(GoalArbiterTest, SupervisorCannotSilentlyPreempt)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "navigation-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  const auto supervisor = arbiter.TryAcquire(
    MakeGoal(
      "supervisor-1",
      Source::kSupervisor,
      1),
    Valid());

  EXPECT_EQ(supervisor.code, Code::kBusy);

  EXPECT_EQ(
    supervisor.reason,
    "active_goal_preemption_forbidden");

  EXPECT_EQ(
    arbiter.ActiveGoal()->goal_id,
    "navigation-1");
}

TEST(GoalArbiterTest, RejectsStaleSequencePerSource)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-2",
        Source::kNavigation,
        2),
      Valid()).accepted);

  ASSERT_TRUE(
    arbiter.Complete(
      "goal-2",
      2).accepted);

  const auto stale = arbiter.TryAcquire(
    MakeGoal(
      "goal-1",
      Source::kNavigation,
      1),
    Valid());

  EXPECT_EQ(
    stale.code,
    Code::kStaleSequence);
}

TEST(GoalArbiterTest, SequenceTrackingIsPerSource)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "navigation-5",
        Source::kNavigation,
        5),
      Valid()).accepted);

  ASSERT_TRUE(
    arbiter.Complete(
      "navigation-5",
      5).accepted);

  const auto exploration = arbiter.TryAcquire(
    MakeGoal(
      "exploration-1",
      Source::kExploration,
      1),
    Valid());

  EXPECT_TRUE(exploration.accepted);
}

TEST(GoalArbiterTest, CancelRequestRetainsOwnership)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  const auto cancel =
    arbiter.RequestCancel("goal-1", 1);

  EXPECT_EQ(
    cancel.code,
    Code::kCancelRequested);

  EXPECT_TRUE(cancel.accepted);
  EXPECT_TRUE(arbiter.HasActiveGoal());
  EXPECT_TRUE(arbiter.CancelRequested());
}

TEST(GoalArbiterTest, DuplicateCancelIsRejected)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  ASSERT_TRUE(
    arbiter.RequestCancel(
      "goal-1",
      1).accepted);

  const auto duplicate =
    arbiter.RequestCancel("goal-1", 1);

  EXPECT_EQ(
    duplicate.code,
    Code::kCancelAlreadyRequested);

  EXPECT_TRUE(arbiter.HasActiveGoal());
}

TEST(GoalArbiterTest, CancelMismatchDoesNotReleaseGoal)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  const auto mismatch =
    arbiter.RequestCancel("goal-2", 1);

  EXPECT_EQ(
    mismatch.code,
    Code::kGoalMismatch);

  EXPECT_TRUE(arbiter.HasActiveGoal());
}

TEST(GoalArbiterTest, CancellationNeedsAcknowledgement)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  ASSERT_TRUE(
    arbiter.RequestCancel(
      "goal-1",
      1).accepted);

  const auto acknowledged =
    arbiter.AcknowledgeCancellation(
    "goal-1",
    1);

  EXPECT_EQ(
    acknowledged.code,
    Code::kCancelAcknowledged);

  EXPECT_FALSE(arbiter.HasActiveGoal());
  EXPECT_TRUE(arbiter.HasSeenGoalId("goal-1"));
  EXPECT_EQ(arbiter.RecentHistorySize(), 1U);
}

TEST(GoalArbiterTest, RejectsCancelAckWithoutRequest)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  const auto acknowledgement =
    arbiter.AcknowledgeCancellation(
    "goal-1",
    1);

  EXPECT_EQ(
    acknowledgement.code,
    Code::kCancelNotRequested);

  EXPECT_TRUE(arbiter.HasActiveGoal());
}

TEST(GoalArbiterTest, LateSuccessAfterCancelCanComplete)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  ASSERT_TRUE(
    arbiter.RequestCancel(
      "goal-1",
      1).accepted);

  const auto completed =
    arbiter.Complete("goal-1", 1);

  EXPECT_EQ(
    completed.code,
    Code::kCompleted);

  EXPECT_FALSE(arbiter.HasActiveGoal());
  EXPECT_TRUE(arbiter.HasSeenGoalId("goal-1"));
}

TEST(GoalArbiterTest, CompletionMismatchIsRejected)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  const auto mismatch =
    arbiter.Complete("goal-1", 2);

  EXPECT_EQ(
    mismatch.code,
    Code::kGoalMismatch);

  EXPECT_TRUE(arbiter.HasActiveGoal());
}

TEST(GoalArbiterTest, CompletedGoalIdCannotBeReused)
{
  savo_nav::GoalArbiter arbiter;

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  ASSERT_TRUE(
    arbiter.Complete(
      "goal-1",
      1).accepted);

  const auto duplicate = arbiter.TryAcquire(
    MakeGoal(
      "goal-1",
      Source::kNavigation,
      2),
    Valid());

  EXPECT_EQ(
    duplicate.code,
    Code::kDuplicateGoal);
}

TEST(GoalArbiterTest, RecentHistoryIsBounded)
{
  savo_nav::GoalArbiter arbiter(2);

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-1",
        Source::kNavigation,
        1),
      Valid()).accepted);

  ASSERT_TRUE(
    arbiter.Complete(
      "goal-1",
      1).accepted);

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-2",
        Source::kNavigation,
        2),
      Valid()).accepted);

  ASSERT_TRUE(
    arbiter.Complete(
      "goal-2",
      2).accepted);

  ASSERT_TRUE(
    arbiter.TryAcquire(
      MakeGoal(
        "goal-3",
        Source::kNavigation,
        3),
      Valid()).accepted);

  ASSERT_TRUE(
    arbiter.Complete(
      "goal-3",
      3).accepted);

  EXPECT_EQ(arbiter.RecentHistorySize(), 2U);
  EXPECT_FALSE(arbiter.HasSeenGoalId("goal-1"));
  EXPECT_TRUE(arbiter.HasSeenGoalId("goal-2"));
  EXPECT_TRUE(arbiter.HasSeenGoalId("goal-3"));
}

TEST(GoalArbiterTest, SourcePrioritiesAreDeterministic)
{
  EXPECT_GT(
    savo_nav::GoalArbiter::SourcePriority(
      Source::kSupervisor),
    savo_nav::GoalArbiter::SourcePriority(
      Source::kOperator));

  EXPECT_GT(
    savo_nav::GoalArbiter::SourcePriority(
      Source::kOperator),
    savo_nav::GoalArbiter::SourcePriority(
      Source::kNavigation));

  EXPECT_GT(
    savo_nav::GoalArbiter::SourcePriority(
      Source::kNavigation),
    savo_nav::GoalArbiter::SourcePriority(
      Source::kExploration));
}

TEST(GoalArbiterTest, ConvertsArbitrationCodes)
{
  EXPECT_EQ(
    savo_nav::GoalArbiter::ToString(
      Code::kAccepted),
    "accepted");

  EXPECT_EQ(
    savo_nav::GoalArbiter::ToString(
      Code::kBusy),
    "busy");

  EXPECT_EQ(
    savo_nav::GoalArbiter::ToString(
      Code::kCancelAcknowledged),
    "cancel_acknowledged");

  EXPECT_EQ(
    savo_nav::GoalArbiter::ToString(
      Code::kCompleted),
    "completed");
}

}  // namespace
