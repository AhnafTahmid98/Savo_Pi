#include "savo_mapping/exploration_goal_handoff.hpp"

#include <gtest/gtest.h>

namespace exploration =
  savo_mapping::exploration;

TEST(ExplorationGoalHandoff, ContractNames)
{
  EXPECT_STREQ(
    exploration::kExplorationActionName,
    "/savo_nav/exploration/navigate_to_pose");

  EXPECT_STREQ(
    exploration::kSelectedGoalTopic,
    "/savo_mapping/exploration/selected_goal");
}

TEST(ExplorationGoalHandoff, SuccessfulLifecycle)
{
  exploration::GoalHandoffMachine machine;

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kIdle);

  EXPECT_TRUE(
    machine.begin("frontier-1").accepted);

  EXPECT_TRUE(
    machine.mark_server_available().accepted);

  EXPECT_TRUE(
    machine.mark_accepted().accepted);

  EXPECT_TRUE(
    machine.mark_executing().accepted);

  EXPECT_TRUE(
    machine.mark_succeeded().accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kSucceeded);

  EXPECT_TRUE(
    exploration::is_terminal(
      machine.state()));

  EXPECT_TRUE(
    machine.reset().accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kIdle);
}

TEST(ExplorationGoalHandoff, RejectsConcurrentGoal)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin("frontier-1").accepted);

  const auto second =
    machine.begin("frontier-2");

  EXPECT_FALSE(second.accepted);
  EXPECT_EQ(
    second.reason,
    "goal_already_active");

  EXPECT_EQ(
    machine.request_id(),
    "frontier-1");
}

TEST(ExplorationGoalHandoff, ServerTimeout)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin("frontier-timeout").accepted);

  EXPECT_TRUE(
    machine.mark_timed_out(
      "savo_nav_unavailable").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kTimedOut);
}

TEST(ExplorationGoalHandoff, GoalResponseTimeout)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin(
      "frontier-response-timeout").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  ASSERT_TRUE(
    machine.mark_timed_out(
      "savo_nav_goal_response_timeout").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kTimedOut);

  EXPECT_TRUE(
    exploration::is_terminal(
      machine.state()));
}

TEST(ExplorationGoalHandoff, ExecutionTimeout)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin(
      "frontier-execution-timeout").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  ASSERT_TRUE(
    machine.mark_accepted().accepted);

  ASSERT_TRUE(
    machine.mark_executing().accepted);

  ASSERT_TRUE(
    machine.mark_timed_out(
      "savo_nav_execution_timeout").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kTimedOut);
}

TEST(ExplorationGoalHandoff, FeedbackStaleTimeout)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin(
      "frontier-feedback-timeout").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  ASSERT_TRUE(
    machine.mark_accepted().accepted);

  ASSERT_TRUE(
    machine.mark_executing().accepted);

  ASSERT_TRUE(
    machine.mark_timed_out(
      "savo_nav_feedback_stale").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kTimedOut);
}

TEST(ExplorationGoalHandoff, CancellationTimeout)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin(
      "frontier-cancel-timeout").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  ASSERT_TRUE(
    machine.mark_accepted().accepted);

  ASSERT_TRUE(
    machine.mark_executing().accepted);

  ASSERT_TRUE(
    machine.request_cancel().accepted);

  ASSERT_TRUE(
    machine.mark_timed_out(
      "savo_nav_cancel_timeout").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kTimedOut);
}

TEST(
  ExplorationGoalHandoff,
  ExecutionTimeoutWaitsForTerminalResult)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin(
      "frontier-execution-timeout").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  ASSERT_TRUE(
    machine.mark_accepted().accepted);

  ASSERT_TRUE(
    machine.mark_executing().accepted);

  ASSERT_TRUE(
    machine.request_cancel(
      "savo_nav_execution_timeout").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kCanceling);

  EXPECT_EQ(
    machine.reason(),
    "savo_nav_execution_timeout");

  EXPECT_TRUE(
    exploration::is_active(
      machine.state()));

  EXPECT_FALSE(
    exploration::is_terminal(
      machine.state()));

  ASSERT_TRUE(
    machine.mark_timed_out(
      "savo_nav_execution_timeout").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kTimedOut);

  EXPECT_FALSE(
    exploration::is_active(
      machine.state()));

  EXPECT_TRUE(
    exploration::is_terminal(
      machine.state()));
}

TEST(
  ExplorationGoalHandoff,
  FeedbackTimeoutWaitsForTerminalResult)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin(
      "frontier-feedback-timeout").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  ASSERT_TRUE(
    machine.mark_accepted().accepted);

  ASSERT_TRUE(
    machine.mark_executing().accepted);

  ASSERT_TRUE(
    machine.request_cancel(
      "savo_nav_feedback_stale").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kCanceling);

  EXPECT_EQ(
    machine.reason(),
    "savo_nav_feedback_stale");

  EXPECT_TRUE(
    exploration::is_active(
      machine.state()));

  EXPECT_FALSE(
    exploration::is_terminal(
      machine.state()));

  ASSERT_TRUE(
    machine.mark_timed_out(
      "savo_nav_feedback_stale").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kTimedOut);

  EXPECT_TRUE(
    exploration::is_terminal(
      machine.state()));
}

TEST(ExplorationGoalHandoff, GoalRejection)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin("frontier-reject").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  EXPECT_TRUE(
    machine.mark_rejected(
      "savo_nav_rejected").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kRejected);
}

TEST(ExplorationGoalHandoff, ActiveCancellation)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin("frontier-cancel").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  ASSERT_TRUE(
    machine.mark_accepted().accepted);

  ASSERT_TRUE(
    machine.mark_executing().accepted);

  EXPECT_TRUE(
    machine.request_cancel().accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kCanceling);

  EXPECT_TRUE(
    machine.mark_canceled(
      "operator_canceled").accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kCanceled);
}

TEST(
  ExplorationGoalHandoff,
  CancelingCanResolveSucceeded)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin(
      "frontier-cancel-rejected-success").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  ASSERT_TRUE(
    machine.mark_accepted().accepted);

  ASSERT_TRUE(
    machine.mark_executing().accepted);

  ASSERT_TRUE(
    machine.request_cancel().accepted);

  ASSERT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kCanceling);

  const auto result =
    machine.mark_succeeded();

  EXPECT_TRUE(result.accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kSucceeded);

  EXPECT_FALSE(
    exploration::is_active(
      machine.state()));

  EXPECT_TRUE(
    exploration::is_terminal(
      machine.state()));
}

TEST(
  ExplorationGoalHandoff,
  CancelingCanResolveAborted)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin(
      "frontier-cancel-rejected-abort").accepted);

  ASSERT_TRUE(
    machine.mark_server_available().accepted);

  ASSERT_TRUE(
    machine.mark_accepted().accepted);

  ASSERT_TRUE(
    machine.mark_executing().accepted);

  ASSERT_TRUE(
    machine.request_cancel().accepted);

  ASSERT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kCanceling);

  const auto result =
    machine.mark_aborted(
      "savo_nav_aborted_after_cancel_rejection");

  EXPECT_TRUE(result.accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kAborted);

  EXPECT_FALSE(
    exploration::is_active(
      machine.state()));

  EXPECT_TRUE(
    exploration::is_terminal(
      machine.state()));
}

TEST(ExplorationGoalHandoff, RejectsInvalidTransition)
{
  exploration::GoalHandoffMachine machine;

  const auto result =
    machine.mark_succeeded();

  EXPECT_FALSE(result.accepted);

  EXPECT_EQ(
    machine.state(),
    exploration::GoalHandoffState::kIdle);
}

TEST(ExplorationGoalHandoff, RequiresRequestId)
{
  exploration::GoalHandoffMachine machine;

  const auto result =
    machine.begin("");

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(
    result.reason,
    "request_id_empty");
}

TEST(ExplorationGoalHandoff, SequenceIncrements)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(
    machine.begin("frontier-1").accepted);

  ASSERT_TRUE(
    machine.mark_timed_out(
      "timeout").accepted);

  ASSERT_TRUE(
    machine.begin("frontier-2").accepted);

  EXPECT_EQ(
    machine.sequence(),
    2U);

  EXPECT_EQ(
    machine.request_id(),
    "frontier-2");
}
