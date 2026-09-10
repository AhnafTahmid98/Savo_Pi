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

TEST(
  ExplorationGoalHandoff,
  ValidationRejectionRemainsCorrelatedAndTerminal)
{
  exploration::GoalHandoffMachine machine;

  ASSERT_TRUE(machine.begin("frontier-invalid").accepted);
  ASSERT_TRUE(
    machine.mark_rejected("goal_frame_mismatch:odom").accepted);

  EXPECT_EQ(machine.sequence(), 1U);
  EXPECT_EQ(machine.request_id(), "frontier-invalid");
  EXPECT_EQ(machine.state(), exploration::GoalHandoffState::kRejected);
  EXPECT_TRUE(exploration::is_terminal(machine.state()));
  EXPECT_EQ(machine.reason(), "goal_frame_mismatch:odom");
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

TEST(
  ExplorationGoalHandoff,
  RapidTerminalResponseAcknowledgesWithoutObservedActiveState)
{
  const exploration::GoalHandoffObservation observation{
    true,
    7U,
    "frontier-7",
    exploration::GoalHandoffState::kRejected,
    "savo_nav_rejected_goal"};

  const auto decision = exploration::evaluate_pending_goal(
    7U, "frontier-7", observation, 0.1, 3.0);

  EXPECT_EQ(
    decision.disposition,
    exploration::PendingGoalDisposition::AcknowledgedTerminal);
  EXPECT_TRUE(decision.acknowledged);
  EXPECT_TRUE(decision.clear_pending);
  EXPECT_EQ(decision.reason, "savo_nav_rejected_goal");
}

TEST(
  ExplorationGoalHandoff,
  UncorrelatedRetainedTerminalDoesNotAcknowledgeCurrentGoal)
{
  const exploration::GoalHandoffObservation observation{
    true,
    6U,
    "frontier-6",
    exploration::GoalHandoffState::kRejected,
    "prior_goal_rejected"};

  const auto decision = exploration::evaluate_pending_goal(
    7U, "frontier-7", observation, 0.1, 3.0);

  EXPECT_EQ(
    decision.disposition,
    exploration::PendingGoalDisposition::WaitingForAcknowledgement);
  EXPECT_FALSE(decision.acknowledged);
  EXPECT_FALSE(decision.clear_pending);
}

TEST(
  ExplorationGoalHandoff,
  NoCorrelatedResponseProducesTrueAcknowledgementTimeout)
{
  const exploration::GoalHandoffObservation observation;

  const auto decision = exploration::evaluate_pending_goal(
    2U, "frontier-2", observation, 3.0, 3.0);

  EXPECT_EQ(
    decision.disposition,
    exploration::PendingGoalDisposition::AcknowledgementTimedOut);
  EXPECT_FALSE(decision.acknowledged);
  EXPECT_FALSE(decision.clear_pending);
  EXPECT_EQ(decision.reason, "handoff_ack_timeout");
}

TEST(
  ExplorationGoalHandoff,
  AcceptedAndExecutingResponsesKeepOneGoalPendingUntilTerminal)
{
  exploration::GoalHandoffObservation observation{
    true,
    3U,
    "frontier-3",
    exploration::GoalHandoffState::kAccepted,
    "goal_accepted"};

  auto decision = exploration::evaluate_pending_goal(
    3U, "frontier-3", observation, 0.1, 3.0);

  EXPECT_EQ(
    decision.disposition,
    exploration::PendingGoalDisposition::AcknowledgedActive);
  EXPECT_TRUE(decision.acknowledged);
  EXPECT_FALSE(decision.clear_pending);

  observation.state = exploration::GoalHandoffState::kExecuting;
  observation.reason = "goal_executing";
  decision = exploration::evaluate_pending_goal(
    3U, "frontier-3", observation, 4.0, 3.0);

  EXPECT_EQ(
    decision.disposition,
    exploration::PendingGoalDisposition::AcknowledgedActive);
  EXPECT_FALSE(decision.clear_pending);

  observation.state = exploration::GoalHandoffState::kSucceeded;
  observation.reason = "goal_succeeded";
  decision = exploration::evaluate_pending_goal(
    3U, "frontier-3", observation, 4.1, 3.0);

  EXPECT_EQ(
    decision.disposition,
    exploration::PendingGoalDisposition::AcknowledgedTerminal);
  EXPECT_TRUE(decision.clear_pending);
  EXPECT_EQ(decision.reason, "goal_succeeded");
}

TEST(
  ExplorationGoalHandoff,
  ParsesEveryPublishedState)
{
  for (const auto state : {
      exploration::GoalHandoffState::kIdle,
      exploration::GoalHandoffState::kWaitingForServer,
      exploration::GoalHandoffState::kSending,
      exploration::GoalHandoffState::kAccepted,
      exploration::GoalHandoffState::kExecuting,
      exploration::GoalHandoffState::kCanceling,
      exploration::GoalHandoffState::kSucceeded,
      exploration::GoalHandoffState::kRejected,
      exploration::GoalHandoffState::kAborted,
      exploration::GoalHandoffState::kCanceled,
      exploration::GoalHandoffState::kTimedOut,
      exploration::GoalHandoffState::kError})
  {
    const auto parsed = exploration::goal_handoff_state_from_string(
      exploration::to_string(state));
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed.value(), state);
  }

  EXPECT_FALSE(
    exploration::goal_handoff_state_from_string("unknown").has_value());
}
