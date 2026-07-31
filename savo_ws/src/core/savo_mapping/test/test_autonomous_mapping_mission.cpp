#include "savo_mapping/autonomous_mapping_mission.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

using savo_mapping::ExplorationMode;
using savo_mapping::MappingMode;
using savo_mapping::SessionState;
using savo_mapping::WorkflowPhase;
using savo_mapping::autonomous::AutonomousMappingMission;
using savo_mapping::autonomous::MissionCommand;
using savo_mapping::autonomous::MissionInputs;
using savo_mapping::autonomous::MissionRequest;
using savo_mapping::autonomous::MissionResult;
using savo_mapping::autonomous::MissionState;
using savo_mapping::autonomous::MissionStrategy;

MissionRequest valid_request()
{
  MissionRequest request;
  request.mission_id = "auto-map-1";
  request.actor_id = "operator";
  request.map_id = "campus-floor-1";
  request.map_revision = 1U;
  request.strategy = MissionStrategy::Frontier;
  request.auto_save = true;
  request.require_quality_approval = true;
  return request;
}

MissionInputs starting_inputs()
{
  MissionInputs inputs;
  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.session_state = SessionState::Idle;
  inputs.readiness_received = true;
  inputs.mapping_ready = true;
  inputs.safety_stop_received = true;
  inputs.safety_stop_active = false;
  inputs.runtime_authority_received = true;
  inputs.runtime_authorized = false;
  inputs.handoff_state_received = true;
  inputs.handoff_active = false;
  inputs.handoff_state = "idle";
  return inputs;
}

MissionInputs exploring_inputs()
{
  auto inputs = starting_inputs();
  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Frontier;
  inputs.workflow_phase = WorkflowPhase::Exploring;
  inputs.session_state = SessionState::Active;
  inputs.runtime_authorized = true;
  return inputs;
}

}  // namespace

TEST(AutonomousMappingMissionTest, RejectsInvalidAndConcurrentStarts)
{
  AutonomousMappingMission mission;
  auto request = valid_request();
  request.map_revision = 0U;

  const auto invalid_start =
    mission.start(request, starting_inputs());
  EXPECT_FALSE(invalid_start.accepted);

  const auto first_start =
    mission.start(valid_request(), starting_inputs());
  ASSERT_TRUE(first_start.accepted);

  const auto second =
    mission.start(valid_request(), starting_inputs());

  EXPECT_FALSE(second.accepted);
  EXPECT_EQ(second.reason, "mission_busy");
}

TEST(AutonomousMappingMissionTest, StartsSessionThenRequestsFrontierMode)
{
  AutonomousMappingMission mission;
  auto inputs = starting_inputs();

  const auto start = mission.start(valid_request(), inputs);

  EXPECT_TRUE(start.request_start_session);
  EXPECT_EQ(
    start.snapshot.state,
    MissionState::Starting);

  inputs.session_state = SessionState::Active;
  const auto mode = mission.observe(inputs);

  EXPECT_TRUE(mode.request_frontier_mode);
  EXPECT_EQ(
    mode.snapshot.state,
    MissionState::Starting);
}

TEST(AutonomousMappingMissionTest, EntersExploringOnlyWithRuntimeAuthority)
{
  AutonomousMappingMission mission;
  ASSERT_TRUE(
    mission.start(valid_request(), starting_inputs()).accepted);

  auto inputs = exploring_inputs();
  inputs.runtime_authorized = false;

  const auto waiting = mission.observe(inputs);
  EXPECT_EQ(
    waiting.snapshot.state,
    MissionState::WaitingForAuthority);

  inputs.runtime_authorized = true;
  const auto exploring = mission.observe(inputs);

  EXPECT_EQ(
    exploring.snapshot.state,
    MissionState::Exploring);
  EXPECT_TRUE(exploring.snapshot.active);
}

TEST(AutonomousMappingMissionTest, PauseCancelsHandoffBeforeChangingMode)
{
  AutonomousMappingMission mission;
  ASSERT_TRUE(
    mission.start(valid_request(), exploring_inputs()).accepted);

  auto inputs = exploring_inputs();
  inputs.handoff_active = true;
  inputs.handoff_state = "executing";

  const auto pause = mission.control(
    MissionCommand::Pause,
    "operator_pause",
    inputs);

  EXPECT_TRUE(pause.request_handoff_cancel);
  EXPECT_FALSE(pause.request_monitor_mode);
  EXPECT_EQ(
    pause.snapshot.state,
    MissionState::Pausing);

  inputs.handoff_active = false;
  inputs.handoff_state = "canceled";
  const auto monitor = mission.observe(inputs);
  EXPECT_TRUE(monitor.request_monitor_mode);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;

  const auto paused = mission.observe(inputs);
  EXPECT_EQ(paused.snapshot.state, MissionState::Paused);
}

TEST(AutonomousMappingMissionTest, ResumeRequiresPausedMission)
{
  AutonomousMappingMission mission;
  ASSERT_TRUE(
    mission.start(valid_request(), exploring_inputs()).accepted);

  const auto rejected = mission.control(
    MissionCommand::Resume,
    "resume",
    exploring_inputs());

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.reason, "mission_not_paused");
}

TEST(AutonomousMappingMissionTest, AuthorityLossLatchesPause)
{
  AutonomousMappingMission mission;
  ASSERT_TRUE(
    mission.start(valid_request(), exploring_inputs()).accepted);

  auto inputs = exploring_inputs();
  inputs.safety_stop_active = true;
  inputs.handoff_active = true;
  inputs.handoff_state = "executing";

  const auto decision = mission.observe(inputs);

  EXPECT_EQ(decision.snapshot.state, MissionState::Pausing);
  EXPECT_TRUE(decision.request_handoff_cancel);
}

TEST(AutonomousMappingMissionTest, CancelStopsGoalModeAndSessionInOrder)
{
  AutonomousMappingMission mission;
  ASSERT_TRUE(
    mission.start(valid_request(), exploring_inputs()).accepted);

  auto inputs = exploring_inputs();
  inputs.handoff_active = true;
  inputs.handoff_state = "executing";

  auto decision = mission.control(
    MissionCommand::Cancel,
    "operator_cancel",
    inputs);

  EXPECT_TRUE(decision.request_handoff_cancel);

  inputs.handoff_active = false;
  inputs.handoff_state = "canceled";
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_monitor_mode);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_cancel_session);

  inputs.session_state = SessionState::Cancelled;
  decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Canceled);
  EXPECT_EQ(decision.snapshot.result, MissionResult::Canceled);
}


TEST(AutonomousMappingMissionTest, CancelBeforeSessionStartRequiresAcknowledgement)
{
  AutonomousMappingMission mission;
  auto inputs = starting_inputs();
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);

  auto decision = mission.control(
    MissionCommand::Cancel,
    "cancel_before_session_start",
    inputs);

  EXPECT_TRUE(decision.request_cancel_session);
  EXPECT_FALSE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Canceling);

  inputs.session_state = SessionState::Cancelled;
  decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Canceled);
}

TEST(AutonomousMappingMissionTest, TracksGoalOutcomesOnce)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);

  inputs.handoff_active = true;
  inputs.handoff_state = "executing";
  mission.observe(inputs);

  inputs.handoff_active = false;
  inputs.handoff_state = "succeeded";
  auto decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.goals_succeeded, 1U);

  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.goals_succeeded, 1U);

  inputs.handoff_active = true;
  inputs.handoff_state = "executing";
  mission.observe(inputs);

  inputs.handoff_active = false;
  inputs.handoff_state = "aborted";
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.goals_failed, 1U);
}


TEST(AutonomousMappingMissionTest, RejectsNonFailureAbortResults)
{
  AutonomousMappingMission mission;
  ASSERT_TRUE(
    mission.start(valid_request(), exploring_inputs()).accepted);

  EXPECT_FALSE(
    mission.abort(
      MissionResult::Succeeded,
      "invalid_success_abort",
      exploring_inputs()).accepted);

  EXPECT_FALSE(
    mission.abort(
      MissionResult::Canceled,
      "use_control_cancel",
      exploring_inputs()).accepted);

  EXPECT_FALSE(
    mission.abort(
      MissionResult::None,
      "not_terminal",
      exploring_inputs()).accepted);
}

TEST(AutonomousMappingMissionTest, RepeatedCancelPreservesPendingFailure)
{
  AutonomousMappingMission mission;
  ASSERT_TRUE(
    mission.start(valid_request(), exploring_inputs()).accepted);

  auto inputs = exploring_inputs();
  auto decision = mission.abort(
    MissionResult::TimedOut,
    "mission_timeout",
    inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::Canceling);

  decision = mission.control(
    MissionCommand::Cancel,
    "operator_cancel_after_timeout",
    inputs);
  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.reason, "mission_cancel_already_in_progress");

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_cancel_session);

  inputs.session_state = SessionState::Cancelled;
  decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Failed);
  EXPECT_EQ(decision.snapshot.result, MissionResult::TimedOut);
  EXPECT_EQ(decision.snapshot.reason, "mission_timeout");
}

TEST(AutonomousMappingMissionTest, TimeoutUsesFailedTerminalState)
{
  AutonomousMappingMission mission;
  ASSERT_TRUE(
    mission.start(valid_request(), exploring_inputs()).accepted);

  auto inputs = exploring_inputs();
  auto decision = mission.abort(
    MissionResult::TimedOut,
    "mission_timeout",
    inputs);

  EXPECT_TRUE(decision.request_monitor_mode);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_cancel_session);

  inputs.session_state = SessionState::Cancelled;
  decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Failed);
  EXPECT_EQ(decision.snapshot.result, MissionResult::TimedOut);
}

TEST(AutonomousMappingMissionTest, ConfirmedExhaustionEntersCompletionPending)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);

  inputs.frontier_status_received = true;
  inputs.frontier_status_fresh = true;
  inputs.frontier_planning_status = "no_frontiers";
  inputs.frontier_plan_sequence = 3U;
  inputs.frontier_map_generation = 12U;
  inputs.exhaustion_observations = 3U;
  inputs.exhaustion_stable_duration_s = 5.0;
  inputs.completion_candidate = true;
  inputs.completion_confirmed = true;
  inputs.completion_reason = "frontier_exhaustion_confirmed:no_frontiers";

  auto decision = mission.observe(inputs);
  EXPECT_EQ(
    decision.snapshot.state,
    MissionState::CompletionPending);
  EXPECT_TRUE(decision.request_monitor_mode);
  EXPECT_FALSE(decision.terminal);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;

  decision = mission.observe(inputs);
  EXPECT_EQ(
    decision.snapshot.state,
    MissionState::Saving);
  EXPECT_FALSE(decision.terminal);
  EXPECT_TRUE(decision.request_map_save);
  EXPECT_EQ(
    decision.reason,
    "requesting_automatic_map_save");
}

TEST(AutonomousMappingMissionTest, ManualSaveMissionCompletesAfterQuiesce)
{
  AutonomousMappingMission mission;
  auto request = valid_request();
  request.auto_save = false;
  auto inputs = exploring_inputs();
  ASSERT_TRUE(mission.start(request, inputs).accepted);

  inputs.frontier_status_received = true;
  inputs.frontier_status_fresh = true;
  inputs.frontier_planning_status = "no_frontiers";
  inputs.frontier_plan_sequence = 3U;
  inputs.exhaustion_observations = 3U;
  inputs.exhaustion_stable_duration_s = 5.0;
  inputs.completion_candidate = true;
  inputs.completion_confirmed = true;
  inputs.completion_reason = "frontier_exhaustion_confirmed:no_frontiers";

  auto decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_monitor_mode);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Completed);
  EXPECT_EQ(decision.snapshot.result, MissionResult::Succeeded);
  EXPECT_EQ(
    decision.snapshot.reason,
    "frontier_exhaustion_confirmed_manual_save_required");
}

TEST(AutonomousMappingMissionTest, NewFrontierRevokesPendingCompletion)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);

  inputs.completion_confirmed = true;
  inputs.completion_reason = "frontier_exhaustion_confirmed:no_frontiers";
  auto decision = mission.observe(inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::CompletionPending);

  inputs.completion_confirmed = false;
  inputs.completion_candidate = false;
  inputs.frontier_planning_status = "goal_selected";
  decision = mission.observe(inputs);

  EXPECT_EQ(decision.snapshot.state, MissionState::Exploring);
  EXPECT_EQ(decision.reason, "completion_evidence_revoked");
}

TEST(AutonomousMappingMissionTest, SessionFailureAbortsCompletionPending)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);

  inputs.completion_confirmed = true;
  inputs.completion_reason = "frontier_exhaustion_confirmed:no_frontiers";
  auto decision = mission.observe(inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::CompletionPending);

  inputs.session_state = SessionState::Failed;
  decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Failed);
  EXPECT_EQ(decision.snapshot.result, MissionResult::InternalError);
}

TEST(AutonomousMappingMissionTest, AutoSaveTransitionsThroughSavingAndVerification)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  inputs.completion_confirmed = true;
  inputs.completion_reason = "frontier_exhaustion_confirmed";

  auto decision = mission.start(valid_request(), inputs);
  ASSERT_TRUE(decision.accepted);
  decision = mission.observe(inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::CompletionPending);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Saving);
  EXPECT_TRUE(decision.request_map_save);

  inputs.map_save_started = true;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Saving);
  EXPECT_FALSE(decision.terminal);

  inputs.map_save_complete = true;
  inputs.map_save_succeeded = true;
  inputs.map_save_reason = "map_session_saved";
  inputs.saved_session_directory = "/tmp/campus-floor-1";
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Verifying);
  EXPECT_TRUE(decision.request_saved_map_verification);

  inputs.verification_started = true;
  inputs.verification_complete = true;
  inputs.verification_succeeded = true;
  inputs.verification_reason = "saved_map_valid";
  decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Completed);
  EXPECT_EQ(decision.snapshot.result, MissionResult::Succeeded);
  EXPECT_TRUE(decision.snapshot.map_saved);
  EXPECT_TRUE(decision.snapshot.map_verified);
}

TEST(AutonomousMappingMissionTest, SaveFailureTerminatesWithSaveFailed)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  inputs.completion_confirmed = true;
  inputs.completion_reason = "frontier_exhaustion_confirmed";

  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);
  mission.observe(inputs);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  mission.observe(inputs);

  inputs.map_save_started = true;
  inputs.map_save_complete = true;
  inputs.map_save_succeeded = false;
  inputs.map_save_reason = "slam_save_map_service_timeout";
  const auto decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Failed);
  EXPECT_EQ(decision.snapshot.result, MissionResult::SaveFailed);
  EXPECT_EQ(decision.snapshot.reason, "slam_save_map_service_timeout");
}

TEST(AutonomousMappingMissionTest, VerificationFailureTerminatesWithSaveFailed)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  inputs.completion_confirmed = true;
  inputs.completion_reason = "frontier_exhaustion_confirmed";

  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);
  mission.observe(inputs);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  mission.observe(inputs);

  inputs.map_save_started = true;
  inputs.map_save_complete = true;
  inputs.map_save_succeeded = true;
  inputs.saved_session_directory = "/tmp/campus-floor-1";
  mission.observe(inputs);

  inputs.verification_started = true;
  inputs.verification_complete = true;
  inputs.verification_succeeded = false;
  inputs.verification_reason = "posegraph_data_missing_or_empty";
  const auto decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Failed);
  EXPECT_EQ(decision.snapshot.result, MissionResult::SaveFailed);
  EXPECT_TRUE(decision.snapshot.map_saved);
  EXPECT_FALSE(decision.snapshot.map_verified);
}
