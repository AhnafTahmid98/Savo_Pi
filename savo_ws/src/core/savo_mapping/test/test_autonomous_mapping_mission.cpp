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

  // AM-1 through AM-3 compatibility fixtures skip the AM-5 prelude.
  inputs.require_start_pose_capture = false;
  inputs.require_initial_scan360 = false;
  inputs.require_initial_head_scan = false;
  inputs.require_coverage = false;
  inputs.require_return_to_start = false;
  inputs.require_final_scan360 = false;
  inputs.require_final_head_scan = false;
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

void enter_coverage_pending(
  AutonomousMappingMission & mission,
  MissionInputs & inputs)
{
  inputs = exploring_inputs();
  inputs.require_coverage = true;
  inputs.completion_confirmed = true;
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);
  ASSERT_TRUE(mission.observe(inputs).request_monitor_mode);
  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  ASSERT_EQ(
    mission.observe(inputs).snapshot.state,
    MissionState::CoveragePending);
}

void enter_coverage_active(
  AutonomousMappingMission & mission,
  MissionInputs & inputs)
{
  enter_coverage_pending(mission, inputs);
  inputs.coverage_plan_generation = 1U;
  inputs.coverage_planning_complete = true;
  inputs.coverage_plan_valid = true;
  inputs.coverage_total_waypoints = 4U;
  ASSERT_TRUE(mission.observe(inputs).request_coverage_approve);
  inputs.coverage_execution_started = true;
  inputs.coverage_execution_active = true;
  inputs.coverage_supervisor_authorized = true;
  ASSERT_EQ(
    mission.observe(inputs).snapshot.state,
    MissionState::Coverage);
}

}  // namespace


TEST(AutonomousMappingMissionTest, SequencesStartPoseInitialScansThenFrontier)
{
  AutonomousMappingMission mission;
  auto inputs = starting_inputs();
  inputs.require_start_pose_capture = true;
  inputs.require_initial_scan360 = true;
  inputs.require_initial_head_scan = true;

  auto decision = mission.start(valid_request(), inputs);
  ASSERT_TRUE(decision.accepted);
  EXPECT_TRUE(decision.request_start_session);

  inputs.session_state = SessionState::Active;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::CapturingStartPose);
  EXPECT_TRUE(decision.request_start_pose_capture);

  inputs.start_pose_capture_started = true;
  inputs.start_pose_capture_complete = true;
  inputs.start_pose_valid = true;
  inputs.start_pose_reason = "start_pose_captured";
  inputs.start_pose_generation = 1U;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::InitialScan360);
  EXPECT_TRUE(decision.request_scan360_mode);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Scan360;
  inputs.workflow_phase = WorkflowPhase::Scan360;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_scan360_start);

  inputs.scan360_started = true;
  inputs.scan360_complete = true;
  inputs.scan360_succeeded = true;
  inputs.scan360_generation = 1U;
  inputs.scan360_state = "complete";
  inputs.scan360_reason = "scan360_complete";
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::InitialHeadScan);
  EXPECT_TRUE(decision.request_monitor_mode);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_head_scan_start);

  inputs.head_scan_started = true;
  inputs.head_scan_complete = true;
  inputs.head_scan_succeeded = true;
  inputs.head_scan_generation = 1U;
  inputs.head_scan_state = "done";
  inputs.head_scan_reason = "head_scan_complete";
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Starting);
  EXPECT_TRUE(decision.request_frontier_mode);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Frontier;
  inputs.workflow_phase = WorkflowPhase::Exploring;
  inputs.runtime_authorized = true;
  decision = mission.observe(inputs);

  EXPECT_EQ(decision.snapshot.state, MissionState::Exploring);
  EXPECT_TRUE(decision.snapshot.start_pose_valid);
  EXPECT_TRUE(decision.snapshot.initial_scan360_complete);
  EXPECT_TRUE(decision.snapshot.initial_scan360_succeeded);
  EXPECT_TRUE(decision.snapshot.initial_head_scan_complete);
  EXPECT_TRUE(decision.snapshot.initial_head_scan_succeeded);
}

TEST(AutonomousMappingMissionTest, InitialScanWithoutHeadEntersFrontier)
{
  AutonomousMappingMission mission;
  auto inputs = starting_inputs();
  inputs.session_state = SessionState::Active;
  inputs.require_initial_scan360 = true;

  auto decision = mission.start(valid_request(), inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::InitialScan360);
  EXPECT_TRUE(decision.request_scan360_mode);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Scan360;
  inputs.workflow_phase = WorkflowPhase::Scan360;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_scan360_start);

  inputs.scan360_started = true;
  inputs.scan360_complete = true;
  inputs.scan360_succeeded = true;
  inputs.scan360_generation = 1U;
  inputs.scan360_state = "complete";
  inputs.scan360_reason = "scan360_complete";
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Starting);
  EXPECT_TRUE(decision.request_frontier_mode);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Frontier;
  inputs.workflow_phase = WorkflowPhase::Exploring;
  inputs.runtime_authorized = true;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Exploring);
}

TEST(AutonomousMappingMissionTest, RunsCoverageReturnFinalScansBeforeSaving)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  inputs.require_coverage = true;
  inputs.require_return_to_start = true;
  inputs.require_final_scan360 = true;
  inputs.require_final_head_scan = true;
  ASSERT_EQ(
    mission.start(valid_request(), inputs).snapshot.state,
    MissionState::Exploring);

  inputs.completion_confirmed = true;
  inputs.completion_reason = "stable_frontier_exhaustion";
  auto decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::CompletionPending);
  EXPECT_TRUE(decision.request_monitor_mode);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::CoveragePending);
  EXPECT_TRUE(decision.request_coverage_plan_reset);
  EXPECT_TRUE(decision.request_coverage_plan);

  inputs.coverage_plan_generation = 1U;
  inputs.coverage_planning_started = true;
  inputs.coverage_planning_complete = true;
  inputs.coverage_plan_valid = true;
  inputs.coverage_total_waypoints = 8U;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_coverage_approve);

  inputs.coverage_execution_started = true;
  inputs.coverage_execution_active = true;
  inputs.coverage_supervisor_authorized = true;
  inputs.coverage_mission_id = "coverage-1-1";
  inputs.coverage_state = "executing";
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Coverage);

  inputs.coverage_execution_active = false;
  inputs.coverage_execution_complete = true;
  inputs.coverage_execution_succeeded = true;
  inputs.coverage_state = "succeeded";
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::ReturningToStart);
  EXPECT_TRUE(decision.request_coverage_reset);

  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_return_to_start);

  inputs.return_to_start_started = true;
  inputs.return_to_start_active = true;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::ReturningToStart);

  inputs.return_to_start_active = false;
  inputs.return_to_start_complete = true;
  inputs.return_to_start_succeeded = true;
  inputs.return_proximity_verified = true;
  inputs.return_within_tolerance = true;
  inputs.return_to_start_distance_m = 0.18;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::FinalScan360);
  EXPECT_TRUE(decision.request_scan360_mode);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Scan360;
  inputs.workflow_phase = WorkflowPhase::Scan360;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_scan360_start);

  inputs.scan360_generation = 1U;
  inputs.scan360_started = true;
  inputs.scan360_complete = true;
  inputs.scan360_succeeded = true;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::FinalHeadScan);
  EXPECT_TRUE(decision.request_monitor_mode);
  EXPECT_TRUE(decision.snapshot.final_scan360_complete);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_head_scan_start);

  inputs.head_scan_generation = 1U;
  inputs.head_scan_started = true;
  inputs.head_scan_complete = true;
  inputs.head_scan_succeeded = true;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Saving);
  EXPECT_TRUE(decision.request_map_save);
  EXPECT_TRUE(decision.snapshot.final_head_scan_complete);

  inputs.map_save_started = true;
  inputs.map_save_complete = true;
  inputs.map_save_succeeded = true;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Verifying);
  EXPECT_TRUE(decision.request_saved_map_verification);

  inputs.verification_started = true;
  inputs.verification_complete = true;
  inputs.verification_succeeded = true;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Completed);
  EXPECT_EQ(decision.snapshot.result, MissionResult::Succeeded);
}

TEST(AutonomousMappingMissionTest, RejectsStaleAndMalformedCoveragePlans)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  inputs.require_coverage = true;
  inputs.completion_confirmed = true;
  inputs.coverage_plan_generation = 4U;
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);
  ASSERT_TRUE(mission.observe(inputs).request_monitor_mode);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  auto decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::CoveragePending);
  EXPECT_TRUE(decision.request_coverage_plan);

  inputs.coverage_planning_complete = true;
  inputs.coverage_plan_valid = true;
  inputs.coverage_total_waypoints = 10U;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.result, MissionResult::NavigationFailed);
  EXPECT_EQ(decision.snapshot.reason, "coverage_plan_generation_stale");

  AutonomousMappingMission malformed_mission;
  inputs = exploring_inputs();
  inputs.require_coverage = true;
  inputs.completion_confirmed = true;
  inputs.coverage_plan_generation = 4U;
  ASSERT_TRUE(malformed_mission.start(valid_request(), inputs).accepted);
  ASSERT_TRUE(malformed_mission.observe(inputs).request_monitor_mode);
  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  ASSERT_EQ(
    malformed_mission.observe(inputs).snapshot.state,
    MissionState::CoveragePending);
  inputs.coverage_plan_generation = 5U;
  inputs.coverage_planning_complete = true;
  inputs.coverage_plan_valid = true;
  inputs.coverage_total_waypoints = 0U;
  decision = malformed_mission.observe(inputs);
  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.result, MissionResult::NavigationFailed);
  EXPECT_EQ(decision.snapshot.reason, "coverage_plan_empty_without_noop");
}

TEST(AutonomousMappingMissionTest, PauseDuringCoverageCancelsAndReplans)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  inputs.require_coverage = true;
  inputs.completion_confirmed = true;
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);
  ASSERT_TRUE(mission.observe(inputs).request_monitor_mode);
  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  static_cast<void>(mission.observe(inputs));
  inputs.coverage_plan_generation = 1U;
  inputs.coverage_planning_complete = true;
  inputs.coverage_plan_valid = true;
  inputs.coverage_total_waypoints = 4U;
  static_cast<void>(mission.observe(inputs));
  inputs.coverage_execution_started = true;
  inputs.coverage_execution_active = true;
  inputs.coverage_supervisor_authorized = true;
  ASSERT_EQ(mission.observe(inputs).snapshot.state, MissionState::Coverage);

  auto decision = mission.control(MissionCommand::Pause, "tag_seen", inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Pausing);
  EXPECT_TRUE(decision.request_coverage_cancel);

  inputs.coverage_execution_active = false;
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Paused);

  decision = mission.control(MissionCommand::Resume, "tag_saved", inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::CoveragePending);
  EXPECT_TRUE(decision.request_coverage_plan_reset);
  EXPECT_TRUE(decision.request_coverage_reset);
}

TEST(AutonomousMappingMissionTest, CancelDuringCoverageApprovalCancelsGateway)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  inputs.require_coverage = true;
  inputs.completion_confirmed = true;
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);
  ASSERT_TRUE(mission.observe(inputs).request_monitor_mode);
  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  static_cast<void>(mission.observe(inputs));
  inputs.coverage_plan_generation = 1U;
  inputs.coverage_planning_complete = true;
  inputs.coverage_plan_valid = true;
  inputs.coverage_total_waypoints = 4U;
  ASSERT_TRUE(mission.observe(inputs).request_coverage_approve);

  inputs.coverage_approval_pending = true;
  const auto decision = mission.control(
    MissionCommand::Cancel, "operator_cancel", inputs);

  EXPECT_EQ(decision.snapshot.state, MissionState::Canceling);
  EXPECT_TRUE(decision.request_coverage_cancel);
  EXPECT_EQ(decision.reason, "cancel_waiting_for_coverage_cancel");
}

TEST(AutonomousMappingMissionTest, CoverageFailureIsNavigationFailure)
{
  AutonomousMappingMission mission;
  MissionInputs inputs;
  enter_coverage_active(mission, inputs);
  inputs.coverage_execution_active = false;
  inputs.coverage_execution_complete = true;
  inputs.coverage_execution_succeeded = false;
  inputs.coverage_reason = "guarded_coverage_aborted";

  const auto decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.result, MissionResult::NavigationFailed);
  EXPECT_EQ(decision.reason, "guarded_coverage_aborted");
}

TEST(AutonomousMappingMissionTest, SupervisorLossCancelsCoverageBeforePause)
{
  AutonomousMappingMission mission;
  MissionInputs inputs;
  enter_coverage_active(mission, inputs);
  inputs.coverage_supervisor_authorized = false;

  const auto decision = mission.observe(inputs);

  EXPECT_EQ(decision.snapshot.state, MissionState::Pausing);
  EXPECT_TRUE(decision.request_coverage_cancel);
  EXPECT_EQ(decision.reason, "pause_waiting_for_coverage_cancel");
}

TEST(AutonomousMappingMissionTest, ReturnOutsideToleranceFailsMission)
{
  AutonomousMappingMission mission;
  MissionInputs inputs;
  enter_coverage_active(mission, inputs);
  inputs.require_return_to_start = true;
  inputs.coverage_execution_active = false;
  inputs.coverage_execution_complete = true;
  inputs.coverage_execution_succeeded = true;
  ASSERT_EQ(
    mission.observe(inputs).snapshot.state,
    MissionState::ReturningToStart);
  inputs.return_to_start_started = true;
  inputs.return_to_start_complete = true;
  inputs.return_to_start_succeeded = true;
  inputs.return_proximity_verified = true;
  inputs.return_within_tolerance = false;

  const auto decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.result, MissionResult::NavigationFailed);
  EXPECT_EQ(decision.reason, "return_to_start_outside_tolerance");
}

TEST(AutonomousMappingMissionTest, ReturnAttemptLimitIsEnforced)
{
  AutonomousMappingMission mission;
  MissionInputs inputs;
  enter_coverage_active(mission, inputs);
  inputs.require_return_to_start = true;
  inputs.coverage_execution_active = false;
  inputs.coverage_execution_complete = true;
  inputs.coverage_execution_succeeded = true;
  ASSERT_EQ(
    mission.observe(inputs).snapshot.state,
    MissionState::ReturningToStart);
  inputs.return_to_start_attempts = 2U;
  inputs.return_to_start_maximum_attempts = 2U;

  const auto decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.result, MissionResult::NavigationFailed);
  EXPECT_EQ(decision.reason, "return_to_start_attempt_limit_reached");
}

TEST(AutonomousMappingMissionTest, FinalScanRequiresFreshSuccessfulGeneration)
{
  AutonomousMappingMission mission;
  MissionInputs inputs;
  enter_coverage_active(mission, inputs);
  inputs.require_final_scan360 = true;
  inputs.scan360_generation = 4U;
  inputs.scan360_complete = true;
  inputs.scan360_succeeded = true;
  inputs.coverage_execution_active = false;
  inputs.coverage_execution_complete = true;
  inputs.coverage_execution_succeeded = true;
  auto decision = mission.observe(inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::FinalScan360);
  EXPECT_TRUE(decision.request_scan360_mode);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Scan360;
  inputs.workflow_phase = WorkflowPhase::Scan360;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_scan360_start);

  inputs.scan360_generation = 5U;
  inputs.scan360_complete = true;
  inputs.scan360_succeeded = false;
  inputs.scan360_reason = "final_scan_fixture_failed";
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.result, MissionResult::ScanFailed);
  EXPECT_EQ(decision.reason, "final_scan_fixture_failed");
}

TEST(AutonomousMappingMissionTest, CancelDuringReturnRequestsActionCancel)
{
  AutonomousMappingMission mission;
  MissionInputs inputs;
  enter_coverage_active(mission, inputs);
  inputs.require_return_to_start = true;
  inputs.coverage_execution_active = false;
  inputs.coverage_execution_complete = true;
  inputs.coverage_execution_succeeded = true;
  ASSERT_EQ(
    mission.observe(inputs).snapshot.state,
    MissionState::ReturningToStart);
  inputs.return_to_start_started = true;
  inputs.return_to_start_active = true;

  const auto decision = mission.control(
    MissionCommand::Cancel, "operator_cancel", inputs);

  EXPECT_EQ(decision.snapshot.state, MissionState::Canceling);
  EXPECT_TRUE(decision.request_return_cancel);
}

TEST(AutonomousMappingMissionTest, InitialScanFailureIsTyped)
{
  AutonomousMappingMission mission;
  auto inputs = starting_inputs();
  inputs.require_initial_scan360 = true;
  inputs.session_state = SessionState::Active;

  auto decision = mission.start(valid_request(), inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::InitialScan360);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Scan360;
  inputs.workflow_phase = WorkflowPhase::Scan360;
  inputs.scan360_generation = 1U;
  inputs.scan360_started = true;
  inputs.scan360_complete = true;
  inputs.scan360_succeeded = false;
  inputs.scan360_state = "failed";
  inputs.scan360_reason = "rotation_action_server_unavailable";
  decision = mission.observe(inputs);

  EXPECT_TRUE(decision.terminal);
  EXPECT_EQ(decision.snapshot.state, MissionState::Failed);
  EXPECT_EQ(decision.snapshot.result, MissionResult::ScanFailed);
  EXPECT_EQ(
    decision.snapshot.reason,
    "rotation_action_server_unavailable");
}

TEST(AutonomousMappingMissionTest, ConditionalScanQuiescesAndResumesFrontier)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);

  inputs.handoff_active = true;
  inputs.handoff_state = "executing";
  auto decision = mission.control(
    MissionCommand::RequestScan360,
    "map_growth_stalled",
    inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Pausing);
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
  EXPECT_EQ(decision.snapshot.state, MissionState::ConditionalScan360);
  EXPECT_TRUE(decision.request_scan360_mode);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Scan360;
  inputs.workflow_phase = WorkflowPhase::Scan360;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_scan360_start);

  inputs.scan360_generation = 1U;
  inputs.scan360_started = true;
  inputs.scan360_complete = true;
  inputs.scan360_succeeded = true;
  inputs.scan360_state = "complete";
  decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::Resuming);
  EXPECT_TRUE(decision.request_frontier_mode);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Frontier;
  inputs.workflow_phase = WorkflowPhase::Exploring;
  inputs.runtime_authorized = true;
  decision = mission.observe(inputs);

  EXPECT_EQ(decision.snapshot.state, MissionState::Exploring);
  EXPECT_EQ(decision.snapshot.conditional_scan360_completed, 1U);
}

TEST(AutonomousMappingMissionTest, PausedHeadScanUsesResumeBoundary)
{
  AutonomousMappingMission mission;
  auto inputs = starting_inputs();
  inputs.require_initial_head_scan = true;
  inputs.session_state = SessionState::Active;

  auto decision = mission.start(valid_request(), inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::InitialHeadScan);
  EXPECT_TRUE(decision.request_head_scan_start);

  inputs.head_scan_generation = 1U;
  inputs.head_scan_started = true;
  inputs.head_scan_active = true;
  inputs.head_scan_state = "running";
  decision = mission.control(MissionCommand::Pause, "operator_pause", inputs);
  EXPECT_TRUE(decision.request_head_scan_pause);

  inputs.head_scan_active = false;
  inputs.head_scan_paused = true;
  inputs.head_scan_state = "paused";
  decision = mission.observe(inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::Paused);

  decision = mission.control(MissionCommand::Resume, "operator_resume", inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::InitialHeadScan);
  EXPECT_TRUE(decision.request_head_scan_resume);
  EXPECT_FALSE(decision.request_head_scan_start);
}

TEST(AutonomousMappingMissionTest, PausedScanRestartsFromAnewGeneration)
{
  AutonomousMappingMission mission;
  auto inputs = starting_inputs();
  inputs.require_initial_scan360 = true;
  inputs.session_state = SessionState::Active;
  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Scan360;
  inputs.workflow_phase = WorkflowPhase::Scan360;

  auto decision = mission.start(valid_request(), inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::InitialScan360);

  inputs.scan360_generation = 1U;
  inputs.scan360_started = true;
  inputs.scan360_active = true;
  inputs.scan360_state = "rotating";
  decision = mission.control(MissionCommand::Pause, "operator_pause", inputs);
  EXPECT_TRUE(decision.request_scan360_cancel);

  inputs.scan360_active = false;
  inputs.scan360_complete = true;
  inputs.scan360_succeeded = false;
  inputs.scan360_state = "canceled";
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_monitor_mode);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  decision = mission.observe(inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::Paused);

  decision = mission.control(MissionCommand::Resume, "operator_resume", inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::InitialScan360);
  EXPECT_TRUE(decision.request_scan360_mode);

  inputs.mode = MappingMode::Autonomous;
  inputs.exploration_mode = ExplorationMode::Scan360;
  inputs.workflow_phase = WorkflowPhase::Scan360;
  decision = mission.observe(inputs);
  EXPECT_TRUE(decision.request_scan360_start);
  EXPECT_EQ(decision.snapshot.scan360_state, "canceled");
}

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

  inputs.frontier_observation_sequence = 1U;
  inputs.frontier_status_fresh = true;
  inputs.completion_confirmed = true;
  inputs.completion_reason = "frontier_exhaustion_confirmed:no_frontiers";
  auto decision = mission.observe(inputs);
  ASSERT_EQ(decision.snapshot.state, MissionState::CompletionPending);

  inputs.completion_confirmed = false;
  inputs.frontier_observation_sequence = 2U;
  inputs.completion_candidate = false;
  inputs.frontier_planning_status = "goal_selected";
  decision = mission.observe(inputs);

  EXPECT_EQ(decision.snapshot.state, MissionState::Exploring);
  EXPECT_EQ(decision.reason, "completion_evidence_revoked");
}

TEST(AutonomousMappingMissionTest, StaleFalseCompletionStatusIsIgnored)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  inputs.frontier_observation_sequence = 10U;
  inputs.completion_confirmed = true;
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);
  ASSERT_EQ(
    mission.observe(inputs).snapshot.state,
    MissionState::CompletionPending);

  inputs.completion_confirmed = false;
  const auto decision = mission.observe(inputs);
  EXPECT_EQ(decision.snapshot.state, MissionState::CompletionPending);
  EXPECT_TRUE(decision.request_monitor_mode);
}

TEST(AutonomousMappingMissionTest, FreshCompletionRevocationAfterMonitorRestoresFrontier)
{
  AutonomousMappingMission mission;
  auto inputs = exploring_inputs();
  inputs.require_coverage = true;
  inputs.frontier_observation_sequence = 20U;
  inputs.frontier_status_fresh = true;
  inputs.completion_confirmed = true;
  ASSERT_TRUE(mission.start(valid_request(), inputs).accepted);
  ASSERT_TRUE(mission.observe(inputs).request_monitor_mode);

  inputs.mode = MappingMode::MonitorOnly;
  inputs.exploration_mode = ExplorationMode::Idle;
  inputs.workflow_phase = WorkflowPhase::Idle;
  inputs.runtime_authorized = false;
  inputs.frontier_observation_sequence = 21U;
  inputs.completion_confirmed = false;
  const auto decision = mission.observe(inputs);

  EXPECT_EQ(decision.snapshot.state, MissionState::WaitingForAuthority);
  EXPECT_TRUE(decision.request_frontier_mode);
  EXPECT_FALSE(decision.request_coverage_plan);
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
