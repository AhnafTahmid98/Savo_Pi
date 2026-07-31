#include "savo_mapping/autonomous_mapping_mission.hpp"

#include <utility>

namespace savo_mapping::autonomous
{

std::string_view to_string(const MissionStrategy strategy)
{
  switch (strategy) {
    case MissionStrategy::None:
      return "none";
    case MissionStrategy::Frontier:
      return "frontier";
    case MissionStrategy::Coverage:
      return "coverage";
  }

  return "unknown";
}

std::string_view to_string(const MissionState state)
{
  switch (state) {
    case MissionState::Idle:
      return "idle";
    case MissionState::Starting:
      return "starting";
    case MissionState::WaitingForAuthority:
      return "waiting_for_authority";
    case MissionState::Exploring:
      return "exploring";
    case MissionState::Pausing:
      return "pausing";
    case MissionState::Paused:
      return "paused";
    case MissionState::Resuming:
      return "resuming";
    case MissionState::Canceling:
      return "canceling";
    case MissionState::Saving:
      return "saving";
    case MissionState::Verifying:
      return "verifying";
    case MissionState::Completed:
      return "completed";
    case MissionState::Canceled:
      return "canceled";
    case MissionState::Failed:
      return "failed";
    case MissionState::CompletionPending:
      return "completion_pending";
    case MissionState::CapturingStartPose:
      return "capturing_start_pose";
    case MissionState::InitialScan360:
      return "initial_scan360";
    case MissionState::InitialHeadScan:
      return "initial_head_scan";
    case MissionState::ConditionalScan360:
      return "conditional_scan360";
    case MissionState::CoveragePending:
      return "coverage_pending";
    case MissionState::Coverage:
      return "coverage";
    case MissionState::ReturningToStart:
      return "returning_to_start";
    case MissionState::FinalScan360:
      return "final_scan360";
    case MissionState::FinalHeadScan:
      return "final_head_scan";
    case MissionState::VerifyingLocations:
      return "verifying_locations";
    case MissionState::AwaitingApproval:
      return "awaiting_approval";
    case MissionState::Releasing:
      return "releasing";
  }

  return "unknown";
}

std::string_view to_string(const MissionResult result)
{
  switch (result) {
    case MissionResult::Succeeded:
      return "succeeded";
    case MissionResult::InvalidRequest:
      return "invalid_request";
    case MissionResult::Busy:
      return "busy";
    case MissionResult::ReadinessLost:
      return "readiness_lost";
    case MissionResult::NavigationFailed:
      return "navigation_failed";
    case MissionResult::TimedOut:
      return "timed_out";
    case MissionResult::Canceled:
      return "canceled";
    case MissionResult::SaveFailed:
      return "save_failed";
    case MissionResult::QualityRejected:
      return "quality_rejected";
    case MissionResult::InternalError:
      return "internal_error";
    case MissionResult::ScanFailed:
      return "scan_failed";
    case MissionResult::StartPoseUnavailable:
      return "start_pose_unavailable";
    case MissionResult::None:
      return "not_terminal";
  }

  return "unknown";
}

std::string_view to_string(const MissionCommand command)
{
  switch (command) {
    case MissionCommand::Pause:
      return "pause";
    case MissionCommand::Resume:
      return "resume";
    case MissionCommand::Cancel:
      return "cancel";
    case MissionCommand::RequestScan360:
      return "request_scan360";
  }

  return "unknown";
}

bool is_active(const MissionState state)
{
  return state != MissionState::Idle && !is_terminal(state);
}

bool is_terminal(const MissionState state)
{
  return
    state == MissionState::Completed ||
    state == MissionState::Canceled ||
    state == MissionState::Failed;
}

bool is_scan_state(const MissionState state)
{
  return
    state == MissionState::InitialScan360 ||
    state == MissionState::ConditionalScan360 ||
    state == MissionState::FinalScan360;
}

bool is_valid_request(const MissionRequest & request)
{
  return
    !request.mission_id.empty() &&
    !request.actor_id.empty() &&
    !request.map_id.empty() &&
    request.map_revision > 0U &&
    request.strategy == MissionStrategy::Frontier;
}

bool is_active_handoff_state(const std::string_view state)
{
  return
    state == "waiting_for_savo_nav" ||
    state == "sending" ||
    state == "accepted" ||
    state == "executing" ||
    state == "canceling";
}

bool is_success_handoff_state(const std::string_view state)
{
  return state == "succeeded";
}

bool is_failed_handoff_state(const std::string_view state)
{
  return
    state == "rejected" ||
    state == "aborted" ||
    state == "timed_out" ||
    state == "error";
}

MissionDecision AutonomousMappingMission::start(
  const MissionRequest & request,
  const MissionInputs & inputs)
{
  if (is_active(snapshot_.state)) {
    return rejected("mission_busy");
  }

  if (!is_valid_request(request)) {
    return rejected("invalid_mission_request");
  }

  snapshot_ = MissionSnapshot{};
  snapshot_.request = request;
  snapshot_.state = MissionState::Starting;
  snapshot_.result = MissionResult::None;
  snapshot_.active = true;
  snapshot_.reason = "mission_start_requested";

  pending_terminal_result_ = MissionResult::Canceled;
  pending_terminal_reason_ = "mission_canceled";
  previous_handoff_state_ = "unavailable";
  resume_state_ = MissionState::Starting;
  transition_to_conditional_scan_ = false;

  start_pose_generation_floor_ = inputs.start_pose_generation;
  scan360_generation_floor_ = inputs.scan360_generation;
  head_scan_generation_floor_ = inputs.head_scan_generation;

  start_pose_captured_ = !inputs.require_start_pose_capture;
  initial_scan360_completed_ = !inputs.require_initial_scan360;
  initial_head_scan_completed_ = !inputs.require_initial_head_scan;
  conditional_scan360_completed_ = 0U;

  return evaluate(inputs);
}

MissionDecision AutonomousMappingMission::control(
  const MissionCommand command,
  std::string reason,
  const MissionInputs & inputs)
{
  if (!is_active(snapshot_.state)) {
    return rejected("no_active_mission");
  }

  if (reason.empty()) {
    reason = std::string{to_string(command)} + "_requested";
  }

  switch (command) {
    case MissionCommand::Pause:
      if (
        snapshot_.state == MissionState::Pausing ||
        snapshot_.state == MissionState::Paused)
      {
        return decision("mission_already_paused_or_pausing");
      }

      if (
        snapshot_.state == MissionState::CompletionPending ||
        snapshot_.state == MissionState::Saving ||
        snapshot_.state == MissionState::Verifying)
      {
        return rejected("mission_completion_pipeline_in_progress");
      }

      if (snapshot_.state == MissionState::Canceling) {
        return rejected("mission_cancel_in_progress");
      }

      begin_pause(snapshot_.state, std::move(reason), false);
      break;

    case MissionCommand::Resume:
      if (snapshot_.state != MissionState::Paused) {
        return rejected("mission_not_paused");
      }

      enter(MissionState::Resuming, std::move(reason));
      break;

    case MissionCommand::Cancel:
      if (snapshot_.state == MissionState::Canceling) {
        return decision("mission_cancel_already_in_progress");
      }

      if (
        snapshot_.state == MissionState::Saving ||
        snapshot_.state == MissionState::Verifying)
      {
        return rejected("mission_save_pipeline_in_progress");
      }

      begin_stop(MissionResult::Canceled, std::move(reason));
      break;

    case MissionCommand::RequestScan360:
      if (snapshot_.state != MissionState::Exploring) {
        return rejected("conditional_scan360_requires_exploring_state");
      }

      begin_pause(
        MissionState::Exploring,
        std::move(reason),
        true);
      break;
  }

  return evaluate(inputs);
}

MissionDecision AutonomousMappingMission::abort(
  const MissionResult result,
  std::string reason,
  const MissionInputs & inputs)
{
  if (!is_active(snapshot_.state)) {
    return rejected("no_active_mission");
  }

  if (
    result == MissionResult::Succeeded ||
    result == MissionResult::Canceled ||
    result == MissionResult::None)
  {
    return rejected("invalid_abort_result");
  }

  if (reason.empty()) {
    reason = std::string{to_string(result)};
  }

  begin_stop(result, std::move(reason));
  return evaluate(inputs);
}

MissionDecision AutonomousMappingMission::observe(
  const MissionInputs & inputs)
{
  return evaluate(inputs);
}

const MissionSnapshot & AutonomousMappingMission::snapshot() const noexcept
{
  return snapshot_;
}

MissionDecision AutonomousMappingMission::evaluate(
  const MissionInputs & inputs)
{
  update_observations(inputs);
  update_handoff_counters(inputs);

  if (!is_active(snapshot_.state)) {
    return decision(snapshot_.reason);
  }

  if (snapshot_.state == MissionState::CompletionPending) {
    MissionDecision output = decision(snapshot_.reason);

    if (
      inputs.session_state.has_value() &&
      inputs.session_state.value() == SessionState::Failed)
    {
      return fail(
        MissionResult::InternalError,
        "mapping_session_failed_during_completion");
    }

    if (
      inputs.session_state.has_value() &&
      inputs.session_state.value() == SessionState::Cancelled)
    {
      snapshot_.state = MissionState::Canceled;
      snapshot_.result = MissionResult::Canceled;
      snapshot_.active = false;
      snapshot_.reason = "mapping_session_canceled_during_completion";
      output = decision(snapshot_.reason);
      output.terminal = true;
      return output;
    }

    if (!inputs.completion_confirmed && workflow_is_frontier(inputs)) {
      enter(MissionState::Exploring, "completion_evidence_revoked");
      return decision(snapshot_.reason);
    }

    if (inputs.handoff_active) {
      output.request_handoff_cancel = true;
      output.reason = "completion_waiting_for_handoff_cancel";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!workflow_is_monitor_only(inputs)) {
      output.request_monitor_mode = true;
      output.reason = "completion_requesting_monitor_mode";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!safe_mapping_state(inputs)) {
      snapshot_.reason = "completion_waiting_for_safe_mapping_state";
      return decision(snapshot_.reason);
    }

    if (snapshot_.request.auto_save) {
      enter(MissionState::Saving, "automatic_map_save_requested");
      return evaluate(inputs);
    }

    snapshot_.state = MissionState::Completed;
    snapshot_.result = MissionResult::Succeeded;
    snapshot_.active = false;
    snapshot_.reason =
      "frontier_exhaustion_confirmed_manual_save_required";

    output = decision(snapshot_.reason);
    output.terminal = true;
    return output;
  }

  if (snapshot_.state == MissionState::Saving) {
    MissionDecision output = decision(snapshot_.reason);

    if (!inputs.map_save_started) {
      output.request_map_save = true;
      output.reason = "requesting_automatic_map_save";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!inputs.map_save_complete) {
      output.request_map_save = true;
      output.reason = "automatic_map_save_in_progress";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!inputs.map_save_succeeded) {
      return fail(
        MissionResult::SaveFailed,
        inputs.map_save_reason.empty() ?
        "automatic_map_save_failed" : inputs.map_save_reason);
    }

    enter(MissionState::Verifying, "saved_map_verification_requested");
    return evaluate(inputs);
  }

  if (snapshot_.state == MissionState::Verifying) {
    MissionDecision output = decision(snapshot_.reason);

    if (!inputs.verification_started) {
      output.request_saved_map_verification = true;
      output.reason = "requesting_saved_map_verification";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!inputs.verification_complete) {
      snapshot_.reason = "saved_map_verification_in_progress";
      return decision(snapshot_.reason);
    }

    if (!inputs.verification_succeeded) {
      return fail(
        MissionResult::SaveFailed,
        inputs.verification_reason.empty() ?
        "saved_map_verification_failed" : inputs.verification_reason);
    }

    snapshot_.state = MissionState::Completed;
    snapshot_.result = MissionResult::Succeeded;
    snapshot_.active = false;
    snapshot_.reason = "automatic_map_save_verified";

    output = decision(snapshot_.reason);
    output.terminal = true;
    return output;
  }

  if (snapshot_.state == MissionState::Canceling) {
    MissionDecision output = decision(snapshot_.reason);

    if (inputs.handoff_active) {
      output.request_handoff_cancel = true;
      output.reason = "cancel_waiting_for_handoff_cancel";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (inputs.scan360_active) {
      output.request_scan360_cancel = true;
      output.reason = "cancel_waiting_for_scan360_cancel";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (inputs.head_scan_active) {
      output.request_head_scan_pause = true;
      output.reason = "cancel_waiting_for_head_scan_pause";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!workflow_is_monitor_only(inputs)) {
      output.request_monitor_mode = true;
      output.reason = "cancel_requesting_monitor_mode";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!session_is_terminal(inputs)) {
      output.request_cancel_session = true;
      output.reason = "cancel_requesting_session_cancel";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    snapshot_.result = pending_terminal_result_;
    snapshot_.active = false;
    snapshot_.reason = pending_terminal_reason_;
    snapshot_.state =
      pending_terminal_result_ == MissionResult::Canceled ?
      MissionState::Canceled : MissionState::Failed;

    output = decision(snapshot_.reason);
    output.terminal = true;
    return output;
  }

  if (snapshot_.state == MissionState::Pausing) {
    MissionDecision output = decision(snapshot_.reason);

    if (inputs.handoff_active) {
      output.request_handoff_cancel = true;
      output.reason = "pause_waiting_for_handoff_cancel";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (inputs.scan360_active) {
      output.request_scan360_cancel = true;
      output.reason = "pause_waiting_for_scan360_cancel";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (inputs.head_scan_active && !inputs.head_scan_paused) {
      output.request_head_scan_pause = true;
      output.reason = "pause_waiting_for_head_scan_pause";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!workflow_is_monitor_only(inputs)) {
      output.request_monitor_mode = true;
      output.reason = "pause_requesting_monitor_mode";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (transition_to_conditional_scan_) {
      transition_to_conditional_scan_ = false;
      begin_scan_stage(
        MissionState::ConditionalScan360,
        inputs,
        "conditional_scan360_quiesced");
      return evaluate(inputs);
    }

    enter(MissionState::Paused, "mission_paused");
    return decision(snapshot_.reason);
  }

  if (snapshot_.state == MissionState::Paused) {
    return decision(snapshot_.reason);
  }

  if (snapshot_.state == MissionState::Resuming) {
    if (!safe_mapping_state(inputs)) {
      snapshot_.reason = "resume_waiting_for_safe_mapping_state";
      return decision(snapshot_.reason);
    }

    if (resume_state_ == MissionState::CapturingStartPose) {
      start_pose_generation_floor_ = inputs.start_pose_generation;
      enter(MissionState::CapturingStartPose, "resuming_start_pose_capture");
      return evaluate(inputs);
    }

    if (is_scan_state(resume_state_)) {
      begin_scan_stage(
        resume_state_,
        inputs,
        "resuming_scan360_stage");
      return evaluate(inputs);
    }

    if (resume_state_ == MissionState::InitialHeadScan) {
      begin_head_scan_stage(
        MissionState::InitialHeadScan,
        inputs,
        "resuming_initial_head_scan");
      return evaluate(inputs);
    }

    return evaluate_frontier_entry(inputs);
  }

  if (snapshot_.state == MissionState::CapturingStartPose) {
    if (!safe_mapping_state(inputs)) {
      begin_pause(
        MissionState::CapturingStartPose,
        "start_pose_capture_authority_lost",
        false);
      return evaluate(inputs);
    }

    if (inputs.start_pose_generation <= start_pose_generation_floor_) {
      MissionDecision output = decision("requesting_start_pose_capture");
      output.request_start_pose_capture = true;
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!inputs.start_pose_capture_complete) {
      MissionDecision output = decision("start_pose_capture_in_progress");
      output.request_start_pose_capture = true;
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!inputs.start_pose_valid) {
      return fail(
        MissionResult::StartPoseUnavailable,
        inputs.start_pose_reason.empty() ?
        "start_pose_capture_failed" : inputs.start_pose_reason);
    }

    start_pose_captured_ = true;
    snapshot_.start_pose_capture_complete = true;
    snapshot_.start_pose_valid = true;
    snapshot_.start_map_generation = inputs.frontier_map_generation;

    if (!initial_scan360_completed_) {
      begin_scan_stage(
        MissionState::InitialScan360,
        inputs,
        "initial_scan360_requested");
      return evaluate(inputs);
    }

    if (!initial_head_scan_completed_) {
      begin_head_scan_stage(
        MissionState::InitialHeadScan,
        inputs,
        "initial_head_scan_requested");
      return evaluate(inputs);
    }

    return evaluate_frontier_entry(inputs);
  }

  if (is_scan_state(snapshot_.state)) {
    return evaluate_scan360_stage(inputs);
  }

  if (snapshot_.state == MissionState::InitialHeadScan) {
    return evaluate_head_scan_stage(inputs);
  }

  if (snapshot_.state == MissionState::Exploring) {
    const bool authority_lost =
      !authority_inputs_complete(inputs) ||
      !safe_mapping_state(inputs) ||
      !inputs.runtime_authorized ||
      !workflow_is_frontier(inputs);

    if (authority_lost) {
      begin_pause(
        MissionState::Exploring,
        "authority_lost_pause_required",
        false);
      return evaluate(inputs);
    }

    if (inputs.completion_confirmed) {
      enter(
        MissionState::CompletionPending,
        inputs.completion_reason.empty() ?
        "frontier_exhaustion_confirmed" : inputs.completion_reason);
      return evaluate(inputs);
    }

    return decision(snapshot_.reason);
  }

  if (snapshot_.state == MissionState::WaitingForAuthority) {
    if (!session_is_active(inputs)) {
      enter(MissionState::Starting, "mapping_session_not_active");
      return evaluate(inputs);
    }

    if (!authority_inputs_complete(inputs) || !safe_mapping_state(inputs)) {
      snapshot_.reason = "waiting_for_mapping_authority";
      return decision(snapshot_.reason);
    }

    if (resume_state_ == MissionState::Exploring) {
      return evaluate_frontier_entry(inputs);
    }

    enter(MissionState::Starting, "mapping_authority_available");
    return evaluate(inputs);
  }

  return evaluate_start_sequence(inputs);
}

MissionDecision AutonomousMappingMission::evaluate_start_sequence(
  const MissionInputs & inputs)
{
  MissionDecision output = decision(snapshot_.reason);

  if (!session_is_active(inputs)) {
    output.request_start_session = true;
    output.reason = "requesting_mapping_session_start";
    snapshot_.reason = output.reason;
    output.snapshot = snapshot_;
    return output;
  }

  if (!authority_inputs_complete(inputs) || !safe_mapping_state(inputs)) {
    resume_state_ = MissionState::Starting;
    enter(MissionState::WaitingForAuthority, "waiting_for_mapping_authority");
    return decision(snapshot_.reason);
  }

  if (!start_pose_captured_) {
    start_pose_generation_floor_ = inputs.start_pose_generation;
    enter(MissionState::CapturingStartPose, "start_pose_capture_requested");
    return evaluate(inputs);
  }

  if (!initial_scan360_completed_) {
    begin_scan_stage(
      MissionState::InitialScan360,
      inputs,
      "initial_scan360_requested");
    return evaluate(inputs);
  }

  if (!initial_head_scan_completed_) {
    begin_head_scan_stage(
      MissionState::InitialHeadScan,
      inputs,
      "initial_head_scan_requested");
    return evaluate(inputs);
  }

  return evaluate_frontier_entry(inputs);
}

MissionDecision AutonomousMappingMission::evaluate_scan360_stage(
  const MissionInputs & inputs)
{
  const MissionState stage = snapshot_.state;

  if (!safe_mapping_state(inputs)) {
    begin_pause(stage, "scan360_authority_lost", false);
    return evaluate(inputs);
  }

  if (!workflow_is_scan360(inputs)) {
    MissionDecision output = decision("requesting_scan360_mode");
    output.request_scan360_mode = true;
    snapshot_.reason = output.reason;
    output.snapshot = snapshot_;
    return output;
  }

  if (inputs.scan360_generation <= scan360_generation_floor_) {
    MissionDecision output = decision("requesting_scan360_start");
    output.request_scan360_start = true;
    snapshot_.reason = output.reason;
    output.snapshot = snapshot_;
    return output;
  }

  if (!inputs.scan360_complete) {
    snapshot_.reason = "scan360_in_progress";
    return decision(snapshot_.reason);
  }

  if (!inputs.scan360_succeeded) {
    return fail(
      MissionResult::ScanFailed,
      inputs.scan360_reason.empty() ?
      "scan360_failed" : inputs.scan360_reason);
  }

  if (stage == MissionState::InitialScan360) {
    initial_scan360_completed_ = true;
    snapshot_.initial_scan360_complete = true;
    snapshot_.initial_scan360_succeeded = true;

    if (!initial_head_scan_completed_) {
      begin_head_scan_stage(
        MissionState::InitialHeadScan,
        inputs,
        "initial_head_scan_requested");
      return evaluate(inputs);
    }

    enter(MissionState::Starting, "initial_scan360_complete");
    return evaluate(inputs);
  }

  if (stage == MissionState::ConditionalScan360) {
    ++conditional_scan360_completed_;
    snapshot_.conditional_scan360_completed =
      conditional_scan360_completed_;
    resume_state_ = MissionState::Exploring;
    enter(MissionState::Resuming, "conditional_scan360_complete");
    return evaluate(inputs);
  }

  resume_state_ = MissionState::Exploring;
  enter(MissionState::Resuming, "scan360_stage_complete");
  return evaluate(inputs);
}

MissionDecision AutonomousMappingMission::evaluate_head_scan_stage(
  const MissionInputs & inputs)
{
  if (!safe_mapping_state(inputs)) {
    begin_pause(
      MissionState::InitialHeadScan,
      "head_scan_authority_lost",
      false);
    return evaluate(inputs);
  }

  if (!workflow_is_monitor_only(inputs)) {
    MissionDecision output = decision("head_scan_requesting_monitor_mode");
    output.request_monitor_mode = true;
    snapshot_.reason = output.reason;
    output.snapshot = snapshot_;
    return output;
  }

  if (inputs.head_scan_paused) {
    MissionDecision output = decision("requesting_initial_head_scan_resume");
    output.request_head_scan_resume = true;
    snapshot_.reason = output.reason;
    output.snapshot = snapshot_;
    return output;
  }

  if (inputs.head_scan_generation <= head_scan_generation_floor_) {
    MissionDecision output = decision("requesting_initial_head_scan_start");
    output.request_head_scan_start = true;
    snapshot_.reason = output.reason;
    output.snapshot = snapshot_;
    return output;
  }

  if (!inputs.head_scan_complete) {
    snapshot_.reason = "initial_head_scan_in_progress";
    return decision(snapshot_.reason);
  }

  if (!inputs.head_scan_succeeded) {
    return fail(
      MissionResult::ScanFailed,
      inputs.head_scan_reason.empty() ?
      "initial_head_scan_failed" : inputs.head_scan_reason);
  }

  initial_head_scan_completed_ = true;
  snapshot_.initial_head_scan_complete = true;
  snapshot_.initial_head_scan_succeeded = true;
  enter(MissionState::Starting, "initial_head_scan_complete");
  return evaluate(inputs);
}

MissionDecision AutonomousMappingMission::evaluate_frontier_entry(
  const MissionInputs & inputs)
{
  MissionDecision output = decision(snapshot_.reason);

  if (!safe_mapping_state(inputs)) {
    resume_state_ = MissionState::Exploring;
    enter(
      MissionState::WaitingForAuthority,
      "waiting_for_mapping_authority");
    return decision(snapshot_.reason);
  }

  if (!workflow_is_frontier(inputs)) {
    output.request_frontier_mode = true;
    output.reason = "requesting_autonomous_frontier_mode";
    snapshot_.reason = output.reason;
    output.snapshot = snapshot_;
    return output;
  }

  if (!inputs.runtime_authorized) {
    resume_state_ = MissionState::Exploring;
    enter(
      MissionState::WaitingForAuthority,
      "waiting_for_frontier_runtime_authority");
    return decision(snapshot_.reason);
  }

  enter(MissionState::Exploring, "frontier_exploration_active");
  return decision(snapshot_.reason);
}

MissionDecision AutonomousMappingMission::fail(
  const MissionResult result,
  std::string reason)
{
  snapshot_.state = MissionState::Failed;
  snapshot_.result = result;
  snapshot_.active = false;
  snapshot_.reason = std::move(reason);

  MissionDecision output = decision(snapshot_.reason);
  output.terminal = true;
  return output;
}

void AutonomousMappingMission::update_observations(
  const MissionInputs & inputs)
{
  snapshot_.runtime_authorized =
    inputs.runtime_authority_received &&
    inputs.runtime_authorized;

  snapshot_.mapping_ready =
    inputs.readiness_received &&
    inputs.mapping_ready;

  snapshot_.safety_stop_active =
    !inputs.safety_stop_received ||
    inputs.safety_stop_active;

  snapshot_.handoff_active =
    inputs.handoff_state_received &&
    inputs.handoff_active;

  snapshot_.start_pose_capture_started =
    inputs.start_pose_capture_started;
  snapshot_.start_pose_capture_complete =
    start_pose_captured_ || inputs.start_pose_capture_complete;
  snapshot_.start_pose_valid =
    start_pose_captured_ || inputs.start_pose_valid;
  snapshot_.start_pose_reason = inputs.start_pose_reason;

  snapshot_.initial_scan360_complete = initial_scan360_completed_;
  snapshot_.initial_scan360_succeeded = initial_scan360_completed_;
  snapshot_.initial_head_scan_complete = initial_head_scan_completed_;
  snapshot_.initial_head_scan_succeeded = initial_head_scan_completed_;
  snapshot_.conditional_scan360_completed =
    conditional_scan360_completed_;

  snapshot_.scan360_started = inputs.scan360_started;
  snapshot_.scan360_active = inputs.scan360_active;
  snapshot_.scan360_complete = inputs.scan360_complete;
  snapshot_.scan360_succeeded = inputs.scan360_succeeded;
  snapshot_.scan360_state = inputs.scan360_state;
  snapshot_.scan360_reason = inputs.scan360_reason;
  snapshot_.scan360_stage = is_scan_state(snapshot_.state) ?
    std::string{to_string(snapshot_.state)} : "none";

  snapshot_.head_scan_started = inputs.head_scan_started;
  snapshot_.head_scan_active = inputs.head_scan_active;
  snapshot_.head_scan_paused = inputs.head_scan_paused;
  snapshot_.head_scan_complete = inputs.head_scan_complete;
  snapshot_.head_scan_succeeded = inputs.head_scan_succeeded;
  snapshot_.head_scan_state = inputs.head_scan_state;
  snapshot_.head_scan_reason = inputs.head_scan_reason;
  snapshot_.head_scan_stage =
    snapshot_.state == MissionState::InitialHeadScan ?
    "initial_head_scan" : "none";

  snapshot_.frontier_status_received =
    inputs.frontier_status_received;
  snapshot_.frontier_status_fresh =
    inputs.frontier_status_fresh;
  snapshot_.frontier_planning_status =
    inputs.frontier_planning_status;
  snapshot_.frontier_plan_sequence =
    inputs.frontier_plan_sequence;
  snapshot_.frontier_map_generation =
    inputs.frontier_map_generation;
  snapshot_.detected_frontiers =
    inputs.detected_frontiers;
  snapshot_.reachable_frontiers =
    inputs.reachable_frontiers;
  snapshot_.exhaustion_observations =
    inputs.exhaustion_observations;
  snapshot_.exhaustion_stable_duration_s =
    inputs.exhaustion_stable_duration_s;
  snapshot_.completion_candidate =
    inputs.completion_candidate;
  snapshot_.completion_confirmed =
    inputs.completion_confirmed;
  snapshot_.completion_reason =
    inputs.completion_reason;

  snapshot_.map_save_started = inputs.map_save_started;
  snapshot_.map_save_complete = inputs.map_save_complete;
  snapshot_.map_saved =
    inputs.map_save_complete &&
    inputs.map_save_succeeded;
  snapshot_.map_save_reason = inputs.map_save_reason;
  snapshot_.saved_session_directory =
    inputs.saved_session_directory;

  snapshot_.verification_started =
    inputs.verification_started;
  snapshot_.verification_complete =
    inputs.verification_complete;
  snapshot_.map_verified =
    inputs.verification_complete &&
    inputs.verification_succeeded;
  snapshot_.verification_reason =
    inputs.verification_reason;
}

void AutonomousMappingMission::update_handoff_counters(
  const MissionInputs & inputs)
{
  if (!is_active(snapshot_.state)) {
    return;
  }

  if (!inputs.handoff_state_received) {
    return;
  }

  if (inputs.handoff_state == previous_handoff_state_) {
    return;
  }

  const bool previous_active =
    is_active_handoff_state(previous_handoff_state_);

  const bool success_transition =
    previous_active &&
    is_success_handoff_state(inputs.handoff_state);
  const bool failed_transition =
    previous_active &&
    is_failed_handoff_state(inputs.handoff_state);

  if (success_transition) {
    ++snapshot_.goals_succeeded;
  }
  if (failed_transition) {
    ++snapshot_.goals_failed;
  }

  previous_handoff_state_ = inputs.handoff_state;
}

bool AutonomousMappingMission::authority_inputs_complete(
  const MissionInputs & inputs) const
{
  return
    inputs.mode.has_value() &&
    inputs.exploration_mode.has_value() &&
    inputs.workflow_phase.has_value() &&
    inputs.session_state.has_value() &&
    inputs.readiness_received &&
    inputs.safety_stop_received &&
    inputs.runtime_authority_received &&
    inputs.handoff_state_received;
}

bool AutonomousMappingMission::safe_mapping_state(
  const MissionInputs & inputs) const
{
  return
    inputs.mapping_ready &&
    !inputs.safety_stop_active;
}

bool AutonomousMappingMission::workflow_is_frontier(
  const MissionInputs & inputs) const
{
  return
    inputs.mode.has_value() &&
    inputs.exploration_mode.has_value() &&
    inputs.workflow_phase.has_value() &&
    inputs.mode.value() == MappingMode::Autonomous &&
    inputs.exploration_mode.value() == ExplorationMode::Frontier &&
    inputs.workflow_phase.value() == WorkflowPhase::Exploring;
}

bool AutonomousMappingMission::workflow_is_scan360(
  const MissionInputs & inputs) const
{
  return
    inputs.mode.has_value() &&
    inputs.exploration_mode.has_value() &&
    inputs.workflow_phase.has_value() &&
    inputs.mode.value() == MappingMode::Autonomous &&
    inputs.exploration_mode.value() == ExplorationMode::Scan360 &&
    inputs.workflow_phase.value() == WorkflowPhase::Scan360;
}

bool AutonomousMappingMission::workflow_is_monitor_only(
  const MissionInputs & inputs) const
{
  return
    inputs.mode.has_value() &&
    inputs.exploration_mode.has_value() &&
    inputs.workflow_phase.has_value() &&
    inputs.mode.value() == MappingMode::MonitorOnly &&
    inputs.exploration_mode.value() == ExplorationMode::Idle &&
    inputs.workflow_phase.value() == WorkflowPhase::Idle;
}

bool AutonomousMappingMission::session_is_active(
  const MissionInputs & inputs) const
{
  return
    inputs.session_state.has_value() &&
    inputs.session_state.value() == SessionState::Active;
}

bool AutonomousMappingMission::session_is_terminal(
  const MissionInputs & inputs) const
{
  return
    inputs.session_state.has_value() &&
    (
    inputs.session_state.value() == SessionState::Saved ||
    inputs.session_state.value() == SessionState::Cancelled ||
    inputs.session_state.value() == SessionState::Failed);
}

void AutonomousMappingMission::begin_scan_stage(
  const MissionState state,
  const MissionInputs & inputs,
  std::string reason)
{
  scan360_generation_floor_ = inputs.scan360_generation;
  enter(state, std::move(reason));
}

void AutonomousMappingMission::begin_head_scan_stage(
  const MissionState state,
  const MissionInputs & inputs,
  std::string reason)
{
  head_scan_generation_floor_ = inputs.head_scan_generation;
  enter(state, std::move(reason));
}

void AutonomousMappingMission::begin_pause(
  const MissionState resume_state,
  std::string reason,
  const bool transition_to_conditional_scan)
{
  resume_state_ = resume_state;
  transition_to_conditional_scan_ = transition_to_conditional_scan;
  enter(MissionState::Pausing, std::move(reason));
}

MissionDecision AutonomousMappingMission::rejected(
  std::string reason) const
{
  MissionDecision output;
  output.accepted = false;
  output.snapshot = snapshot_;
  output.reason = std::move(reason);
  return output;
}

MissionDecision AutonomousMappingMission::decision(
  std::string reason) const
{
  MissionDecision output;
  output.snapshot = snapshot_;
  output.reason = std::move(reason);
  output.terminal = is_terminal(snapshot_.state);
  return output;
}

void AutonomousMappingMission::enter(
  const MissionState state,
  std::string reason)
{
  snapshot_.state = state;
  snapshot_.active = is_active(state);
  snapshot_.reason = std::move(reason);
}

void AutonomousMappingMission::begin_stop(
  const MissionResult result,
  std::string reason)
{
  pending_terminal_result_ = result;
  pending_terminal_reason_ = std::move(reason);
  enter(MissionState::Canceling, pending_terminal_reason_);
}

}  // namespace savo_mapping::autonomous
