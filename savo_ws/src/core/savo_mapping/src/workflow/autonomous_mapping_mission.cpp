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

      enter(MissionState::Pausing, std::move(reason));
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
      snapshot_.state = MissionState::Failed;
      snapshot_.result = MissionResult::InternalError;
      snapshot_.active = false;
      snapshot_.reason = "mapping_session_failed_during_completion";
      output = decision(snapshot_.reason);
      output.terminal = true;
      return output;
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
      enter(
        MissionState::Exploring,
        "completion_evidence_revoked");
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

    if (!inputs.mapping_ready || inputs.safety_stop_active) {
      snapshot_.reason = "completion_waiting_for_safe_mapping_state";
      return decision(snapshot_.reason);
    }

    if (snapshot_.request.auto_save) {
      enter(
        MissionState::Saving,
        "automatic_map_save_requested");
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
      output.reason = inputs.map_save_started ?
        "automatic_map_save_in_progress" :
        "requesting_automatic_map_save";
      snapshot_.reason = output.reason;
      output.snapshot = snapshot_;
      return output;
    }

    if (!inputs.map_save_succeeded) {
      snapshot_.state = MissionState::Failed;
      snapshot_.result = MissionResult::SaveFailed;
      snapshot_.active = false;
      snapshot_.reason = inputs.map_save_reason.empty() ?
        "automatic_map_save_failed" : inputs.map_save_reason;
      output = decision(snapshot_.reason);
      output.terminal = true;
      return output;
    }

    enter(
      MissionState::Verifying,
      "saved_map_verification_requested");
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
      snapshot_.state = MissionState::Failed;
      snapshot_.result = MissionResult::SaveFailed;
      snapshot_.active = false;
      snapshot_.reason = inputs.verification_reason.empty() ?
        "saved_map_verification_failed" : inputs.verification_reason;
      output = decision(snapshot_.reason);
      output.terminal = true;
      return output;
    }

    snapshot_.state = MissionState::Completed;
    snapshot_.result = MissionResult::Succeeded;
    snapshot_.active = false;
    snapshot_.reason = "automatic_map_save_verified";

    output = decision(snapshot_.reason);
    output.terminal = true;
    return output;
  }

  if (snapshot_.state == MissionState::Exploring) {
    const bool authority_lost =
      !authority_inputs_complete(inputs) ||
      !inputs.mapping_ready ||
      inputs.safety_stop_active ||
      !inputs.runtime_authorized ||
      !workflow_is_frontier(inputs);

    if (authority_lost) {
      enter(
        MissionState::Pausing,
        "authority_lost_pause_required");
    } else if (inputs.completion_confirmed) {
      enter(
        MissionState::CompletionPending,
        inputs.completion_reason.empty() ?
        "frontier_exhaustion_confirmed" :
        inputs.completion_reason);
      return evaluate(inputs);
    }
  }

  MissionDecision output = decision(snapshot_.reason);

  if (snapshot_.state == MissionState::Pausing) {
    if (inputs.handoff_active) {
      output.request_handoff_cancel = true;
      output.reason = "pause_waiting_for_handoff_cancel";
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

    enter(MissionState::Paused, "mission_paused");
    return decision(snapshot_.reason);
  }

  if (snapshot_.state == MissionState::Canceling) {
    if (inputs.handoff_active) {
      output.request_handoff_cancel = true;
      output.reason = "cancel_waiting_for_handoff_cancel";
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

  if (snapshot_.state == MissionState::Paused) {
    return decision(snapshot_.reason);
  }

  if (!session_is_active(inputs)) {
    output.request_start_session = true;
    output.reason = "requesting_mapping_session_start";
    snapshot_.reason = output.reason;
    output.snapshot = snapshot_;
    return output;
  }

  if (
    !authority_inputs_complete(inputs) ||
    !inputs.mapping_ready ||
    inputs.safety_stop_active)
  {
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
    enter(
      MissionState::WaitingForAuthority,
      "waiting_for_frontier_runtime_authority");
    return decision(snapshot_.reason);
  }

  enter(MissionState::Exploring, "frontier_exploration_active");
  return decision(snapshot_.reason);
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

  snapshot_.map_save_started =
    inputs.map_save_started;
  snapshot_.map_save_complete =
    inputs.map_save_complete;
  snapshot_.map_saved =
    inputs.map_save_complete &&
    inputs.map_save_succeeded;
  snapshot_.map_save_reason =
    inputs.map_save_reason;
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
