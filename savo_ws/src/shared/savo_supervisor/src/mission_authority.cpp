// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/mission_authority.hpp"

#include <utility>

namespace savo_supervisor
{
namespace
{

bool has_live_map(const ActiveMapContext & context)
{
  return context.type == MapContextType::kLiveMapping && !context.map_id.empty();
}

bool has_saved_map(
  const ActiveMapContext & context,
  const bool require_approved)
{
  return context.type == MapContextType::kSavedRelease &&
         !context.map_id.empty() && context.map_revision > 0U &&
         !context.map_release_id.empty() && (!require_approved || context.approved);
}

bool map_matches(
  const MissionAuthorizationRequest & request,
  const ActiveMapContext & context)
{
  if (!request.map_id.empty() && request.map_id != context.map_id) {
    return false;
  }
  if (request.map_revision > 0U && request.map_revision != context.map_revision) {
    return false;
  }
  if (!request.map_release_id.empty() && request.map_release_id != context.map_release_id) {
    return false;
  }
  return true;
}

MissionAuthorizationDecision deny(
  MissionAuthorizationCode code,
  std::string reason)
{
  MissionAuthorizationDecision decision;
  decision.code = code;
  decision.reason = std::move(reason);
  return decision;
}

MissionAuthorizationDecision allow(std::string reason)
{
  MissionAuthorizationDecision decision;
  decision.authorized = true;
  decision.code = MissionAuthorizationCode::kAuthorized;
  decision.reason = std::move(reason);
  return decision;
}

}  // namespace

MissionAuthority::MissionAuthority(MissionAuthorityPolicy policy)
: policy_(std::move(policy))
{
}

MissionCapabilities MissionAuthority::EvaluateCapabilities(
  const MissionDependencySnapshot & dependencies) const
{
  MissionCapabilities capabilities;
  const auto & core = dependencies.core.capabilities;
  const bool core_running = dependencies.core.lifecycle == Lifecycle::RUNNING;
  const bool no_stop = dependencies.core.safety != SafetyObservation::STOPPED;

  capabilities.mapping_available = dependencies.mapping.received &&
    dependencies.mapping.fresh && dependencies.mapping.valid && dependencies.mapping.healthy;
  capabilities.navigation_ready = dependencies.navigation.received &&
    dependencies.navigation.fresh && dependencies.navigation.valid &&
    dependencies.navigation.ready && dependencies.navigation.goal_acceptance_allowed;
  capabilities.head_ready = dependencies.head.received && dependencies.head.fresh &&
    dependencies.head.valid && dependencies.head.operational &&
    dependencies.head.pan_tilt_ready && dependencies.head.camera_ready &&
    dependencies.head.camera_pose_ready &&
    dependencies.endpoints.apriltag_confirmation_action;
  capabilities.locations_read_ready = dependencies.locations.received &&
    dependencies.locations.fresh && dependencies.locations.valid &&
    dependencies.locations.read_ready && dependencies.locations.storage_healthy;
  capabilities.locations_write_ready = capabilities.locations_read_ready &&
    dependencies.locations.write_ready && !dependencies.locations.mutation_in_progress;
  capabilities.semantic_mapping_ready = capabilities.head_ready &&
    capabilities.locations_write_ready;

  const bool live_map = has_live_map(dependencies.map_context);
  const bool saved_map = has_saved_map(
    dependencies.map_context, policy_.require_approved_release_for_navigation);

  capabilities.can_start_manual_mapping = policy_.allow_manual_mapping &&
    core_running && core.can_start_geometric_mapping && capabilities.mapping_available &&
    dependencies.mapping.ready;
  capabilities.can_start_autonomous_mapping = policy_.allow_autonomous_mapping &&
    capabilities.can_start_manual_mapping && capabilities.navigation_ready &&
    dependencies.endpoints.autonomous_mapping_action &&
    (!policy_.require_semantic_autonomous_mapping || capabilities.semantic_mapping_ready);
  capabilities.can_run_scan360 = policy_.allow_scan360 && core_running &&
    core.can_rotate && capabilities.mapping_available &&
    dependencies.endpoints.rotate_to_heading_action;
  capabilities.can_run_coverage = policy_.allow_coverage && core_running &&
    core.core_motion_ready && capabilities.mapping_available &&
    dependencies.mapping.ready && capabilities.navigation_ready &&
    dependencies.endpoints.coverage_action && live_map;
  capabilities.can_navigate = policy_.allow_navigation && core_running && no_stop &&
    core.core_motion_ready && core.can_start_geometric_mapping &&
    capabilities.navigation_ready && saved_map;
  capabilities.can_register_location = policy_.allow_location_registration &&
    capabilities.semantic_mapping_ready && capabilities.mapping_available && live_map;
  capabilities.can_review_location = policy_.allow_location_review &&
    capabilities.locations_write_ready;
  capabilities.can_confirm_arrival = policy_.allow_arrival_confirmation &&
    capabilities.can_navigate && capabilities.head_ready &&
    capabilities.locations_read_ready;
  return capabilities;
}

MissionAuthorizationDecision MissionAuthority::CheckOperation(
  const MissionAuthorizationRequest & request,
  const MissionDependencySnapshot & dependencies) const
{
  if (request.operation == MissionOperation::kNone || request.request_id.empty() ||
    request.actor_id.empty())
  {
    return deny(MissionAuthorizationCode::kInvalidRequest, "invalid_operation_request");
  }

  if (dependencies.system.shutdown_requested) {
    return deny(MissionAuthorizationCode::kNotReady, "system_shutting_down");
  }
  if (dependencies.system.fault_latched) {
    return deny(MissionAuthorizationCode::kHealthBlocked, "system_fault_latched");
  }
  if (IsExclusiveOperation(request.operation) && !dependencies.system.armed) {
    return deny(MissionAuthorizationCode::kNotReady, "system_not_armed");
  }
  if (request.remote_origin && !dependencies.system.remote_commands_ready) {
    return deny(MissionAuthorizationCode::kDependencyBlocked, "remote_command_path_not_ready");
  }
  if (dependencies.core.lifecycle == Lifecycle::STARTING) {
    return deny(MissionAuthorizationCode::kNotReady, "supervisor_starting");
  }
  if (dependencies.core.lifecycle == Lifecycle::STOPPING) {
    return deny(MissionAuthorizationCode::kNotReady, "supervisor_stopping");
  }
  if (dependencies.core.lifecycle == Lifecycle::FAULTED) {
    return deny(MissionAuthorizationCode::kHealthBlocked, "supervisor_faulted");
  }
  if (dependencies.core.safety == SafetyObservation::UNKNOWN) {
    return deny(MissionAuthorizationCode::kSafetyBlocked, "safety_unknown");
  }
  if (request.motion_required && dependencies.core.safety == SafetyObservation::STOPPED) {
    return deny(MissionAuthorizationCode::kSafetyBlocked, "safety_stop_active");
  }

  const auto capabilities = EvaluateCapabilities(dependencies);
  bool allowed = false;
  const char * unavailable_reason = "operation_dependency_blocked";
  switch (request.operation) {
    case MissionOperation::kManualControl:
      allowed = policy_.allow_manual_control && dependencies.core.capabilities.can_manual_drive;
      unavailable_reason = "manual_control_not_ready";
      break;
    case MissionOperation::kManualMapping:
      allowed = capabilities.can_start_manual_mapping;
      unavailable_reason = "manual_mapping_not_ready";
      break;
    case MissionOperation::kAutonomousMapping:
      allowed = capabilities.can_start_autonomous_mapping &&
        (!request.require_semantic || capabilities.semantic_mapping_ready);
      unavailable_reason = request.require_semantic ?
        "autonomous_semantic_mapping_not_ready" : "autonomous_mapping_not_ready";
      break;
    case MissionOperation::kScan360:
      allowed = capabilities.can_run_scan360;
      unavailable_reason = "scan360_not_ready";
      break;
    case MissionOperation::kCoverage:
      allowed = capabilities.can_run_coverage;
      unavailable_reason = "coverage_not_ready";
      break;
    case MissionOperation::kNavigateToPose:
    case MissionOperation::kNavigateToLocation:
      allowed = capabilities.can_navigate && map_matches(request, dependencies.map_context);
      unavailable_reason = "navigation_not_ready_or_map_mismatch";
      break;
    case MissionOperation::kRegisterLocation:
      allowed = capabilities.can_register_location &&
        map_matches(request, dependencies.map_context);
      unavailable_reason = "location_registration_not_ready_or_map_mismatch";
      break;
    case MissionOperation::kReviewLocation:
      allowed = capabilities.can_review_location;
      unavailable_reason = "location_review_not_ready";
      break;
    case MissionOperation::kConfirmArrival:
      allowed = capabilities.can_confirm_arrival && map_matches(request, dependencies.map_context);
      unavailable_reason = "arrival_confirmation_not_ready_or_map_mismatch";
      break;
    case MissionOperation::kNone:
      break;
  }
  if (!allowed) {
    const bool map_operation = request.operation == MissionOperation::kNavigateToPose ||
      request.operation == MissionOperation::kNavigateToLocation ||
      request.operation == MissionOperation::kRegisterLocation ||
      request.operation == MissionOperation::kConfirmArrival;
    return deny(
      map_operation ? MissionAuthorizationCode::kMapContextBlocked :
      MissionAuthorizationCode::kDependencyBlocked,
      unavailable_reason);
  }
  return allow("operation_authorized");
}

MissionAuthorizationDecision MissionAuthority::Handle(
  const MissionAuthorizationRequest & request,
  const MissionDependencySnapshot & dependencies)
{
  if (request.command == AuthorityCommand::kCheck) {
    if (IsExclusiveOperation(request.operation) && state_.state != OperationState::kIdle) {
      if (state_.request_id != request.request_id || state_.operation != request.operation) {
        return deny(MissionAuthorizationCode::kOperationConflict, "major_operation_already_active");
      }
      if (state_.actor_id != request.actor_id ||
        state_.map_id != request.map_id ||
        state_.map_revision != request.map_revision ||
        state_.map_release_id != request.map_release_id ||
        state_.semantic_required != request.require_semantic ||
        (request.expected_generation > 0U && request.expected_generation != state_.generation))
      {
        return deny(MissionAuthorizationCode::kOwnershipMismatch, "operation_ownership_mismatch");
      }
      if (state_.state == OperationState::kPaused || state_.state == OperationState::kRevoked) {
        return deny(
          MissionAuthorizationCode::kOperationConflict,
          "operation_requires_explicit_resume");
      }
    }
    return CheckOperation(request, dependencies);
  }

  if (request.command == AuthorityCommand::kAcquire) {
    const auto checked = CheckOperation(request, dependencies);
    if (!checked.authorized) {
      return checked;
    }
    if (!IsExclusiveOperation(request.operation)) {
      return allow("nonexclusive_operation_authorized");
    }
    if (state_.state != OperationState::kIdle) {
      if (state_.request_id == request.request_id && state_.operation == request.operation &&
        state_.actor_id == request.actor_id && state_.map_id == request.map_id &&
        state_.map_revision == request.map_revision &&
        state_.map_release_id == request.map_release_id &&
        state_.semantic_required == request.require_semantic &&
        (request.expected_generation == 0U ||
        request.expected_generation == state_.generation))
      {
        return allow("operation_already_owned");
      }
      return deny(MissionAuthorizationCode::kOperationConflict, "major_operation_already_active");
    }
    state_.operation = request.operation;
    state_.state = OperationState::kActive;
    state_.request_id = request.request_id;
    state_.actor_id = request.actor_id;
    state_.map_id = request.map_id;
    state_.map_revision = request.map_revision;
    state_.map_release_id = request.map_release_id;
    state_.semantic_required = request.require_semantic;
    state_.remote_origin = request.remote_origin;
    ++state_.generation;
    state_.reason = "operation_acquired";
    return allow(state_.reason);
  }

  if (state_.state == OperationState::kIdle || request.request_id != state_.request_id ||
    request.actor_id != state_.actor_id || request.operation != state_.operation ||
    request.map_id != state_.map_id ||
    request.map_revision != state_.map_revision ||
    request.map_release_id != state_.map_release_id ||
    request.require_semantic != state_.semantic_required ||
    (request.expected_generation > 0U && request.expected_generation != state_.generation))
  {
    return deny(MissionAuthorizationCode::kOwnershipMismatch, "operation_ownership_mismatch");
  }

  if (request.command == AuthorityCommand::kRelease) {
    state_.operation = MissionOperation::kNone;
    state_.state = OperationState::kIdle;
    state_.request_id.clear();
    state_.actor_id.clear();
    state_.map_id.clear();
    state_.map_revision = 0U;
    state_.map_release_id.clear();
    state_.semantic_required = false;
    state_.remote_origin = false;
    ++state_.generation;
    state_.reason = "operation_released";
    return allow(state_.reason);
  }
  if (request.command == AuthorityCommand::kPause) {
    if (state_.state != OperationState::kActive) {
      return deny(MissionAuthorizationCode::kOperationConflict, "operation_not_active");
    }
    state_.state = OperationState::kPaused;
    ++state_.generation;
    state_.reason = "operation_paused";
    return allow(state_.reason);
  }
  if (request.command == AuthorityCommand::kResume) {
    MissionAuthorizationRequest resumed = request;
    resumed.operation = state_.operation;
    resumed.map_id = state_.map_id;
    resumed.map_revision = state_.map_revision;
    resumed.map_release_id = state_.map_release_id;
    resumed.require_semantic = state_.semantic_required;
    resumed.remote_origin = state_.remote_origin;
    const auto checked = CheckOperation(resumed, dependencies);
    if (!checked.authorized) {
      return checked;
    }
    if (state_.state != OperationState::kPaused && state_.state != OperationState::kRevoked) {
      return deny(MissionAuthorizationCode::kOperationConflict, "operation_not_paused_or_revoked");
    }
    state_.state = OperationState::kActive;
    ++state_.generation;
    state_.reason = "operation_resumed";
    return allow(state_.reason);
  }
  return deny(MissionAuthorizationCode::kInvalidRequest, "unsupported_authority_command");
}

bool MissionAuthority::Revalidate(const MissionDependencySnapshot & dependencies)
{
  if (state_.state != OperationState::kActive) {
    return false;
  }
  MissionAuthorizationRequest request;
  request.command = AuthorityCommand::kCheck;
  request.operation = state_.operation;
  request.request_id = state_.request_id;
  request.actor_id = state_.actor_id;
  request.map_id = state_.map_id;
  request.map_revision = state_.map_revision;
  request.map_release_id = state_.map_release_id;
  request.require_semantic = state_.semantic_required;
  request.remote_origin = state_.remote_origin;
  request.motion_required = IsExclusiveOperation(state_.operation);
  const auto decision = CheckOperation(request, dependencies);
  if (decision.authorized) {
    return false;
  }
  state_.state = OperationState::kRevoked;
  ++state_.generation;
  state_.reason = "runtime_authorization_revoked:" + decision.reason;
  return true;
}

const MissionAuthorityState & MissionAuthority::state() const noexcept
{
  return state_;
}

bool IsExclusiveOperation(MissionOperation operation) noexcept
{
  switch (operation) {
    case MissionOperation::kManualControl:
    case MissionOperation::kManualMapping:
    case MissionOperation::kAutonomousMapping:
    case MissionOperation::kScan360:
    case MissionOperation::kCoverage:
    case MissionOperation::kNavigateToPose:
    case MissionOperation::kNavigateToLocation:
      return true;
    case MissionOperation::kNone:
    case MissionOperation::kRegisterLocation:
    case MissionOperation::kReviewLocation:
    case MissionOperation::kConfirmArrival:
      return false;
  }
  return false;
}

OperatingMode ModeForAuthority(
  const SupervisorState & core,
  const MissionAuthorityState & authority) noexcept
{
  if (core.lifecycle == Lifecycle::STARTING) {
    return OperatingMode::BOOTING;
  }
  if (core.lifecycle == Lifecycle::FAULTED) {
    return OperatingMode::ERROR;
  }
  if (core.safety == SafetyObservation::STOPPED) {
    return OperatingMode::ESTOP;
  }
  if (authority.state == OperationState::kPaused ||
    authority.state == OperationState::kRevoked)
  {
    return OperatingMode::RECOVERY;
  }
  switch (authority.operation) {
    case MissionOperation::kManualControl: return OperatingMode::MANUAL;
    case MissionOperation::kManualMapping:
    case MissionOperation::kAutonomousMapping:
    case MissionOperation::kScan360:
    case MissionOperation::kCoverage:
      return OperatingMode::MAPPING;
    case MissionOperation::kNavigateToPose:
    case MissionOperation::kNavigateToLocation:
      return OperatingMode::NAVIGATE;
    case MissionOperation::kNone:
    case MissionOperation::kRegisterLocation:
    case MissionOperation::kReviewLocation:
    case MissionOperation::kConfirmArrival:
      return OperatingMode::IDLE;
  }
  return OperatingMode::UNKNOWN;
}

const char * ToString(MissionOperation value) noexcept
{
  switch (value) {
    case MissionOperation::kNone: return "NONE";
    case MissionOperation::kManualControl: return "MANUAL_CONTROL";
    case MissionOperation::kManualMapping: return "MANUAL_MAPPING";
    case MissionOperation::kAutonomousMapping: return "AUTONOMOUS_MAPPING";
    case MissionOperation::kScan360: return "SCAN360";
    case MissionOperation::kCoverage: return "COVERAGE";
    case MissionOperation::kNavigateToPose: return "NAVIGATE_TO_POSE";
    case MissionOperation::kNavigateToLocation: return "NAVIGATE_TO_LOCATION";
    case MissionOperation::kRegisterLocation: return "REGISTER_LOCATION";
    case MissionOperation::kReviewLocation: return "REVIEW_LOCATION";
    case MissionOperation::kConfirmArrival: return "CONFIRM_ARRIVAL";
  }
  return "UNKNOWN";
}

const char * ToString(OperationState value) noexcept
{
  switch (value) {
    case OperationState::kIdle: return "IDLE";
    case OperationState::kActive: return "ACTIVE";
    case OperationState::kPaused: return "PAUSED";
    case OperationState::kRevoked: return "REVOKED";
  }
  return "UNKNOWN";
}

const char * ToString(MapContextType value) noexcept
{
  switch (value) {
    case MapContextType::kNone: return "NONE";
    case MapContextType::kLiveMapping: return "LIVE_MAPPING";
    case MapContextType::kSavedRelease: return "SAVED_RELEASE";
  }
  return "UNKNOWN";
}

}  // namespace savo_supervisor
