// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/system_authority.hpp"

#include <string>
#include <utility>

namespace savo_supervisor
{
namespace
{

SystemAuthorityDecision accept(
  std::string reason,
  const SystemAuthorityCode code = SystemAuthorityCode::kAccepted)
{
  SystemAuthorityDecision result;
  result.accepted = true;
  result.code = code;
  result.reason = std::move(reason);
  return result;
}

SystemAuthorityDecision reject(SystemAuthorityCode code, std::string reason)
{
  SystemAuthorityDecision result;
  result.code = code;
  result.reason = std::move(reason);
  return result;
}

}  // namespace

SystemDependencySnapshot EvaluateCoreSystemDependencies(const SupervisorState & core)
{
  SystemDependencySnapshot dependencies;
  dependencies.core_ready = core.lifecycle == Lifecycle::RUNNING && core.ready &&
    core.capabilities.core_health_ready && core.capabilities.core_safety_ready;
  dependencies.safety_known = core.safety != SafetyObservation::UNKNOWN;
  // Examine all required components: an earlier stale source must not hide a critical fault.
  for (const auto & component : core.component_summaries) {
    if (!component.required || component.ready) {
      continue;
    }
    const bool temporary = component.state == ComponentState::STALE ||
      component.state == ComponentState::INITIALIZING || component.state == ComponentState::UNKNOWN;
    CoreFaultEvidence fault{
      temporary ? CoreFaultKind::kUnavailable : CoreFaultKind::kCritical,
      component.name + ":" + ToString(component.state) + ":" + component.reason_code};
    if (!temporary) {
      dependencies.core_fault = std::move(fault);
      return dependencies;
    }
    if (dependencies.core_fault.kind == CoreFaultKind::kNone) {
      dependencies.core_fault = std::move(fault);
    }
  }
  if (!core.safety_summary.ready || !dependencies.safety_known) {
    const auto & reason = core.safety_summary.reason_code;
    const bool temporary = reason == "safety_stop_missing" || reason == "safety_stop_stale" ||
      reason == "safety_slowdown_missing" || reason == "safety_slowdown_stale" ||
      reason == "safety_state_unknown";
    if (!temporary || dependencies.core_fault.kind == CoreFaultKind::kNone) {
      dependencies.core_fault = {
        temporary ? CoreFaultKind::kUnavailable : CoreFaultKind::kCritical,
        "safety:UNKNOWN:" + reason};
    }
  }
  if (dependencies.core_fault.kind == CoreFaultKind::kNone &&
    (core.lifecycle == Lifecycle::FAULTED || core.health == AggregateHealth::ERROR))
  {
    dependencies.core_fault = {CoreFaultKind::kCritical, "core:ERROR:" + core.reason_code};
  }
  return dependencies;
}

SystemAuthority::SystemAuthority(SystemAuthorityPolicy policy)
: policy_(std::move(policy))
{
}

void SystemAuthority::change(std::string reason, std::string actor)
{
  ++generation_;
  reason_ = std::move(reason);
  if (!actor.empty()) {
    last_actor_ = std::move(actor);
  }
}

bool SystemAuthority::Update(
  const SystemDependencySnapshot & dependencies,
  const std::chrono::steady_clock::time_point now)
{
  if (shutdown_requested_ || fault_latched_) {
    return false;
  }
  const bool unavailable = !dependencies.core_ready || !dependencies.safety_known ||
    dependencies.core_fault.kind != CoreFaultKind::kNone;
  if (!unavailable) {
    unavailable_since_.reset();
  } else if (armed_ || unavailable_since_.has_value()) {
    const bool was_armed = armed_;
    armed_ = false;
    rearm_required_ = true;
    if (!unavailable_since_.has_value()) {
      unavailable_since_ = now;
    }
    const auto evidence = dependencies.core_fault.reason.empty() ?
      "core:UNKNOWN:required_core_dependency_unavailable" : dependencies.core_fault.reason;
    // Persistence qualification only. Authority is lost on the FIRST observation.
    // Service calls cannot accelerate this window; it uses monotonic elapsed time.
    const bool confirmed = dependencies.core_fault.kind == CoreFaultKind::kCritical ||
      now - *unavailable_since_ >= std::chrono::seconds(1);
    if (confirmed && policy_.latch_core_faults_after_arm) {
      fault_latched_ = true;
      change("core_fault_latched:" + evidence, "automatic_revalidation");
      return true;
    }
    if (was_armed) {
      change("core_unavailable:" + evidence, "automatic_revalidation");
      return true;
    }
  }
  if (policy_.auto_arm && !rearm_required_ && !armed_ && !fault_latched_ &&
    !unavailable && dependencies.startup_dependencies_ready)
  {
    armed_ = true;
    change("system_auto_armed", "automatic_startup");
    return true;
  }
  return false;
}

SystemAuthorityDecision SystemAuthority::Handle(
  const SystemAuthorityRequest & request,
  const SystemDependencySnapshot & dependencies)
{
  if (request.command == SystemCommand::kStatus) {
    return accept("system_status_reported");
  }
  if (request.request_id.empty() || request.actor_id.empty()) {
    return reject(SystemAuthorityCode::kInvalidRequest, "invalid_system_request");
  }
  if (request.expected_generation > 0U && request.expected_generation != generation_) {
    return reject(SystemAuthorityCode::kGenerationMismatch, "system_generation_mismatch");
  }
  if (request.command == SystemCommand::kArm) {
    if (shutdown_requested_) {
      return reject(SystemAuthorityCode::kNotReady, "shutdown_already_requested");
    }
    if (fault_latched_) {
      return reject(SystemAuthorityCode::kFaultLatched, "fault_latch_must_be_cleared");
    }
    if (armed_) {
      return accept(
        "system_already_armed", SystemAuthorityCode::kAlreadyInState);
    }
    if (!dependencies.core_ready || !dependencies.safety_known ||
      dependencies.core_fault.kind != CoreFaultKind::kNone ||
      !dependencies.startup_dependencies_ready)
    {
      return reject(SystemAuthorityCode::kNotReady, "startup_dependencies_not_ready");
    }
    armed_ = true;
    rearm_required_ = false;
    unavailable_since_.reset();
    change(request.reason.empty() ? "system_armed" : request.reason, request.actor_id);
    return accept(reason_);
  }
  if (request.command == SystemCommand::kDisarm) {
    if (!armed_) {
      return accept(
        "system_already_disarmed", SystemAuthorityCode::kAlreadyInState);
    }
    armed_ = false;
    rearm_required_ = true;
    change(request.reason.empty() ? "system_disarmed" : request.reason, request.actor_id);
    return accept(reason_);
  }
  if (request.command == SystemCommand::kBeginShutdown) {
    if (shutdown_requested_) {
      return accept(
        "shutdown_already_requested", SystemAuthorityCode::kAlreadyInState);
    }
    armed_ = false;
    shutdown_requested_ = true;
    change(
      request.reason.empty() ? "controlled_shutdown_requested" : request.reason,
      request.actor_id);
    return accept(reason_);
  }
  if (request.command == SystemCommand::kClearFaultLatch) {
    if (!fault_latched_) {
      return accept(
        "fault_latch_already_clear", SystemAuthorityCode::kAlreadyInState);
    }
    const bool safe = dependencies.core_ready && dependencies.safety_known &&
      dependencies.core_fault.kind == CoreFaultKind::kNone &&
      dependencies.startup_dependencies_ready && dependencies.mission_idle;
    if (!safe) {
      return reject(SystemAuthorityCode::kUnsafeToClear, "fault_latch_clear_not_safe");
    }
    fault_latched_ = false;
    rearm_required_ = true;
    unavailable_since_.reset();
    change(request.reason.empty() ? "fault_latch_cleared" : request.reason, request.actor_id);
    return accept(reason_);
  }
  return reject(SystemAuthorityCode::kInvalidRequest, "unsupported_system_command");
}

void SystemAuthority::RestoreFaultLatch(
  const bool fault_latched,
  const std::uint64_t generation,
  const std::string & reason)
{
  armed_ = false;
  fault_latched_ = fault_latched;
  shutdown_requested_ = false;
  rearm_required_ = true;
  unavailable_since_.reset();
  generation_ = generation;
  reason_ = fault_latched ?
    (reason.empty() ? "fault_latch_restored" : reason) :
    "system_disarmed_after_restart";
  last_actor_ = "persistent_state_restore";
}

SystemAuthoritySnapshot SystemAuthority::snapshot(
  const SystemDependencySnapshot & dependencies) const
{
  SystemAuthoritySnapshot result;
  result.startup_ready = dependencies.core_ready && dependencies.startup_dependencies_ready;
  result.armed = armed_;
  result.fault_latched = fault_latched_;
  result.shutdown_requested = shutdown_requested_;
  result.remote_commands_ready = armed_ && dependencies.remote_commands_ready &&
    !fault_latched_ && !shutdown_requested_;
  result.generation = generation_;
  result.last_actor = last_actor_;
  if (shutdown_requested_) {
    result.state = SystemAuthorityState::kShutdownRequested;
    result.reason = reason_;
  } else if (fault_latched_) {
    result.state = SystemAuthorityState::kFaultLatched;
    result.reason = reason_;
  } else if (armed_ && result.startup_ready && !dependencies.degraded) {
    result.state = SystemAuthorityState::kArmed;
    result.reason = reason_;
  } else if (armed_) {
    result.state = SystemAuthorityState::kArmedDegraded;
    result.reason = result.startup_ready ?
      "system_armed_degraded" : "required_startup_dependency_lost";
  } else if (result.startup_ready) {
    result.state = SystemAuthorityState::kReadyToArm;
    result.reason = generation_ == 0U ? "system_ready_to_arm" : reason_;
  } else if (generation_ > 0U) {
    result.state = SystemAuthorityState::kDisarmed;
    result.reason = reason_;
  } else {
    result.state = SystemAuthorityState::kBooting;
    result.reason = "system_booting";
  }
  return result;
}

const char * ToString(const SystemAuthorityState value) noexcept
{
  switch (value) {
    case SystemAuthorityState::kBooting: return "BOOTING";
    case SystemAuthorityState::kReadyToArm: return "READY_TO_ARM";
    case SystemAuthorityState::kArmed: return "ARMED";
    case SystemAuthorityState::kArmedDegraded: return "ARMED_DEGRADED";
    case SystemAuthorityState::kDisarmed: return "DISARMED";
    case SystemAuthorityState::kFaultLatched: return "FAULT_LATCHED";
    case SystemAuthorityState::kShutdownRequested: return "SHUTDOWN_REQUESTED";
  }
  return "UNKNOWN";
}

}  // namespace savo_supervisor
