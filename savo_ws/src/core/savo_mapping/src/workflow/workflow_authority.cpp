#include "savo_mapping/workflow_authority.hpp"

#include <utility>

namespace savo_mapping::workflow
{
namespace
{

bool autonomous_strategy_is_valid(ExplorationMode mode)
{
  return mode == ExplorationMode::Frontier ||
         mode == ExplorationMode::Scan360 ||
         mode == ExplorationMode::Coverage;
}

bool mode_requires_active_session(MappingMode mode)
{
  return mode != MappingMode::MonitorOnly;
}

bool mode_requires_mapping_readiness(MappingMode mode)
{
  return mode == MappingMode::Autonomous ||
         mode == MappingMode::Frontier ||
         mode == MappingMode::Scan360 ||
         mode == MappingMode::Coverage ||
         mode == MappingMode::SemanticReview;
}

bool exploration_requires_navigation_handoff(ExplorationMode mode)
{
  return mode == ExplorationMode::Frontier ||
         mode == ExplorationMode::Coverage;
}

bool states_match(
  const WorkflowAuthorityState & left,
  const WorkflowAuthorityState & right)
{
  return left.mode == right.mode &&
         left.exploration_mode == right.exploration_mode &&
         left.workflow_phase == right.workflow_phase &&
         left.session_state == right.session_state;
}

WorkflowAuthorityDecision make_decision(
  AuthorityDisposition disposition,
  const WorkflowAuthorityState & next_state,
  bool cancel_active_exploration,
  bool movement_authorized,
  bool frontier_enabled,
  std::string reason)
{
  WorkflowAuthorityDecision decision;
  decision.disposition = disposition;
  decision.next_state = next_state;
  decision.cancel_active_exploration = cancel_active_exploration;
  decision.movement_authorized = movement_authorized;
  decision.frontier_enabled = frontier_enabled;
  decision.reason = std::move(reason);
  return decision;
}

}  // namespace

std::string_view to_string(AuthorityDisposition disposition)
{
  switch (disposition) {
    case AuthorityDisposition::Accepted:
      return "accepted";

    case AuthorityDisposition::Rejected:
      return "rejected";

    case AuthorityDisposition::CancelRequired:
      return "cancel_required";

    case AuthorityDisposition::SafetyHold:
      return "safety_hold";
  }

  return "rejected";
}

WorkflowAuthorityState make_default_workflow_authority_state()
{
  return WorkflowAuthorityState{};
}

std::optional<ExplorationMode> resolve_exploration_mode(
  const ModeChangeRequest & request)
{
  switch (request.target_mode) {
    case MappingMode::MonitorOnly:
    case MappingMode::Manual:
      return ExplorationMode::Idle;

    case MappingMode::Autonomous:
      if (autonomous_strategy_is_valid(
          request.autonomous_strategy))
      {
        return request.autonomous_strategy;
      }

      return std::nullopt;

    case MappingMode::Frontier:
      return ExplorationMode::Frontier;

    case MappingMode::Scan360:
      return ExplorationMode::Scan360;

    case MappingMode::Coverage:
      return ExplorationMode::Coverage;

    case MappingMode::SemanticReview:
      return ExplorationMode::SemanticReview;
  }

  return std::nullopt;
}

WorkflowPhase workflow_phase_for(
  MappingMode mode,
  ExplorationMode exploration_mode)
{
  if (mode == MappingMode::MonitorOnly) {
    return WorkflowPhase::Idle;
  }

  if (mode == MappingMode::Manual) {
    return WorkflowPhase::Mapping;
  }

  switch (exploration_mode) {
    case ExplorationMode::Idle:
      return WorkflowPhase::Idle;

    case ExplorationMode::Frontier:
      return WorkflowPhase::Exploring;

    case ExplorationMode::Scan360:
      return WorkflowPhase::Scan360;

    case ExplorationMode::Coverage:
      return WorkflowPhase::Coverage;

    case ExplorationMode::SemanticReview:
      return WorkflowPhase::SemanticReview;
  }

  return WorkflowPhase::Error;
}

bool is_workflow_authority_state_consistent(
  const WorkflowAuthorityState & state)
{
  const ModeChangeRequest request{
    state.mode,
    state.exploration_mode
  };

  const auto resolved = resolve_exploration_mode(request);

  if (!resolved.has_value() ||
    resolved.value() != state.exploration_mode)
  {
    return false;
  }

  return state.workflow_phase ==
         workflow_phase_for(
    state.mode,
    state.exploration_mode);
}

bool is_frontier_runtime_enabled(
  const WorkflowAuthorityDecision & decision)
{
  return decision.disposition ==
         AuthorityDisposition::Accepted &&
         decision.frontier_enabled;
}

WorkflowAuthorityDecision evaluate_mode_change(
  const WorkflowAuthorityState & current,
  const WorkflowAuthorityInputs & inputs,
  const ModeChangeRequest & request)
{
  if (!is_workflow_authority_state_consistent(current)) {
    return make_decision(
      AuthorityDisposition::Rejected,
      current,
      false,
      false,
      false,
      "rejected: inconsistent_current_state");
  }

  const auto resolved = resolve_exploration_mode(request);

  if (!resolved.has_value()) {
    return make_decision(
      AuthorityDisposition::Rejected,
      current,
      false,
      false,
      false,
      "rejected: invalid_autonomous_strategy");
  }

  WorkflowAuthorityState target;
  target.mode = request.target_mode;
  target.exploration_mode = resolved.value();
  target.workflow_phase = workflow_phase_for(
    target.mode,
    target.exploration_mode);
  target.session_state = current.session_state;

  const bool target_differs =
    !states_match(current, target);

  if (inputs.exploration_goal_active &&
    target_differs)
  {
    return make_decision(
      AuthorityDisposition::CancelRequired,
      current,
      true,
      false,
      false,
      "cancel_required: active_navigation_goal");
  }

  const bool target_requests_movement =
    is_movement_requesting_mode(target.mode);

  if (inputs.safety_stop_active &&
    target_requests_movement)
  {
    return make_decision(
      AuthorityDisposition::SafetyHold,
      current,
      inputs.exploration_goal_active,
      false,
      false,
      "safety_hold: safety_stop_active");
  }

  if (mode_requires_active_session(target.mode) &&
    current.session_state != SessionState::Active)
  {
    return make_decision(
      inputs.exploration_goal_active ?
      AuthorityDisposition::CancelRequired :
      AuthorityDisposition::Rejected,
      current,
      inputs.exploration_goal_active,
      false,
      false,
      inputs.exploration_goal_active ?
      "cancel_required: session_not_active" :
      "rejected: session_not_active");
  }

  if (mode_requires_mapping_readiness(target.mode) &&
    !inputs.mapping_ready)
  {
    return make_decision(
      inputs.exploration_goal_active ?
      AuthorityDisposition::CancelRequired :
      AuthorityDisposition::Rejected,
      current,
      inputs.exploration_goal_active,
      false,
      false,
      inputs.exploration_goal_active ?
      "cancel_required: mapping_not_ready" :
      "rejected: mapping_not_ready");
  }

  if (exploration_requires_navigation_handoff(
      target.exploration_mode) &&
    !inputs.navigation_handoff_ready)
  {
    return make_decision(
      inputs.exploration_goal_active ?
      AuthorityDisposition::CancelRequired :
      AuthorityDisposition::Rejected,
      current,
      inputs.exploration_goal_active,
      false,
      false,
      inputs.exploration_goal_active ?
      "cancel_required: navigation_handoff_not_ready" :
      "rejected: navigation_handoff_not_ready");
  }

  const bool frontier_enabled =
    target.exploration_mode ==
    ExplorationMode::Frontier;

  return make_decision(
    AuthorityDisposition::Accepted,
    target,
    false,
    target_requests_movement,
    frontier_enabled,
    target_differs ?
    "accepted: mode_changed" :
    "accepted: no_change");
}

}  // namespace savo_mapping::workflow
