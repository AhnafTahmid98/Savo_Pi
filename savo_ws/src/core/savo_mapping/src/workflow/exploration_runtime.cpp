#include "savo_mapping/exploration_runtime.hpp"

#include <utility>

namespace savo_mapping::exploration_runtime
{
namespace
{

RuntimeDecision make_decision(
  RuntimeDisposition disposition,
  bool frontier_enabled,
  bool cancel_active_goal,
  std::string reason)
{
  RuntimeDecision decision;

  decision.disposition =
    disposition;

  decision.frontier_enabled =
    frontier_enabled;

  decision.cancel_active_goal =
    cancel_active_goal;

  decision.reason =
    std::move(reason);

  return decision;
}

RuntimeDecision disabled_or_cancel(
  const RuntimeInputs & inputs,
  std::string reason)
{
  if (
    inputs.handoff_state_received &&
    inputs.handoff_active)
  {
    return make_decision(
      RuntimeDisposition::CancelRequired,
      false,
      true,
      "cancel_required: " + reason);
  }

  return make_decision(
    RuntimeDisposition::Disabled,
    false,
    false,
    "disabled: " + reason);
}

}  // namespace

std::string_view to_string(
  RuntimeDisposition disposition)
{
  switch (disposition) {
    case RuntimeDisposition::
      WaitingForAuthority:
      return "waiting_for_authority";

    case RuntimeDisposition::Disabled:
      return "disabled";

    case RuntimeDisposition::Enabled:
      return "enabled";

    case RuntimeDisposition::
      CancelRequired:
      return "cancel_required";
  }

  return "waiting_for_authority";
}

bool authority_state_complete(
  const RuntimeInputs & inputs)
{
  return
    inputs.mode.has_value() &&
    inputs.exploration_mode.has_value() &&
    inputs.workflow_phase.has_value() &&
    inputs.session_state.has_value() &&
    inputs.readiness_received &&
    inputs.safety_stop_received &&
    inputs.handoff_state_received;
}

bool frontier_authorized_by_workflow(
  const RuntimeInputs & inputs)
{
  if (
    !inputs.mode.has_value() ||
    !inputs.exploration_mode.has_value() ||
    !inputs.workflow_phase.has_value() ||
    !inputs.session_state.has_value())
  {
    return false;
  }

  const bool frontier_mode =
    inputs.mode.value() ==
    MappingMode::Frontier ||
    (
    inputs.mode.value() ==
    MappingMode::Autonomous &&
    inputs.exploration_mode.value() ==
    ExplorationMode::Frontier);

  return
    frontier_mode &&
    inputs.exploration_mode.value() ==
    ExplorationMode::Frontier &&
    inputs.workflow_phase.value() ==
    WorkflowPhase::Exploring &&
    inputs.session_state.value() ==
    SessionState::Active;
}

RuntimeDecision evaluate(
  const RuntimeInputs & inputs)
{
  if (!authority_state_complete(inputs)) {
    if (
      inputs.handoff_state_received &&
      inputs.handoff_active)
    {
      return make_decision(
        RuntimeDisposition::
        CancelRequired,
        false,
        true,
        "cancel_required: "
        "authority_state_incomplete");
    }

    return make_decision(
      RuntimeDisposition::
      WaitingForAuthority,
      false,
      false,
      "waiting_for_authority");
  }

  if (inputs.safety_stop_active) {
    return disabled_or_cancel(
      inputs,
      "safety_stop_active");
  }

  if (!inputs.mapping_ready) {
    return disabled_or_cancel(
      inputs,
      "mapping_not_ready");
  }

  if (
    inputs.session_state.value() !=
    SessionState::Active)
  {
    return disabled_or_cancel(
      inputs,
      "session_not_active");
  }

  if (!frontier_authorized_by_workflow(
      inputs))
  {
    return disabled_or_cancel(
      inputs,
      "frontier_not_authorized");
  }

  return make_decision(
    RuntimeDisposition::Enabled,
    true,
    false,
    "enabled: frontier_authorized");
}

}  // namespace savo_mapping::exploration_runtime
