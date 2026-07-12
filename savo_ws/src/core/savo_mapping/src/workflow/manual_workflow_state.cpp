#include "savo_mapping/manual_workflow_state.hpp"

#include <string_view>

namespace savo_mapping::workflow
{

std::string_view to_string(
  ManualWorkflowState state)
{
  switch (state) {
    case ManualWorkflowState::STARTING:
      return "starting";

    case ManualWorkflowState::WAITING_FOR_INPUTS:
      return "waiting_for_inputs";

    case ManualWorkflowState::MAPPING:
      return "mapping";

    case ManualWorkflowState::PAUSED:
      return "paused";

    case ManualWorkflowState::ERROR:
      return "error";

    case ManualWorkflowState::COMPLETED:
      return "completed";
  }

  return "error";
}

bool lifecycle_state_is_active(
  std::uint8_t state_id)
{
  return state_id ==
         lifecycle_state_id::ACTIVE;
}

bool lifecycle_state_is_terminal_error(
  std::uint8_t state_id)
{
  return
    state_id ==
      lifecycle_state_id::FINALIZED ||
    state_id ==
      lifecycle_state_id::ERROR_PROCESSING;
}

ManualWorkflowState evaluate_manual_workflow_state(
  const SlamLifecycleObservation & lifecycle,
  bool mapping_ready,
  std::string_view session_state)
{
  if (session_state == "error") {
    return ManualWorkflowState::ERROR;
  }

  if (session_state == "completed") {
    return ManualWorkflowState::COMPLETED;
  }

  if (session_state == "paused") {
    return ManualWorkflowState::PAUSED;
  }

  if (lifecycle_state_is_terminal_error(
      lifecycle.state_id))
  {
    return ManualWorkflowState::ERROR;
  }

  if (lifecycle.startup_grace_expired &&
      (!lifecycle.response_received ||
       !lifecycle.response_fresh))
  {
    return ManualWorkflowState::ERROR;
  }

  if (lifecycle.state_id ==
        lifecycle_state_id::INACTIVE &&
      lifecycle.ever_active)
  {
    return ManualWorkflowState::PAUSED;
  }

  if (!lifecycle_state_is_active(
      lifecycle.state_id))
  {
    return ManualWorkflowState::STARTING;
  }

  if (!mapping_ready) {
    return ManualWorkflowState::WAITING_FOR_INPUTS;
  }

  return ManualWorkflowState::MAPPING;
}

}  // namespace savo_mapping::workflow
