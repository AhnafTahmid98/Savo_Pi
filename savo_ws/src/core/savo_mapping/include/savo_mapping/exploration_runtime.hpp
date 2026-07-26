#pragma once

#include "savo_mapping/exploration_mode.hpp"
#include "savo_mapping/mapping_mode.hpp"
#include "savo_mapping/session_state.hpp"
#include "savo_mapping/workflow_phase.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace savo_mapping::exploration_runtime
{

enum class RuntimeDisposition
{
  WaitingForAuthority,
  Disabled,
  Enabled,
  CancelRequired
};

struct RuntimeInputs
{
  std::optional<MappingMode> mode;
  std::optional<ExplorationMode> exploration_mode;
  std::optional<WorkflowPhase> workflow_phase;
  std::optional<SessionState> session_state;

  bool readiness_received{false};
  bool mapping_ready{false};

  bool safety_stop_received{false};
  bool safety_stop_active{true};

  bool handoff_state_received{false};
  bool handoff_active{false};
};

struct RuntimeDecision
{
  RuntimeDisposition disposition{
    RuntimeDisposition::WaitingForAuthority};

  bool frontier_enabled{false};
  bool cancel_active_goal{false};

  std::string reason{
    "waiting_for_authority"};
};

std::string_view to_string(
  RuntimeDisposition disposition);

bool authority_state_complete(
  const RuntimeInputs & inputs);

bool frontier_authorized_by_workflow(
  const RuntimeInputs & inputs);

RuntimeDecision evaluate(
  const RuntimeInputs & inputs);

}  // namespace savo_mapping::exploration_runtime
