#pragma once

#include "savo_mapping/exploration_mode.hpp"
#include "savo_mapping/mapping_mode.hpp"
#include "savo_mapping/session_state.hpp"
#include "savo_mapping/workflow_phase.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace savo_mapping::workflow
{

enum class AuthorityDisposition
{
  Accepted,
  Rejected,
  CancelRequired,
  SafetyHold
};

struct WorkflowAuthorityState
{
  MappingMode mode{MappingMode::MonitorOnly};
  ExplorationMode exploration_mode{ExplorationMode::Idle};
  WorkflowPhase workflow_phase{WorkflowPhase::Idle};
  SessionState session_state{SessionState::Idle};
};

struct WorkflowAuthorityInputs
{
  bool mapping_ready{false};
  bool navigation_handoff_ready{false};
  bool safety_stop_active{false};
  bool exploration_goal_active{false};
};

struct ModeChangeRequest
{
  MappingMode target_mode{MappingMode::MonitorOnly};
  ExplorationMode autonomous_strategy{ExplorationMode::Idle};
};

struct WorkflowAuthorityDecision
{
  AuthorityDisposition disposition{AuthorityDisposition::Rejected};
  WorkflowAuthorityState next_state{};

  bool cancel_active_exploration{false};
  bool movement_authorized{false};
  bool frontier_enabled{false};

  std::string reason{"rejected: not_evaluated"};
};

std::string_view to_string(AuthorityDisposition disposition);

WorkflowAuthorityState make_default_workflow_authority_state();

std::optional<ExplorationMode> resolve_exploration_mode(
  const ModeChangeRequest & request);

WorkflowPhase workflow_phase_for(
  MappingMode mode,
  ExplorationMode exploration_mode);

bool is_workflow_authority_state_consistent(
  const WorkflowAuthorityState & state);

bool is_frontier_runtime_enabled(
  const WorkflowAuthorityDecision & decision);

WorkflowAuthorityDecision evaluate_mode_change(
  const WorkflowAuthorityState & current,
  const WorkflowAuthorityInputs & inputs,
  const ModeChangeRequest & request);

}  // namespace savo_mapping::workflow
