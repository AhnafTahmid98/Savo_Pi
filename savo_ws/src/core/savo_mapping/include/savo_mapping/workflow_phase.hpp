#pragma once

#include <optional>
#include <string_view>

namespace savo_mapping
{

enum class WorkflowPhase
{
  Idle,
  Preflight,
  StartingSlam,
  Mapping,
  Exploring,
  Scan360,
  Coverage,
  SemanticReview,
  Saving,
  Validating,
  Complete,
  Error
};

std::string_view to_string(WorkflowPhase phase);
std::optional<WorkflowPhase> workflow_phase_from_string(std::string_view text);

bool is_terminal_phase(WorkflowPhase phase);
bool is_active_phase(WorkflowPhase phase);
bool is_motion_related_phase(WorkflowPhase phase);
bool is_map_output_phase(WorkflowPhase phase);

}  // namespace savo_mapping
