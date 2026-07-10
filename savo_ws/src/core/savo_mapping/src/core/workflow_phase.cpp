#include "savo_mapping/workflow_phase.hpp"

namespace savo_mapping
{

std::string_view to_string(WorkflowPhase phase)
{
  switch (phase) {
    case WorkflowPhase::Idle:
      return "idle";
    case WorkflowPhase::Preflight:
      return "preflight";
    case WorkflowPhase::StartingSlam:
      return "starting_slam";
    case WorkflowPhase::Mapping:
      return "mapping";
    case WorkflowPhase::Exploring:
      return "exploring";
    case WorkflowPhase::Scan360:
      return "scan360";
    case WorkflowPhase::Coverage:
      return "coverage";
    case WorkflowPhase::SemanticReview:
      return "semantic_review";
    case WorkflowPhase::Saving:
      return "saving";
    case WorkflowPhase::Validating:
      return "validating";
    case WorkflowPhase::Complete:
      return "complete";
    case WorkflowPhase::Error:
      return "error";
  }

  return "unknown";
}

std::optional<WorkflowPhase> workflow_phase_from_string(std::string_view text)
{
  if (text == "idle") {
    return WorkflowPhase::Idle;
  }
  if (text == "preflight") {
    return WorkflowPhase::Preflight;
  }
  if (text == "starting_slam") {
    return WorkflowPhase::StartingSlam;
  }
  if (text == "mapping") {
    return WorkflowPhase::Mapping;
  }
  if (text == "exploring") {
    return WorkflowPhase::Exploring;
  }
  if (text == "scan360") {
    return WorkflowPhase::Scan360;
  }
  if (text == "coverage") {
    return WorkflowPhase::Coverage;
  }
  if (text == "semantic_review") {
    return WorkflowPhase::SemanticReview;
  }
  if (text == "saving") {
    return WorkflowPhase::Saving;
  }
  if (text == "validating") {
    return WorkflowPhase::Validating;
  }
  if (text == "complete") {
    return WorkflowPhase::Complete;
  }
  if (text == "error") {
    return WorkflowPhase::Error;
  }

  return std::nullopt;
}

bool is_terminal_phase(WorkflowPhase phase)
{
  return phase == WorkflowPhase::Complete ||
         phase == WorkflowPhase::Error;
}

bool is_active_phase(WorkflowPhase phase)
{
  return phase == WorkflowPhase::Mapping ||
         phase == WorkflowPhase::Exploring ||
         phase == WorkflowPhase::Scan360 ||
         phase == WorkflowPhase::Coverage ||
         phase == WorkflowPhase::SemanticReview;
}

bool is_motion_related_phase(WorkflowPhase phase)
{
  return phase == WorkflowPhase::Exploring ||
         phase == WorkflowPhase::Scan360 ||
         phase == WorkflowPhase::Coverage;
}

bool is_map_output_phase(WorkflowPhase phase)
{
  return phase == WorkflowPhase::Saving ||
         phase == WorkflowPhase::Validating ||
         phase == WorkflowPhase::Complete;
}

}  // namespace savo_mapping
