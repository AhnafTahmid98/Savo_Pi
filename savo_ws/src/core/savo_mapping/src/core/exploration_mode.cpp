#include "savo_mapping/exploration_mode.hpp"

namespace savo_mapping
{

std::string_view to_string(ExplorationMode mode)
{
  switch (mode) {
    case ExplorationMode::Idle:
      return "idle";
    case ExplorationMode::Frontier:
      return "frontier";
    case ExplorationMode::Scan360:
      return "scan360";
    case ExplorationMode::Coverage:
      return "coverage";
    case ExplorationMode::SemanticReview:
      return "semantic_review";
  }

  return "unknown";
}

std::optional<ExplorationMode> exploration_mode_from_string(std::string_view text)
{
  if (text == "idle") {
    return ExplorationMode::Idle;
  }
  if (text == "frontier") {
    return ExplorationMode::Frontier;
  }
  if (text == "scan360") {
    return ExplorationMode::Scan360;
  }
  if (text == "coverage") {
    return ExplorationMode::Coverage;
  }
  if (text == "semantic_review") {
    return ExplorationMode::SemanticReview;
  }

  return std::nullopt;
}

bool requires_navigation_goal(ExplorationMode mode)
{
  return mode == ExplorationMode::Frontier ||
         mode == ExplorationMode::Coverage;
}

bool requires_rotation_command(ExplorationMode mode)
{
  return mode == ExplorationMode::Scan360;
}

bool requires_semantic_confirmation(ExplorationMode mode)
{
  return mode == ExplorationMode::SemanticReview;
}

bool is_active_exploration_mode(ExplorationMode mode)
{
  return mode != ExplorationMode::Idle;
}

}  // namespace savo_mapping
