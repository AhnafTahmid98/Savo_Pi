#pragma once

#include <optional>
#include <string_view>

namespace savo_mapping
{

enum class ExplorationMode
{
  Idle,
  Frontier,
  Scan360,
  Coverage,
  SemanticReview
};

std::string_view to_string(ExplorationMode mode);
std::optional<ExplorationMode> exploration_mode_from_string(std::string_view text);

bool requires_navigation_goal(ExplorationMode mode);
bool requires_rotation_command(ExplorationMode mode);
bool requires_semantic_confirmation(ExplorationMode mode);
bool is_active_exploration_mode(ExplorationMode mode);

}  // namespace savo_mapping
