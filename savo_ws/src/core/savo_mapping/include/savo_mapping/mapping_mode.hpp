#pragma once

#include <optional>
#include <string_view>

namespace savo_mapping
{

enum class MappingMode
{
  MonitorOnly,
  Manual,
  Autonomous,
  Frontier,
  Scan360,
  Coverage,
  SemanticReview
};

std::string_view to_string(MappingMode mode);
std::optional<MappingMode> mapping_mode_from_string(std::string_view text);

bool is_autonomous_mode(MappingMode mode);
bool is_movement_requesting_mode(MappingMode mode);
bool is_semantic_mode(MappingMode mode);

}  // namespace savo_mapping
