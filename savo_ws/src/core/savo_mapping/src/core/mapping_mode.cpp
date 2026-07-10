#include "savo_mapping/mapping_mode.hpp"

namespace savo_mapping
{

std::string_view to_string(MappingMode mode)
{
  switch (mode) {
    case MappingMode::MonitorOnly:
      return "monitor_only";
    case MappingMode::Manual:
      return "manual";
    case MappingMode::Autonomous:
      return "autonomous";
    case MappingMode::Frontier:
      return "frontier";
    case MappingMode::Scan360:
      return "scan360";
    case MappingMode::Coverage:
      return "coverage";
    case MappingMode::SemanticReview:
      return "semantic_review";
  }

  return "unknown";
}

std::optional<MappingMode> mapping_mode_from_string(std::string_view text)
{
  if (text == "monitor_only") {
    return MappingMode::MonitorOnly;
  }
  if (text == "manual") {
    return MappingMode::Manual;
  }
  if (text == "autonomous") {
    return MappingMode::Autonomous;
  }
  if (text == "frontier") {
    return MappingMode::Frontier;
  }
  if (text == "scan360") {
    return MappingMode::Scan360;
  }
  if (text == "coverage") {
    return MappingMode::Coverage;
  }
  if (text == "semantic_review") {
    return MappingMode::SemanticReview;
  }

  return std::nullopt;
}

bool is_autonomous_mode(MappingMode mode)
{
  return mode == MappingMode::Autonomous ||
         mode == MappingMode::Frontier ||
         mode == MappingMode::Scan360 ||
         mode == MappingMode::Coverage;
}

bool is_movement_requesting_mode(MappingMode mode)
{
  return mode == MappingMode::Autonomous ||
         mode == MappingMode::Frontier ||
         mode == MappingMode::Scan360 ||
         mode == MappingMode::Coverage;
}

bool is_semantic_mode(MappingMode mode)
{
  return mode == MappingMode::SemanticReview;
}

}  // namespace savo_mapping
