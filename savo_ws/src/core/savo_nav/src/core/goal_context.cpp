// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/goal_context.hpp"

#include <cctype>

#include "savo_nav/frame_names.hpp"

namespace
{

bool ContainsWhitespace(const std::string & value)
{
  for (const unsigned char character : value) {
    if (std::isspace(character) != 0) {
      return true;
    }
  }

  return false;
}

}  // namespace

namespace savo_nav
{

ValidationResult GoalContextContract::Validate(
  const GoalContext & context)
{
  if (context.goal_id.empty()) {
    return {
      ValidationCode::kEmptyIdentifier,
      "goal_id_is_empty"
    };
  }

  if (ContainsWhitespace(context.goal_id)) {
    return {
      ValidationCode::kInvalidIdentifier,
      "goal_id_contains_whitespace"
    };
  }

  switch (context.source) {
    case GoalSource::kUnknown:
      return {
        ValidationCode::kInvalidState,
        "goal_source_is_unknown"
      };

    case GoalSource::kNavigation:
    case GoalSource::kExploration:
    case GoalSource::kWaypoint:
    case GoalSource::kArea:
    case GoalSource::kSemantic:
    case GoalSource::kOperator:
    case GoalSource::kSupervisor:
      break;

    default:
      return {
        ValidationCode::kInvalidState,
        "goal_source_is_invalid"
      };
  }

  if (context.target_frame != frames::kMap) {
    return {
      ValidationCode::kInvalidFrame,
      "goal_target_frame_must_be_map"
    };
  }

  if (context.sequence == 0) {
    return {
      ValidationCode::kInvalidCombination,
      "goal_sequence_must_be_nonzero"
    };
  }

  return {};
}

std::string_view GoalContextContract::ToString(
  const GoalSource source) noexcept
{
  switch (source) {
    case GoalSource::kUnknown:
      return "unknown";

    case GoalSource::kNavigation:
      return "navigation";

    case GoalSource::kExploration:
      return "exploration";

    case GoalSource::kWaypoint:
      return "waypoint";

    case GoalSource::kArea:
      return "area";

    case GoalSource::kSemantic:
      return "semantic";

    case GoalSource::kOperator:
      return "operator";

    case GoalSource::kSupervisor:
      return "supervisor";
  }

  return "unknown";
}

}  // namespace savo_nav
