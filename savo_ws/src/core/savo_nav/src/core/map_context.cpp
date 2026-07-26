// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/map_context.hpp"

#include "savo_nav/frame_names.hpp"

namespace savo_nav
{

ValidationResult MapContextContract::Validate(
  const MapContext & context)
{
  if (context.frame_id != frames::kMap) {
    return {
      ValidationCode::kInvalidFrame,
      "map_context_frame_must_be_map"
    };
  }

  if (!context.available) {
    const bool unavailable_state_is_clean =
      context.mode == NavigationMapMode::kUnknown &&
      context.authority == MapToOdomAuthority::kNone &&
      context.map_id.empty() &&
      context.revision == 0 &&
      !context.localization_ready &&
      !context.mapping_active;

    if (!unavailable_state_is_clean) {
      return {
        ValidationCode::kInvalidCombination,
        "unavailable_map_context_contains_active_state"
      };
    }

    return {};
  }

  if (context.revision == 0) {
    return {
      ValidationCode::kInvalidCombination,
      "available_map_revision_must_be_nonzero"
    };
  }

  if (context.mode == NavigationMapMode::kSavedMap) {
    if (context.map_id.empty()) {
      return {
        ValidationCode::kMissingMapId,
        "saved_map_requires_map_id"
      };
    }

    if (context.authority != MapToOdomAuthority::kAmcl) {
      return {
        ValidationCode::kInvalidAuthority,
        "saved_map_requires_amcl_authority"
      };
    }

    if (
      !context.localization_ready ||
      context.mapping_active)
    {
      return {
        ValidationCode::kInvalidCombination,
        "saved_map_state_is_inconsistent"
      };
    }

    return {};
  }

  if (context.mode == NavigationMapMode::kLiveMapping) {
    if (
      context.authority !=
      MapToOdomAuthority::kSlamToolbox)
    {
      return {
        ValidationCode::kInvalidAuthority,
        "live_mapping_requires_slam_toolbox_authority"
      };
    }

    if (
      !context.localization_ready ||
      !context.mapping_active)
    {
      return {
        ValidationCode::kInvalidCombination,
        "live_mapping_state_is_inconsistent"
      };
    }

    return {};
  }

  return {
    ValidationCode::kInvalidState,
    "available_map_mode_is_unknown"
  };
}

std::string_view MapContextContract::ToString(
  const NavigationMapMode mode) noexcept
{
  switch (mode) {
    case NavigationMapMode::kUnknown:
      return "unknown";

    case NavigationMapMode::kSavedMap:
      return "saved_map";

    case NavigationMapMode::kLiveMapping:
      return "live_mapping";
  }

  return "unknown";
}

std::string_view MapContextContract::ToString(
  const MapToOdomAuthority authority) noexcept
{
  switch (authority) {
    case MapToOdomAuthority::kNone:
      return "none";

    case MapToOdomAuthority::kAmcl:
      return "amcl";

    case MapToOdomAuthority::kSlamToolbox:
      return "slam_toolbox";
  }

  return "unknown";
}

}  // namespace savo_nav
