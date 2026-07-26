// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/types.hpp"

namespace savo_nav
{

bool ValidationResult::IsValid() const noexcept
{
  return code == ValidationCode::kValid;
}

std::string_view ToString(
  const ValidationCode code) noexcept
{
  switch (code) {
    case ValidationCode::kValid:
      return "valid";

    case ValidationCode::kEmptyIdentifier:
      return "empty_identifier";

    case ValidationCode::kInvalidIdentifier:
      return "invalid_identifier";

    case ValidationCode::kInvalidFrame:
      return "invalid_frame";

    case ValidationCode::kInvalidState:
      return "invalid_state";

    case ValidationCode::kInvalidCombination:
      return "invalid_combination";

    case ValidationCode::kMissingMapId:
      return "missing_map_id";

    case ValidationCode::kInvalidAuthority:
      return "invalid_authority";

    case ValidationCode::kInvalidResult:
      return "invalid_result";

    case ValidationCode::kNotReady:
      return "not_ready";

    case ValidationCode::kInvalidPose:
      return "invalid_pose";

    case ValidationCode::kCoordinateOutOfBounds:
      return "coordinate_out_of_bounds";

    case ValidationCode::kMapUnavailable:
      return "map_unavailable";

    case ValidationCode::kMapMismatch:
      return "map_mismatch";

    case ValidationCode::kCancellationRequested:
      return "cancellation_requested";
  }

  return "unknown";
}

}  // namespace savo_nav
