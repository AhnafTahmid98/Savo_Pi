// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace savo_head::apriltag_contract
{

inline constexpr std::string_view kTypedObservationTopic{
  "/savo_head/apriltag/observations"};

inline constexpr std::string_view kConfirmAction{
  "/savo_head/apriltag/confirm"};

enum class Duty : std::uint8_t
{
  kRegisterLocation = 1U,
  kConfirmArrival = 2U,
};

enum class IdentityEvidenceDisposition : std::uint8_t
{
  kAccepted = 0U,
  kWrongTag = 1U,
  kUnstable = 2U,
};

[[nodiscard]] constexpr bool IsValidDuty(const std::uint8_t value)
{
  return value == static_cast<std::uint8_t>(Duty::kRegisterLocation) ||
         value == static_cast<std::uint8_t>(Duty::kConfirmArrival);
}

[[nodiscard]] constexpr bool RequiresSpatialEvidence(
  const std::uint8_t duty,
  const bool require_map_pose)
{
  return duty == static_cast<std::uint8_t>(Duty::kRegisterLocation) ||
         require_map_pose;
}

[[nodiscard]] constexpr bool IsIdentityOnlyArrival(
  const std::uint8_t duty,
  const bool require_map_pose)
{
  return duty == static_cast<std::uint8_t>(Duty::kConfirmArrival) &&
         !require_map_pose;
}

[[nodiscard]] constexpr IdentityEvidenceDisposition ClassifyIdentityEvidence(
  const bool family_matches,
  const bool tag_matches,
  const bool observation_is_fresh,
  const bool detection_quality_is_acceptable,
  const bool hamming_distance_is_acceptable)
{
  if (!family_matches || !tag_matches) {
    return IdentityEvidenceDisposition::kWrongTag;
  }
  if (!observation_is_fresh ||
    !detection_quality_is_acceptable ||
    !hamming_distance_is_acceptable)
  {
    return IdentityEvidenceDisposition::kUnstable;
  }
  return IdentityEvidenceDisposition::kAccepted;
}

[[nodiscard]] constexpr bool HasMinimumEvidence(
  const std::size_t accepted_observations,
  const std::size_t minimum_observations)
{
  return minimum_observations > 0U &&
         accepted_observations >= minimum_observations;
}

}  // namespace savo_head::apriltag_contract
