// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: Apache-2.0

#pragma once

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

[[nodiscard]] constexpr bool IsValidDuty(const std::uint8_t value)
{
  return value == static_cast<std::uint8_t>(Duty::kRegisterLocation) ||
         value == static_cast<std::uint8_t>(Duty::kConfirmArrival);
}

}  // namespace savo_head::apriltag_contract
