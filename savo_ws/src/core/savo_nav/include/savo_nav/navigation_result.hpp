// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "savo_nav/types.hpp"

namespace savo_nav
{

enum class NavigationResultCode : std::uint8_t
{
  kNone = 0,
  kSucceeded,
  kRejectedNotReady,
  kRejectedInvalidGoal,
  kRejectedBusy,
  kCanceledByClient,
  kCanceledBySafety,
  kNav2Rejected,
  kNav2Aborted,
  kNav2TimedOut,
  kControlUnavailable,
  kLocalizationUnavailable,
  kMapUnavailable,
  kInternalError
};

struct NavigationResult
{
  NavigationResultCode code{NavigationResultCode::kNone};
  std::string goal_id{};
  std::string reason{"not_set"};
  bool terminal{false};
  bool success{false};
};

class NavigationResultContract
{
public:
  [[nodiscard]] static ValidationResult Validate(
    const NavigationResult & result);

  [[nodiscard]] static std::string_view ToString(
    NavigationResultCode code) noexcept;

  [[nodiscard]] static NavigationResult MakePending(
    std::string goal_id,
    std::string reason);

  [[nodiscard]] static NavigationResult MakeTerminal(
    std::string goal_id,
    NavigationResultCode code,
    std::string reason);
};

}  // namespace savo_nav
