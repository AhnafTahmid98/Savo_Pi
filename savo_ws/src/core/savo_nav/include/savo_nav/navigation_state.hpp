// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string_view>

namespace savo_nav
{

enum class NavigationState : std::uint8_t
{
  kOffline = 0,
  kStarting,
  kIdle,
  kValidatingGoal,
  kWaitingForControl,
  kPlanning,
  kNavigating,
  kRecovering,
  kCanceling,
  kSucceeded,
  kCanceled,
  kRejected,
  kFailed
};

class NavigationStateContract
{
public:
  [[nodiscard]] static std::string_view ToString(
    NavigationState state) noexcept;

  [[nodiscard]] static bool IsTerminal(
    NavigationState state) noexcept;

  [[nodiscard]] static bool AcceptsNewGoal(
    NavigationState state) noexcept;

  [[nodiscard]] static bool HasActiveGoal(
    NavigationState state) noexcept;

  [[nodiscard]] static bool IsTransitionAllowed(
    NavigationState from,
    NavigationState to) noexcept;
};

}  // namespace savo_nav
