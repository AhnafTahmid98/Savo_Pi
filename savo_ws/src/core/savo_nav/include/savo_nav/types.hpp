// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace savo_nav
{

enum class NavigationReadinessState : std::uint8_t
{
  kOffline = 0,
  kStarting,
  kWaitingForMap,
  kWaitingForTf,
  kWaitingForLocalization,
  kWaitingForLidar,
  kWaitingForPointCloud,
  kWaitingForNav2,
  kWaitingForCostmaps,
  kReady,
  kDegraded,
  kBlocked,
  kFault
};

struct NavigationReadinessResult
{
  NavigationReadinessState state{
    NavigationReadinessState::kStarting};

  bool goal_acceptance_allowed{false};

  std::string reason{"initializing"};

  std::vector<std::string> failed_dependencies{};
};

enum class ValidationCode : std::uint8_t
{
  kValid = 0,
  kEmptyIdentifier,
  kInvalidIdentifier,
  kInvalidFrame,
  kInvalidState,
  kInvalidCombination,
  kMissingMapId,
  kInvalidAuthority,
  kInvalidResult,
  kNotReady,
  kInvalidPose,
  kCoordinateOutOfBounds,
  kMapUnavailable,
  kMapMismatch,
  kCancellationRequested
};

struct ValidationResult
{
  ValidationCode code{ValidationCode::kValid};
  std::string reason{"valid"};

  [[nodiscard]] bool IsValid() const noexcept;
};

[[nodiscard]] std::string_view ToString(
  ValidationCode code) noexcept;

}  // namespace savo_nav
