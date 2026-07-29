// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "savo_nav/types.hpp"

namespace savo_nav
{

enum class GoalSource : std::uint8_t
{
  kUnknown = 0,
  kNavigation,
  kExploration,
  kWaypoint,
  kArea,
  kSemantic,
  kOperator,
  kSupervisor,
  kCoverage
};

struct GoalContext
{
  std::string goal_id{};
  GoalSource source{GoalSource::kUnknown};
  std::string target_frame{"map"};
  std::string map_id{};
  std::uint64_t sequence{0};
  bool allow_recovery{true};
  bool cancel_requested{false};
};

class GoalContextContract
{
public:
  [[nodiscard]] static ValidationResult Validate(
    const GoalContext & context);

  [[nodiscard]] static std::string_view ToString(
    GoalSource source) noexcept;
};

}  // namespace savo_nav
