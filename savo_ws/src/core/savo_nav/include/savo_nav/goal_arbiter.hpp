// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <string_view>

#include "savo_nav/goal_context.hpp"
#include "savo_nav/types.hpp"

namespace savo_nav
{

enum class GoalArbitrationCode : std::uint8_t
{
  kAccepted = 0,
  kInvalidGoal,
  kBusy,
  kDuplicateGoal,
  kStaleSequence,
  kNoActiveGoal,
  kGoalMismatch,
  kCancelRequested,
  kCancelAlreadyRequested,
  kCancelNotRequested,
  kCancelAcknowledged,
  kCompleted
};

struct GoalArbitrationDecision
{
  GoalArbitrationCode code{
    GoalArbitrationCode::kInvalidGoal};

  bool accepted{false};

  bool state_changed{false};

  std::string reason{"not_evaluated"};

  std::string active_goal_id{};
};

class GoalArbiter
{
public:
  explicit GoalArbiter(
    std::size_t recent_history_capacity = 32);

  [[nodiscard]] GoalArbitrationDecision TryAcquire(
    const GoalContext & context,
    const ValidationResult & validation);

  [[nodiscard]] GoalArbitrationDecision RequestCancel(
    const std::string & goal_id,
    std::uint64_t sequence);

  [[nodiscard]] GoalArbitrationDecision
  AcknowledgeCancellation(
    const std::string & goal_id,
    std::uint64_t sequence);

  [[nodiscard]] GoalArbitrationDecision Complete(
    const std::string & goal_id,
    std::uint64_t sequence);

  [[nodiscard]] bool HasActiveGoal() const noexcept;

  [[nodiscard]] bool CancelRequested() const noexcept;

  [[nodiscard]] std::optional<GoalContext>
  ActiveGoal() const;

  [[nodiscard]] bool HasSeenGoalId(
    const std::string & goal_id) const;

  [[nodiscard]] std::size_t RecentHistorySize()
  const noexcept;

  [[nodiscard]] static int SourcePriority(
    GoalSource source) noexcept;

  [[nodiscard]] static std::string_view ToString(
    GoalArbitrationCode code) noexcept;

private:
  static constexpr std::size_t kGoalSourceCount = 9;

  [[nodiscard]] static std::size_t SourceIndex(
    GoalSource source) noexcept;

  [[nodiscard]] bool MatchesActive(
    const std::string & goal_id,
    std::uint64_t sequence) const noexcept;

  void RememberAndReleaseActive();

  std::size_t recent_history_capacity_;

  std::optional<GoalContext> active_goal_{};

  std::deque<GoalContext> recent_history_{};

  std::array<std::uint64_t, kGoalSourceCount>
  last_sequence_by_source_{};
};

}  // namespace savo_nav
