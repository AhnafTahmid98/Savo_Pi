// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "savo_nav/goal_arbiter.hpp"
#include "savo_nav/goal_validator.hpp"

namespace savo_nav
{

enum class GoalGatewayState : std::uint8_t
{
  kIdle = 0,
  kReserved,
  kForwarding,
  kActive,
  kCanceling
};

struct GoalGatewayDecision
{
  bool accepted{false};
  bool state_changed{false};

  std::string reason{"not_evaluated"};

  ValidationCode validation_code{
    ValidationCode::kValid};

  GoalArbitrationCode arbitration_code{
    GoalArbitrationCode::kInvalidGoal};
};

struct GoalGatewaySnapshot
{
  GoalGatewayState state{GoalGatewayState::kIdle};

  std::string active_goal_id{};

  GoalSource source{GoalSource::kUnknown};

  std::uint64_t sequence{0};

  std::string reason{"idle"};
};

class GoalGateway
{
public:
  explicit GoalGateway(
    GoalValidationPolicy validation_policy = {},
    std::size_t recent_history_capacity = 32);

  [[nodiscard]] GoalGatewayDecision Admit(
    const GoalValidationRequest & request);

  [[nodiscard]] bool MarkForwarding(
    const std::string & goal_id,
    std::uint64_t sequence);

  [[nodiscard]] bool MarkAcceptedByNav2(
    const std::string & goal_id,
    std::uint64_t sequence);

  [[nodiscard]] GoalGatewayDecision RequestCancel(
    const std::string & goal_id,
    std::uint64_t sequence,
    std::string reason);

  [[nodiscard]] GoalGatewayDecision
  AcknowledgeCancellation(
    const std::string & goal_id,
    std::uint64_t sequence);

  [[nodiscard]] GoalGatewayDecision Complete(
    const std::string & goal_id,
    std::uint64_t sequence,
    std::string reason);

  [[nodiscard]] bool HasActiveGoal() const noexcept;

  [[nodiscard]] bool CancelRequested() const noexcept;

  [[nodiscard]] std::optional<GoalContext>
  ActiveGoal() const;

  [[nodiscard]] GoalGatewaySnapshot Snapshot() const;

  [[nodiscard]] static std::string_view ToString(
    GoalGatewayState state) noexcept;

private:
  void SetState(
    GoalGatewayState state,
    std::string reason);

  GoalValidationPolicy validation_policy_;
  GoalArbiter arbiter_;

  GoalGatewayState state_{GoalGatewayState::kIdle};
  std::string reason_{"idle"};
};

}  // namespace savo_nav
