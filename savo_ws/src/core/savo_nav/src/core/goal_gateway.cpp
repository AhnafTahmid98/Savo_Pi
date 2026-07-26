// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/goal_gateway.hpp"

#include <utility>

namespace savo_nav
{

GoalGateway::GoalGateway(
  GoalValidationPolicy validation_policy,
  const std::size_t recent_history_capacity)
: validation_policy_(std::move(validation_policy)),
  arbiter_(recent_history_capacity)
{
}

GoalGatewayDecision GoalGateway::Admit(
  const GoalValidationRequest & request)
{
  const auto validation =
    GoalValidator::Validate(
    request,
    validation_policy_);

  const auto arbitration =
    arbiter_.TryAcquire(
    request.context,
    validation);

  GoalGatewayDecision decision;

  decision.accepted = arbitration.accepted;
  decision.state_changed =
    arbitration.state_changed;

  decision.reason = arbitration.reason;
  decision.validation_code = validation.code;

  decision.arbitration_code =
    arbitration.code;

  if (decision.accepted) {
    SetState(
      GoalGatewayState::kReserved,
      "goal_reserved_for_nav2_forwarding");
  }

  return decision;
}

bool GoalGateway::MarkForwarding(
  const std::string & goal_id,
  const std::uint64_t sequence)
{
  const auto active = arbiter_.ActiveGoal();

  if (
    !active ||
    active->goal_id != goal_id ||
    active->sequence != sequence ||
    state_ != GoalGatewayState::kReserved)
  {
    return false;
  }

  SetState(
    GoalGatewayState::kForwarding,
    "waiting_for_nav2_goal_response");

  return true;
}

bool GoalGateway::MarkAcceptedByNav2(
  const std::string & goal_id,
  const std::uint64_t sequence)
{
  const auto active = arbiter_.ActiveGoal();

  if (
    !active ||
    active->goal_id != goal_id ||
    active->sequence != sequence)
  {
    return false;
  }

  if (state_ == GoalGatewayState::kCanceling) {
    reason_ =
      "nav2_accepted_goal_while_cancel_pending";

    return true;
  }

  if (state_ != GoalGatewayState::kForwarding) {
    return false;
  }

  SetState(
    GoalGatewayState::kActive,
    "nav2_goal_active");

  return true;
}

GoalGatewayDecision GoalGateway::RequestCancel(
  const std::string & goal_id,
  const std::uint64_t sequence,
  std::string reason)
{
  const auto arbitration =
    arbiter_.RequestCancel(
    goal_id,
    sequence);

  GoalGatewayDecision decision;

  decision.accepted = arbitration.accepted;
  decision.state_changed =
    arbitration.state_changed;

  decision.reason = arbitration.reason;
  decision.arbitration_code =
    arbitration.code;

  if (decision.accepted) {
    SetState(
      GoalGatewayState::kCanceling,
      std::move(reason));
  }

  return decision;
}

GoalGatewayDecision
GoalGateway::AcknowledgeCancellation(
  const std::string & goal_id,
  const std::uint64_t sequence)
{
  const auto arbitration =
    arbiter_.AcknowledgeCancellation(
    goal_id,
    sequence);

  GoalGatewayDecision decision;

  decision.accepted = arbitration.accepted;
  decision.state_changed =
    arbitration.state_changed;

  decision.reason = arbitration.reason;
  decision.arbitration_code =
    arbitration.code;

  if (decision.accepted) {
    SetState(
      GoalGatewayState::kIdle,
      "cancel_acknowledged_goal_released");
  }

  return decision;
}

GoalGatewayDecision GoalGateway::Complete(
  const std::string & goal_id,
  const std::uint64_t sequence,
  std::string reason)
{
  const auto arbitration =
    arbiter_.Complete(
    goal_id,
    sequence);

  GoalGatewayDecision decision;

  decision.accepted = arbitration.accepted;
  decision.state_changed =
    arbitration.state_changed;

  decision.reason = arbitration.reason;
  decision.arbitration_code =
    arbitration.code;

  if (decision.accepted) {
    SetState(
      GoalGatewayState::kIdle,
      std::move(reason));
  }

  return decision;
}

bool GoalGateway::HasActiveGoal() const noexcept
{
  return arbiter_.HasActiveGoal();
}

bool GoalGateway::CancelRequested() const noexcept
{
  return arbiter_.CancelRequested();
}

std::optional<GoalContext>
GoalGateway::ActiveGoal() const
{
  return arbiter_.ActiveGoal();
}

GoalGatewaySnapshot GoalGateway::Snapshot() const
{
  GoalGatewaySnapshot snapshot;

  snapshot.state = state_;
  snapshot.reason = reason_;

  const auto active = arbiter_.ActiveGoal();

  if (active) {
    snapshot.active_goal_id =
      active->goal_id;

    snapshot.source = active->source;
    snapshot.sequence = active->sequence;
  }

  return snapshot;
}

std::string_view GoalGateway::ToString(
  const GoalGatewayState state) noexcept
{
  switch (state) {
    case GoalGatewayState::kIdle:
      return "idle";

    case GoalGatewayState::kReserved:
      return "reserved";

    case GoalGatewayState::kForwarding:
      return "forwarding";

    case GoalGatewayState::kActive:
      return "active";

    case GoalGatewayState::kCanceling:
      return "canceling";
  }

  return "unknown";
}

void GoalGateway::SetState(
  const GoalGatewayState state,
  std::string reason)
{
  state_ = state;
  reason_ = std::move(reason);
}

}  // namespace savo_nav
