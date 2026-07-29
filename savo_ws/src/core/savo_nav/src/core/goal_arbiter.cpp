// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/goal_arbiter.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace
{

savo_nav::GoalArbitrationDecision MakeDecision(
  const savo_nav::GoalArbitrationCode code,
  const bool accepted,
  const bool state_changed,
  std::string reason,
  std::string active_goal_id)
{
  savo_nav::GoalArbitrationDecision decision;

  decision.code = code;
  decision.accepted = accepted;
  decision.state_changed = state_changed;
  decision.reason = std::move(reason);
  decision.active_goal_id =
    std::move(active_goal_id);

  return decision;
}

}  // namespace

namespace savo_nav
{

GoalArbiter::GoalArbiter(
  const std::size_t recent_history_capacity)
: recent_history_capacity_(recent_history_capacity)
{
  if (recent_history_capacity_ == 0) {
    throw std::invalid_argument(
            "recent goal history capacity must be positive");
  }
}

GoalArbitrationDecision GoalArbiter::TryAcquire(
  const GoalContext & context,
  const ValidationResult & validation)
{
  if (!validation.IsValid()) {
    return MakeDecision(
      GoalArbitrationCode::kInvalidGoal,
      false,
      false,
      validation.reason,
      active_goal_ ?
      active_goal_->goal_id :
      std::string{});
  }

  if (HasSeenGoalId(context.goal_id)) {
    return MakeDecision(
      GoalArbitrationCode::kDuplicateGoal,
      false,
      false,
      "goal_id_was_already_seen",
      active_goal_ ?
      active_goal_->goal_id :
      std::string{});
  }

  const std::size_t source_index =
    SourceIndex(context.source);

  if (
    context.sequence <=
    last_sequence_by_source_[source_index])
  {
    return MakeDecision(
      GoalArbitrationCode::kStaleSequence,
      false,
      false,
      "goal_sequence_is_not_newer_for_source",
      active_goal_ ?
      active_goal_->goal_id :
      std::string{});
  }

  if (active_goal_) {
    const int incoming_priority =
      SourcePriority(context.source);

    const int active_priority =
      SourcePriority(active_goal_->source);

    const std::string reason =
      incoming_priority > active_priority ?
      "active_goal_preemption_forbidden" :
      "active_goal_has_equal_or_higher_priority";

    return MakeDecision(
      GoalArbitrationCode::kBusy,
      false,
      false,
      reason,
      active_goal_->goal_id);
  }

  active_goal_ = context;

  last_sequence_by_source_[source_index] =
    context.sequence;

  return MakeDecision(
    GoalArbitrationCode::kAccepted,
    true,
    true,
    "goal_ownership_acquired",
    context.goal_id);
}

GoalArbitrationDecision GoalArbiter::RequestCancel(
  const std::string & goal_id,
  const std::uint64_t sequence)
{
  if (!active_goal_) {
    return MakeDecision(
      GoalArbitrationCode::kNoActiveGoal,
      false,
      false,
      "no_active_goal_to_cancel",
      {});
  }

  if (!MatchesActive(goal_id, sequence)) {
    return MakeDecision(
      GoalArbitrationCode::kGoalMismatch,
      false,
      false,
      "cancel_request_does_not_match_active_goal",
      active_goal_->goal_id);
  }

  if (active_goal_->cancel_requested) {
    return MakeDecision(
      GoalArbitrationCode::kCancelAlreadyRequested,
      false,
      false,
      "cancel_was_already_requested",
      active_goal_->goal_id);
  }

  active_goal_->cancel_requested = true;

  return MakeDecision(
    GoalArbitrationCode::kCancelRequested,
    true,
    true,
    "cancel_request_recorded_waiting_for_acknowledgement",
    active_goal_->goal_id);
}

GoalArbitrationDecision
GoalArbiter::AcknowledgeCancellation(
  const std::string & goal_id,
  const std::uint64_t sequence)
{
  if (!active_goal_) {
    return MakeDecision(
      GoalArbitrationCode::kNoActiveGoal,
      false,
      false,
      "no_active_goal_for_cancel_acknowledgement",
      {});
  }

  if (!MatchesActive(goal_id, sequence)) {
    return MakeDecision(
      GoalArbitrationCode::kGoalMismatch,
      false,
      false,
      "cancel_acknowledgement_does_not_match_active_goal",
      active_goal_->goal_id);
  }

  if (!active_goal_->cancel_requested) {
    return MakeDecision(
      GoalArbitrationCode::kCancelNotRequested,
      false,
      false,
      "cancel_acknowledgement_without_request",
      active_goal_->goal_id);
  }

  const std::string completed_goal_id =
    active_goal_->goal_id;

  RememberAndReleaseActive();

  return MakeDecision(
    GoalArbitrationCode::kCancelAcknowledged,
    true,
    true,
    "cancel_acknowledged_goal_released",
    completed_goal_id);
}

GoalArbitrationDecision GoalArbiter::Complete(
  const std::string & goal_id,
  const std::uint64_t sequence)
{
  if (!active_goal_) {
    return MakeDecision(
      GoalArbitrationCode::kNoActiveGoal,
      false,
      false,
      "no_active_goal_to_complete",
      {});
  }

  if (!MatchesActive(goal_id, sequence)) {
    return MakeDecision(
      GoalArbitrationCode::kGoalMismatch,
      false,
      false,
      "completion_does_not_match_active_goal",
      active_goal_->goal_id);
  }

  const std::string completed_goal_id =
    active_goal_->goal_id;

  RememberAndReleaseActive();

  return MakeDecision(
    GoalArbitrationCode::kCompleted,
    true,
    true,
    "goal_completed_and_released",
    completed_goal_id);
}

bool GoalArbiter::HasActiveGoal() const noexcept
{
  return active_goal_.has_value();
}

bool GoalArbiter::CancelRequested() const noexcept
{
  return
    active_goal_.has_value() &&
    active_goal_->cancel_requested;
}

std::optional<GoalContext>
GoalArbiter::ActiveGoal() const
{
  return active_goal_;
}

bool GoalArbiter::HasSeenGoalId(
  const std::string & goal_id) const
{
  if (
    active_goal_ &&
    active_goal_->goal_id == goal_id)
  {
    return true;
  }

  return std::any_of(
    recent_history_.begin(),
    recent_history_.end(),
    [&goal_id](const GoalContext & context)
    {
      return context.goal_id == goal_id;
    });
}

std::size_t GoalArbiter::RecentHistorySize()
const noexcept
{
  return recent_history_.size();
}

int GoalArbiter::SourcePriority(
  const GoalSource source) noexcept
{
  switch (source) {
    case GoalSource::kSupervisor:
      return 700;

    case GoalSource::kOperator:
      return 600;

    case GoalSource::kNavigation:
      return 500;

    case GoalSource::kWaypoint:
      return 400;

    case GoalSource::kArea:
      return 300;

    case GoalSource::kSemantic:
      return 200;

    case GoalSource::kCoverage:
      return 150;

    case GoalSource::kExploration:
      return 100;

    case GoalSource::kUnknown:
      return 0;
  }

  return 0;
}

std::string_view GoalArbiter::ToString(
  const GoalArbitrationCode code) noexcept
{
  switch (code) {
    case GoalArbitrationCode::kAccepted:
      return "accepted";

    case GoalArbitrationCode::kInvalidGoal:
      return "invalid_goal";

    case GoalArbitrationCode::kBusy:
      return "busy";

    case GoalArbitrationCode::kDuplicateGoal:
      return "duplicate_goal";

    case GoalArbitrationCode::kStaleSequence:
      return "stale_sequence";

    case GoalArbitrationCode::kNoActiveGoal:
      return "no_active_goal";

    case GoalArbitrationCode::kGoalMismatch:
      return "goal_mismatch";

    case GoalArbitrationCode::kCancelRequested:
      return "cancel_requested";

    case GoalArbitrationCode::kCancelAlreadyRequested:
      return "cancel_already_requested";

    case GoalArbitrationCode::kCancelNotRequested:
      return "cancel_not_requested";

    case GoalArbitrationCode::kCancelAcknowledged:
      return "cancel_acknowledged";

    case GoalArbitrationCode::kCompleted:
      return "completed";
  }

  return "unknown";
}

std::size_t GoalArbiter::SourceIndex(
  const GoalSource source) noexcept
{
  const auto index =
    static_cast<std::size_t>(source);

  if (index >= kGoalSourceCount) {
    return 0;
  }

  return index;
}

bool GoalArbiter::MatchesActive(
  const std::string & goal_id,
  const std::uint64_t sequence) const noexcept
{
  return
    active_goal_.has_value() &&
    active_goal_->goal_id == goal_id &&
    active_goal_->sequence == sequence;
}

void GoalArbiter::RememberAndReleaseActive()
{
  if (!active_goal_) {
    return;
  }

  recent_history_.push_back(*active_goal_);

  while (
    recent_history_.size() >
    recent_history_capacity_)
  {
    recent_history_.pop_front();
  }

  active_goal_.reset();
}

}  // namespace savo_nav
