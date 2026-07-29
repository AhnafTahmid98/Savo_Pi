// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/coverage_execution_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace savo_nav
{

CoverageExecutionModel::CoverageExecutionModel(
  CoverageExecutionPolicy policy)
: policy_(policy)
{
  if (!IsPolicyValid()) {
    throw std::invalid_argument(
            "coverage execution policy is invalid");
  }
}

bool CoverageExecutionModel::Start(
  const double now_seconds,
  const double requested_timeout_seconds)
{
  if (
    snapshot_.state != CoverageExecutionState::kIdle ||
    !IsTimeValid(now_seconds) ||
    !std::isfinite(requested_timeout_seconds) ||
    requested_timeout_seconds < 0.0)
  {
    return false;
  }

  const double resolved_timeout =
    requested_timeout_seconds == 0.0 ?
    policy_.default_execution_timeout_seconds :
    std::min(
    requested_timeout_seconds,
    policy_.maximum_execution_timeout_seconds);

  snapshot_ = {};
  snapshot_.state =
    CoverageExecutionState::kWaitingForBackend;

  snapshot_.reason = "waiting_for_backend";
  snapshot_.resolved_execution_timeout_seconds =
    resolved_timeout;

  started_at_seconds_ = now_seconds;
  last_event_seconds_ = now_seconds;
  last_feedback_seconds_ = now_seconds;

  return true;
}

bool CoverageExecutionModel::MarkBackendAccepted(
  const double now_seconds)
{
  if (
    snapshot_.state !=
    CoverageExecutionState::kWaitingForBackend ||
    !IsTimeValid(now_seconds) ||
    now_seconds < last_event_seconds_)
  {
    return false;
  }

  snapshot_.state = CoverageExecutionState::kExecuting;
  snapshot_.reason = "backend_executing";

  backend_accepted_ = true;
  backend_accepted_at_seconds_ = now_seconds;
  last_feedback_seconds_ = now_seconds;
  last_event_seconds_ = now_seconds;

  return true;
}

bool CoverageExecutionModel::MarkFeedback(
  const double now_seconds)
{
  if (
    (
      snapshot_.state !=
      CoverageExecutionState::kExecuting &&
      snapshot_.state !=
      CoverageExecutionState::kCanceling
    ) ||
    !IsTimeValid(now_seconds) ||
    now_seconds < last_feedback_seconds_)
  {
    return false;
  }

  last_feedback_seconds_ = now_seconds;
  last_event_seconds_ =
    std::max(last_event_seconds_, now_seconds);

  return true;
}

bool CoverageExecutionModel::RequestCancel(
  const double now_seconds,
  std::string reason)
{
  if (
    (
      snapshot_.state !=
      CoverageExecutionState::kWaitingForBackend &&
      snapshot_.state !=
      CoverageExecutionState::kExecuting
    ) ||
    snapshot_.cancel_requested ||
    !IsTimeValid(now_seconds) ||
    now_seconds < last_event_seconds_ ||
    reason.empty())
  {
    return false;
  }

  snapshot_.state = CoverageExecutionState::kCanceling;
  snapshot_.reason = reason;
  snapshot_.cancel_requested = true;

  cancellation_reason_ = std::move(reason);
  cancel_started_at_seconds_ = now_seconds;
  last_event_seconds_ = now_seconds;

  return true;
}

CoverageWatchdogEvent CoverageExecutionModel::CheckWatchdogs(
  const double now_seconds)
{
  if (
    !IsTimeValid(now_seconds) ||
    now_seconds < last_event_seconds_)
  {
    return CoverageWatchdogEvent::kNone;
  }

  if (
    snapshot_.state == CoverageExecutionState::kExecuting &&
    backend_accepted_)
  {
    const double execution_age =
      now_seconds - backend_accepted_at_seconds_;

    if (
      execution_age >
      snapshot_.resolved_execution_timeout_seconds)
    {
      static_cast<void>(
        RequestCancel(
          now_seconds,
          "execution_timeout"));

      return CoverageWatchdogEvent::kExecutionTimeout;
    }

    const double feedback_age =
      now_seconds - last_feedback_seconds_;

    if (
      feedback_age >
      policy_.feedback_stale_timeout_seconds)
    {
      static_cast<void>(
        RequestCancel(
          now_seconds,
          "feedback_stale"));

      return CoverageWatchdogEvent::kFeedbackStale;
    }
  }

  if (
    snapshot_.state == CoverageExecutionState::kCanceling &&
    now_seconds - cancel_started_at_seconds_ >
    policy_.cancel_timeout_seconds)
  {
    const bool feedback_timeout =
      cancellation_reason_ == "feedback_stale";

    SetPublicTerminal(
      feedback_timeout ?
      CoverageExecutionState::kFeedbackStale :
      CoverageExecutionState::kTimedOut,
      feedback_timeout ?
      CoverageTerminalCode::kFeedbackStale :
      CoverageTerminalCode::kTimedOut,
      cancellation_reason_ + "_cancel_timeout",
      false);

    snapshot_.backend_terminal_pending = true;

    return CoverageWatchdogEvent::kCancelTimeout;
  }

  return CoverageWatchdogEvent::kNone;
}

bool CoverageExecutionModel::FailBeforeBackend(
  const CoverageTerminalCode code,
  std::string reason)
{
  if (
    snapshot_.state !=
    CoverageExecutionState::kWaitingForBackend ||
    code == CoverageTerminalCode::kNone ||
    code == CoverageTerminalCode::kSucceeded ||
    reason.empty())
  {
    return false;
  }

  const bool rejected =
    code == CoverageTerminalCode::kInvalidRequest ||
    code == CoverageTerminalCode::kNotReady ||
    code == CoverageTerminalCode::kBusy ||
    code == CoverageTerminalCode::kBackendRejected;

  SetPublicTerminal(
    rejected ?
    CoverageExecutionState::kRejected :
    CoverageExecutionState::kFailed,
    code,
    std::move(reason),
    true);

  return true;
}

bool CoverageExecutionModel::ObserveBackendTerminal(
  const CoverageBackendTerminal terminal,
  std::string reason)
{
  if (reason.empty()) {
    reason = "backend_terminal";
  }

  if (snapshot_.backend_terminal_pending) {
    snapshot_.backend_terminal_pending = false;
    snapshot_.release_ownership = true;
    return true;
  }

  if (
    snapshot_.public_terminal ||
    snapshot_.state == CoverageExecutionState::kIdle)
  {
    return false;
  }

  if (terminal == CoverageBackendTerminal::kSucceeded) {
    SetPublicTerminal(
      CoverageExecutionState::kSucceeded,
      CoverageTerminalCode::kSucceeded,
      std::move(reason),
      true);

    return true;
  }

  if (terminal == CoverageBackendTerminal::kRejected) {
    SetPublicTerminal(
      CoverageExecutionState::kRejected,
      CoverageTerminalCode::kBackendRejected,
      std::move(reason),
      true);

    return true;
  }

  if (terminal == CoverageBackendTerminal::kCanceled) {
    if (cancellation_reason_ == "execution_timeout") {
      SetPublicTerminal(
        CoverageExecutionState::kTimedOut,
        CoverageTerminalCode::kTimedOut,
        "execution_timeout",
        true);
    } else if (cancellation_reason_ == "feedback_stale") {
      SetPublicTerminal(
        CoverageExecutionState::kFeedbackStale,
        CoverageTerminalCode::kFeedbackStale,
        "feedback_stale",
        true);
    } else {
      SetPublicTerminal(
        CoverageExecutionState::kCanceled,
        CoverageTerminalCode::kCanceled,
        std::move(reason),
        true);
    }

    return true;
  }

  if (terminal == CoverageBackendTerminal::kAborted) {
    SetPublicTerminal(
      CoverageExecutionState::kFailed,
      CoverageTerminalCode::kAborted,
      std::move(reason),
      true);

    return true;
  }

  SetPublicTerminal(
    CoverageExecutionState::kFailed,
    CoverageTerminalCode::kInternalError,
    std::move(reason),
    true);

  return true;
}

void CoverageExecutionModel::Reset() noexcept
{
  snapshot_ = {};

  started_at_seconds_ = 0.0;
  backend_accepted_at_seconds_ = 0.0;
  last_feedback_seconds_ = 0.0;
  cancel_started_at_seconds_ = 0.0;
  last_event_seconds_ = 0.0;

  cancellation_reason_.clear();
  backend_accepted_ = false;
}

const CoverageExecutionSnapshot &
CoverageExecutionModel::Snapshot() const noexcept
{
  return snapshot_;
}

bool CoverageExecutionModel::IsPolicyValid() const noexcept
{
  return
    std::isfinite(
      policy_.default_execution_timeout_seconds) &&
    policy_.default_execution_timeout_seconds > 0.0 &&
    std::isfinite(
      policy_.maximum_execution_timeout_seconds) &&
    policy_.maximum_execution_timeout_seconds >=
    policy_.default_execution_timeout_seconds &&
    std::isfinite(
      policy_.feedback_stale_timeout_seconds) &&
    policy_.feedback_stale_timeout_seconds > 0.0 &&
    std::isfinite(policy_.cancel_timeout_seconds) &&
    policy_.cancel_timeout_seconds > 0.0;
}

bool CoverageExecutionModel::IsTimeValid(
  const double value) const noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

void CoverageExecutionModel::SetPublicTerminal(
  const CoverageExecutionState state,
  const CoverageTerminalCode code,
  std::string reason,
  const bool release_ownership)
{
  snapshot_.state = state;
  snapshot_.terminal_code = code;
  snapshot_.reason = std::move(reason);
  snapshot_.public_terminal = true;
  snapshot_.release_ownership = release_ownership;
}

std::string_view CoverageExecutionModel::ToString(
  const CoverageExecutionState state) noexcept
{
  switch (state) {
    case CoverageExecutionState::kIdle:
      return "idle";

    case CoverageExecutionState::kWaitingForBackend:
      return "waiting_for_backend";

    case CoverageExecutionState::kExecuting:
      return "executing";

    case CoverageExecutionState::kCanceling:
      return "canceling";

    case CoverageExecutionState::kSucceeded:
      return "succeeded";

    case CoverageExecutionState::kRejected:
      return "rejected";

    case CoverageExecutionState::kFailed:
      return "failed";

    case CoverageExecutionState::kCanceled:
      return "canceled";

    case CoverageExecutionState::kTimedOut:
      return "timed_out";

    case CoverageExecutionState::kFeedbackStale:
      return "feedback_stale";
  }

  return "unknown";
}

std::string_view CoverageExecutionModel::ToString(
  const CoverageTerminalCode code) noexcept
{
  switch (code) {
    case CoverageTerminalCode::kNone:
      return "none";

    case CoverageTerminalCode::kSucceeded:
      return "succeeded";

    case CoverageTerminalCode::kInvalidRequest:
      return "invalid_request";

    case CoverageTerminalCode::kNotReady:
      return "not_ready";

    case CoverageTerminalCode::kBusy:
      return "busy";

    case CoverageTerminalCode::kBackendUnavailable:
      return "backend_unavailable";

    case CoverageTerminalCode::kBackendRejected:
      return "backend_rejected";

    case CoverageTerminalCode::kAborted:
      return "aborted";

    case CoverageTerminalCode::kCanceled:
      return "canceled";

    case CoverageTerminalCode::kTimedOut:
      return "timed_out";

    case CoverageTerminalCode::kFeedbackStale:
      return "feedback_stale";

    case CoverageTerminalCode::kInternalError:
      return "internal_error";
  }

  return "unknown";
}

}  // namespace savo_nav
