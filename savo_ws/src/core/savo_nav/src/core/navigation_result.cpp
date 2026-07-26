// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/navigation_result.hpp"

#include <stdexcept>
#include <utility>

namespace savo_nav
{

ValidationResult NavigationResultContract::Validate(
  const NavigationResult & result)
{
  if (result.reason.empty()) {
    return {
      ValidationCode::kInvalidResult,
      "result_reason_is_empty"
    };
  }

  if (result.code == NavigationResultCode::kNone) {
    if (result.terminal || result.success) {
      return {
        ValidationCode::kInvalidResult,
        "pending_result_has_terminal_flags"
      };
    }

    return {};
  }

  if (result.goal_id.empty()) {
    return {
      ValidationCode::kEmptyIdentifier,
      "terminal_result_goal_id_is_empty"
    };
  }

  if (!result.terminal) {
    return {
      ValidationCode::kInvalidResult,
      "non_pending_result_must_be_terminal"
    };
  }

  if (result.code == NavigationResultCode::kSucceeded) {
    if (!result.success) {
      return {
        ValidationCode::kInvalidResult,
        "succeeded_result_must_set_success"
      };
    }

    return {};
  }

  if (result.success) {
    return {
      ValidationCode::kInvalidResult,
      "failure_result_must_not_set_success"
    };
  }

  return {};
}

std::string_view NavigationResultContract::ToString(
  const NavigationResultCode code) noexcept
{
  switch (code) {
    case NavigationResultCode::kNone:
      return "none";

    case NavigationResultCode::kSucceeded:
      return "succeeded";

    case NavigationResultCode::kRejectedNotReady:
      return "rejected_not_ready";

    case NavigationResultCode::kRejectedInvalidGoal:
      return "rejected_invalid_goal";

    case NavigationResultCode::kRejectedBusy:
      return "rejected_busy";

    case NavigationResultCode::kCanceledByClient:
      return "canceled_by_client";

    case NavigationResultCode::kCanceledBySafety:
      return "canceled_by_safety";

    case NavigationResultCode::kNav2Rejected:
      return "nav2_rejected";

    case NavigationResultCode::kNav2Aborted:
      return "nav2_aborted";

    case NavigationResultCode::kNav2TimedOut:
      return "nav2_timed_out";

    case NavigationResultCode::kControlUnavailable:
      return "control_unavailable";

    case NavigationResultCode::kLocalizationUnavailable:
      return "localization_unavailable";

    case NavigationResultCode::kMapUnavailable:
      return "map_unavailable";

    case NavigationResultCode::kInternalError:
      return "internal_error";
  }

  return "unknown";
}

NavigationResult NavigationResultContract::MakePending(
  std::string goal_id,
  std::string reason)
{
  NavigationResult result;

  result.goal_id = std::move(goal_id);
  result.reason = std::move(reason);

  return result;
}

NavigationResult NavigationResultContract::MakeTerminal(
  std::string goal_id,
  const NavigationResultCode code,
  std::string reason)
{
  if (code == NavigationResultCode::kNone) {
    throw std::invalid_argument(
            "terminal result code must not be none");
  }

  NavigationResult result;

  result.code = code;
  result.goal_id = std::move(goal_id);
  result.reason = std::move(reason);
  result.terminal = true;
  result.success =
    code == NavigationResultCode::kSucceeded;

  return result;
}

}  // namespace savo_nav
