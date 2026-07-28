// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/goal_admission_policy.hpp"

namespace savo_nav
{

GoalAdmissionDecision GoalAdmissionPolicy::Evaluate(
  const GoalAdmissionInput & input)
{
  GoalAdmissionDecision decision;

  if (!input.guard_observed) {
    decision.reason =
      "control_recovery_guard_unobserved";

    decision.request_active_cancel =
      input.active_goal &&
      !input.cancellation_requested;

    return decision;
  }

  if (!input.guard_fresh) {
    decision.reason =
      "control_recovery_guard_stale";

    decision.request_active_cancel =
      input.active_goal &&
      !input.cancellation_requested;

    return decision;
  }

  if (!input.guard_allowed) {
    decision.reason =
      input.guard_reason.empty() ?
      "control_recovery_guard_blocked" :
      input.guard_reason;

    decision.request_active_cancel =
      input.active_goal &&
      !input.cancellation_requested;

    return decision;
  }

  if (input.active_goal || input.slot_reserved) {
    decision.reason = "goal_gateway_busy";
    return decision;
  }

  decision.accept_new_goal = true;
  decision.reason = "goal_admission_allowed";

  return decision;
}

}  // namespace savo_nav
