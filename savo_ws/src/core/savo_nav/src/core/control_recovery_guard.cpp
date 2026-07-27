// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/control_recovery_guard.hpp"

namespace savo_nav
{

ControlRecoveryDecision
ControlRecoveryGuard::Evaluate(
  const ControlModeObservation & control,
  const RecoveryObservation & recovery)
{
  ControlRecoveryDecision decision;

  decision.control = control;
  decision.recovery = recovery;

  if (!control.navigation_allowed) {
    decision.reason = control.reason_code;
    return decision;
  }

  if (!recovery.navigation_allowed) {
    decision.reason = recovery.reason_code;
    return decision;
  }

  decision.navigation_allowed = true;
  decision.cancel_active_goal = false;

  decision.reason =
    "control_recovery_ready";

  return decision;
}

}  // namespace savo_nav
