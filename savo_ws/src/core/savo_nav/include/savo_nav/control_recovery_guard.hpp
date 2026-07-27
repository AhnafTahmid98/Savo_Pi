// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>

#include "savo_nav/control_mode_client.hpp"
#include "savo_nav/recovery_bridge.hpp"

namespace savo_nav
{

struct ControlRecoveryDecision
{
  bool navigation_allowed{false};
  bool cancel_active_goal{true};

  std::string reason{
    "control_recovery_not_evaluated"};

  ControlModeObservation control{};
  RecoveryObservation recovery{};
};

class ControlRecoveryGuard
{
public:
  [[nodiscard]] static ControlRecoveryDecision
  Evaluate(
    const ControlModeObservation & control,
    const RecoveryObservation & recovery);
};

}  // namespace savo_nav
