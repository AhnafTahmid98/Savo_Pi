// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>

namespace savo_nav
{

struct GoalAdmissionInput
{
  bool guard_observed{false};
  bool guard_fresh{false};
  bool guard_allowed{false};

  bool active_goal{false};
  bool slot_reserved{false};
  bool cancellation_requested{false};

  std::string guard_reason{
    "control_recovery_guard_unobserved"};
};

struct GoalAdmissionDecision
{
  bool accept_new_goal{false};
  bool request_active_cancel{false};

  std::string reason{
    "control_recovery_guard_unobserved"};
};

class GoalAdmissionPolicy
{
public:
  [[nodiscard]] static GoalAdmissionDecision Evaluate(
    const GoalAdmissionInput & input);
};

}  // namespace savo_nav
