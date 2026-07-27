// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>

namespace savo_nav
{

struct RecoveryObservation
{
  bool observed{false};
  bool fresh{false};

  bool active{false};

  bool navigation_allowed{false};
  bool cancel_active_goal{true};

  double age_seconds{0.0};

  std::string state_text{};
  std::string status_text{};

  std::string reason_code{
    "recovery_state_unobserved"};
};

class RecoveryBridge
{
public:
  [[nodiscard]] bool UpdateActive(
    bool active,
    double now_seconds);

  void UpdateState(std::string state_text);

  void UpdateStatus(std::string status_text);

  void Reset() noexcept;

  [[nodiscard]] RecoveryObservation Evaluate(
    double now_seconds,
    double timeout_seconds) const;

private:
  bool observed_{false};
  bool active_{false};

  double last_active_seconds_{0.0};

  std::string state_text_{};
  std::string status_text_{};
};

}  // namespace savo_nav
