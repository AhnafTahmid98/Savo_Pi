// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/recovery_bridge.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace savo_nav
{

bool RecoveryBridge::UpdateActive(
  const bool active,
  const double now_seconds)
{
  if (!std::isfinite(now_seconds)) {
    return false;
  }

  observed_ = true;
  active_ = active;

  last_active_seconds_ = now_seconds;

  return true;
}

void RecoveryBridge::UpdateState(
  std::string state_text)
{
  state_text_ = std::move(state_text);
}

void RecoveryBridge::UpdateStatus(
  std::string status_text)
{
  status_text_ = std::move(status_text);
}

void RecoveryBridge::Reset() noexcept
{
  observed_ = false;
  active_ = false;

  last_active_seconds_ = 0.0;

  state_text_.clear();
  status_text_.clear();
}

RecoveryObservation RecoveryBridge::Evaluate(
  const double now_seconds,
  const double timeout_seconds) const
{
  RecoveryObservation observation;

  observation.observed = observed_;
  observation.active = active_;

  observation.state_text = state_text_;
  observation.status_text = status_text_;

  if (!observed_) {
    observation.age_seconds =
      std::numeric_limits<double>::infinity();

    observation.reason_code =
      "recovery_state_unobserved";

    return observation;
  }

  if (
    !std::isfinite(now_seconds) ||
    !std::isfinite(timeout_seconds) ||
    timeout_seconds <= 0.0 ||
    now_seconds < last_active_seconds_)
  {
    observation.age_seconds =
      std::numeric_limits<double>::infinity();

    observation.reason_code =
      "recovery_state_invalid_time";

    return observation;
  }

  observation.age_seconds =
    now_seconds - last_active_seconds_;

  observation.fresh =
    observation.age_seconds <= timeout_seconds;

  if (!observation.fresh) {
    observation.reason_code =
      "recovery_state_stale";

    return observation;
  }

  if (observation.active) {
    observation.reason_code =
      "recovery_active";

    return observation;
  }

  observation.navigation_allowed = true;
  observation.cancel_active_goal = false;

  observation.reason_code =
    "recovery_clear";

  return observation;
}

}  // namespace savo_nav
