// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/safety_state_monitor.hpp"

#include <cmath>

namespace savo_nav
{

SafetyStateMonitor::SafetyStateMonitor(
  const double state_timeout_seconds)
: stop_monitor_(state_timeout_seconds),
  slowdown_monitor_(state_timeout_seconds)
{
}

void SafetyStateMonitor::UpdateStop(
  const bool stop_active,
  const double monotonic_seconds)
{
  stop_active_ = stop_active;
  stop_monitor_.MarkReceived(monotonic_seconds);
}

void SafetyStateMonitor::UpdateSlowdown(
  const double slowdown_factor,
  const double monotonic_seconds)
{
  slowdown_factor_ = slowdown_factor;

  slowdown_valid_ =
    std::isfinite(slowdown_factor_) &&
    slowdown_factor_ >= 0.0 &&
    slowdown_factor_ <= 1.0;

  slowdown_monitor_.MarkReceived(monotonic_seconds);
}

void SafetyStateMonitor::Reset() noexcept
{
  stop_monitor_.Reset();
  slowdown_monitor_.Reset();

  stop_active_ = false;
  slowdown_valid_ = false;
  slowdown_factor_ = 0.0;
}

SafetyHealthSnapshot SafetyStateMonitor::GetSnapshot(
  const double monotonic_seconds) const noexcept
{
  SafetyHealthSnapshot snapshot;

  snapshot.stop_state_received =
    stop_monitor_.HasReceived();

  snapshot.slowdown_received =
    slowdown_monitor_.HasReceived();

  snapshot.state_fresh =
    stop_monitor_.IsFresh(monotonic_seconds) &&
    slowdown_monitor_.IsFresh(monotonic_seconds);

  snapshot.stop_active = stop_active_;
  snapshot.slowdown_valid = slowdown_valid_;
  snapshot.slowdown_factor = slowdown_factor_;

  return snapshot;
}

}  // namespace savo_nav
