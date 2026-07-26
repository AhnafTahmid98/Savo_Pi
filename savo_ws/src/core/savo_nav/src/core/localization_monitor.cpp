// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/localization_monitor.hpp"

namespace savo_nav
{

LocalizationMonitor::LocalizationMonitor(
  const double odometry_timeout_seconds)
: odometry_monitor_(odometry_timeout_seconds)
{
}

void LocalizationMonitor::MarkOdometryReceived(
  const double monotonic_seconds)
{
  odometry_monitor_.MarkReceived(monotonic_seconds);
}

void LocalizationMonitor::Reset() noexcept
{
  odometry_monitor_.Reset();
}

LocalizationHealthSnapshot LocalizationMonitor::GetSnapshot(
  const double monotonic_seconds) const noexcept
{
  LocalizationHealthSnapshot snapshot;

  snapshot.odometry_received =
    odometry_monitor_.HasReceived();

  snapshot.odometry_fresh =
    odometry_monitor_.IsFresh(monotonic_seconds);

  return snapshot;
}

}  // namespace savo_nav
