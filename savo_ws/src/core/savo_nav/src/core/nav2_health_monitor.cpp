// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/nav2_health_monitor.hpp"

namespace savo_nav
{

Nav2HealthMonitor::Nav2HealthMonitor(
  const double costmap_timeout_seconds)
: global_costmap_monitor_(costmap_timeout_seconds),
  local_costmap_monitor_(costmap_timeout_seconds)
{
}

void Nav2HealthMonitor::UpdateActionServerAvailable(
  const bool available) noexcept
{
  action_server_available_ = available;
}

void Nav2HealthMonitor::MarkGlobalCostmapReceived(
  const double monotonic_seconds)
{
  global_costmap_monitor_.MarkReceived(monotonic_seconds);
}

void Nav2HealthMonitor::MarkLocalCostmapReceived(
  const double monotonic_seconds)
{
  local_costmap_monitor_.MarkReceived(monotonic_seconds);
}

void Nav2HealthMonitor::Reset() noexcept
{
  global_costmap_monitor_.Reset();
  local_costmap_monitor_.Reset();

  action_server_available_ = false;
}

Nav2HealthSnapshot Nav2HealthMonitor::GetSnapshot(
  const double monotonic_seconds) const noexcept
{
  Nav2HealthSnapshot snapshot;

  snapshot.action_server_available =
    action_server_available_;

  snapshot.global_costmap_received =
    global_costmap_monitor_.HasReceived();

  snapshot.global_costmap_fresh =
    global_costmap_monitor_.IsFresh(monotonic_seconds);

  snapshot.local_costmap_received =
    local_costmap_monitor_.HasReceived();

  snapshot.local_costmap_fresh =
    local_costmap_monitor_.IsFresh(monotonic_seconds);

  return snapshot;
}

}  // namespace savo_nav
