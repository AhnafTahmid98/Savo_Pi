// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include "savo_nav/sensor_freshness_monitor.hpp"

namespace savo_nav
{

struct Nav2HealthSnapshot
{
  bool action_server_available{false};
  bool global_costmap_received{false};
  bool global_costmap_fresh{false};
  bool local_costmap_received{false};
  bool local_costmap_fresh{false};
};

class Nav2HealthMonitor
{
public:
  explicit Nav2HealthMonitor(
    double costmap_timeout_seconds);

  void UpdateActionServerAvailable(
    bool available) noexcept;

  void MarkGlobalCostmapReceived(
    double monotonic_seconds);

  void MarkLocalCostmapReceived(
    double monotonic_seconds);

  void Reset() noexcept;

  [[nodiscard]] Nav2HealthSnapshot GetSnapshot(
    double monotonic_seconds) const noexcept;

private:
  SensorFreshnessMonitor global_costmap_monitor_;
  SensorFreshnessMonitor local_costmap_monitor_;

  bool action_server_available_{false};
};

}  // namespace savo_nav
