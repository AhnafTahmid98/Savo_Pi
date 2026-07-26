// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include "savo_nav/sensor_freshness_monitor.hpp"

namespace savo_nav
{

struct LocalizationHealthSnapshot
{
  bool odometry_received{false};
  bool odometry_fresh{false};
};

class LocalizationMonitor
{
public:
  explicit LocalizationMonitor(
    double odometry_timeout_seconds);

  void MarkOdometryReceived(double monotonic_seconds);

  void Reset() noexcept;

  [[nodiscard]] LocalizationHealthSnapshot GetSnapshot(
    double monotonic_seconds) const noexcept;

private:
  SensorFreshnessMonitor odometry_monitor_;
};

}  // namespace savo_nav
