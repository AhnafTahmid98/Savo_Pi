// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include "savo_nav/sensor_freshness_monitor.hpp"

namespace savo_nav
{

struct SafetyHealthSnapshot
{
  bool stop_state_received{false};
  bool slowdown_received{false};
  bool state_fresh{false};
  bool stop_active{false};
  bool slowdown_valid{false};
  double slowdown_factor{0.0};
};

class SafetyStateMonitor
{
public:
  explicit SafetyStateMonitor(
    double state_timeout_seconds);

  void UpdateStop(
    bool stop_active,
    double monotonic_seconds);

  void UpdateSlowdown(
    double slowdown_factor,
    double monotonic_seconds);

  void Reset() noexcept;

  [[nodiscard]] SafetyHealthSnapshot GetSnapshot(
    double monotonic_seconds) const noexcept;

private:
  SensorFreshnessMonitor stop_monitor_;
  SensorFreshnessMonitor slowdown_monitor_;

  bool stop_active_{false};
  bool slowdown_valid_{false};
  double slowdown_factor_{0.0};
};

}  // namespace savo_nav
