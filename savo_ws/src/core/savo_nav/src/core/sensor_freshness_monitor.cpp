// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/sensor_freshness_monitor.hpp"

#include <cmath>
#include <stdexcept>

namespace savo_nav
{

SensorFreshnessMonitor::SensorFreshnessMonitor(
  const double timeout_seconds)
: timeout_seconds_(timeout_seconds)
{
  if (
    !std::isfinite(timeout_seconds_) ||
    timeout_seconds_ <= 0.0)
  {
    throw std::invalid_argument(
            "freshness timeout must be finite and positive");
  }
}

void SensorFreshnessMonitor::MarkReceived(
  const double monotonic_seconds)
{
  if (
    !std::isfinite(monotonic_seconds) ||
    monotonic_seconds < 0.0)
  {
    throw std::invalid_argument(
            "receipt time must be finite and nonnegative");
  }

  last_received_seconds_ = monotonic_seconds;
  received_ = true;
}

void SensorFreshnessMonitor::Reset() noexcept
{
  last_received_seconds_ = 0.0;
  received_ = false;
}

bool SensorFreshnessMonitor::HasReceived() const noexcept
{
  return received_;
}

bool SensorFreshnessMonitor::IsFresh(
  const double monotonic_seconds) const noexcept
{
  if (
    !received_ ||
    !std::isfinite(monotonic_seconds) ||
    monotonic_seconds < last_received_seconds_)
  {
    return false;
  }

  return
    monotonic_seconds - last_received_seconds_ <=
    timeout_seconds_;
}

double SensorFreshnessMonitor::TimeoutSeconds() const noexcept
{
  return timeout_seconds_;
}

}  // namespace savo_nav
