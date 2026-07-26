// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

namespace savo_nav
{

class SensorFreshnessMonitor
{
public:
  explicit SensorFreshnessMonitor(double timeout_seconds);

  void MarkReceived(double monotonic_seconds);

  void Reset() noexcept;

  [[nodiscard]] bool HasReceived() const noexcept;

  [[nodiscard]] bool IsFresh(
    double monotonic_seconds) const noexcept;

  [[nodiscard]] double TimeoutSeconds() const noexcept;

private:
  double timeout_seconds_;
  double last_received_seconds_{0.0};
  bool received_{false};
};

}  // namespace savo_nav
