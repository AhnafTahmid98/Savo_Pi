// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include "savo_nav/coverage_path_validator.hpp"

namespace savo_nav
{

struct CoverageProgressSnapshot
{
  std::uint32_t current_waypoint{
    std::numeric_limits<std::uint32_t>::max()};

  std::uint32_t completed_waypoints{0};
  std::uint32_t total_waypoints{0};

  double completion_ratio{0.0};
  double remaining_distance_m{0.0};
  double elapsed_seconds{0.0};
  double estimated_remaining_seconds{0.0};
};

class CoverageProgressTracker
{
public:
  [[nodiscard]] bool Configure(
    const std::vector<CoveragePathPoint> & points);

  [[nodiscard]] bool Start(double now_seconds);

  [[nodiscard]] bool UpdateRemainingDistance(
    double remaining_distance_m,
    double now_seconds);

  [[nodiscard]] bool MarkSucceeded(
    double now_seconds);

  void Reset() noexcept;

  [[nodiscard]] bool IsConfigured() const noexcept;
  [[nodiscard]] bool IsStarted() const noexcept;

  [[nodiscard]] const CoverageProgressSnapshot &
  Snapshot() const noexcept;

private:
  void Refresh(double now_seconds);

  std::vector<double> cumulative_distances_{};

  CoverageProgressSnapshot snapshot_{};

  double total_distance_m_{0.0};
  double started_at_seconds_{0.0};
  double last_update_seconds_{0.0};
  double monotonic_remaining_distance_m_{0.0};

  bool configured_{false};
  bool started_{false};
  bool succeeded_{false};
};

}  // namespace savo_nav
