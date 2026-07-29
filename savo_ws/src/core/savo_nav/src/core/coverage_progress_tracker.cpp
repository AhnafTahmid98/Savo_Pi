// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/coverage_progress_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

constexpr double kDistanceEpsilon = 1.0e-9;

bool IsValidTime(const double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

namespace savo_nav
{

bool CoverageProgressTracker::Configure(
  const std::vector<CoveragePathPoint> & points)
{
  Reset();

  if (
    points.empty() ||
    points.size() >
    std::numeric_limits<std::uint32_t>::max())
  {
    return false;
  }

  cumulative_distances_.reserve(points.size());

  double cumulative_distance = 0.0;

  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto & point = points[index];

    if (
      !std::isfinite(point.x) ||
      !std::isfinite(point.y))
    {
      Reset();
      return false;
    }

    if (index > 0) {
      const auto & previous = points[index - 1];

      cumulative_distance += std::hypot(
        point.x - previous.x,
        point.y - previous.y);

      if (!std::isfinite(cumulative_distance)) {
        Reset();
        return false;
      }
    }

    cumulative_distances_.push_back(
      cumulative_distance);
  }

  total_distance_m_ = cumulative_distance;
  monotonic_remaining_distance_m_ = total_distance_m_;

  snapshot_.total_waypoints =
    static_cast<std::uint32_t>(points.size());

  snapshot_.remaining_distance_m =
    total_distance_m_;

  configured_ = true;
  return true;
}

bool CoverageProgressTracker::Start(
  const double now_seconds)
{
  if (
    !configured_ ||
    started_ ||
    !IsValidTime(now_seconds))
  {
    return false;
  }

  started_ = true;
  started_at_seconds_ = now_seconds;
  last_update_seconds_ = now_seconds;

  snapshot_.current_waypoint = 0;
  snapshot_.elapsed_seconds = 0.0;

  return true;
}

bool CoverageProgressTracker::UpdateRemainingDistance(
  const double remaining_distance_m,
  const double now_seconds)
{
  if (
    !started_ ||
    succeeded_ ||
    !IsValidTime(now_seconds) ||
    now_seconds < last_update_seconds_ ||
    !std::isfinite(remaining_distance_m) ||
    remaining_distance_m < 0.0)
  {
    return false;
  }

  monotonic_remaining_distance_m_ =
    std::min(
    {
      monotonic_remaining_distance_m_,
      remaining_distance_m,
      total_distance_m_
    });

  last_update_seconds_ = now_seconds;
  Refresh(now_seconds);

  return true;
}

bool CoverageProgressTracker::MarkSucceeded(
  const double now_seconds)
{
  if (
    !started_ ||
    succeeded_ ||
    !IsValidTime(now_seconds) ||
    now_seconds < last_update_seconds_)
  {
    return false;
  }

  succeeded_ = true;
  last_update_seconds_ = now_seconds;
  monotonic_remaining_distance_m_ = 0.0;

  snapshot_.completed_waypoints =
    snapshot_.total_waypoints;

  snapshot_.current_waypoint =
    snapshot_.total_waypoints - 1;

  snapshot_.completion_ratio = 1.0;
  snapshot_.remaining_distance_m = 0.0;
  snapshot_.estimated_remaining_seconds = 0.0;
  snapshot_.elapsed_seconds =
    now_seconds - started_at_seconds_;

  return true;
}

void CoverageProgressTracker::Reset() noexcept
{
  cumulative_distances_.clear();
  snapshot_ = {};

  total_distance_m_ = 0.0;
  started_at_seconds_ = 0.0;
  last_update_seconds_ = 0.0;
  monotonic_remaining_distance_m_ = 0.0;

  configured_ = false;
  started_ = false;
  succeeded_ = false;
}

bool CoverageProgressTracker::IsConfigured() const noexcept
{
  return configured_;
}

bool CoverageProgressTracker::IsStarted() const noexcept
{
  return started_;
}

const CoverageProgressSnapshot &
CoverageProgressTracker::Snapshot() const noexcept
{
  return snapshot_;
}

void CoverageProgressTracker::Refresh(
  const double now_seconds)
{
  snapshot_.elapsed_seconds =
    now_seconds - started_at_seconds_;

  snapshot_.remaining_distance_m =
    monotonic_remaining_distance_m_;

  if (total_distance_m_ <= kDistanceEpsilon) {
    snapshot_.completed_waypoints =
      snapshot_.total_waypoints;

    snapshot_.current_waypoint =
      snapshot_.total_waypoints - 1;

    snapshot_.completion_ratio = 1.0;
    snapshot_.estimated_remaining_seconds = 0.0;
    return;
  }

  const double traveled_distance_m =
    total_distance_m_ -
    monotonic_remaining_distance_m_;

  std::uint32_t completed = 0;

  for (
    std::size_t index = 0;
    index < cumulative_distances_.size();
    ++index)
  {
    const bool first_waypoint_reached =
      index == 0 &&
      traveled_distance_m > kDistanceEpsilon;

    const bool later_waypoint_reached =
      index > 0 &&
      cumulative_distances_[index] <=
      traveled_distance_m + kDistanceEpsilon;

    if (
      first_waypoint_reached ||
      later_waypoint_reached)
    {
      completed =
        static_cast<std::uint32_t>(index + 1);
    }
  }

  snapshot_.completed_waypoints = completed;

  snapshot_.current_waypoint = std::min(
    completed,
    snapshot_.total_waypoints - 1);

  snapshot_.completion_ratio = std::clamp(
    traveled_distance_m / total_distance_m_,
    0.0,
    1.0);

  snapshot_.estimated_remaining_seconds = 0.0;

  if (
    traveled_distance_m > kDistanceEpsilon &&
    snapshot_.elapsed_seconds > kDistanceEpsilon)
  {
    const double average_speed =
      traveled_distance_m /
      snapshot_.elapsed_seconds;

    if (
      std::isfinite(average_speed) &&
      average_speed > kDistanceEpsilon)
    {
      snapshot_.estimated_remaining_seconds =
        monotonic_remaining_distance_m_ /
        average_speed;
    }
  }
}

}  // namespace savo_nav
