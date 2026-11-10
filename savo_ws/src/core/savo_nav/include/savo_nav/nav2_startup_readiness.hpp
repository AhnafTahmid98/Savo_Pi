// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <chrono>
#include <string>
#include <vector>

namespace savo_nav
{

// Pure, monotonic-clock state machine; independent of ROS transport/executor.
// Pending requests do not revoke a fresh, confirmed ACTIVE observation.
enum class LifecycleObservationState
{
  kUnknown,
  kActive,
  kInactive,
  kStale,
};

class LifecycleObservation
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void RequestStarted(TimePoint now) noexcept;
  void RequestCancelled() noexcept;
  void ResponseReceived(bool active, TimePoint now) noexcept;
  void Invalidate() noexcept;
  [[nodiscard]] bool request_pending() const noexcept;
  [[nodiscard]] bool RequestTimedOut(
    TimePoint now, Clock::duration timeout) const noexcept;
  [[nodiscard]] LifecycleObservationState StateAt(
    TimePoint now, Clock::duration freshness) const noexcept;

private:
  bool pending_{false};
  bool observed_{false};
  bool last_active_{false};
  TimePoint requested_at_{};
  TimePoint observed_at_{};
};

enum class StartupQuality
{
  kBelowMinimum,
  kMinimum,
};

struct Nav2StartupEvidence
{
  std::vector<std::string> inactive_lifecycle_nodes;
  std::vector<std::string> unconfirmed_lifecycle_nodes;
  bool navigate_to_pose_action_ready{false};
  bool follow_waypoints_action_ready{false};
  bool odom_to_base_footprint_tf_ready{false};
};

struct Nav2StartupDecision
{
  bool process_started{false};
  bool startup_ready{false};
  StartupQuality quality{StartupQuality::kBelowMinimum};
  std::string reason{"nav2_starting"};
};

[[nodiscard]] Nav2StartupDecision EvaluateNav2Startup(
  const Nav2StartupEvidence & evidence);
[[nodiscard]] const char * ToString(StartupQuality quality) noexcept;

}  // namespace savo_nav
