// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/nav2_startup_readiness.hpp"

namespace savo_nav
{

void LifecycleObservation::RequestStarted(const TimePoint now) noexcept
{
  pending_ = true;
  requested_at_ = now;
}

void LifecycleObservation::RequestCancelled() noexcept
{
  pending_ = false;
}

void LifecycleObservation::ResponseReceived(const bool active, const TimePoint now) noexcept
{
  pending_ = false;
  observed_ = true;
  last_active_ = active;
  observed_at_ = now;
}

void LifecycleObservation::Invalidate() noexcept
{
  pending_ = false;
  observed_ = false;
}

bool LifecycleObservation::request_pending() const noexcept
{
  return pending_;
}

bool LifecycleObservation::RequestTimedOut(
  const TimePoint now, const Clock::duration timeout) const noexcept
{
  return pending_ && (now < requested_at_ || now - requested_at_ >= timeout);
}

LifecycleObservationState LifecycleObservation::StateAt(
  const TimePoint now, const Clock::duration freshness) const noexcept
{
  if (!observed_) {
    return LifecycleObservationState::kUnknown;
  }
  if (now < observed_at_ || now - observed_at_ > freshness) {
    return LifecycleObservationState::kStale;
  }
  return last_active_ ? LifecycleObservationState::kActive :
         LifecycleObservationState::kInactive;
}

Nav2StartupDecision EvaluateNav2Startup(const Nav2StartupEvidence & evidence)
{
  Nav2StartupDecision decision;
  // Preserve the existing JSON field semantics for consumers: this legacy
  // field means all required lifecycle nodes are freshly verified ACTIVE,
  // not that OS processes were inspected.
  decision.process_started = evidence.inactive_lifecycle_nodes.empty() &&
    evidence.unconfirmed_lifecycle_nodes.empty();
  if (!evidence.inactive_lifecycle_nodes.empty()) {
    decision.reason = "nav2_lifecycle_nodes_not_active";
    return decision;
  }
  if (!evidence.unconfirmed_lifecycle_nodes.empty()) {
    decision.reason = "nav2_lifecycle_states_unconfirmed";
    return decision;
  }
  if (!evidence.navigate_to_pose_action_ready) {
    decision.reason = "navigate_to_pose_action_unavailable";
    return decision;
  }
  if (!evidence.follow_waypoints_action_ready) {
    decision.reason = "follow_waypoints_action_unavailable";
    return decision;
  }
  if (!evidence.odom_to_base_footprint_tf_ready) {
    decision.reason = "odom_to_base_footprint_tf_unavailable";
    return decision;
  }
  decision.startup_ready = true;
  decision.quality = StartupQuality::kMinimum;
  decision.reason = "nav2_processes_ready_without_map_requirement";
  return decision;
}

const char * ToString(const StartupQuality quality) noexcept
{
  switch (quality) {
    case StartupQuality::kBelowMinimum: return "BELOW_MINIMUM";
    case StartupQuality::kMinimum: return "MINIMUM";
  }
  return "BELOW_MINIMUM";
}

}  // namespace savo_nav
