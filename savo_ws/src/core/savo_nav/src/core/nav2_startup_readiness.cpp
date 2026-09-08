// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/nav2_startup_readiness.hpp"

namespace savo_nav
{

Nav2StartupDecision EvaluateNav2Startup(const Nav2StartupEvidence & evidence)
{
  Nav2StartupDecision decision;
  decision.process_started = evidence.inactive_lifecycle_nodes.empty();
  if (!decision.process_started) {
    decision.reason = "nav2_lifecycle_nodes_not_active";
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
