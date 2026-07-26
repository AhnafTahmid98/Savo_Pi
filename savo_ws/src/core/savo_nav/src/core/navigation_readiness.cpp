// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/navigation_readiness.hpp"

#include <utility>

namespace
{

void AddFailure(
  std::vector<std::string> & failures,
  const bool failed,
  const std::string & dependency)
{
  if (failed) {
    failures.push_back(dependency);
  }
}

savo_nav::NavigationReadinessResult MakeResult(
  const savo_nav::NavigationReadinessState state,
  const bool goal_acceptance_allowed,
  std::string reason,
  std::vector<std::string> failed_dependencies)
{
  savo_nav::NavigationReadinessResult result;

  result.state = state;
  result.goal_acceptance_allowed =
    goal_acceptance_allowed;

  result.reason = std::move(reason);

  result.failed_dependencies =
    std::move(failed_dependencies);

  return result;
}

}  // namespace

namespace savo_nav
{

NavigationReadiness::NavigationReadiness() = default;

const NavigationReadinessResult &
NavigationReadiness::GetResult() const noexcept
{
  return result_;
}

bool NavigationReadiness::Update(
  const NavigationReadinessState state,
  const bool goal_acceptance_allowed,
  std::string reason,
  std::vector<std::string> failed_dependencies)
{
  if (!IsUpdateValid(
      state,
      goal_acceptance_allowed,
      reason,
      failed_dependencies))
  {
    return false;
  }

  result_.state = state;
  result_.goal_acceptance_allowed =
    goal_acceptance_allowed;

  result_.reason = std::move(reason);

  result_.failed_dependencies =
    std::move(failed_dependencies);

  return true;
}

NavigationReadinessResult NavigationReadiness::Evaluate(
  const NavigationDependencySnapshot & snapshot,
  const NavigationReadinessPolicy & policy)
{
  std::vector<std::string> failures;

  AddFailure(
    failures,
    policy.require_safety_state &&
    !snapshot.safety_state_fresh,
    "safety_state");

  AddFailure(
    failures,
    snapshot.safety_stop_active,
    "safety_stop");

  AddFailure(
    failures,
    policy.require_safety_state &&
    !snapshot.slowdown_valid,
    "safety_slowdown");

  AddFailure(
    failures,
    policy.require_safety_state &&
    snapshot.slowdown_valid &&
    snapshot.slowdown_factor <= 0.0,
    "safety_motion_permission");

  AddFailure(
    failures,
    policy.require_control_mode &&
    !snapshot.control_state_fresh,
    "control_mode_state");

  AddFailure(
    failures,
    policy.require_control_mode &&
    snapshot.control_state_fresh &&
    !snapshot.control_allows_navigation,
    "control_mode_permission");

  AddFailure(
    failures,
    !snapshot.map_available,
    "map");

  AddFailure(
    failures,
    !snapshot.map_to_odom_fresh,
    "map_to_odom");

  AddFailure(
    failures,
    !snapshot.odom_to_base_fresh,
    "odom_to_base_footprint");

  AddFailure(
    failures,
    !snapshot.base_footprint_to_base_link_available,
    "base_footprint_to_base_link");

  AddFailure(
    failures,
    !snapshot.localization_fresh,
    "localization");

  AddFailure(
    failures,
    !snapshot.lidar_fresh,
    "lidar");

  AddFailure(
    failures,
    policy.require_pointcloud &&
    !snapshot.pointcloud_fresh,
    "obstacle_pointcloud");

  AddFailure(
    failures,
    !snapshot.nav2_action_server_available,
    "nav2_action_server");

  AddFailure(
    failures,
    policy.require_global_costmap &&
    !snapshot.global_costmap_fresh,
    "global_costmap");

  AddFailure(
    failures,
    policy.require_local_costmap &&
    !snapshot.local_costmap_fresh,
    "local_costmap");

  if (
    policy.require_safety_state &&
    !snapshot.safety_state_fresh)
  {
    return MakeResult(
      NavigationReadinessState::kBlocked,
      false,
      "safety_state_unavailable",
      std::move(failures));
  }

  if (snapshot.safety_stop_active) {
    return MakeResult(
      NavigationReadinessState::kBlocked,
      false,
      "safety_stop_active",
      std::move(failures));
  }

  if (
    policy.require_safety_state &&
    !snapshot.slowdown_valid)
  {
    return MakeResult(
      NavigationReadinessState::kBlocked,
      false,
      "safety_slowdown_invalid",
      std::move(failures));
  }

  if (
    policy.require_safety_state &&
    snapshot.slowdown_factor <= 0.0)
  {
    return MakeResult(
      NavigationReadinessState::kBlocked,
      false,
      "safety_motion_not_permitted",
      std::move(failures));
  }

  if (
    policy.require_control_mode &&
    !snapshot.control_state_fresh)
  {
    return MakeResult(
      NavigationReadinessState::kBlocked,
      false,
      "control_mode_state_unavailable",
      std::move(failures));
  }

  if (
    policy.require_control_mode &&
    !snapshot.control_allows_navigation)
  {
    return MakeResult(
      NavigationReadinessState::kBlocked,
      false,
      "control_mode_not_navigation",
      std::move(failures));
  }

  if (!snapshot.map_available) {
    return MakeResult(
      NavigationReadinessState::kWaitingForMap,
      false,
      "waiting_for_map",
      std::move(failures));
  }

  if (
    !snapshot.map_to_odom_fresh ||
    !snapshot.odom_to_base_fresh ||
    !snapshot.base_footprint_to_base_link_available)
  {
    return MakeResult(
      NavigationReadinessState::kWaitingForTf,
      false,
      "waiting_for_required_tf_chain",
      std::move(failures));
  }

  if (!snapshot.localization_fresh) {
    return MakeResult(
      NavigationReadinessState::kWaitingForLocalization,
      false,
      "waiting_for_filtered_odometry",
      std::move(failures));
  }

  if (!snapshot.lidar_fresh) {
    return MakeResult(
      NavigationReadinessState::kWaitingForLidar,
      false,
      "waiting_for_lidar_scan",
      std::move(failures));
  }

  if (
    policy.require_pointcloud &&
    !snapshot.pointcloud_fresh)
  {
    return MakeResult(
      NavigationReadinessState::kWaitingForPointCloud,
      false,
      "waiting_for_filtered_obstacle_pointcloud",
      std::move(failures));
  }

  if (!snapshot.nav2_action_server_available) {
    return MakeResult(
      NavigationReadinessState::kWaitingForNav2,
      false,
      "waiting_for_nav2_action_server",
      std::move(failures));
  }

  if (
    (
      policy.require_global_costmap &&
      !snapshot.global_costmap_fresh
    ) ||
    (
      policy.require_local_costmap &&
      !snapshot.local_costmap_fresh
    ))
  {
    return MakeResult(
      NavigationReadinessState::kWaitingForCostmaps,
      false,
      "waiting_for_required_costmaps",
      std::move(failures));
  }

  return MakeResult(
    NavigationReadinessState::kReady,
    true,
    "all_required_navigation_dependencies_ready",
    {});
}

std::string_view NavigationReadiness::ToString(
  const NavigationReadinessState state) noexcept
{
  switch (state) {
    case NavigationReadinessState::kOffline:
      return "offline";

    case NavigationReadinessState::kStarting:
      return "starting";

    case NavigationReadinessState::kWaitingForMap:
      return "waiting_for_map";

    case NavigationReadinessState::kWaitingForTf:
      return "waiting_for_tf";

    case NavigationReadinessState::kWaitingForLocalization:
      return "waiting_for_localization";

    case NavigationReadinessState::kWaitingForLidar:
      return "waiting_for_lidar";

    case NavigationReadinessState::kWaitingForPointCloud:
      return "waiting_for_pointcloud";

    case NavigationReadinessState::kWaitingForNav2:
      return "waiting_for_nav2";

    case NavigationReadinessState::kWaitingForCostmaps:
      return "waiting_for_costmaps";

    case NavigationReadinessState::kReady:
      return "ready";

    case NavigationReadinessState::kDegraded:
      return "degraded";

    case NavigationReadinessState::kBlocked:
      return "blocked";

    case NavigationReadinessState::kFault:
      return "fault";
  }

  return "unknown";
}

bool NavigationReadiness::IsUpdateValid(
  const NavigationReadinessState state,
  const bool goal_acceptance_allowed,
  const std::string & reason,
  const std::vector<std::string> & failed_dependencies) noexcept
{
  if (reason.empty()) {
    return false;
  }

  if (state == NavigationReadinessState::kReady) {
    return
      goal_acceptance_allowed &&
      failed_dependencies.empty();
  }

  if (
    state == NavigationReadinessState::kOffline ||
    state == NavigationReadinessState::kStarting ||
    state == NavigationReadinessState::kWaitingForMap ||
    state == NavigationReadinessState::kWaitingForTf ||
    state == NavigationReadinessState::kWaitingForLocalization ||
    state == NavigationReadinessState::kWaitingForLidar ||
    state == NavigationReadinessState::kWaitingForPointCloud ||
    state == NavigationReadinessState::kWaitingForNav2 ||
    state == NavigationReadinessState::kWaitingForCostmaps ||
    state == NavigationReadinessState::kBlocked ||
    state == NavigationReadinessState::kFault)
  {
    return !goal_acceptance_allowed;
  }

  return state == NavigationReadinessState::kDegraded;
}

}  // namespace savo_nav
