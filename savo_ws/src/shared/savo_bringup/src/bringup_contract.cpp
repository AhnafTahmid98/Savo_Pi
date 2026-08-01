// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_bringup/bringup_contract.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace savo_bringup
{
namespace
{

std::string Normalize(std::string_view value)
{
  std::string normalized(value);
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](const unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  return normalized;
}

}  // namespace

std::optional<HostRole> ParseHostRole(const std::string_view value) noexcept
{
  const auto normalized = Normalize(value);
  if (normalized == "core") {
    return HostRole::kCore;
  }
  if (normalized == "edge") {
    return HostRole::kEdge;
  }
  if (normalized == "all") {
    return HostRole::kAll;
  }
  return std::nullopt;
}

std::optional<RobotMode> ParseRobotMode(const std::string_view value) noexcept
{
  const auto normalized = Normalize(value);
  if (normalized == "safe_idle") {
    return RobotMode::kSafeIdle;
  }
  if (normalized == "manual") {
    return RobotMode::kManual;
  }
  if (normalized == "manual_mapping") {
    return RobotMode::kManualMapping;
  }
  if (normalized == "autonomous_mapping") {
    return RobotMode::kAutonomousMapping;
  }
  if (normalized == "saved_map_navigation") {
    return RobotMode::kSavedMapNavigation;
  }
  if (normalized == "diagnostics") {
    return RobotMode::kDiagnostics;
  }
  return std::nullopt;
}

std::optional<BringupProfile> ParseBringupProfile(
  const std::string_view value) noexcept
{
  const auto normalized = Normalize(value);
  if (normalized == "bench") {
    return BringupProfile::kBench;
  }
  if (normalized == "lidar_only") {
    return BringupProfile::kLidarOnly;
  }
  if (normalized == "lidar_d435_voxel") {
    return BringupProfile::kLidarD435Voxel;
  }
  if (normalized == "production") {
    return BringupProfile::kProduction;
  }
  return std::nullopt;
}

std::optional<ReadinessState> ParseReadinessState(
  const std::string_view value) noexcept
{
  const auto normalized = Normalize(value);
  for (const auto state : {
      ReadinessState::kStarting,
      ReadinessState::kWaitingForDependencies,
      ReadinessState::kValidatingGeometry,
      ReadinessState::kWaitingForSafety,
      ReadinessState::kWaitingForLocalization,
      ReadinessState::kWaitingForMapContext,
      ReadinessState::kWaitingForNavigation,
      ReadinessState::kReady,
      ReadinessState::kDegraded,
      ReadinessState::kBlocked,
      ReadinessState::kShuttingDown})
  {
    if (normalized == ToString(state)) {
      return state;
    }
  }
  return std::nullopt;
}

std::string_view ToString(const HostRole value) noexcept
{
  switch (value) {
    case HostRole::kCore:
      return "core";
    case HostRole::kEdge:
      return "edge";
    case HostRole::kAll:
      return "all";
  }
  return "unknown";
}

std::string_view ToString(const RobotMode value) noexcept
{
  switch (value) {
    case RobotMode::kSafeIdle:
      return "safe_idle";
    case RobotMode::kManual:
      return "manual";
    case RobotMode::kManualMapping:
      return "manual_mapping";
    case RobotMode::kAutonomousMapping:
      return "autonomous_mapping";
    case RobotMode::kSavedMapNavigation:
      return "saved_map_navigation";
    case RobotMode::kDiagnostics:
      return "diagnostics";
  }
  return "unknown";
}

std::string_view ToString(const BringupProfile value) noexcept
{
  switch (value) {
    case BringupProfile::kBench:
      return "bench";
    case BringupProfile::kLidarOnly:
      return "lidar_only";
    case BringupProfile::kLidarD435Voxel:
      return "lidar_d435_voxel";
    case BringupProfile::kProduction:
      return "production";
  }
  return "unknown";
}

std::string_view ToString(const ReadinessState value) noexcept
{
  switch (value) {
    case ReadinessState::kStarting:
      return "starting";
    case ReadinessState::kWaitingForDependencies:
      return "waiting_for_dependencies";
    case ReadinessState::kValidatingGeometry:
      return "validating_geometry";
    case ReadinessState::kWaitingForSafety:
      return "waiting_for_safety";
    case ReadinessState::kWaitingForLocalization:
      return "waiting_for_localization";
    case ReadinessState::kWaitingForMapContext:
      return "waiting_for_map_context";
    case ReadinessState::kWaitingForNavigation:
      return "waiting_for_navigation";
    case ReadinessState::kReady:
      return "ready";
    case ReadinessState::kDegraded:
      return "degraded";
    case ReadinessState::kBlocked:
      return "blocked";
    case ReadinessState::kShuttingDown:
      return "shutting_down";
  }
  return "unknown";
}

ReadinessDecision EvaluateReadiness(
  const std::vector<DependencyStatus> & dependencies,
  const bool configuration_valid,
  const bool startup_timeout_expired,
  const bool shutting_down)
{
  ReadinessDecision decision;
  if (shutting_down) {
    decision.state = ReadinessState::kShuttingDown;
    return decision;
  }
  if (!configuration_valid) {
    decision.state = ReadinessState::kBlocked;
    decision.failed.push_back("invalid_configuration");
    return decision;
  }

  ReadinessState waiting_state = ReadinessState::kWaitingForDependencies;
  bool waiting_state_selected = false;
  for (const auto & dependency : dependencies) {
    const std::string detail = dependency.detail.empty() ? "unspecified" : dependency.detail;
    if (dependency.required) {
      if (dependency.failed) {
        decision.failed.push_back(dependency.name + ":" + detail);
      } else if (!dependency.observed) {
        decision.missing.push_back(dependency.name + ":not_observed");
      } else if (!dependency.fresh) {
        decision.missing.push_back(dependency.name + ":stale");
      } else if (!dependency.ready) {
        decision.missing.push_back(dependency.name + ":" + detail);
      } else {
        continue;
      }
      if (!waiting_state_selected) {
        waiting_state = dependency.waiting_state;
        waiting_state_selected = true;
      }
    } else {
      if (
        dependency.failed ||
        (dependency.observed && (!dependency.fresh || !dependency.ready)))
      {
        decision.degraded.push_back(dependency.name + ":" + detail);
      }
    }
  }

  if (!decision.failed.empty() ||
    (startup_timeout_expired && !decision.missing.empty()))
  {
    decision.state = ReadinessState::kBlocked;
  } else if (!decision.missing.empty()) {
    decision.state = waiting_state;
  } else if (!decision.degraded.empty()) {
    decision.state = ReadinessState::kDegraded;
    decision.ready = true;
  } else {
    decision.state = ReadinessState::kReady;
    decision.ready = true;
  }
  return decision;
}

BringupRequirements RequirementsFor(
  const HostRole role,
  const RobotMode mode,
  const BringupProfile profile,
  const bool start_bridge,
  const bool start_realsense,
  const bool start_vo,
  const bool start_speech) noexcept
{
  BringupRequirements requirements;
  requirements.core_required = role == HostRole::kCore || role == HostRole::kAll;
  requirements.edge_required = role == HostRole::kEdge || role == HostRole::kAll;

  if (requirements.core_required) {
    requirements.supervisor_required = mode != RobotMode::kDiagnostics;
    requirements.navigation_required =
      mode == RobotMode::kAutonomousMapping ||
      mode == RobotMode::kSavedMapNavigation;
    requirements.mapping_required =
      mode == RobotMode::kManualMapping ||
      mode == RobotMode::kAutonomousMapping;
    requirements.motion_capable =
      mode == RobotMode::kManual ||
      mode == RobotMode::kManualMapping ||
      mode == RobotMode::kAutonomousMapping ||
      mode == RobotMode::kSavedMapNavigation;
  }

  if (requirements.edge_required) {
    requirements.bridge_required = start_bridge;
    requirements.realsense_required = start_realsense;
    requirements.vo_required = start_vo;
    requirements.speech_required = start_speech;
  }

  requirements.voxel_layer_enabled =
    profile == BringupProfile::kLidarD435Voxel;
  requirements.locked_geometry_required =
    requirements.motion_capable && profile != BringupProfile::kBench;
  return requirements;
}

std::string ValidateCombination(
  const HostRole role,
  const RobotMode mode,
  const BringupProfile profile,
  const bool d435_voxel_validated,
  const bool require_locked_geometry,
  const bool allow_provisional_geometry)
{
  if (
    profile == BringupProfile::kLidarD435Voxel &&
    !d435_voxel_validated)
  {
    return "d435_voxel_profile_requires_explicit_hardware_validation";
  }

  if (role == HostRole::kAll && profile != BringupProfile::kBench) {
    return "all_host_role_is_bench_only";
  }

  if (
    profile == BringupProfile::kProduction &&
    !require_locked_geometry)
  {
    return "production_profile_requires_locked_geometry";
  }

  const auto requirements = RequirementsFor(
    role, mode, profile, true, true, true, true);

  if (
    requirements.locked_geometry_required &&
    !require_locked_geometry)
  {
    return "motion_profile_requires_locked_geometry_validation";
  }

  if (require_locked_geometry && allow_provisional_geometry) {
    return "locked_geometry_and_provisional_override_are_mutually_exclusive";
  }

  if (profile == BringupProfile::kProduction && allow_provisional_geometry) {
    return "production_profile_cannot_allow_provisional_geometry";
  }

  return {};
}

}  // namespace savo_bringup
