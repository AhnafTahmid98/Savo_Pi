// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_bringup/bringup_contract.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

#include <nlohmann/json.hpp>

namespace savo_bringup
{
namespace
{

using Json = nlohmann::json;

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

std::string_view ToString(const QualityLevel value) noexcept
{
  switch (value) {
    case QualityLevel::kBelowMinimum:
      return "BELOW_MINIMUM";
    case QualityLevel::kMinimum:
      return "MINIMUM";
    case QualityLevel::kGood:
      return "GOOD";
    case QualityLevel::kExcellent:
      return "EXCELLENT";
  }
  return "BELOW_MINIMUM";
}

std::string_view ToString(const StartupStageState value) noexcept
{
  switch (value) {
    case StartupStageState::kStarting:
      return "STARTING";
    case StartupStageState::kWaitingForDependencies:
      return "WAITING_FOR_DEPENDENCIES";
    case StartupStageState::kStabilizing:
      return "STABILIZING";
    case StartupStageState::kReady:
      return "READY";
    case StartupStageState::kFailed:
      return "FAILED";
    case StartupStageState::kShuttingDown:
      return "SHUTTING_DOWN";
  }
  return "FAILED";
}

std::optional<QualityLevel> ParseQualityLevel(const std::string_view value) noexcept
{
  const auto normalized = Normalize(value);
  if (normalized == "below_minimum") {
    return QualityLevel::kBelowMinimum;
  }
  if (normalized == "minimum") {
    return QualityLevel::kMinimum;
  }
  if (normalized == "good") {
    return QualityLevel::kGood;
  }
  if (normalized == "excellent") {
    return QualityLevel::kExcellent;
  }
  return std::nullopt;
}

std::optional<StructuredHealthEvaluation> EvaluateStructuredHealthPayload(
  const std::string_view payload) noexcept
{
  const auto first_content = std::find_if_not(
    payload.begin(), payload.end(),
    [](const unsigned char character) {return std::isspace(character) != 0;});
  if (first_content == payload.end() || *first_content != '{') {
    return std::nullopt;
  }

  const auto invalid = []() {return StructuredHealthEvaluation{};};
  try {
    const Json object = Json::parse(payload, nullptr, false);
    if (object.is_discarded() || !object.is_object()) {
      return invalid();
    }

    bool has_authoritative_health = false;
    bool authoritative_health = true;
    for (const auto * field : {"overall_ok", "startup_ready", "ready", "ok", "healthy"}) {
      if (!object.contains(field)) {
        continue;
      }
      has_authoritative_health = true;
      if (!object.at(field).is_boolean()) {
        return invalid();
      }
      authoritative_health = authoritative_health && object.at(field).get<bool>();
    }
    if (!has_authoritative_health) {
      return std::nullopt;
    }

    bool authoritative_failure = false;
    for (const auto * field : {"overall_status", "status", "state"}) {
      if (!object.contains(field)) {
        continue;
      }
      if (!object.at(field).is_string()) {
        return invalid();
      }
      const auto status = Normalize(object.at(field).get<std::string>());
      authoritative_failure = authoritative_failure ||
        status == "blocked" || status == "critical" || status == "error" ||
        status == "fault" || status == "stale" || status == "unknown";
    }

    std::optional<QualityLevel> quality;
    for (const auto * field : {"quality", "rate_quality"}) {
      if (!object.contains(field)) {
        continue;
      }
      if (!object.at(field).is_string()) {
        return invalid();
      }
      const auto parsed = ParseQualityLevel(object.at(field).get<std::string>());
      if (!parsed) {
        return invalid();
      }
      quality = quality ? std::min(*quality, *parsed) : *parsed;
    }

    StructuredHealthEvaluation evaluation;
    evaluation.valid = true;
    evaluation.failed = !authoritative_health || authoritative_failure;
    evaluation.ready = !evaluation.failed &&
      quality != QualityLevel::kBelowMinimum;
    evaluation.quality = quality;
    return evaluation;
  } catch (const Json::exception &) {
    return invalid();
  } catch (...) {
    return invalid();
  }
}

bool ValidateStartupStageTiming(const StartupStageTiming & timing) noexcept
{
  return std::isfinite(timing.minimum_settle_s) &&
         std::isfinite(timing.stable_ready_s) &&
         std::isfinite(timing.startup_timeout_s) &&
         timing.minimum_settle_s >= 0.0 &&
         timing.stable_ready_s >= 0.0 &&
         timing.startup_timeout_s > timing.minimum_settle_s + timing.stable_ready_s;
}

QualityLevel WorstRequiredQuality(
  const std::vector<QualityLevel> & required_quality,
  const bool dependencies_ready) noexcept
{
  if (!dependencies_ready) {
    return QualityLevel::kBelowMinimum;
  }
  if (required_quality.empty()) {
    return QualityLevel::kMinimum;
  }
  return *std::min_element(required_quality.begin(), required_quality.end());
}

StartupStageTracker::StartupStageTracker(StartupStageTiming timing)
: timing_(timing)
{
}

const StartupStageTiming & StartupStageTracker::timing() const noexcept
{
  return timing_;
}

StartupStageDecision StartupStageTracker::Update(
  const double elapsed_s,
  const StartupStageInput & input)
{
  StartupStageDecision decision;
  decision.quality = input.dependencies_ready ? input.quality : QualityLevel::kBelowMinimum;

  if (input.shutting_down) {
    decision.state = StartupStageState::kShuttingDown;
    decision.reason = "shutdown_requested";
    return decision;
  }
  if (terminal_failed_) {
    decision.state = StartupStageState::kFailed;
    decision.failed = true;
    decision.quality = QualityLevel::kBelowMinimum;
    decision.reason = terminal_failure_reason_;
    return decision;
  }
  if (terminal_ready_) {
    decision.state = StartupStageState::kReady;
    decision.ready = true;
    decision.quality = input.dependencies_ready ? input.quality : QualityLevel::kBelowMinimum;
    decision.reason = input.dependencies_ready ? "stable_ready" : input.reason;
    return decision;
  }
  if (!ValidateStartupStageTiming(timing_) || !input.configuration_valid ||
    input.unrecoverable_failure)
  {
    terminal_failed_ = true;
    terminal_failure_reason_ = !input.configuration_valid ? "invalid_stage_configuration" :
      (input.reason.empty() ? "unrecoverable_stage_failure" : input.reason);
    decision.state = StartupStageState::kFailed;
    decision.failed = true;
    decision.reason = terminal_failure_reason_;
    return decision;
  }
  if (!std::isfinite(elapsed_s) || elapsed_s < 0.0) {
    terminal_failed_ = true;
    terminal_failure_reason_ = "invalid_stage_clock";
    decision.state = StartupStageState::kFailed;
    decision.failed = true;
    decision.reason = terminal_failure_reason_;
    return decision;
  }
  if (elapsed_s >= timing_.startup_timeout_s) {
    terminal_failed_ = true;
    terminal_failure_reason_ = input.reason.empty() ? "stage_startup_timeout" :
      "stage_startup_timeout:" + input.reason;
    decision.state = StartupStageState::kFailed;
    decision.failed = true;
    decision.reason = terminal_failure_reason_;
    return decision;
  }
  if (!input.processes_started || elapsed_s < timing_.minimum_settle_s) {
    stable_since_s_ = -1.0;
    decision.state = input.processes_started ?
      StartupStageState::kWaitingForDependencies : StartupStageState::kStarting;
    decision.reason = input.processes_started ? "minimum_settle_pending" :
      "waiting_for_processes";
    return decision;
  }
  if (!input.dependencies_ready || input.quality == QualityLevel::kBelowMinimum) {
    stable_since_s_ = -1.0;
    decision.state = StartupStageState::kWaitingForDependencies;
    decision.reason = input.quality == QualityLevel::kBelowMinimum ?
      "required_quality_below_minimum" :
      (input.reason.empty() ? "waiting_for_dependencies" : input.reason);
    return decision;
  }
  if (stable_since_s_ < 0.0) {
    stable_since_s_ = elapsed_s;
  }
  decision.stable_for_s = std::max(0.0, elapsed_s - stable_since_s_);
  if (decision.stable_for_s >= timing_.stable_ready_s) {
    terminal_ready_ = true;
    decision.state = StartupStageState::kReady;
    decision.ready = true;
    decision.reason = "stable_ready";
    return decision;
  }
  decision.state = StartupStageState::kStabilizing;
  decision.reason = "waiting_for_stable_ready_window";
  return decision;
}

EstablishedDependencyTracker::EstablishedDependencyTracker(
  const std::size_t confirmation_samples)
: confirmation_samples_(std::max<std::size_t>(1U, confirmation_samples))
{
}

EstablishedDependencyDecision EstablishedDependencyTracker::Update(
  const std::string_view loss_signature) noexcept
{
  if (loss_signature.empty()) {
    consecutive_loss_samples_ = 0U;
    last_loss_signature_.clear();
    return {};
  }
  if (loss_signature != last_loss_signature_) {
    consecutive_loss_samples_ = 0U;
    last_loss_signature_ = loss_signature;
  }
  if (consecutive_loss_samples_ < confirmation_samples_) {
    ++consecutive_loss_samples_;
  }
  return {true, consecutive_loss_samples_ >= confirmation_samples_};
}

std::size_t EstablishedDependencyTracker::confirmation_samples() const noexcept
{
  return confirmation_samples_;
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
