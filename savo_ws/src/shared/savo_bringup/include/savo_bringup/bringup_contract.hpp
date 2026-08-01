// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_BRINGUP__BRINGUP_CONTRACT_HPP_
#define SAVO_BRINGUP__BRINGUP_CONTRACT_HPP_

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace savo_bringup
{

enum class HostRole
{
  kCore,
  kEdge,
  kAll,
};

enum class RobotMode
{
  kSafeIdle,
  kManual,
  kManualMapping,
  kAutonomousMapping,
  kSavedMapNavigation,
  kDiagnostics,
};

enum class BringupProfile
{
  kBench,
  kLidarOnly,
  kLidarD435Voxel,
  kProduction,
};

enum class ReadinessState
{
  kStarting,
  kWaitingForDependencies,
  kValidatingGeometry,
  kWaitingForSafety,
  kWaitingForLocalization,
  kWaitingForMapContext,
  kWaitingForNavigation,
  kReady,
  kDegraded,
  kBlocked,
  kShuttingDown,
};

struct DependencyStatus
{
  std::string name;
  ReadinessState waiting_state{ReadinessState::kWaitingForDependencies};
  bool required{true};
  bool observed{false};
  bool fresh{false};
  bool ready{false};
  bool failed{false};
  std::string detail{"not_observed"};
};

struct ReadinessDecision
{
  ReadinessState state{ReadinessState::kStarting};
  bool ready{false};
  std::vector<std::string> missing;
  std::vector<std::string> failed;
  std::vector<std::string> degraded;
};

struct BringupRequirements
{
  bool core_required{false};
  bool edge_required{false};
  bool supervisor_required{false};
  bool navigation_required{false};
  bool mapping_required{false};
  bool bridge_required{false};
  bool realsense_required{false};
  bool vo_required{false};
  bool speech_required{false};
  bool locked_geometry_required{false};
  bool motion_capable{false};
  bool voxel_layer_enabled{false};
};

std::optional<HostRole> ParseHostRole(std::string_view value) noexcept;
std::optional<RobotMode> ParseRobotMode(std::string_view value) noexcept;
std::optional<BringupProfile> ParseBringupProfile(std::string_view value) noexcept;
std::optional<ReadinessState> ParseReadinessState(std::string_view value) noexcept;

std::string_view ToString(HostRole value) noexcept;
std::string_view ToString(RobotMode value) noexcept;
std::string_view ToString(BringupProfile value) noexcept;
std::string_view ToString(ReadinessState value) noexcept;

ReadinessDecision EvaluateReadiness(
  const std::vector<DependencyStatus> & dependencies,
  bool configuration_valid,
  bool startup_timeout_expired,
  bool shutting_down = false);

BringupRequirements RequirementsFor(
  HostRole role,
  RobotMode mode,
  BringupProfile profile,
  bool start_bridge,
  bool start_realsense,
  bool start_vo,
  bool start_speech) noexcept;

std::string ValidateCombination(
  HostRole role,
  RobotMode mode,
  BringupProfile profile,
  bool d435_voxel_validated,
  bool require_locked_geometry,
  bool allow_provisional_geometry);

}  // namespace savo_bringup

#endif  // SAVO_BRINGUP__BRINGUP_CONTRACT_HPP_
