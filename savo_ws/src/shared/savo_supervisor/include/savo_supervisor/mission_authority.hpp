// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>

#include "savo_supervisor/supervisor_state.hpp"

namespace savo_supervisor
{

enum class MissionOperation : std::uint8_t
{
  kNone = 0U,
  kManualControl,
  kManualMapping,
  kAutonomousMapping,
  kScan360,
  kCoverage,
  kNavigateToPose,
  kNavigateToLocation,
  kRegisterLocation,
  kReviewLocation,
  kConfirmArrival,
};

enum class AuthorityCommand : std::uint8_t
{
  kCheck = 0U,
  kAcquire,
  kRelease,
  kPause,
  kResume,
};

enum class OperationState : std::uint8_t
{
  kIdle = 0U,
  kActive,
  kPaused,
  kRevoked,
};

enum class MapContextType : std::uint8_t
{
  kNone = 0U,
  kLiveMapping,
  kSavedRelease,
};

enum class MissionAuthorizationCode : std::uint8_t
{
  kAuthorized = 0U,
  kInvalidRequest,
  kNotReady,
  kHealthBlocked,
  kSafetyBlocked,
  kDependencyBlocked,
  kMapContextBlocked,
  kOperationConflict,
  kOwnershipMismatch,
  kOperationDisabled,
};

struct MappingObservation
{
  bool received{false};
  bool fresh{false};
  bool valid{false};
  bool healthy{false};
  bool ready{false};
  bool slam_active{false};
  std::string mode{"unknown"};
  std::string workflow_phase{"unknown"};
  std::string session_state{"unknown"};
  std::string active_map_name{};
  std::string reason{"mapping_not_observed"};
};

struct NavigationObservation
{
  bool received{false};
  bool fresh{false};
  bool valid{false};
  bool ready{false};
  bool goal_acceptance_allowed{false};
  std::string state{"OFFLINE"};
  std::string reason{"navigation_not_observed"};
};

struct HeadObservation
{
  bool received{false};
  bool fresh{false};
  bool valid{false};
  bool operational{false};
  bool pan_tilt_ready{false};
  bool camera_ready{false};
  bool camera_pose_ready{false};
  std::string reason{"head_not_observed"};
};

struct LocationsObservation
{
  bool received{false};
  bool fresh{false};
  bool valid{false};
  bool read_ready{false};
  bool write_ready{false};
  bool storage_healthy{false};
  bool mutation_in_progress{false};
  std::string reason{"locations_not_observed"};
};

struct EndpointAvailability
{
  bool autonomous_mapping_action{false};
  bool rotate_to_heading_action{false};
  bool coverage_action{false};
  bool apriltag_confirmation_action{false};
};

struct ActiveMapContext
{
  MapContextType type{MapContextType::kNone};
  std::string map_id{};
  std::uint32_t map_revision{0U};
  std::string map_release_id{};
  std::string mapping_session_id{};
  bool approved{false};
  std::uint64_t generation{0U};
};

struct MissionCapabilities
{
  bool mapping_available{false};
  bool navigation_ready{false};
  bool head_ready{false};
  bool locations_read_ready{false};
  bool locations_write_ready{false};
  bool semantic_mapping_ready{false};
  bool can_start_manual_mapping{false};
  bool can_start_autonomous_mapping{false};
  bool can_run_scan360{false};
  bool can_run_coverage{false};
  bool can_navigate{false};
  bool can_register_location{false};
  bool can_review_location{false};
  bool can_confirm_arrival{false};
};

struct SystemMissionGate
{
  bool armed{false};
  bool fault_latched{false};
  bool shutdown_requested{false};
  bool remote_commands_ready{false};
};

struct MissionDependencySnapshot
{
  SupervisorState core{};
  MappingObservation mapping{};
  NavigationObservation navigation{};
  HeadObservation head{};
  LocationsObservation locations{};
  EndpointAvailability endpoints{};
  ActiveMapContext map_context{};
  SystemMissionGate system{};
};

struct MissionAuthorityPolicy
{
  bool allow_manual_control{true};
  bool allow_manual_mapping{true};
  bool allow_autonomous_mapping{true};
  bool allow_scan360{true};
  bool allow_coverage{true};
  bool allow_navigation{true};
  bool allow_location_registration{true};
  bool allow_location_review{true};
  bool allow_arrival_confirmation{true};
  bool require_semantic_autonomous_mapping{true};
  bool require_approved_release_for_navigation{true};
};

struct MissionAuthorizationRequest
{
  AuthorityCommand command{AuthorityCommand::kCheck};
  MissionOperation operation{MissionOperation::kNone};
  std::string request_id{};
  std::string actor_id{};
  std::string map_id{};
  std::uint32_t map_revision{0U};
  std::string map_release_id{};
  bool require_semantic{false};
  bool motion_required{false};
  bool remote_origin{false};
  std::uint64_t expected_generation{0U};
};

struct MissionAuthorizationDecision
{
  bool authorized{false};
  MissionAuthorizationCode code{MissionAuthorizationCode::kInvalidRequest};
  std::string reason{"not_evaluated"};
};

struct MissionAuthorityState
{
  MissionOperation operation{MissionOperation::kNone};
  OperationState state{OperationState::kIdle};
  std::string request_id{};
  std::string actor_id{};
  std::string map_id{};
  std::uint32_t map_revision{0U};
  std::string map_release_id{};
  bool semantic_required{false};
  bool remote_origin{false};
  std::uint64_t generation{0U};
  std::string reason{"idle"};
};

class MissionAuthority
{
public:
  explicit MissionAuthority(MissionAuthorityPolicy policy = {});

  [[nodiscard]] MissionCapabilities EvaluateCapabilities(
    const MissionDependencySnapshot & dependencies) const;

  [[nodiscard]] MissionAuthorizationDecision Handle(
    const MissionAuthorizationRequest & request,
    const MissionDependencySnapshot & dependencies);

  [[nodiscard]] bool Revalidate(
    const MissionDependencySnapshot & dependencies);

  [[nodiscard]] const MissionAuthorityState & state() const noexcept;

private:
  [[nodiscard]] MissionAuthorizationDecision CheckOperation(
    const MissionAuthorizationRequest & request,
    const MissionDependencySnapshot & dependencies) const;

  MissionAuthorityPolicy policy_{};
  MissionAuthorityState state_{};
};

[[nodiscard]] bool IsExclusiveOperation(MissionOperation operation) noexcept;
[[nodiscard]] OperatingMode ModeForAuthority(
  const SupervisorState & core,
  const MissionAuthorityState & authority) noexcept;
[[nodiscard]] const char * ToString(MissionOperation value) noexcept;
[[nodiscard]] const char * ToString(OperationState value) noexcept;
[[nodiscard]] const char * ToString(MapContextType value) noexcept;

}  // namespace savo_supervisor
