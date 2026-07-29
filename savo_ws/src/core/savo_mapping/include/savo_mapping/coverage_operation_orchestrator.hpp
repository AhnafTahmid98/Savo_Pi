// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>

namespace savo_mapping::coverage
{

inline constexpr char kCoverageOperationStateTopic[] =
  "/savo_mapping/coverage_operation/state";
inline constexpr char kCoverageOperationStatusTopic[] =
  "/savo_mapping/coverage_operation/status";
inline constexpr char kCoverageOperationEventsTopic[] =
  "/savo_mapping/coverage_operation/events";
inline constexpr char kCoverageOperationApproveService[] =
  "/savo_mapping/coverage_operation/approve";
inline constexpr char kCoverageOperationCancelService[] =
  "/savo_mapping/coverage_operation/cancel";
inline constexpr char kCoverageOperationResetService[] =
  "/savo_mapping/coverage_operation/reset";
inline constexpr char kInternalCoverageApproveService[] =
  "/savo_mapping/_internal/coverage_execution/approve";
inline constexpr char kInternalCoverageCancelService[] =
  "/savo_mapping/_internal/coverage_execution/cancel";
inline constexpr char kInternalCoverageResetService[] =
  "/savo_mapping/_internal/coverage_execution/reset";
inline constexpr char kSupervisorStateTopic[] =
  "/savo_supervisor/state_summary";

struct CoverageOperationPolicy
{
  double supervisor_timeout_sec{1.5};
  double maximum_candidate_age_sec{300.0};
  double internal_service_timeout_sec{2.0};
  double supervisor_loss_cancel_delay_sec{0.5};
  bool allow_degraded_supervisor{false};
  bool cancel_on_supervisor_loss{true};
};

struct SupervisorAuthorizationSnapshot
{
  bool valid{false};
  bool ready{false};
  bool degraded{false};
  std::string lifecycle;
  std::string health;
  std::string reason;
};

struct CoverageHandoffSnapshot
{
  bool valid{false};
  bool enabled{false};
  bool candidate_valid{false};
  std::uint64_t candidate_generation{0U};
  double candidate_age_sec{0.0};
  std::string state;
  std::string reason;
  std::string mission_id;
  std::string terminal_state;
  std::string result_reason;
};

struct CoverageApprovalDecision
{
  bool accepted{false};
  std::string reason;
  std::uint64_t candidate_generation{0U};
};

std::string validate_coverage_operation_policy(
  const CoverageOperationPolicy & policy);

SupervisorAuthorizationSnapshot parse_supervisor_authorization(
  const std::string & payload);

CoverageHandoffSnapshot parse_coverage_handoff_snapshot(
  const std::string & payload);

bool supervisor_authorized(
  const SupervisorAuthorizationSnapshot & snapshot,
  const CoverageOperationPolicy & policy);

CoverageApprovalDecision evaluate_coverage_approval(
  const SupervisorAuthorizationSnapshot & supervisor,
  double supervisor_age_sec,
  const CoverageHandoffSnapshot & handoff,
  double current_candidate_age_sec,
  const CoverageOperationPolicy & policy);

bool coverage_handoff_state_active(const std::string & state);
bool coverage_handoff_state_terminal(const std::string & state);

bool mission_id_matches_candidate_generation(
  const std::string & mission_id,
  std::uint64_t candidate_generation);

std::string coverage_operation_effective_state(
  bool enabled,
  bool approval_pending,
  bool supervisor_is_authorized,
  const CoverageHandoffSnapshot & handoff);

}  // namespace savo_mapping::coverage
