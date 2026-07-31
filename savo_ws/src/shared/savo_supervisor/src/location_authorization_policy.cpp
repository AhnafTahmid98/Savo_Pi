// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/location_authorization_policy.hpp"

#include <utility>

namespace savo_supervisor
{
namespace
{

LocationAuthorizationDecision reject(
  const LocationAuthorizationCode code,
  std::string reason)
{
  return {false, code, std::move(reason)};
}

bool operation_enabled(
  const LocationAuthorizationPolicy & policy,
  const LocationOperation operation)
{
  switch (operation) {
    case LocationOperation::kRegisterCandidate:
      return policy.allow_registration;
    case LocationOperation::kApproveLocation:
      return policy.allow_approval;
    case LocationOperation::kNavigateToLocation:
      return policy.allow_navigation;
    case LocationOperation::kConfirmArrival:
      return policy.allow_arrival_confirmation;
    case LocationOperation::kRejectLocationCandidate:
      return policy.allow_rejection;
  }
  return false;
}

}  // namespace

LocationAuthorizationEvaluator::LocationAuthorizationEvaluator(
  LocationAuthorizationPolicy policy)
: policy_(std::move(policy))
{
}

LocationAuthorizationDecision LocationAuthorizationEvaluator::Evaluate(
  const LocationAuthorizationRequest & request,
  const SupervisorState & state) const
{
  if (request.request_id.empty() || request.actor_id.empty() ||
    request.map_id.empty() || request.map_revision == 0U)
  {
    return reject(
      LocationAuthorizationCode::kInvalidRequest,
      "request_actor_and_map_context_required");
  }

  if (
    (request.operation == LocationOperation::kRegisterCandidate ||
    request.operation == LocationOperation::kApproveLocation ||
    request.operation == LocationOperation::kRejectLocationCandidate) &&
    request.candidate_id.empty())
  {
    return reject(
      LocationAuthorizationCode::kInvalidRequest,
      "candidate_identity_required");
  }

  if (
    (request.operation == LocationOperation::kNavigateToLocation ||
    request.operation == LocationOperation::kConfirmArrival) &&
    request.location_id.empty())
  {
    return reject(
      LocationAuthorizationCode::kInvalidRequest,
      "location_identity_required");
  }

  if (!operation_enabled(policy_, request.operation)) {
    return reject(
      LocationAuthorizationCode::kOperationDisabled,
      "location_operation_disabled_by_policy");
  }

  if (state.lifecycle != Lifecycle::RUNNING || !state.ready) {
    return reject(
      LocationAuthorizationCode::kSupervisorNotReady,
      "supervisor_not_ready");
  }

  if (state.health == AggregateHealth::ERROR ||
    state.health == AggregateHealth::UNKNOWN)
  {
    return reject(
      LocationAuthorizationCode::kHealthBlocked,
      "aggregate_health_blocks_operation");
  }

  if (state.health == AggregateHealth::DEGRADED) {
    const bool allowed = request.motion_required ?
      policy_.allow_degraded_motion :
      policy_.allow_degraded_non_motion;

    if (!allowed) {
      return reject(
        LocationAuthorizationCode::kHealthBlocked,
        "degraded_health_not_authorized");
    }
  }

  if (request.motion_required) {
    if (state.safety == SafetyObservation::STOPPED) {
      return reject(
        LocationAuthorizationCode::kSafetyBlocked,
        "safety_stop_active");
    }

    if (policy_.require_known_safety_for_motion &&
      state.safety == SafetyObservation::UNKNOWN)
    {
      return reject(
        LocationAuthorizationCode::kSafetyBlocked,
        "safety_state_unknown");
    }
  }

  return {
    true,
    LocationAuthorizationCode::kAuthorized,
    "location_operation_authorized"};
}

}  // namespace savo_supervisor
