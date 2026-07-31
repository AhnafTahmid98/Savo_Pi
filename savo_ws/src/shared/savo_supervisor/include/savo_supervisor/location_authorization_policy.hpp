// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>

#include "savo_supervisor/supervisor_state.hpp"

namespace savo_supervisor
{

enum class LocationOperation : std::uint8_t
{
  kRegisterCandidate = 1U,
  kApproveLocation = 2U,
  kNavigateToLocation = 3U,
  kConfirmArrival = 4U,
  kRejectLocationCandidate = 5U,
};

enum class LocationAuthorizationCode : std::uint8_t
{
  kAuthorized = 0U,
  kInvalidRequest,
  kSupervisorNotReady,
  kHealthBlocked,
  kSafetyBlocked,
  kMapContextBlocked,
  kOperationDisabled,
};

struct LocationAuthorizationRequest
{
  LocationOperation operation{LocationOperation::kRegisterCandidate};
  std::string request_id{};
  std::string actor_id{};
  std::string candidate_id{};
  std::string location_id{};
  std::string map_id{};
  std::uint32_t map_revision{0U};
  bool motion_required{false};
};

struct LocationAuthorizationPolicy
{
  bool allow_registration{true};
  bool allow_approval{true};
  bool allow_rejection{true};
  bool allow_navigation{true};
  bool allow_arrival_confirmation{true};
  bool allow_degraded_non_motion{true};
  bool allow_degraded_motion{false};
  bool require_known_safety_for_motion{false};
};

struct LocationAuthorizationDecision
{
  bool authorized{false};
  LocationAuthorizationCode code{
    LocationAuthorizationCode::kInvalidRequest};
  std::string reason{"not_evaluated"};
};

class LocationAuthorizationEvaluator
{
public:
  explicit LocationAuthorizationEvaluator(
    LocationAuthorizationPolicy policy = {});

  [[nodiscard]] LocationAuthorizationDecision Evaluate(
    const LocationAuthorizationRequest & request,
    const SupervisorState & state) const;

private:
  LocationAuthorizationPolicy policy_{};
};

}  // namespace savo_supervisor
