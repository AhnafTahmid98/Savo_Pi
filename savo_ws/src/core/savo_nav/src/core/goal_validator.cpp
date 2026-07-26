// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/goal_validator.hpp"

#include <cmath>

namespace
{

inline constexpr double kPi =
  3.14159265358979323846;

bool IsFinitePose(
  const savo_nav::GoalPose2D & pose) noexcept
{
  return
    std::isfinite(pose.x) &&
    std::isfinite(pose.y) &&
    std::isfinite(pose.yaw);
}

}  // namespace

namespace savo_nav
{

ValidationResult GoalValidator::Validate(
  const GoalValidationRequest & request,
  const GoalValidationPolicy & policy)
{
  const auto context_validation =
    GoalContextContract::Validate(request.context);

  if (!context_validation.IsValid()) {
    return context_validation;
  }

  if (
    !std::isfinite(policy.max_abs_coordinate_m) ||
    policy.max_abs_coordinate_m <= 0.0)
  {
    return {
      ValidationCode::kInvalidCombination,
      "goal_validation_coordinate_limit_is_invalid"
    };
  }

  if (request.context.cancel_requested) {
    return {
      ValidationCode::kCancellationRequested,
      "new_goal_contains_cancel_requested_flag"
    };
  }

  if (!IsFinitePose(request.pose)) {
    return {
      ValidationCode::kInvalidPose,
      "goal_pose_contains_nonfinite_value"
    };
  }

  if (
    std::abs(request.pose.x) >
    policy.max_abs_coordinate_m ||
    std::abs(request.pose.y) >
    policy.max_abs_coordinate_m)
  {
    return {
      ValidationCode::kCoordinateOutOfBounds,
      "goal_pose_exceeds_coordinate_limit"
    };
  }

  if (
    request.pose.yaw < -kPi ||
    request.pose.yaw > kPi)
  {
    return {
      ValidationCode::kInvalidPose,
      "goal_yaw_must_be_normalized"
    };
  }

  const auto map_validation =
    MapContextContract::Validate(
    request.map_context);

  if (!map_validation.IsValid()) {
    return map_validation;
  }

  if (!request.map_context.available) {
    return {
      ValidationCode::kMapUnavailable,
      "goal_validation_map_is_unavailable"
    };
  }

  if (policy.require_readiness) {
    const bool accepted_state =
      request.readiness.state ==
      NavigationReadinessState::kReady ||
      (
      policy.allow_degraded_readiness &&
      request.readiness.state ==
      NavigationReadinessState::kDegraded
      );

    if (
      !request.readiness.goal_acceptance_allowed ||
      !accepted_state)
    {
      return {
        ValidationCode::kNotReady,
        "navigation_readiness_rejects_goal"
      };
    }
  }

  if (
    policy.require_map_id_match &&
    request.map_context.mode ==
    NavigationMapMode::kSavedMap)
  {
    if (request.context.map_id.empty()) {
      return {
        ValidationCode::kMissingMapId,
        "saved_map_goal_requires_map_id"
      };
    }

    if (
      request.context.map_id !=
      request.map_context.map_id)
    {
      return {
        ValidationCode::kMapMismatch,
        "goal_map_id_does_not_match_active_map"
      };
    }
  }

  return {};
}

}  // namespace savo_nav
