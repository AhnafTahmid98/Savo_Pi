// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/coverage_path_validator.hpp"

#include <cctype>
#include <cmath>
#include <string>
#include <utility>

#include "savo_nav/frame_names.hpp"

namespace
{

savo_nav::CoveragePathValidationResult MakeFailure(
  const savo_nav::ValidationCode code,
  std::string reason)
{
  savo_nav::CoveragePathValidationResult result;

  result.validation.code = code;
  result.validation.reason = std::move(reason);

  return result;
}

savo_nav::CoveragePathValidationResult MakeFailure(
  const savo_nav::ValidationResult & validation)
{
  savo_nav::CoveragePathValidationResult result;
  result.validation = validation;

  return result;
}

bool ContainsWhitespace(const std::string & value)
{
  for (const unsigned char character : value) {
    if (std::isspace(character) != 0) {
      return true;
    }
  }

  return false;
}

bool IsFinitePositive(const double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

bool IsPolicyValid(
  const savo_nav::CoveragePathValidationPolicy & policy)
{
  return
    policy.expected_contract_version != 0 &&
    policy.maximum_waypoints != 0 &&
    policy.maximum_mission_id_length != 0 &&
    IsFinitePositive(
      policy.maximum_absolute_coordinate_m) &&
    std::isfinite(policy.maximum_absolute_z_m) &&
    policy.maximum_absolute_z_m >= 0.0 &&
    IsFinitePositive(
      policy.maximum_total_distance_m) &&
    IsFinitePositive(
      policy.maximum_execution_timeout_seconds) &&
    IsFinitePositive(
      policy.quaternion_norm_tolerance) &&
    std::isfinite(
      policy.planar_quaternion_tolerance) &&
    policy.planar_quaternion_tolerance >= 0.0;
}

bool IsPointFinite(
  const savo_nav::CoveragePathPoint & point) noexcept
{
  return
    std::isfinite(point.x) &&
    std::isfinite(point.y) &&
    std::isfinite(point.z) &&
    std::isfinite(point.orientation_x) &&
    std::isfinite(point.orientation_y) &&
    std::isfinite(point.orientation_z) &&
    std::isfinite(point.orientation_w);
}

bool ReadinessAllowsCoverage(
  const savo_nav::NavigationReadinessResult & readiness,
  const savo_nav::CoveragePathValidationPolicy & policy)
{
  const bool accepted_state =
    readiness.state ==
    savo_nav::NavigationReadinessState::kReady ||
    (
    policy.allow_degraded_readiness &&
    readiness.state ==
    savo_nav::NavigationReadinessState::kDegraded
    );

  return
    readiness.goal_acceptance_allowed &&
    accepted_state;
}

}  // namespace

namespace savo_nav
{

CoveragePathValidationResult CoveragePathValidator::Validate(
  const CoveragePathValidationRequest & request,
  const CoveragePathValidationPolicy & policy)
{
  if (!IsPolicyValid(policy)) {
    return MakeFailure(
      ValidationCode::kInvalidCombination,
      "coverage_path_validation_policy_is_invalid");
  }

  if (
    request.contract_version !=
    policy.expected_contract_version)
  {
    return MakeFailure(
      ValidationCode::kInvalidCombination,
      "coverage_contract_version_mismatch");
  }

  if (request.mission_id.empty()) {
    return MakeFailure(
      ValidationCode::kEmptyIdentifier,
      "coverage_mission_id_is_empty");
  }

  if (
    request.mission_id.size() >
    policy.maximum_mission_id_length)
  {
    return MakeFailure(
      ValidationCode::kInvalidIdentifier,
      "coverage_mission_id_exceeds_length_limit");
  }

  if (ContainsWhitespace(request.mission_id)) {
    return MakeFailure(
      ValidationCode::kInvalidIdentifier,
      "coverage_mission_id_contains_whitespace");
  }

  if (request.path_frame != frames::kMap) {
    return MakeFailure(
      ValidationCode::kInvalidFrame,
      "coverage_path_frame_must_be_map");
  }

  if (request.points.empty()) {
    return MakeFailure(
      ValidationCode::kInvalidCombination,
      "coverage_path_is_empty");
  }

  if (
    request.points.size() >
    policy.maximum_waypoints)
  {
    return MakeFailure(
      ValidationCode::kInvalidCombination,
      "coverage_path_exceeds_waypoint_limit");
  }

  if (
    !std::isfinite(
      request.execution_timeout_seconds) ||
    request.execution_timeout_seconds < 0.0 ||
    request.execution_timeout_seconds >
    policy.maximum_execution_timeout_seconds)
  {
    return MakeFailure(
      ValidationCode::kInvalidCombination,
      "coverage_execution_timeout_is_invalid");
  }

  const auto map_validation =
    MapContextContract::Validate(
    request.map_context);

  if (!map_validation.IsValid()) {
    return MakeFailure(map_validation);
  }

  if (!request.map_context.available) {
    return MakeFailure(
      ValidationCode::kMapUnavailable,
      "coverage_active_map_is_unavailable");
  }

  if (
    policy.require_readiness &&
    !ReadinessAllowsCoverage(
      request.readiness,
      policy))
  {
    return MakeFailure(
      ValidationCode::kNotReady,
      "navigation_readiness_rejects_coverage_path");
  }

  double total_distance_m = 0.0;

  for (
    std::size_t index = 0;
    index < request.points.size();
    ++index)
  {
    const auto & point = request.points[index];

    if (
      !point.frame_id.empty() &&
      point.frame_id != request.path_frame)
    {
      return MakeFailure(
        ValidationCode::kInvalidFrame,
        "coverage_waypoint_frame_mismatch");
    }

    if (!IsPointFinite(point)) {
      return MakeFailure(
        ValidationCode::kInvalidPose,
        "coverage_waypoint_contains_nonfinite_value");
    }

    if (
      std::abs(point.x) >
      policy.maximum_absolute_coordinate_m ||
      std::abs(point.y) >
      policy.maximum_absolute_coordinate_m ||
      std::abs(point.z) >
      policy.maximum_absolute_z_m)
    {
      return MakeFailure(
        ValidationCode::kCoordinateOutOfBounds,
        "coverage_waypoint_exceeds_coordinate_limit");
    }

    if (
      std::abs(point.orientation_x) >
      policy.planar_quaternion_tolerance ||
      std::abs(point.orientation_y) >
      policy.planar_quaternion_tolerance)
    {
      return MakeFailure(
        ValidationCode::kInvalidPose,
        "coverage_waypoint_orientation_is_not_planar");
    }

    const double quaternion_norm =
      std::sqrt(
      point.orientation_x *
      point.orientation_x +
      point.orientation_y *
      point.orientation_y +
      point.orientation_z *
      point.orientation_z +
      point.orientation_w *
      point.orientation_w);

    if (
      !std::isfinite(quaternion_norm) ||
      std::abs(quaternion_norm - 1.0) >
      policy.quaternion_norm_tolerance)
    {
      return MakeFailure(
        ValidationCode::kInvalidPose,
        "coverage_waypoint_orientation_is_not_normalized");
    }

    if (index == 0) {
      continue;
    }

    const auto & previous =
      request.points[index - 1];

    const double segment_distance_m =
      std::hypot(
      point.x - previous.x,
      point.y - previous.y);

    total_distance_m += segment_distance_m;

    if (
      !std::isfinite(total_distance_m) ||
      total_distance_m >
      policy.maximum_total_distance_m)
    {
      return MakeFailure(
        ValidationCode::kInvalidCombination,
        "coverage_path_exceeds_distance_limit");
    }
  }

  CoveragePathValidationResult result;

  result.validation.code = ValidationCode::kValid;
  result.validation.reason = "coverage_path_valid";
  result.total_distance_m = total_distance_m;

  return result;
}

}  // namespace savo_nav
