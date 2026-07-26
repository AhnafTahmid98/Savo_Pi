// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_localization/localization_health_core.hpp"

#include <array>
#include <utility>

namespace savo_localization
{
namespace
{

std::string reason_with_detail(
  const std::string & name,
  const std::string & suffix,
  const std::string & detail)
{
  std::string reason = name + "_" + suffix;
  if (!detail.empty()) {
    reason += ":" + detail;
  }
  return reason;
}

LocalizationHealthResult make_result(
  const LocalizationHealthState state,
  const bool ready,
  const bool degraded,
  std::string reason_code,
  std::vector<std::string> reasons)
{
  LocalizationHealthResult result;
  result.state = state;
  result.ready = ready;
  result.degraded = degraded;
  result.reason_code = std::move(reason_code);
  result.reasons = std::move(reasons);
  return result;
}

}  // namespace

LocalizationHealthResult LocalizationHealthCore::Evaluate(
  const LocalizationHealthInputs & inputs) const
{
  const std::array<const SourceHealthObservation *, 4> sources{
    &inputs.imu,
    &inputs.wheel_odom,
    &inputs.filtered_odom,
    &inputs.vo_odom};

  std::vector<std::string> errors;
  std::vector<std::string> missing_or_stale;
  std::vector<std::string> degraded;

  for (const auto * source : sources) {
    if (source == nullptr || !source->enabled) {
      continue;
    }

    AppendSourceErrors(*source, errors);
    AppendSourceStaleReasons(*source, missing_or_stale);
    AppendSourceDegradedReasons(*source, degraded);
  }

  if (inputs.filtered_pose_jump_detected) {
    errors.emplace_back("filtered_odom_pose_jump_detected");
  }
  if (inputs.filtered_yaw_jump_detected) {
    errors.emplace_back("filtered_odom_yaw_jump_detected");
  }

  if (!errors.empty()) {
    return make_result(
      LocalizationHealthState::kError,
      false,
      false,
      "invalid_localization_data",
      std::move(errors));
  }

  const std::array<const TransformHealthObservation *, 2> transforms{
    &inputs.odom_to_base,
    &inputs.base_to_imu};

  for (const auto * transform : transforms) {
    if (transform == nullptr || !transform->required) {
      continue;
    }

    if (!transform->available) {
      missing_or_stale.push_back(
        reason_with_detail(transform->name, "unavailable", transform->detail));
    } else if (!transform->fresh) {
      missing_or_stale.push_back(
        reason_with_detail(transform->name, "stale", transform->detail));
    }
  }

  if (!missing_or_stale.empty()) {
    if (inputs.startup_age_s < inputs.startup_grace_s) {
      return make_result(
        LocalizationHealthState::kInitializing,
        false,
        false,
        "waiting_for_localization_inputs",
        std::move(missing_or_stale));
    }

    return make_result(
      LocalizationHealthState::kStale,
      false,
      false,
      "required_localization_input_unavailable",
      std::move(missing_or_stale));
  }

  if (!degraded.empty()) {
    return make_result(
      LocalizationHealthState::kDegraded,
      true,
      true,
      "localization_operational_degraded",
      std::move(degraded));
  }

  return make_result(
    LocalizationHealthState::kOk,
    true,
    false,
    "localization_operational",
    {});
}

std::string_view LocalizationHealthCore::ToString(
  const LocalizationHealthState state) noexcept
{
  switch (state) {
    case LocalizationHealthState::kUnknown:
      return "UNKNOWN";
    case LocalizationHealthState::kInitializing:
      return "INITIALIZING";
    case LocalizationHealthState::kOk:
      return "OK";
    case LocalizationHealthState::kDegraded:
      return "DEGRADED";
    case LocalizationHealthState::kStale:
      return "STALE";
    case LocalizationHealthState::kError:
      return "ERROR";
  }

  return "UNKNOWN";
}

void LocalizationHealthCore::AppendSourceErrors(
  const SourceHealthObservation & source,
  std::vector<std::string> & errors)
{
  if (!source.received) {
    return;
  }

  if (!source.data_valid) {
    errors.push_back(reason_with_detail(source.name, "invalid_data", source.detail));
  }
  if (!source.frame_valid) {
    errors.push_back(reason_with_detail(source.name, "invalid_frame", source.detail));
  }
  if (!source.timestamp_valid) {
    errors.push_back(reason_with_detail(source.name, "timestamp_regression", source.detail));
  }
  if (source.diagnostic_error) {
    errors.push_back(reason_with_detail(source.name, "diagnostic_error", source.detail));
  }
}

void LocalizationHealthCore::AppendSourceStaleReasons(
  const SourceHealthObservation & source,
  std::vector<std::string> & stale_reasons)
{
  if (!source.required) {
    return;
  }

  if (!source.received) {
    stale_reasons.push_back(reason_with_detail(source.name, "missing", source.detail));
    return;
  }

  if (!source.fresh) {
    stale_reasons.push_back(reason_with_detail(source.name, "stale", source.detail));
    return;
  }

  if (!source.rate_valid) {
    stale_reasons.push_back(
      reason_with_detail(source.name, "rate_below_minimum", source.detail));
  }
}

void LocalizationHealthCore::AppendSourceDegradedReasons(
  const SourceHealthObservation & source,
  std::vector<std::string> & degraded_reasons)
{
  if (!source.received) {
    if (!source.required) {
      degraded_reasons.push_back(reason_with_detail(source.name, "missing_optional", source.detail));
    }
    return;
  }

  if (!source.fresh && !source.required) {
    degraded_reasons.push_back(reason_with_detail(source.name, "stale_optional", source.detail));
  }
  if (!source.rate_valid && !source.required) {
    degraded_reasons.push_back(reason_with_detail(source.name, "rate_low", source.detail));
  }
  if (source.diagnostic_warning) {
    degraded_reasons.push_back(reason_with_detail(source.name, "diagnostic_warning", source.detail));
  }
}

}  // namespace savo_localization
