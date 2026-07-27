// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/control_mode_client.hpp"

#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace
{

std::string Normalize(const std::string_view input)
{
  std::string normalized;
  normalized.reserve(input.size());

  for (const char character : input) {
    const auto value =
      static_cast<unsigned char>(character);

    if (std::isspace(value) != 0) {
      continue;
    }

    if (character == '-') {
      normalized.push_back('_');
      continue;
    }

    normalized.push_back(
      static_cast<char>(std::toupper(value)));
  }

  return normalized;
}

}  // namespace

namespace savo_nav
{

bool ControlModeClient::UpdateMode(
  std::string mode_text,
  const double now_seconds)
{
  if (!std::isfinite(now_seconds)) {
    return false;
  }

  mode_ = Parse(mode_text);

  mode_text_ =
    std::string(ToString(mode_));

  observed_ = true;
  last_mode_seconds_ = now_seconds;

  return true;
}

void ControlModeClient::UpdateReason(
  std::string reason_text)
{
  reason_text_ = std::move(reason_text);
}

void ControlModeClient::UpdateStatus(
  std::string status_text)
{
  status_text_ = std::move(status_text);
}

void ControlModeClient::Reset() noexcept
{
  observed_ = false;
  last_mode_seconds_ = 0.0;

  mode_ = ObservedControlMode::kUnknown;

  mode_text_ = "UNKNOWN";
  reason_text_.clear();
  status_text_.clear();
}

ControlModeObservation ControlModeClient::Evaluate(
  const double now_seconds,
  const double timeout_seconds) const
{
  ControlModeObservation observation;

  observation.observed = observed_;
  observation.mode = mode_;
  observation.mode_text = mode_text_;
  observation.reason_text = reason_text_;
  observation.status_text = status_text_;

  if (!observed_) {
    observation.age_seconds =
      std::numeric_limits<double>::infinity();

    observation.reason_code =
      "control_mode_unobserved";

    return observation;
  }

  if (
    !std::isfinite(now_seconds) ||
    !std::isfinite(timeout_seconds) ||
    timeout_seconds <= 0.0 ||
    now_seconds < last_mode_seconds_)
  {
    observation.age_seconds =
      std::numeric_limits<double>::infinity();

    observation.reason_code =
      "control_mode_invalid_time";

    return observation;
  }

  observation.age_seconds =
    now_seconds - last_mode_seconds_;

  observation.fresh =
    observation.age_seconds <= timeout_seconds;

  if (!observation.fresh) {
    observation.reason_code =
      "control_mode_stale";

    return observation;
  }

  if (
    observation.mode ==
    ObservedControlMode::kUnknown)
  {
    observation.reason_code =
      "control_mode_unknown";

    return observation;
  }

  if (
    observation.mode !=
    ObservedControlMode::kNav)
  {
    observation.reason_code =
      "control_mode_not_nav";

    return observation;
  }

  observation.navigation_allowed = true;
  observation.cancel_active_goal = false;

  observation.reason_code =
    "control_mode_nav";

  return observation;
}

ObservedControlMode ControlModeClient::Parse(
  const std::string_view mode_text)
{
  const std::string normalized =
    Normalize(mode_text);

  if (
    normalized == "STOP" ||
    normalized == "IDLE" ||
    normalized == "DISABLED")
  {
    return ObservedControlMode::kStop;
  }

  if (
    normalized == "MANUAL" ||
    normalized == "TELEOP" ||
    normalized == "MAN")
  {
    return ObservedControlMode::kManual;
  }

  if (
    normalized == "AUTO" ||
    normalized == "AUTONOMOUS")
  {
    return ObservedControlMode::kAuto;
  }

  if (
    normalized == "NAV" ||
    normalized == "NAVIGATION")
  {
    return ObservedControlMode::kNav;
  }

  if (
    normalized == "RECOVERY" ||
    normalized == "RECOVER")
  {
    return ObservedControlMode::kRecovery;
  }

  return ObservedControlMode::kUnknown;
}

std::string_view ControlModeClient::ToString(
  const ObservedControlMode mode) noexcept
{
  switch (mode) {
    case ObservedControlMode::kUnknown:
      return "UNKNOWN";

    case ObservedControlMode::kStop:
      return "STOP";

    case ObservedControlMode::kManual:
      return "MANUAL";

    case ObservedControlMode::kAuto:
      return "AUTO";

    case ObservedControlMode::kNav:
      return "NAV";

    case ObservedControlMode::kRecovery:
      return "RECOVERY";
  }

  return "UNKNOWN";
}

}  // namespace savo_nav
