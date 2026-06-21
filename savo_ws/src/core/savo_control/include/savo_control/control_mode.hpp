#pragma once

#include <algorithm>
#include <cctype>
#include <ostream>
#include <string>

namespace savo_control
{

enum class ControlMode
{
  STOP,
  MANUAL,
  AUTO,
  NAV,
  RECOVERY
};

enum class ControlModeReason
{
  NONE,
  STARTUP,
  REQUESTED,
  MANUAL_OVERRIDE,
  RECOVERY_ACTIVE,
  RECOVERY_LATCHED,
  SAFETY_STOP_ACTIVE,
  EXTERNAL_STOP,
  INVALID_TIME,
  TIMEOUT,
  NO_SOURCE_ALLOWED,
  UNKNOWN
};

inline std::string normalize_mode_text(const std::string & value)
{
  std::string out;
  out.reserve(value.size());

  for (const char ch : value) {
    if (ch == '-' || ch == ' ') {
      out.push_back('_');
    } else {
      out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
    }
  }

  const auto first = out.find_first_not_of("\t\n\r ");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = out.find_last_not_of("\t\n\r ");
  return out.substr(first, last - first + 1);
}

inline std::string to_string(const ControlMode mode)
{
  switch (mode) {
    case ControlMode::STOP:
      return "STOP";
    case ControlMode::MANUAL:
      return "MANUAL";
    case ControlMode::AUTO:
      return "AUTO";
    case ControlMode::NAV:
      return "NAV";
    case ControlMode::RECOVERY:
      return "RECOVERY";
  }

  return "STOP";
}

inline std::string control_mode_to_string(const ControlMode mode)
{
  return to_string(mode);
}

inline std::string mode_to_string(const ControlMode mode)
{
  return to_string(mode);
}

inline bool try_parse_control_mode(const std::string & value, ControlMode & mode)
{
  const std::string key = normalize_mode_text(value);

  if (key == "STOP" || key == "IDLE" || key == "DISABLED") {
    mode = ControlMode::STOP;
    return true;
  }

  if (key == "MANUAL" || key == "TELEOP" || key == "MAN") {
    mode = ControlMode::MANUAL;
    return true;
  }

  if (key == "AUTO" || key == "AUTONOMOUS" || key == "AUTON") {
    mode = ControlMode::AUTO;
    return true;
  }

  if (key == "NAV" || key == "NAV2") {
    mode = ControlMode::NAV;
    return true;
  }

  if (key == "RECOVERY") {
    mode = ControlMode::RECOVERY;
    return true;
  }

  return false;
}

inline ControlMode parse_control_mode(
  const std::string & value,
  const ControlMode default_mode = ControlMode::STOP)
{
  ControlMode mode = default_mode;
  if (try_parse_control_mode(value, mode)) {
    return mode;
  }

  return default_mode;
}

inline ControlMode control_mode_from_string(
  const std::string & value,
  const ControlMode default_mode = ControlMode::STOP)
{
  return parse_control_mode(value, default_mode);
}

inline ControlMode mode_from_string(
  const std::string & value,
  const ControlMode default_mode = ControlMode::STOP)
{
  return parse_control_mode(value, default_mode);
}

inline bool is_stop_like(const ControlMode mode)
{
  return mode == ControlMode::STOP;
}

inline bool is_stop_like(const std::string & value)
{
  return parse_control_mode(value, ControlMode::STOP) == ControlMode::STOP;
}

inline bool is_motion_mode(const ControlMode mode)
{
  return mode == ControlMode::MANUAL ||
         mode == ControlMode::AUTO ||
         mode == ControlMode::NAV ||
         mode == ControlMode::RECOVERY;
}

inline bool is_motion_mode(const std::string & value)
{
  ControlMode mode = ControlMode::STOP;
  if (!try_parse_control_mode(value, mode)) {
    return false;
  }

  return is_motion_mode(mode);
}

inline ControlMode safe_mode_fallback()
{
  return ControlMode::STOP;
}

inline std::string plain_mux_mode_string(const ControlMode mode)
{
  if (mode == ControlMode::MANUAL) {
    return "MANUAL";
  }

  if (mode == ControlMode::AUTO) {
    return "AUTO";
  }

  if (mode == ControlMode::NAV) {
    return "NAV";
  }

  return "STOP";
}

inline std::string plain_mux_mode_string(const std::string & value)
{
  return plain_mux_mode_string(parse_control_mode(value, ControlMode::STOP));
}

inline std::string mode_display_label(const ControlMode mode)
{
  switch (mode) {
    case ControlMode::STOP:
      return "STOP (safe hold)";
    case ControlMode::MANUAL:
      return "MANUAL (teleop/app)";
    case ControlMode::AUTO:
      return "AUTO (local auto)";
    case ControlMode::NAV:
      return "NAV (Nav2 goal)";
    case ControlMode::RECOVERY:
      return "RECOVERY (override path)";
  }

  return "STOP (safe hold)";
}

inline std::string mode_display_label(const std::string & value)
{
  return mode_display_label(parse_control_mode(value, ControlMode::STOP));
}

inline std::string to_string(const ControlModeReason reason)
{
  switch (reason) {
    case ControlModeReason::NONE:
      return "NONE";
    case ControlModeReason::STARTUP:
      return "STARTUP";
    case ControlModeReason::REQUESTED:
      return "REQUESTED";
    case ControlModeReason::MANUAL_OVERRIDE:
      return "MANUAL_OVERRIDE";
    case ControlModeReason::RECOVERY_ACTIVE:
      return "RECOVERY_ACTIVE";
    case ControlModeReason::RECOVERY_LATCHED:
      return "RECOVERY_LATCHED";
    case ControlModeReason::SAFETY_STOP_ACTIVE:
      return "SAFETY_STOP_ACTIVE";
    case ControlModeReason::EXTERNAL_STOP:
      return "EXTERNAL_STOP";
    case ControlModeReason::INVALID_TIME:
      return "INVALID_TIME";
    case ControlModeReason::TIMEOUT:
      return "TIMEOUT";
    case ControlModeReason::NO_SOURCE_ALLOWED:
      return "NO_SOURCE_ALLOWED";
    case ControlModeReason::UNKNOWN:
      return "UNKNOWN";
  }

  return "UNKNOWN";
}

inline std::string control_mode_reason_to_string(const ControlModeReason reason)
{
  return to_string(reason);
}

inline bool try_parse_control_mode_reason(
  const std::string & value,
  ControlModeReason & reason)
{
  const std::string key = normalize_mode_text(value);

  if (key == "NONE") {
    reason = ControlModeReason::NONE;
    return true;
  }
  if (key == "STARTUP") {
    reason = ControlModeReason::STARTUP;
    return true;
  }
  if (key == "REQUESTED") {
    reason = ControlModeReason::REQUESTED;
    return true;
  }
  if (key == "MANUAL_OVERRIDE") {
    reason = ControlModeReason::MANUAL_OVERRIDE;
    return true;
  }
  if (key == "RECOVERY_ACTIVE") {
    reason = ControlModeReason::RECOVERY_ACTIVE;
    return true;
  }
  if (key == "RECOVERY_LATCHED") {
    reason = ControlModeReason::RECOVERY_LATCHED;
    return true;
  }
  if (key == "SAFETY_STOP_ACTIVE") {
    reason = ControlModeReason::SAFETY_STOP_ACTIVE;
    return true;
  }
  if (key == "EXTERNAL_STOP") {
    reason = ControlModeReason::EXTERNAL_STOP;
    return true;
  }
  if (key == "INVALID_TIME") {
    reason = ControlModeReason::INVALID_TIME;
    return true;
  }
  if (key == "TIMEOUT") {
    reason = ControlModeReason::TIMEOUT;
    return true;
  }
  if (key == "NO_SOURCE_ALLOWED") {
    reason = ControlModeReason::NO_SOURCE_ALLOWED;
    return true;
  }
  if (key == "UNKNOWN") {
    reason = ControlModeReason::UNKNOWN;
    return true;
  }

  return false;
}

inline ControlModeReason parse_control_mode_reason(
  const std::string & value,
  const ControlModeReason default_reason = ControlModeReason::UNKNOWN)
{
  ControlModeReason reason = default_reason;
  if (try_parse_control_mode_reason(value, reason)) {
    return reason;
  }

  return default_reason;
}

inline ControlModeReason control_mode_reason_from_string(
  const std::string & value,
  const ControlModeReason default_reason = ControlModeReason::UNKNOWN)
{
  return parse_control_mode_reason(value, default_reason);
}

inline std::ostream & operator<<(std::ostream & os, const ControlMode mode)
{
  os << to_string(mode);
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ControlModeReason reason)
{
  os << to_string(reason);
  return os;
}

}  // namespace savo_control
