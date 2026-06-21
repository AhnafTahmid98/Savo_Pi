#pragma once

#include <algorithm>
#include <cctype>
#include <ostream>
#include <string>

namespace savo_control
{

enum class RecoveryState
{
  IDLE,
  REQUESTED,
  ACTIVE,
  BACKING_UP,
  ROTATING,
  ESCAPING,
  COMPLETED,
  FAILED,
  BLOCKED,
  TIMEOUT,
  CANCELLED
};

enum class RecoveryTrigger
{
  NONE,
  STUCK_DETECTED,
  SAFETY_STOP,
  MANUAL_REQUEST,
  NAV_FAILURE,
  WATCHDOG,
  TEST_REQUEST,
  UNKNOWN
};

enum class RecoveryAction
{
  NONE,
  STOP,
  BACKUP,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  BACKUP_THEN_LEFT,
  BACKUP_THEN_RIGHT,
  ESCAPE_SEQUENCE
};

inline std::string normalize_recovery_text(const std::string & value)
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

inline std::string to_string(const RecoveryState state)
{
  switch (state) {
    case RecoveryState::IDLE:
      return "IDLE";
    case RecoveryState::REQUESTED:
      return "REQUESTED";
    case RecoveryState::ACTIVE:
      return "ACTIVE";
    case RecoveryState::BACKING_UP:
      return "BACKING_UP";
    case RecoveryState::ROTATING:
      return "ROTATING";
    case RecoveryState::ESCAPING:
      return "ESCAPING";
    case RecoveryState::COMPLETED:
      return "COMPLETED";
    case RecoveryState::FAILED:
      return "FAILED";
    case RecoveryState::BLOCKED:
      return "BLOCKED";
    case RecoveryState::TIMEOUT:
      return "TIMEOUT";
    case RecoveryState::CANCELLED:
      return "CANCELLED";
  }

  return "IDLE";
}

inline std::string recovery_state_to_string(const RecoveryState state)
{
  return to_string(state);
}

inline bool try_parse_recovery_state(const std::string & value, RecoveryState & state)
{
  const std::string key = normalize_recovery_text(value);

  if (key == "IDLE") {
    state = RecoveryState::IDLE;
    return true;
  }
  if (key == "REQUESTED" || key == "REQUEST") {
    state = RecoveryState::REQUESTED;
    return true;
  }
  if (key == "ACTIVE" || key == "RUNNING") {
    state = RecoveryState::ACTIVE;
    return true;
  }
  if (key == "BACKING_UP" || key == "BACKUP" || key == "BACKING") {
    state = RecoveryState::BACKING_UP;
    return true;
  }
  if (key == "ROTATING" || key == "ROTATE") {
    state = RecoveryState::ROTATING;
    return true;
  }
  if (key == "ESCAPING" || key == "ESCAPE") {
    state = RecoveryState::ESCAPING;
    return true;
  }
  if (key == "COMPLETED" || key == "DONE" || key == "SUCCESS") {
    state = RecoveryState::COMPLETED;
    return true;
  }
  if (key == "FAILED" || key == "FAIL") {
    state = RecoveryState::FAILED;
    return true;
  }
  if (key == "BLOCKED" || key == "SAFETY_STOP") {
    state = RecoveryState::BLOCKED;
    return true;
  }
  if (key == "TIMEOUT" || key == "TIMED_OUT") {
    state = RecoveryState::TIMEOUT;
    return true;
  }
  if (key == "CANCELLED" || key == "CANCELED" || key == "CANCEL") {
    state = RecoveryState::CANCELLED;
    return true;
  }

  return false;
}

inline RecoveryState parse_recovery_state(
  const std::string & value,
  const RecoveryState default_state = RecoveryState::IDLE)
{
  RecoveryState state = default_state;
  if (try_parse_recovery_state(value, state)) {
    return state;
  }

  return default_state;
}

inline RecoveryState recovery_state_from_string(
  const std::string & value,
  const RecoveryState default_state = RecoveryState::IDLE)
{
  return parse_recovery_state(value, default_state);
}

inline bool is_recovery_active(const RecoveryState state)
{
  return state == RecoveryState::REQUESTED ||
         state == RecoveryState::ACTIVE ||
         state == RecoveryState::BACKING_UP ||
         state == RecoveryState::ROTATING ||
         state == RecoveryState::ESCAPING;
}

inline bool is_recovery_terminal(const RecoveryState state)
{
  return state == RecoveryState::COMPLETED ||
         state == RecoveryState::FAILED ||
         state == RecoveryState::BLOCKED ||
         state == RecoveryState::TIMEOUT ||
         state == RecoveryState::CANCELLED;
}

inline bool recovery_allows_motion(const RecoveryState state)
{
  return state == RecoveryState::ACTIVE ||
         state == RecoveryState::BACKING_UP ||
         state == RecoveryState::ROTATING ||
         state == RecoveryState::ESCAPING;
}

inline std::string to_string(const RecoveryTrigger trigger)
{
  switch (trigger) {
    case RecoveryTrigger::NONE:
      return "NONE";
    case RecoveryTrigger::STUCK_DETECTED:
      return "STUCK_DETECTED";
    case RecoveryTrigger::SAFETY_STOP:
      return "SAFETY_STOP";
    case RecoveryTrigger::MANUAL_REQUEST:
      return "MANUAL_REQUEST";
    case RecoveryTrigger::NAV_FAILURE:
      return "NAV_FAILURE";
    case RecoveryTrigger::WATCHDOG:
      return "WATCHDOG";
    case RecoveryTrigger::TEST_REQUEST:
      return "TEST_REQUEST";
    case RecoveryTrigger::UNKNOWN:
      return "UNKNOWN";
  }

  return "UNKNOWN";
}

inline std::string recovery_trigger_to_string(const RecoveryTrigger trigger)
{
  return to_string(trigger);
}

inline bool try_parse_recovery_trigger(const std::string & value, RecoveryTrigger & trigger)
{
  const std::string key = normalize_recovery_text(value);

  if (key == "NONE") {
    trigger = RecoveryTrigger::NONE;
    return true;
  }
  if (key == "STUCK_DETECTED" || key == "STUCK") {
    trigger = RecoveryTrigger::STUCK_DETECTED;
    return true;
  }
  if (key == "SAFETY_STOP" || key == "SAFETY") {
    trigger = RecoveryTrigger::SAFETY_STOP;
    return true;
  }
  if (key == "MANUAL_REQUEST" || key == "MANUAL") {
    trigger = RecoveryTrigger::MANUAL_REQUEST;
    return true;
  }
  if (key == "NAV_FAILURE" || key == "NAV") {
    trigger = RecoveryTrigger::NAV_FAILURE;
    return true;
  }
  if (key == "WATCHDOG") {
    trigger = RecoveryTrigger::WATCHDOG;
    return true;
  }
  if (key == "TEST_REQUEST" || key == "TEST") {
    trigger = RecoveryTrigger::TEST_REQUEST;
    return true;
  }
  if (key == "UNKNOWN") {
    trigger = RecoveryTrigger::UNKNOWN;
    return true;
  }

  return false;
}

inline RecoveryTrigger parse_recovery_trigger(
  const std::string & value,
  const RecoveryTrigger default_trigger = RecoveryTrigger::UNKNOWN)
{
  RecoveryTrigger trigger = default_trigger;
  if (try_parse_recovery_trigger(value, trigger)) {
    return trigger;
  }

  return default_trigger;
}

inline RecoveryTrigger recovery_trigger_from_string(
  const std::string & value,
  const RecoveryTrigger default_trigger = RecoveryTrigger::UNKNOWN)
{
  return parse_recovery_trigger(value, default_trigger);
}

inline std::string to_string(const RecoveryAction action)
{
  switch (action) {
    case RecoveryAction::NONE:
      return "NONE";
    case RecoveryAction::STOP:
      return "STOP";
    case RecoveryAction::BACKUP:
      return "BACKUP";
    case RecoveryAction::ROTATE_LEFT:
      return "ROTATE_LEFT";
    case RecoveryAction::ROTATE_RIGHT:
      return "ROTATE_RIGHT";
    case RecoveryAction::BACKUP_THEN_LEFT:
      return "BACKUP_THEN_LEFT";
    case RecoveryAction::BACKUP_THEN_RIGHT:
      return "BACKUP_THEN_RIGHT";
    case RecoveryAction::ESCAPE_SEQUENCE:
      return "ESCAPE_SEQUENCE";
  }

  return "NONE";
}

inline std::string recovery_action_to_string(const RecoveryAction action)
{
  return to_string(action);
}

inline bool try_parse_recovery_action(const std::string & value, RecoveryAction & action)
{
  const std::string key = normalize_recovery_text(value);

  if (key == "NONE") {
    action = RecoveryAction::NONE;
    return true;
  }
  if (key == "STOP") {
    action = RecoveryAction::STOP;
    return true;
  }
  if (key == "BACKUP" || key == "BACK_UP" || key == "BACKING_UP") {
    action = RecoveryAction::BACKUP;
    return true;
  }
  if (key == "ROTATE_LEFT" || key == "LEFT") {
    action = RecoveryAction::ROTATE_LEFT;
    return true;
  }
  if (key == "ROTATE_RIGHT" || key == "RIGHT") {
    action = RecoveryAction::ROTATE_RIGHT;
    return true;
  }
  if (key == "BACKUP_THEN_LEFT" || key == "BACKUP_LEFT") {
    action = RecoveryAction::BACKUP_THEN_LEFT;
    return true;
  }
  if (key == "BACKUP_THEN_RIGHT" || key == "BACKUP_RIGHT") {
    action = RecoveryAction::BACKUP_THEN_RIGHT;
    return true;
  }
  if (key == "ESCAPE_SEQUENCE" || key == "ESCAPE") {
    action = RecoveryAction::ESCAPE_SEQUENCE;
    return true;
  }

  return false;
}

inline RecoveryAction parse_recovery_action(
  const std::string & value,
  const RecoveryAction default_action = RecoveryAction::NONE)
{
  RecoveryAction action = default_action;
  if (try_parse_recovery_action(value, action)) {
    return action;
  }

  return default_action;
}

inline RecoveryAction recovery_action_from_string(
  const std::string & value,
  const RecoveryAction default_action = RecoveryAction::NONE)
{
  return parse_recovery_action(value, default_action);
}

inline bool recovery_action_outputs_motion(const RecoveryAction action)
{
  return action == RecoveryAction::BACKUP ||
         action == RecoveryAction::ROTATE_LEFT ||
         action == RecoveryAction::ROTATE_RIGHT ||
         action == RecoveryAction::BACKUP_THEN_LEFT ||
         action == RecoveryAction::BACKUP_THEN_RIGHT ||
         action == RecoveryAction::ESCAPE_SEQUENCE;
}

inline std::ostream & operator<<(std::ostream & os, const RecoveryState state)
{
  os << to_string(state);
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const RecoveryTrigger trigger)
{
  os << to_string(trigger);
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const RecoveryAction action)
{
  os << to_string(action);
  return os;
}

}  // namespace savo_control
