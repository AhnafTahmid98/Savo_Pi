// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/command_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace savo_bridge
{
namespace
{

using Json = nlohmann::json;
using ErrorCode = CommandProtocolErrorCode;

struct ValidationFailure
{
  ErrorCode code;
  std::string reason;
};

using ValidationResult = std::optional<ValidationFailure>;

[[nodiscard]] CommandParseResult failure(
  const ErrorCode code,
  std::string reason)
{
  CommandParseResult result;
  result.error = CommandProtocolError{
    code,
    std::move(reason),
  };
  return result;
}

[[nodiscard]] bool is_ascii_alphanumeric(
  const char character) noexcept
{
  return
    (character >= 'A' && character <= 'Z') ||
    (character >= 'a' && character <= 'z') ||
    (character >= '0' && character <= '9');
}

[[nodiscard]] bool is_safe_identifier(
  const std::string_view value) noexcept
{
  if (value.empty() || value.size() > 128U) {
    return false;
  }

  if (!is_ascii_alphanumeric(value.front())) {
    return false;
  }

  return std::all_of(
    value.begin() + 1,
    value.end(),
    [](const char character) {
      return
        is_ascii_alphanumeric(character) ||
        character == '.' ||
        character == '_' ||
        character == ':' ||
        character == '-';
    });
}

[[nodiscard]] bool is_valid_source(
  const std::string_view value) noexcept
{
  if (value.empty() || value.size() > 64U) {
    return false;
  }

  if (value.front() < 'a' || value.front() > 'z') {
    return false;
  }

  return std::all_of(
    value.begin() + 1,
    value.end(),
    [](const char character) {
      return
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9') ||
        character == '.' ||
        character == '_' ||
        character == ':' ||
        character == '-';
    });
}

[[nodiscard]] bool is_valid_origin_agent(
  const std::string_view value) noexcept
{
  if (value.empty() || value.size() > 64U) {
    return false;
  }

  if (value.front() < 'a' || value.front() > 'z') {
    return false;
  }

  return std::all_of(
    value.begin() + 1,
    value.end(),
    [](const char character) {
      return
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9') ||
        character == '_';
    });
}

[[nodiscard]] std::string trim_copy(
  const std::string_view value)
{
  const auto not_whitespace = [](const char character) {
      return
        character != ' ' &&
        character != '\t' &&
        character != '\n' &&
        character != '\r' &&
        character != '\f' &&
        character != '\v';
    };

  const auto begin = std::find_if(
    value.begin(),
    value.end(),
    not_whitespace);

  const auto end = std::find_if(
    value.rbegin(),
    value.rend(),
    not_whitespace).base();

  if (begin >= end) {
    return {};
  }

  return std::string(begin, end);
}

[[nodiscard]] ValidationResult validate_reason(
  const std::string & reason)
{
  const auto trimmed = trim_copy(reason);

  if (trimmed.empty() || trimmed.size() > 128U) {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "reason must contain 1 through 128 characters after trimming",
    };
  }

  return std::nullopt;
}

[[nodiscard]] ValidationResult reject_unknown_fields(
  const Json & object,
  const std::set<std::string> & allowed,
  const ErrorCode code,
  const std::string & description)
{
  for (auto iterator = object.begin();
    iterator != object.end();
    ++iterator)
  {
    if (allowed.count(iterator.key()) == 0U) {
      return ValidationFailure{
        code,
        description + ": " + iterator.key(),
      };
    }
  }

  return std::nullopt;
}

[[nodiscard]] ValidationResult require_fields(
  const Json & object,
  const std::set<std::string> & required)
{
  for (const auto & field : required) {
    if (!object.contains(field)) {
      return ValidationFailure{
        ErrorCode::MissingRequiredField,
        "missing required field: " + field,
      };
    }
  }

  return std::nullopt;
}

[[nodiscard]] bool read_integer(
  const Json & value,
  std::int64_t & output) noexcept
{
  try {
    if (value.is_number_unsigned()) {
      const auto unsigned_value =
        value.get<std::uint64_t>();

      if (unsigned_value >
        static_cast<std::uint64_t>(
          std::numeric_limits<std::int64_t>::max()))
      {
        return false;
      }

      output = static_cast<std::int64_t>(
        unsigned_value);

      return true;
    }

    if (value.is_number_integer()) {
      output = value.get<std::int64_t>();
      return true;
    }
  } catch (...) {
    return false;
  }

  return false;
}

[[nodiscard]] bool read_number(
  const Json & value,
  double & output) noexcept
{
  if (!value.is_number()) {
    return false;
  }

  try {
    output = value.get<double>();
  } catch (...) {
    return false;
  }

  return std::isfinite(output);
}

[[nodiscard]] ValidationResult read_required_string(
  const Json & object,
  const std::string & field,
  std::string & output)
{
  if (!object.at(field).is_string()) {
    return ValidationFailure{
      ErrorCode::InvalidFieldType,
      field + " must be a string",
    };
  }

  output = object.at(field).get<std::string>();
  return std::nullopt;
}

[[nodiscard]] ValidationResult read_optional_string(
  const Json & object,
  const std::string & field,
  std::optional<std::string> & output)
{
  if (!object.contains(field)) {
    return std::nullopt;
  }

  if (!object.at(field).is_string()) {
    return ValidationFailure{
      ErrorCode::InvalidFieldType,
      field + " must be a string",
    };
  }

  output = object.at(field).get<std::string>();
  return std::nullopt;
}

[[nodiscard]] ValidationResult read_nullable_string(
  const Json & object,
  const std::string & field,
  std::optional<std::string> & output)
{
  if (!object.contains(field) || object.at(field).is_null()) {
    return std::nullopt;
  }

  return read_optional_string(object, field, output);
}

[[nodiscard]] std::optional<CommandType> parse_command_type(
  const std::string_view value) noexcept
{
  if (value == "stop") {
    return CommandType::Stop;
  }

  if (value == "cancel_action") {
    return CommandType::CancelAction;
  }

  if (value == "teleop_nudge") {
    return CommandType::TeleopNudge;
  }

  if (value == "navigate_to_location") {
    return CommandType::NavigateToLocation;
  }

  if (value == "cancel_navigation") {
    return CommandType::CancelNavigation;
  }

  if (value == "query_navigation_state") {
    return CommandType::QueryNavigationState;
  }

  if (value == "start_autonomous_mapping") {
    return CommandType::StartAutonomousMapping;
  }

  if (value == "control_mapping") {
    return CommandType::ControlMapping;
  }

  if (value == "query_mapping_state") {
    return CommandType::QueryMappingState;
  }

  if (value == "query_supervisor_state") {
    return CommandType::QuerySupervisorState;
  }

  return std::nullopt;
}

[[nodiscard]] std::optional<CommandPriority> parse_priority(
  const std::string_view value) noexcept
{
  if (value == "normal") {
    return CommandPriority::Normal;
  }

  if (value == "high") {
    return CommandPriority::High;
  }

  if (value == "emergency") {
    return CommandPriority::Emergency;
  }

  return std::nullopt;
}

[[nodiscard]] ValidationResult validate_identifier(
  const std::string & value,
  const std::string & field)
{
  if (!is_safe_identifier(value)) {
    return ValidationFailure{
      ErrorCode::InvalidIdentifier,
      field + " has invalid identifier syntax",
    };
  }

  return std::nullopt;
}

[[nodiscard]] ValidationResult parse_stop_payload(
  const Json & payload,
  CommandPayload & output)
{
  const std::set<std::string> allowed{
    "reason",
    "scope",
  };

  if (const auto error = reject_unknown_fields(
      payload,
      allowed,
      ErrorCode::UnknownPayloadField,
      "unknown stop payload field"))
  {
    return error;
  }

  StopCommandPayload result;
  result.reason = "user_requested_stop";
  result.scope = "all_movement";

  if (payload.contains("reason")) {
    if (!payload.at("reason").is_string()) {
      return ValidationFailure{
        ErrorCode::InvalidFieldType,
        "reason must be a string",
      };
    }

    result.reason =
      payload.at("reason").get<std::string>();
  }

  if (payload.contains("scope")) {
    if (!payload.at("scope").is_string()) {
      return ValidationFailure{
        ErrorCode::InvalidFieldType,
        "scope must be a string",
      };
    }

    result.scope =
      payload.at("scope").get<std::string>();
  }

  if (result.scope != "all_movement") {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "stop scope must equal all_movement",
    };
  }

  if (const auto error = validate_reason(result.reason)) {
    return error;
  }

  output = std::move(result);
  return std::nullopt;
}

[[nodiscard]] ValidationResult parse_cancel_action_payload(
  const Json & payload,
  CommandPayload & output)
{
  const std::set<std::string> allowed{
    "reason",
    "target_command_id",
  };

  if (const auto error = reject_unknown_fields(
      payload,
      allowed,
      ErrorCode::UnknownPayloadField,
      "unknown cancel_action payload field"))
  {
    return error;
  }

  if (const auto error = require_fields(
      payload,
      {"target_command_id"}))
  {
    return error;
  }

  CancelActionCommandPayload result;
  result.reason = "user_requested_cancel";

  if (const auto error = read_required_string(
      payload,
      "target_command_id",
      result.target_command_id))
  {
    return error;
  }

  if (const auto error = validate_identifier(
      result.target_command_id,
      "target_command_id"))
  {
    return error;
  }

  if (payload.contains("reason")) {
    if (const auto error = read_required_string(
        payload,
        "reason",
        result.reason))
    {
      return error;
    }
  }

  if (const auto error = validate_reason(result.reason)) {
    return error;
  }

  output = std::move(result);
  return std::nullopt;
}

[[nodiscard]] ValidationResult parse_teleop_payload(
  const Json & payload,
  CommandPayload & output)
{
  const std::set<std::string> allowed{
    "angular_z_radps",
    "direction",
    "duration_ms",
    "linear_x_mps",
    "linear_y_mps",
    "speed_label",
  };

  if (const auto error = reject_unknown_fields(
      payload,
      allowed,
      ErrorCode::UnknownPayloadField,
      "unknown teleop_nudge payload field"))
  {
    return error;
  }

  if (const auto error = require_fields(
      payload,
      {"direction", "duration_ms"}))
  {
    return error;
  }

  TeleopNudgeCommandPayload result;

  if (const auto error = read_required_string(
      payload,
      "direction",
      result.direction))
  {
    return error;
  }

  if (!read_integer(
      payload.at("duration_ms"),
      result.duration_ms))
  {
    return ValidationFailure{
      ErrorCode::InvalidFieldType,
      "duration_ms must be an integer",
    };
  }

  const auto read_optional_number =
    [&payload](
    const std::string & field,
    double & value) -> ValidationResult
    {
      if (!payload.contains(field)) {
        return std::nullopt;
      }

      if (!read_number(payload.at(field), value)) {
        return ValidationFailure{
        ErrorCode::InvalidFieldType,
        field + " must be a finite number",
        };
      }

      return std::nullopt;
    };

  if (const auto error = read_optional_number(
      "linear_x_mps",
      result.linear_x_mps))
  {
    return error;
  }

  if (const auto error = read_optional_number(
      "linear_y_mps",
      result.linear_y_mps))
  {
    return error;
  }

  if (const auto error = read_optional_number(
      "angular_z_radps",
      result.angular_z_radps))
  {
    return error;
  }

  if (const auto error = read_nullable_string(
      payload,
      "speed_label",
      result.speed_label))
  {
    return error;
  }

  if (result.speed_label.has_value() &&
    result.speed_label->size() > 32U)
  {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "speed_label must not exceed 32 characters",
    };
  }

  if (std::abs(result.linear_x_mps) >
    MAX_TELEOP_LINEAR_SPEED_MPS)
  {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "linear_x_mps exceeds the safe limit",
    };
  }

  if (std::abs(result.linear_y_mps) >
    MAX_TELEOP_LINEAR_SPEED_MPS)
  {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "linear_y_mps exceeds the safe limit",
    };
  }

  if (std::abs(result.angular_z_radps) >
    MAX_TELEOP_ANGULAR_SPEED_RADPS)
  {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "angular_z_radps exceeds the safe limit",
    };
  }

  if (result.duration_ms < 1 ||
    result.duration_ms > MAX_TELEOP_DURATION_MS)
  {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "duration_ms must be from 1 through 1000",
    };
  }

  const int nonzero_axes =
    (result.linear_x_mps != 0.0 ? 1 : 0) +
    (result.linear_y_mps != 0.0 ? 1 : 0) +
    (result.angular_z_radps != 0.0 ? 1 : 0);

  if (nonzero_axes != 1) {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "exactly one teleop motion axis must be non-zero",
    };
  }

  const bool direction_matches =
    (result.direction == "forward" &&
    result.linear_x_mps > 0.0) ||
    (result.direction == "backward" &&
    result.linear_x_mps < 0.0) ||
    (result.direction == "strafe_left" &&
    result.linear_y_mps > 0.0) ||
    (result.direction == "strafe_right" &&
    result.linear_y_mps < 0.0) ||
    (result.direction == "turn_left" &&
    result.angular_z_radps > 0.0) ||
    (result.direction == "turn_right" &&
    result.angular_z_radps < 0.0);

  if (!direction_matches) {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "teleop direction does not match the active axis and sign",
    };
  }

  output = std::move(result);
  return std::nullopt;
}

[[nodiscard]] ValidationResult parse_navigation_payload(
  const Json & payload,
  CommandPayload & output)
{
  const std::set<std::string> allowed{
    "location_id",
    "map_id",
    "requested_label",
  };

  if (const auto error = reject_unknown_fields(
      payload,
      allowed,
      ErrorCode::UnknownPayloadField,
      "unknown navigate_to_location payload field"))
  {
    return error;
  }

  if (const auto error = require_fields(
      payload,
      {"location_id"}))
  {
    return error;
  }

  NavigateToLocationCommandPayload result;

  if (const auto error = read_required_string(
      payload,
      "location_id",
      result.location_id))
  {
    return error;
  }

  if (const auto error = validate_identifier(
      result.location_id,
      "location_id"))
  {
    return error;
  }

  if (const auto error = read_nullable_string(
      payload,
      "requested_label",
      result.requested_label))
  {
    return error;
  }

  if (result.requested_label.has_value() &&
    result.requested_label->size() > 128U)
  {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "requested_label must not exceed 128 characters",
    };
  }

  if (const auto error = read_nullable_string(
      payload,
      "map_id",
      result.map_id))
  {
    return error;
  }

  if (result.map_id.has_value()) {
    if (const auto error = validate_identifier(
        result.map_id.value(),
        "map_id"))
    {
      return error;
    }
  }

  output = std::move(result);
  return std::nullopt;
}

[[nodiscard]] ValidationResult parse_cancel_navigation_payload(
  const Json & payload,
  CommandPayload & output)
{
  const std::set<std::string> allowed{
    "reason",
    "target_command_id",
  };

  if (const auto error = reject_unknown_fields(
      payload,
      allowed,
      ErrorCode::UnknownPayloadField,
      "unknown cancel_navigation payload field"))
  {
    return error;
  }

  CancelNavigationCommandPayload result;
  result.reason = "user_requested_cancel_navigation";

  if (const auto error = read_nullable_string(
      payload,
      "target_command_id",
      result.target_command_id))
  {
    return error;
  }

  if (result.target_command_id.has_value()) {
    if (const auto error = validate_identifier(
        result.target_command_id.value(),
        "target_command_id"))
    {
      return error;
    }
  }

  if (payload.contains("reason")) {
    if (const auto error = read_required_string(
        payload,
        "reason",
        result.reason))
    {
      return error;
    }
  }

  if (const auto error = validate_reason(result.reason)) {
    return error;
  }

  output = std::move(result);
  return std::nullopt;
}

[[nodiscard]] ValidationResult parse_query_payload(
  const Json & payload,
  const std::string & expected_scope,
  CommandPayload & output)
{
  if (const auto error = reject_unknown_fields(
      payload, {"scope"}, ErrorCode::UnknownPayloadField,
      "unknown query payload field"))
  {
    return error;
  }
  QueryStateCommandPayload result{expected_scope};
  if (payload.contains("scope")) {
    if (const auto error = read_required_string(payload, "scope", result.scope)) {
      return error;
    }
  }
  if (result.scope != expected_scope) {
    return ValidationFailure{ErrorCode::InvalidPayload, "query scope does not match command type"};
  }
  output = std::move(result);
  return std::nullopt;
}

[[nodiscard]] ValidationResult parse_start_mapping_payload(
  const Json & payload,
  CommandPayload & output)
{
  const std::set<std::string> allowed{
    "auto_save", "map_id", "map_revision", "mission_id",
    "mission_timeout_ms", "require_quality_approval", "require_semantic"};
  if (const auto error = reject_unknown_fields(
      payload, allowed, ErrorCode::UnknownPayloadField,
      "unknown start_autonomous_mapping payload field"))
  {
    return error;
  }
  if (const auto error = require_fields(
      payload, {"map_id", "map_revision", "mission_id"}))
  {
    return error;
  }
  StartAutonomousMappingCommandPayload result;
  if (const auto error = read_required_string(payload, "mission_id", result.mission_id)) {
    return error;
  }
  if (const auto error = read_required_string(payload, "map_id", result.map_id)) {
    return error;
  }
  if (const auto error = validate_identifier(result.mission_id, "mission_id")) {
    return error;
  }
  if (const auto error = validate_identifier(result.map_id, "map_id")) {
    return error;
  }
  std::int64_t revision = 0;
  if (!read_integer(payload.at("map_revision"), revision) || revision <= 0 ||
    static_cast<std::uint64_t>(revision) > std::numeric_limits<std::uint32_t>::max())
  {
    return ValidationFailure{ErrorCode::InvalidPayload, "map_revision must be a positive uint32"};
  }
  result.map_revision = static_cast<std::uint32_t>(revision);
  if (payload.contains("mission_timeout_ms")) {
    if (!read_integer(payload.at("mission_timeout_ms"), result.mission_timeout_ms) ||
      result.mission_timeout_ms < 0 || result.mission_timeout_ms > 86400000)
    {
      return ValidationFailure{ErrorCode::InvalidPayload,
        "mission_timeout_ms must be 0 through 86400000"};
    }
  }
  for (const std::string field : {
      "auto_save", "require_quality_approval", "require_semantic"})
  {
    if (payload.contains(field) && !payload.at(field).is_boolean()) {
      return ValidationFailure{ErrorCode::InvalidFieldType, field + " must be boolean"};
    }
  }
  if (payload.contains("auto_save")) {
    result.auto_save = payload.at("auto_save").get<bool>();
  }
  if (payload.contains("require_quality_approval")) {
    result.require_quality_approval = payload.at("require_quality_approval").get<bool>();
  }
  if (payload.contains("require_semantic")) {
    result.require_semantic = payload.at("require_semantic").get<bool>();
  }
  if (!result.auto_save || !result.require_quality_approval) {
    return ValidationFailure{
      ErrorCode::InvalidPayload,
      "autonomous mapping requires auto_save and operator quality approval"};
  }
  output = std::move(result);
  return std::nullopt;
}

[[nodiscard]] ValidationResult parse_control_mapping_payload(
  const Json & payload,
  CommandPayload & output)
{
  if (const auto error = reject_unknown_fields(
      payload, {"mission_id", "operation", "reason"},
      ErrorCode::UnknownPayloadField, "unknown control_mapping payload field"))
  {
    return error;
  }
  if (const auto error = require_fields(payload, {"mission_id", "operation"})) {
    return error;
  }
  ControlMappingCommandPayload result;
  result.reason = "savomind_mapping_control";
  if (const auto error = read_required_string(payload, "mission_id", result.mission_id)) {
    return error;
  }
  if (const auto error = read_required_string(payload, "operation", result.operation)) {
    return error;
  }
  if (payload.contains("reason")) {
    if (const auto error = read_required_string(payload, "reason", result.reason)) {
      return error;
    }
  }
  if (const auto error = validate_identifier(result.mission_id, "mission_id")) {
    return error;
  }
  if (result.operation != "pause" && result.operation != "resume" &&
    result.operation != "cancel" && result.operation != "request_scan360")
  {
    return ValidationFailure{ErrorCode::InvalidPayload,
      "mapping operation must be pause, resume, cancel, or request_scan360"};
  }
  if (const auto error = validate_reason(result.reason)) {
    return error;
  }
  output = std::move(result);
  return std::nullopt;
}

[[nodiscard]] ValidationResult parse_payload(
  const CommandType type,
  const Json & payload,
  CommandPayload & output)
{
  if (!payload.is_object()) {
    return ValidationFailure{
      ErrorCode::PayloadNotObject,
      "payload must be a JSON object",
    };
  }

  switch (type) {
    case CommandType::Stop:
      return parse_stop_payload(payload, output);
    case CommandType::CancelAction:
      return parse_cancel_action_payload(payload, output);
    case CommandType::TeleopNudge:
      return parse_teleop_payload(payload, output);
    case CommandType::NavigateToLocation:
      return parse_navigation_payload(payload, output);
    case CommandType::CancelNavigation:
      return parse_cancel_navigation_payload(payload, output);
    case CommandType::QueryNavigationState:
      return parse_query_payload(payload, "navigation", output);
    case CommandType::StartAutonomousMapping:
      return parse_start_mapping_payload(payload, output);
    case CommandType::ControlMapping:
      return parse_control_mapping_payload(payload, output);
    case CommandType::QueryMappingState:
      return parse_query_payload(payload, "mapping", output);
    case CommandType::QuerySupervisorState:
      return parse_query_payload(payload, "supervisor", output);
  }

  return ValidationFailure{
    ErrorCode::InvalidCommandType,
    "unsupported command type",
  };
}

[[nodiscard]] CommandParseResult validate_document(
  const Json & document,
  const std::int64_t now_unix_ms)
{
  const std::set<std::string> allowed{
    "command_id",
    "command_type",
    "context_snapshot_epoch",
    "context_snapshot_sequence",
    "conversation_id",
    "expires_at_unix_ms",
    "issued_at_unix_ms",
    "message_type",
    "origin_agent",
    "payload",
    "priority",
    "protocol_version",
    "request_id",
    "source",
  };

  const std::set<std::string> required{
    "command_id",
    "command_type",
    "expires_at_unix_ms",
    "issued_at_unix_ms",
    "payload",
    "source",
  };

  if (const auto error = reject_unknown_fields(
      document,
      allowed,
      ErrorCode::UnknownTopLevelField,
      "unknown top-level field"))
  {
    return failure(error->code, error->reason);
  }

  if (const auto error = require_fields(document, required)) {
    return failure(error->code, error->reason);
  }

  ValidatedCommand command;

  if (document.contains("protocol_version")) {
    if (!read_integer(
        document.at("protocol_version"),
        command.protocol_version))
    {
      return failure(
        ErrorCode::InvalidFieldType,
        "protocol_version must be an integer");
    }
  }

  if (command.protocol_version !=
    COMMAND_PROTOCOL_VERSION)
  {
    return failure(
      ErrorCode::UnsupportedProtocolVersion,
      "protocol_version must equal 1");
  }

  if (document.contains("message_type")) {
    if (const auto error = read_required_string(
        document,
        "message_type",
        command.message_type))
    {
      return failure(error->code, error->reason);
    }
  }

  if (command.message_type != "command_request") {
    return failure(
      ErrorCode::InvalidMessageType,
      "message_type must equal command_request");
  }

  if (const auto error = read_required_string(
      document,
      "command_id",
      command.command_id))
  {
    return failure(error->code, error->reason);
  }

  if (const auto error = validate_identifier(
      command.command_id,
      "command_id"))
  {
    return failure(error->code, error->reason);
  }

  std::string command_type_text;

  if (const auto error = read_required_string(
      document,
      "command_type",
      command_type_text))
  {
    return failure(error->code, error->reason);
  }

  const auto command_type =
    parse_command_type(command_type_text);

  if (!command_type.has_value()) {
    return failure(
      ErrorCode::InvalidCommandType,
      "unsupported command_type");
  }

  command.command_type = command_type.value();

  if (const auto error = read_required_string(
      document,
      "source",
      command.source))
  {
    return failure(error->code, error->reason);
  }

  if (!is_valid_source(command.source)) {
    return failure(
      ErrorCode::InvalidSource,
      "source has invalid syntax");
  }

  if (!read_integer(
      document.at("issued_at_unix_ms"),
      command.issued_at_unix_ms))
  {
    return failure(
      ErrorCode::InvalidFieldType,
      "issued_at_unix_ms must be an integer");
  }

  if (!read_integer(
      document.at("expires_at_unix_ms"),
      command.expires_at_unix_ms))
  {
    return failure(
      ErrorCode::InvalidFieldType,
      "expires_at_unix_ms must be an integer");
  }

  if (command.issued_at_unix_ms < 0 ||
    command.expires_at_unix_ms < 0)
  {
    return failure(
      ErrorCode::InvalidTimestamp,
      "timestamps must be nonnegative");
  }

  if (command.expires_at_unix_ms <=
    command.issued_at_unix_ms)
  {
    return failure(
      ErrorCode::InvalidTtl,
      "expires_at_unix_ms must be greater than issued_at_unix_ms");
  }

  if (
    command.expires_at_unix_ms -
    command.issued_at_unix_ms >
    MAX_COMMAND_TTL_MS)
  {
    return failure(
      ErrorCode::InvalidTtl,
      "command TTL must not exceed 60000 milliseconds");
  }

  if (command.expires_at_unix_ms <= now_unix_ms) {
    return failure(
      ErrorCode::ExpiredCommand,
      "command is already expired");
  }

  if (const auto error = read_optional_string(
      document,
      "origin_agent",
      command.origin_agent))
  {
    return failure(error->code, error->reason);
  }

  if (command.origin_agent.has_value() &&
    !is_valid_origin_agent(command.origin_agent.value()))
  {
    return failure(
      ErrorCode::InvalidOriginAgent,
      "origin_agent has invalid syntax");
  }

  if (const auto error = read_optional_string(
      document,
      "request_id",
      command.request_id))
  {
    return failure(error->code, error->reason);
  }

  if (command.request_id.has_value()) {
    if (const auto error = validate_identifier(
        command.request_id.value(),
        "request_id"))
    {
      return failure(error->code, error->reason);
    }
  }

  const bool whole_plan_command =
    command.command_type == CommandType::QueryNavigationState ||
    command.command_type == CommandType::StartAutonomousMapping ||
    command.command_type == CommandType::ControlMapping ||
    command.command_type == CommandType::QueryMappingState ||
    command.command_type == CommandType::QuerySupervisorState;
  if (whole_plan_command && !command.request_id.has_value()) {
    return failure(
      ErrorCode::MissingRequiredField,
      "whole-plan commands require request_id");
  }
  if (whole_plan_command && !command.origin_agent.has_value()) {
    return failure(
      ErrorCode::MissingRequiredField,
      "whole-plan commands require origin_agent");
  }

  if (const auto error = read_optional_string(
      document,
      "conversation_id",
      command.conversation_id))
  {
    return failure(error->code, error->reason);
  }

  if (command.conversation_id.has_value()) {
    if (const auto error = validate_identifier(
        command.conversation_id.value(),
        "conversation_id"))
    {
      return failure(error->code, error->reason);
    }
  }

  std::string priority_text{"normal"};

  if (document.contains("priority")) {
    if (const auto error = read_required_string(
        document,
        "priority",
        priority_text))
    {
      return failure(error->code, error->reason);
    }
  }

  const auto priority = parse_priority(priority_text);

  if (!priority.has_value()) {
    return failure(
      ErrorCode::InvalidPriority,
      "unsupported priority");
  }

  command.priority = priority.value();

  const bool has_sequence =
    document.contains("context_snapshot_sequence") &&
    !document.at("context_snapshot_sequence").is_null();

  const bool has_epoch =
    document.contains("context_snapshot_epoch") &&
    !document.at("context_snapshot_epoch").is_null();

  if (has_sequence != has_epoch) {
    return failure(
      ErrorCode::InvalidSnapshotContext,
      "snapshot sequence and epoch must be supplied together");
  }

  if (has_sequence) {
    std::int64_t sequence = 0;
    std::int64_t epoch = 0;

    if (!read_integer(
        document.at("context_snapshot_sequence"),
        sequence))
    {
      return failure(
        ErrorCode::InvalidFieldType,
        "context_snapshot_sequence must be an integer");
    }

    if (!read_integer(
        document.at("context_snapshot_epoch"),
        epoch))
    {
      return failure(
        ErrorCode::InvalidFieldType,
        "context_snapshot_epoch must be an integer");
    }

    if (sequence < 0 || epoch < 1) {
      return failure(
        ErrorCode::InvalidSnapshotContext,
        "snapshot sequence must be at least 0 and epoch at least 1");
    }

    command.context_snapshot_sequence = sequence;
    command.context_snapshot_epoch = epoch;
  }

  if (command.command_type == CommandType::Stop) {
    if (command.priority != CommandPriority::Emergency) {
      return failure(
        ErrorCode::InvalidCommandPriority,
        "stop requires emergency priority");
    }
  } else if (command.priority == CommandPriority::Emergency) {
    return failure(
      ErrorCode::InvalidCommandPriority,
      "emergency priority is reserved for stop");
  }

  if (const auto error = parse_payload(
      command.command_type,
      document.at("payload"),
      command.payload))
  {
    return failure(error->code, error->reason);
  }

  CommandParseResult result;
  result.command = std::move(command);
  return result;
}

}  // namespace

std::string_view to_string(
  const CommandType command_type) noexcept
{
  switch (command_type) {
    case CommandType::Stop:
      return "stop";
    case CommandType::CancelAction:
      return "cancel_action";
    case CommandType::TeleopNudge:
      return "teleop_nudge";
    case CommandType::NavigateToLocation:
      return "navigate_to_location";
    case CommandType::CancelNavigation:
      return "cancel_navigation";
    case CommandType::QueryNavigationState:
      return "query_navigation_state";
    case CommandType::StartAutonomousMapping:
      return "start_autonomous_mapping";
    case CommandType::ControlMapping:
      return "control_mapping";
    case CommandType::QueryMappingState:
      return "query_mapping_state";
    case CommandType::QuerySupervisorState:
      return "query_supervisor_state";
  }

  return "unknown";
}

std::string_view to_string(
  const CommandPriority priority) noexcept
{
  switch (priority) {
    case CommandPriority::Normal:
      return "normal";
    case CommandPriority::High:
      return "high";
    case CommandPriority::Emergency:
      return "emergency";
  }

  return "unknown";
}

std::string_view to_string(
  const CommandProtocolErrorCode code) noexcept
{
  switch (code) {
    case ErrorCode::None:
      return "none";
    case ErrorCode::MalformedJson:
      return "malformed_json";
    case ErrorCode::DuplicateObjectKey:
      return "duplicate_object_key";
    case ErrorCode::TopLevelNotObject:
      return "top_level_not_object";
    case ErrorCode::MissingRequiredField:
      return "missing_required_field";
    case ErrorCode::UnknownTopLevelField:
      return "unknown_top_level_field";
    case ErrorCode::InvalidFieldType:
      return "invalid_field_type";
    case ErrorCode::UnsupportedProtocolVersion:
      return "unsupported_protocol_version";
    case ErrorCode::InvalidMessageType:
      return "invalid_message_type";
    case ErrorCode::InvalidCommandType:
      return "invalid_command_type";
    case ErrorCode::InvalidPriority:
      return "invalid_priority";
    case ErrorCode::InvalidIdentifier:
      return "invalid_identifier";
    case ErrorCode::InvalidSource:
      return "invalid_source";
    case ErrorCode::InvalidOriginAgent:
      return "invalid_origin_agent";
    case ErrorCode::InvalidTimestamp:
      return "invalid_timestamp";
    case ErrorCode::InvalidTtl:
      return "invalid_ttl";
    case ErrorCode::ExpiredCommand:
      return "expired_command";
    case ErrorCode::InvalidSnapshotContext:
      return "invalid_snapshot_context";
    case ErrorCode::InvalidCommandPriority:
      return "invalid_command_priority";
    case ErrorCode::PayloadNotObject:
      return "payload_not_object";
    case ErrorCode::UnknownPayloadField:
      return "unknown_payload_field";
    case ErrorCode::InvalidPayload:
      return "invalid_payload";
  }

  return "unknown";
}

CommandParseResult parse_command_request(
  const std::string_view encoded_json_request,
  const std::int64_t now_unix_ms) noexcept
{
  try {
    bool duplicate_key = false;
    std::vector<std::set<std::string>> object_keys;

    const auto callback =
      [&duplicate_key, &object_keys](
      const int,
      const Json::parse_event_t event,
      Json & parsed)
      {
        if (event == Json::parse_event_t::object_start) {
          object_keys.emplace_back();
        } else if (event == Json::parse_event_t::key) {
          if (object_keys.empty() ||
            !object_keys.back().insert(
              parsed.get<std::string>()).second)
          {
            duplicate_key = true;
          }
        } else if (event == Json::parse_event_t::object_end) {
          if (!object_keys.empty()) {
            object_keys.pop_back();
          }
        }

        return true;
      };

    const Json document = Json::parse(
      encoded_json_request.begin(),
      encoded_json_request.end(),
      callback,
      false);

    if (document.is_discarded()) {
      return failure(
        ErrorCode::MalformedJson,
        "request is not valid JSON");
    }

    if (duplicate_key) {
      return failure(
        ErrorCode::DuplicateObjectKey,
        "request contains a duplicate object key");
    }

    if (!document.is_object()) {
      return failure(
        ErrorCode::TopLevelNotObject,
        "top-level JSON value must be an object");
    }

    return validate_document(document, now_unix_ms);
  } catch (...) {
    return failure(
      ErrorCode::MalformedJson,
      "request could not be parsed safely");
  }
}

}  // namespace savo_bridge
