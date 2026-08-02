// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__COMMAND_PROTOCOL_HPP_
#define SAVO_BRIDGE__COMMAND_PROTOCOL_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

namespace savo_bridge
{

inline constexpr std::int64_t COMMAND_PROTOCOL_VERSION = 1;
inline constexpr std::int64_t MAX_COMMAND_TTL_MS = 60000;
inline constexpr double MAX_TELEOP_LINEAR_SPEED_MPS = 0.18;
inline constexpr double MAX_TELEOP_ANGULAR_SPEED_RADPS = 0.65;
inline constexpr std::int64_t MAX_TELEOP_DURATION_MS = 1000;

enum class CommandType
{
  Stop,
  CancelAction,
  TeleopNudge,
  NavigateToLocation,
  CancelNavigation,
  QueryNavigationState,
  StartAutonomousMapping,
  ControlMapping,
  QueryMappingState,
  QuerySupervisorState,
};

enum class CommandPriority
{
  Normal,
  High,
  Emergency,
};

enum class CommandProtocolErrorCode
{
  None,
  MalformedJson,
  DuplicateObjectKey,
  TopLevelNotObject,
  MissingRequiredField,
  UnknownTopLevelField,
  InvalidFieldType,
  UnsupportedProtocolVersion,
  InvalidMessageType,
  InvalidCommandType,
  InvalidPriority,
  InvalidIdentifier,
  InvalidSource,
  InvalidOriginAgent,
  InvalidTimestamp,
  InvalidTtl,
  ExpiredCommand,
  InvalidSnapshotContext,
  InvalidCommandPriority,
  PayloadNotObject,
  UnknownPayloadField,
  InvalidPayload,
};

[[nodiscard]] std::string_view to_string(
  CommandType command_type) noexcept;

[[nodiscard]] std::string_view to_string(
  CommandPriority priority) noexcept;

[[nodiscard]] std::string_view to_string(
  CommandProtocolErrorCode code) noexcept;

struct StopCommandPayload
{
  std::string reason;
  std::string scope;
};

struct CancelActionCommandPayload
{
  std::string target_command_id;
  std::string reason;
};

struct TeleopNudgeCommandPayload
{
  std::string direction;
  double linear_x_mps{0.0};
  double linear_y_mps{0.0};
  double angular_z_radps{0.0};
  std::int64_t duration_ms{0};
  std::optional<std::string> speed_label;
};

struct NavigateToLocationCommandPayload
{
  std::string location_id;
  std::optional<std::string> requested_label;
  std::optional<std::string> map_id;
};

struct CancelNavigationCommandPayload
{
  std::optional<std::string> target_command_id;
  std::string reason;
};

struct QueryStateCommandPayload
{
  std::string scope;
};

struct StartAutonomousMappingCommandPayload
{
  std::string mission_id;
  std::string map_id;
  std::uint32_t map_revision{0U};
  std::int64_t mission_timeout_ms{0};
  bool auto_save{true};
  bool require_quality_approval{true};
};

struct ControlMappingCommandPayload
{
  std::string operation;
  std::string mission_id;
  std::string reason;
};

using CommandPayload = std::variant<
  StopCommandPayload,
  CancelActionCommandPayload,
  TeleopNudgeCommandPayload,
  NavigateToLocationCommandPayload,
  CancelNavigationCommandPayload,
  QueryStateCommandPayload,
  StartAutonomousMappingCommandPayload,
  ControlMappingCommandPayload>;

struct ValidatedCommand
{
  std::int64_t protocol_version{COMMAND_PROTOCOL_VERSION};
  std::string message_type{"command_request"};
  std::string command_id;
  CommandType command_type{CommandType::Stop};
  std::string source;
  std::int64_t issued_at_unix_ms{0};
  std::int64_t expires_at_unix_ms{0};
  std::optional<std::string> origin_agent;
  std::optional<std::string> request_id;
  std::optional<std::string> conversation_id;
  CommandPriority priority{CommandPriority::Normal};
  std::optional<std::int64_t> context_snapshot_sequence;
  std::optional<std::int64_t> context_snapshot_epoch;
  CommandPayload payload{StopCommandPayload{}};
};

struct CommandProtocolError
{
  CommandProtocolErrorCode code{CommandProtocolErrorCode::None};
  std::string reason;
};

struct CommandParseResult
{
  std::optional<ValidatedCommand> command;
  std::optional<CommandProtocolError> error;

  [[nodiscard]] bool succeeded() const noexcept
  {
    return command.has_value() && !error.has_value();
  }
};

[[nodiscard]] CommandParseResult parse_command_request(
  std::string_view encoded_json_request,
  std::int64_t now_unix_ms) noexcept;

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__COMMAND_PROTOCOL_HPP_
