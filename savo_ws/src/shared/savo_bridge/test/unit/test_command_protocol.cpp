// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

#include "savo_bridge/command_protocol.hpp"

namespace
{

using Json = nlohmann::json;
using savo_bridge::CommandProtocolErrorCode;

inline constexpr std::int64_t NOW_UNIX_MS =
  1800000000500;

inline constexpr std::string_view CANONICAL_STOP =
  R"json(
{
  "command_id": "fixture-stop-001",
  "command_type": "stop",
  "conversation_id": "fixture-conversation-001",
  "expires_at_unix_ms": 1800000001000,
  "issued_at_unix_ms": 1800000000000,
  "message_type": "command_request",
  "origin_agent": "safety_agent",
  "payload": {
    "reason": "fixture_stop",
    "scope": "all_movement"
  },
  "priority": "emergency",
  "protocol_version": 1,
  "request_id": "fixture-request-stop-001",
  "source": "savomind"
}
)json";

inline constexpr std::string_view CANONICAL_TELEOP =
  R"json(
{
  "command_id": "fixture-teleop-001",
  "command_type": "teleop_nudge",
  "context_snapshot_epoch": 3,
  "context_snapshot_sequence": 42,
  "expires_at_unix_ms": 1800000001000,
  "issued_at_unix_ms": 1800000000000,
  "message_type": "command_request",
  "origin_agent": "teleop_agent",
  "payload": {
    "angular_z_radps": 0.0,
    "direction": "forward",
    "duration_ms": 600,
    "linear_x_mps": 0.12,
    "linear_y_mps": 0.0,
    "speed_label": "normal"
  },
  "priority": "normal",
  "protocol_version": 1,
  "source": "savomind"
}
)json";

inline constexpr std::string_view CANONICAL_NAVIGATION =
  R"json(
{
  "command_id": "fixture-navigation-001",
  "command_type": "navigate_to_location",
  "context_snapshot_epoch": 3,
  "context_snapshot_sequence": 42,
  "expires_at_unix_ms": 1800000001000,
  "issued_at_unix_ms": 1800000000000,
  "message_type": "command_request",
  "origin_agent": "navigation_agent",
  "payload": {
    "location_id": "A201",
    "map_id": "campus-main",
    "requested_label": "Room A201"
  },
  "priority": "normal",
  "protocol_version": 1,
  "source": "savomind"
}
)json";

[[nodiscard]] std::optional<std::string> read_fixture(
  const std::string & filename)
{
  const std::filesystem::path path =
    std::filesystem::path{
    "/tmp/savo_bridge_phase2e2a/gate1"} /
  filename;

  std::ifstream input(path);

  if (!input) {
    return std::nullopt;
  }

  return std::string(
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>());
}

void expect_success(
  const std::string_view encoded,
  const std::int64_t now_unix_ms = NOW_UNIX_MS)
{
  const auto result =
    savo_bridge::parse_command_request(
    encoded,
    now_unix_ms);

  ASSERT_TRUE(result.succeeded())
    << (
    result.error.has_value() ?
    result.error->reason :
    "missing command without error");

  EXPECT_TRUE(result.command.has_value());
  EXPECT_FALSE(result.error.has_value());
}

void expect_error(
  const std::string_view encoded,
  const CommandProtocolErrorCode expected,
  const std::int64_t now_unix_ms = NOW_UNIX_MS)
{
  const auto result =
    savo_bridge::parse_command_request(
    encoded,
    now_unix_ms);

  EXPECT_FALSE(result.succeeded());
  EXPECT_FALSE(result.command.has_value());
  ASSERT_TRUE(result.error.has_value());
  EXPECT_EQ(result.error->code, expected);
  EXPECT_FALSE(result.error->reason.empty());
}

[[nodiscard]] Json teleop_document()
{
  return Json::parse(CANONICAL_TELEOP);
}

[[nodiscard]] Json navigation_document()
{
  return Json::parse(CANONICAL_NAVIGATION);
}

TEST(SavoBridgeCommandProtocol, CanonicalStopRequestAccepted)
{
  const auto result =
    savo_bridge::parse_command_request(
    CANONICAL_STOP,
    NOW_UNIX_MS);

  ASSERT_TRUE(result.succeeded());
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(
    result.command->command_type,
    savo_bridge::CommandType::Stop);
  EXPECT_EQ(
    result.command->priority,
    savo_bridge::CommandPriority::Emergency);

  const auto * payload =
    std::get_if<savo_bridge::StopCommandPayload>(
    &result.command->payload);

  ASSERT_NE(payload, nullptr);
  EXPECT_EQ(payload->reason, "fixture_stop");
  EXPECT_EQ(payload->scope, "all_movement");

  const auto fixture =
    read_fixture("request_stop.json");

  if (fixture.has_value()) {
    expect_success(fixture.value());
  }
}

TEST(SavoBridgeCommandProtocol, CanonicalTeleopRequestAccepted)
{
  const auto result =
    savo_bridge::parse_command_request(
    CANONICAL_TELEOP,
    NOW_UNIX_MS);

  ASSERT_TRUE(result.succeeded());
  ASSERT_TRUE(result.command.has_value());

  const auto * payload =
    std::get_if<
    savo_bridge::TeleopNudgeCommandPayload>(
    &result.command->payload);

  ASSERT_NE(payload, nullptr);
  EXPECT_EQ(payload->direction, "forward");
  EXPECT_DOUBLE_EQ(payload->linear_x_mps, 0.12);
  EXPECT_EQ(payload->duration_ms, 600);
  ASSERT_TRUE(payload->speed_label.has_value());
  EXPECT_EQ(payload->speed_label.value(), "normal");

  const auto fixture =
    read_fixture("request_teleop.json");

  if (fixture.has_value()) {
    expect_success(fixture.value());
  }
}

TEST(
  SavoBridgeCommandProtocol,
  CanonicalNavigationByLocationRequestAccepted)
{
  const auto result =
    savo_bridge::parse_command_request(
    CANONICAL_NAVIGATION,
    NOW_UNIX_MS);

  ASSERT_TRUE(result.succeeded());
  ASSERT_TRUE(result.command.has_value());

  const auto * payload =
    std::get_if<
    savo_bridge::NavigateToLocationCommandPayload>(
    &result.command->payload);

  ASSERT_NE(payload, nullptr);
  EXPECT_EQ(payload->location_id, "A201");
  ASSERT_TRUE(payload->map_id.has_value());
  EXPECT_EQ(payload->map_id.value(), "campus-main");

  const auto fixture =
    read_fixture("request_navigation.json");

  if (fixture.has_value()) {
    expect_success(fixture.value());
  }
}

TEST(SavoBridgeCommandProtocol, CancelActionAccepted)
{
  expect_success(
      R"json(
  {
    "command_id": "cancel-action-1",
    "command_type": "cancel_action",
    "source": "savomind",
    "issued_at_unix_ms": 1800000000000,
    "expires_at_unix_ms": 1800000001000,
    "payload": {"target_command_id": "active-command-1"}
  }
  )json");
}

TEST(SavoBridgeCommandProtocol, CancelNavigationAccepted)
{
  const auto result =
    savo_bridge::parse_command_request(
    R"json(
    {
      "command_id": "cancel-navigation-1",
      "command_type": "cancel_navigation",
      "source": "savomind",
      "issued_at_unix_ms": 1800000000000,
      "expires_at_unix_ms": 1800000001000,
      "priority": "high",
      "payload": {}
    }
    )json",
    NOW_UNIX_MS);

  ASSERT_TRUE(result.succeeded());
  const auto * payload =
    std::get_if<
    savo_bridge::CancelNavigationCommandPayload>(
    &result.command->payload);

  ASSERT_NE(payload, nullptr);
  EXPECT_FALSE(payload->target_command_id.has_value());
  EXPECT_EQ(
    payload->reason,
    "user_requested_cancel_navigation");
}

TEST(SavoBridgeCommandProtocol, DefaultsAreApplied)
{
  const auto result =
    savo_bridge::parse_command_request(
    R"json(
    {
      "command_id": "cancel-navigation-defaults",
      "command_type": "cancel_navigation",
      "source": "savomind",
      "issued_at_unix_ms": 1800000000000,
      "expires_at_unix_ms": 1800000001000,
      "payload": {}
    }
    )json",
    NOW_UNIX_MS);

  ASSERT_TRUE(result.succeeded());
  EXPECT_EQ(
    result.command->protocol_version,
    savo_bridge::COMMAND_PROTOCOL_VERSION);
  EXPECT_EQ(
    result.command->message_type,
    "command_request");
  EXPECT_EQ(
    result.command->priority,
    savo_bridge::CommandPriority::Normal);
}

TEST(SavoBridgeCommandProtocol, WrongProtocolVersionRejected)
{
  auto document = teleop_document();
  document["protocol_version"] = 2;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::
    UnsupportedProtocolVersion);
}

TEST(SavoBridgeCommandProtocol, UnknownTopLevelFieldRejected)
{
  auto document = teleop_document();
  document["unsafe_extension"] = true;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::UnknownTopLevelField);
}

TEST(SavoBridgeCommandProtocol, UnknownPayloadFieldRejected)
{
  auto document = teleop_document();
  document["payload"]["frame_id"] = "base_link";
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::UnknownPayloadField);
}

TEST(SavoBridgeCommandProtocol, DuplicateTopLevelKeyRejected)
{
  expect_error(
    R"json(
    {
      "command_id": "first",
      "command_id": "second",
      "command_type": "cancel_navigation",
      "source": "savomind",
      "issued_at_unix_ms": 1800000000000,
      "expires_at_unix_ms": 1800000001000,
      "payload": {}
    }
    )json",
    CommandProtocolErrorCode::DuplicateObjectKey);
}

TEST(SavoBridgeCommandProtocol, DuplicatePayloadKeyRejected)
{
  expect_error(
    R"json(
    {
      "command_id": "duplicate-payload",
      "command_type": "cancel_navigation",
      "source": "savomind",
      "issued_at_unix_ms": 1800000000000,
      "expires_at_unix_ms": 1800000001000,
      "payload": {
        "reason": "first",
        "reason": "second"
      }
    }
    )json",
    CommandProtocolErrorCode::DuplicateObjectKey);
}

TEST(SavoBridgeCommandProtocol, StringifiedIntegerRejected)
{
  auto document = teleop_document();
  document["protocol_version"] = "1";
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidFieldType);
}

TEST(SavoBridgeCommandProtocol, BooleanIntegerRejected)
{
  auto document = teleop_document();
  document["context_snapshot_sequence"] = true;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidFieldType);
}

TEST(SavoBridgeCommandProtocol, FloatingTimestampRejected)
{
  auto document = teleop_document();
  document["issued_at_unix_ms"] = 1800000000000.5;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidFieldType);
}

TEST(SavoBridgeCommandProtocol, InvalidTtlRejected)
{
  auto document = teleop_document();
  document["expires_at_unix_ms"] =
    1800000060001;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidTtl);
}

TEST(SavoBridgeCommandProtocol, ExpiredCommandRejected)
{
  expect_error(
    CANONICAL_TELEOP,
    CommandProtocolErrorCode::ExpiredCommand,
    1800000001000);
}

TEST(SavoBridgeCommandProtocol, SnapshotSequenceWithoutEpochRejected)
{
  auto document = teleop_document();
  document.erase("context_snapshot_epoch");
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidSnapshotContext);
}

TEST(SavoBridgeCommandProtocol, SnapshotEpochWithoutSequenceRejected)
{
  auto document = teleop_document();
  document.erase("context_snapshot_sequence");
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidSnapshotContext);
}

TEST(SavoBridgeCommandProtocol, NullSnapshotPairAccepted)
{
  auto document = teleop_document();
  document["context_snapshot_sequence"] = nullptr;
  document["context_snapshot_epoch"] = nullptr;
  expect_success(document.dump());
}

TEST(SavoBridgeCommandProtocol, StopWithoutEmergencyPriorityRejected)
{
  auto document = Json::parse(CANONICAL_STOP);
  document["priority"] = "normal";
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidCommandPriority);
}

TEST(SavoBridgeCommandProtocol, EmergencyTeleopRejected)
{
  auto document = teleop_document();
  document["priority"] = "emergency";
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidCommandPriority);
}

TEST(SavoBridgeCommandProtocol, MismatchedCommandPayloadRejected)
{
  auto document = teleop_document();
  document["command_type"] = "stop";
  document["priority"] = "emergency";
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::UnknownPayloadField);
}

TEST(SavoBridgeCommandProtocol, TeleopMultipleAxesRejected)
{
  auto document = teleop_document();
  document["payload"]["linear_y_mps"] = 0.1;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidPayload);
}

TEST(SavoBridgeCommandProtocol, TeleopDirectionSignMismatchRejected)
{
  auto document = teleop_document();
  document["payload"]["linear_x_mps"] = -0.12;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidPayload);
}

TEST(SavoBridgeCommandProtocol, LinearSpeedAboveLimitRejected)
{
  auto document = teleop_document();
  document["payload"]["linear_x_mps"] = 0.180001;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidPayload);
}

TEST(SavoBridgeCommandProtocol, AngularSpeedAboveLimitRejected)
{
  auto document = teleop_document();
  document["payload"]["direction"] = "turn_left";
  document["payload"]["linear_x_mps"] = 0.0;
  document["payload"]["angular_z_radps"] = 0.650001;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidPayload);
}

TEST(SavoBridgeCommandProtocol, DurationAboveLimitRejected)
{
  auto document = teleop_document();
  document["payload"]["duration_ms"] = 1001;
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidPayload);
}

TEST(SavoBridgeCommandProtocol, BoundaryTeleopValuesAccepted)
{
  auto linear = teleop_document();
  linear["payload"]["linear_x_mps"] =
    savo_bridge::MAX_TELEOP_LINEAR_SPEED_MPS;
  linear["payload"]["duration_ms"] =
    savo_bridge::MAX_TELEOP_DURATION_MS;
  expect_success(linear.dump());

  auto angular = teleop_document();
  angular["payload"]["direction"] = "turn_right";
  angular["payload"]["linear_x_mps"] = 0.0;
  angular["payload"]["angular_z_radps"] =
    -savo_bridge::MAX_TELEOP_ANGULAR_SPEED_RADPS;
  angular["payload"]["duration_ms"] = 1;
  expect_success(angular.dump());
}

TEST(SavoBridgeCommandProtocol, NavigationPoseFieldRejected)
{
  auto document = navigation_document();
  document["payload"]["pose"] = {
    {"x", 1.0},
    {"y", 2.0},
  };
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::UnknownPayloadField);
}

TEST(SavoBridgeCommandProtocol, NavigationRawRosFieldRejected)
{
  auto document = navigation_document();
  document["payload"]["action_name"] =
    "/navigate_to_pose";
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::UnknownPayloadField);
}

TEST(SavoBridgeCommandProtocol, InvalidIdentifiersRejected)
{
  auto command = teleop_document();
  command["command_id"] = "/unsafe";
  expect_error(
    command.dump(),
    CommandProtocolErrorCode::InvalidIdentifier);

  auto navigation = navigation_document();
  navigation["payload"]["location_id"] = "bad id";
  expect_error(
    navigation.dump(),
    CommandProtocolErrorCode::InvalidIdentifier);
}

TEST(SavoBridgeCommandProtocol, InvalidSourceRejected)
{
  auto document = teleop_document();
  document["source"] = "SavoMind";
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidSource);
}

TEST(SavoBridgeCommandProtocol, InvalidOriginAgentRejected)
{
  auto document = teleop_document();
  document["origin_agent"] = "agent-with-dash";
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidOriginAgent);
}

TEST(SavoBridgeCommandProtocol, EmptyTrimmedReasonRejected)
{
  auto document = Json::parse(CANONICAL_STOP);
  document["payload"]["reason"] = " \t\n ";
  expect_error(
    document.dump(),
    CommandProtocolErrorCode::InvalidPayload);
}

TEST(SavoBridgeCommandProtocol, MalformedJsonRejected)
{
  expect_error(
    R"json({"command_id":)json",
    CommandProtocolErrorCode::MalformedJson);
}

TEST(SavoBridgeCommandProtocol, NonObjectJsonRejected)
{
  expect_error(
    R"json(["not", "a", "command"])json",
    CommandProtocolErrorCode::TopLevelNotObject);
}

TEST(SavoBridgeCommandProtocol, ParsesAuthorizedWholePlanMappingCommands)
{
  const auto start = savo_bridge::parse_command_request(
    R"json({
      "command_id":"mapping-start-1",
      "command_type":"start_autonomous_mapping",
      "expires_at_unix_ms":1800000001000,
      "issued_at_unix_ms":1800000000000,
      "origin_agent":"mapping_agent",
      "request_id":"mapping-request-1",
      "payload":{"mission_id":"mission-1","map_id":"map-1","map_revision":1,
        "auto_save":true,"require_quality_approval":true},
      "source":"savomind"
    })json",
    NOW_UNIX_MS);
  ASSERT_TRUE(start.succeeded());
  EXPECT_EQ(start.command->command_type,
    savo_bridge::CommandType::StartAutonomousMapping);
  const auto * start_payload =
    std::get_if<savo_bridge::StartAutonomousMappingCommandPayload>(
    &start.command->payload);
  ASSERT_NE(start_payload, nullptr);
  EXPECT_TRUE(start_payload->auto_save);
  EXPECT_TRUE(start_payload->require_quality_approval);
  EXPECT_TRUE(start_payload->require_semantic);

  const auto map_only = savo_bridge::parse_command_request(
    R"json({
      "command_id":"mapping-start-map-only-1",
      "command_type":"start_autonomous_mapping",
      "expires_at_unix_ms":1800000001000,
      "issued_at_unix_ms":1800000000000,
      "origin_agent":"mapping_agent",
      "request_id":"mapping-request-map-only-1",
      "payload":{"mission_id":"mission-map-only-1","map_id":"map-1",
        "map_revision":1,"auto_save":true,
        "require_quality_approval":true,"require_semantic":false},
      "source":"savomind"
    })json",
    NOW_UNIX_MS);
  ASSERT_TRUE(map_only.succeeded());
  const auto * map_only_payload =
    std::get_if<savo_bridge::StartAutonomousMappingCommandPayload>(
    &map_only.command->payload);
  ASSERT_NE(map_only_payload, nullptr);
  EXPECT_FALSE(map_only_payload->require_semantic);

  const auto control = savo_bridge::parse_command_request(
    R"json({
      "command_id":"mapping-pause-1",
      "command_type":"control_mapping",
      "expires_at_unix_ms":1800000001000,
      "issued_at_unix_ms":1800000000000,
      "origin_agent":"mapping_agent",
      "request_id":"mapping-request-2",
      "payload":{"mission_id":"mission-1","operation":"pause"},
      "source":"savomind"
    })json",
    NOW_UNIX_MS);
  ASSERT_TRUE(control.succeeded());
  EXPECT_EQ(control.command->command_type,
    savo_bridge::CommandType::ControlMapping);

  const auto scan360 = savo_bridge::parse_command_request(
    R"json({
      "command_id":"mapping-scan360-1",
      "command_type":"control_mapping",
      "expires_at_unix_ms":1800000001000,
      "issued_at_unix_ms":1800000000000,
      "origin_agent":"mapping_agent",
      "request_id":"mapping-request-3",
      "payload":{"mission_id":"mission-1","operation":"request_scan360"},
      "source":"savomind"
    })json",
    NOW_UNIX_MS);
  ASSERT_TRUE(scan360.succeeded());
  const auto * scan_payload =
    std::get_if<savo_bridge::ControlMappingCommandPayload>(
    &scan360.command->payload);
  ASSERT_NE(scan_payload, nullptr);
  EXPECT_EQ(scan_payload->operation, "request_scan360");
}

TEST(SavoBridgeCommandProtocol, MappingCannotDisableSaveOrOperatorApproval)
{
  const auto result = savo_bridge::parse_command_request(
    R"json({
      "command_id":"mapping-unsafe-1",
      "command_type":"start_autonomous_mapping",
      "expires_at_unix_ms":1800000001000,
      "issued_at_unix_ms":1800000000000,
      "origin_agent":"mapping_agent",
      "request_id":"mapping-request-unsafe",
      "payload":{"mission_id":"mission-1","map_id":"map-1","map_revision":1,
        "auto_save":true,"require_quality_approval":false},
      "source":"savomind"
    })json",
    NOW_UNIX_MS);
  ASSERT_FALSE(result.succeeded());
  ASSERT_TRUE(result.error.has_value());
  EXPECT_EQ(result.error->code, CommandProtocolErrorCode::InvalidPayload);
}

TEST(SavoBridgeCommandProtocol, WholePlanQueriesRequireCorrelationAndActor)
{
  const auto result = savo_bridge::parse_command_request(
    R"json({
      "command_id":"query-1",
      "command_type":"query_supervisor_state",
      "expires_at_unix_ms":1800000001000,
      "issued_at_unix_ms":1800000000000,
      "payload":{"scope":"supervisor"},
      "source":"savomind"
    })json",
    NOW_UNIX_MS);
  ASSERT_FALSE(result.succeeded());
  ASSERT_TRUE(result.error.has_value());
  EXPECT_EQ(result.error->code, CommandProtocolErrorCode::MissingRequiredField);
}

TEST(SavoBridgeCommandProtocol, ErrorCodesHaveStableStrings)
{
  EXPECT_EQ(
    savo_bridge::to_string(
      CommandProtocolErrorCode::DuplicateObjectKey),
    "duplicate_object_key");
  EXPECT_EQ(
    savo_bridge::to_string(
      CommandProtocolErrorCode::InvalidPayload),
    "invalid_payload");
}

TEST(
  SavoBridgeCommandProtocol,
  ParserHasNoRosPublicationSocketOrPhysicalDispatch)
{
  const std::filesystem::path package_root =
    std::filesystem::path{__FILE__}
  .parent_path()
  .parent_path()
  .parent_path();

  const std::vector<std::filesystem::path> files{
    package_root /
    "include/savo_bridge/command_protocol.hpp",
    package_root / "src/command_protocol.cpp",
  };

  const std::vector<std::string> forbidden{
    "AF_UNIX",
    "SOCK_STREAM",
    "sockaddr_un",
    "command.sock",
    "create_publisher",
    "rclcpp",
    "cmd_vel",
    "physical_dispatch",
  };

  for (const auto & file : files) {
    std::ifstream input(file);
    ASSERT_TRUE(input) << file;

    const std::string content{
      std::istreambuf_iterator<char>{input},
      std::istreambuf_iterator<char>{}};

    for (const auto & token : forbidden) {
      EXPECT_EQ(content.find(token), std::string::npos)
        << file << " contains " << token;
    }
  }
}

}  // namespace
