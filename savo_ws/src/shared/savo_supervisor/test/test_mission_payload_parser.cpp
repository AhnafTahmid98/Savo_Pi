// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "savo_supervisor/mission_payload_parser.hpp"

namespace
{

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

TEST(MissionPayloadParser, ParsesMappingStatus)
{
  savo_supervisor::MissionPayloadParser parser;
  const auto result = parser.ParseMappingStatus(
    R"({"mode":"online_async","workflow_phase":"active","session_state":"active","healthy":true,"ready":true,"slam_active":true,"map_received":true,"scan_received":true,"tf_ok":true,"odom_ok":true,"active_map_name":"floor_2"})");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.healthy);
  EXPECT_TRUE(result.ready);
  EXPECT_EQ(result.active_map_name, "floor_2");
}

TEST(MissionPayloadParser, RejectsIncompleteMappingStatus)
{
  savo_supervisor::MissionPayloadParser parser;
  const auto result = parser.ParseMappingStatus(R"({"healthy":true})");
  EXPECT_FALSE(result.valid);
}

TEST(MissionPayloadParser, ParsesNavigationStatus)
{
  savo_supervisor::MissionPayloadParser parser;
  const auto result = parser.ParseNavigationStatus(
    "state=READY;goal_acceptance_allowed=true;reason=navigation_ready;failed_dependencies=");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.goal_acceptance_allowed);
}

TEST(MissionPayloadParser, ParsesLocationsStatus)
{
  savo_supervisor::MissionPayloadParser parser;
  const auto result = parser.ParseLocationsStatus(
    R"({"component":"savo_locations","read_ready":true,"write_ready":true,"storage_healthy":true,"mutation_in_progress":false,"reason":"ready"})");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.read_ready);
  EXPECT_TRUE(result.write_ready);
}

TEST(MissionPayloadParser, ParsesHeadSemanticReadiness)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "savo_head.head_status";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "OK";
  status.values = {
    kv("pan_tilt_state", "OK"),
    kv("camera_stream_healthy", "true"),
    kv("camera_pose_ready", "true")};
  diagnostic_msgs::msg::DiagnosticArray message;
  message.status = {status};

  savo_supervisor::MissionPayloadParser parser;
  const auto result = parser.ParseHeadStatus(message);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.operational);
  EXPECT_TRUE(result.pan_tilt_ready);
  EXPECT_TRUE(result.camera_ready);
  EXPECT_TRUE(result.camera_pose_ready);
}

TEST(MissionPayloadParser, StaleHeadDiagnosticIsNotOperational)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "savo_head.head_status";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  status.message = "stale";
  status.values = {
    kv("pan_tilt_state", "OK"),
    kv("camera_stream_healthy", "true"),
    kv("camera_pose_ready", "true")};
  diagnostic_msgs::msg::DiagnosticArray message;
  message.status = {status};

  savo_supervisor::MissionPayloadParser parser;
  const auto result = parser.ParseHeadStatus(message);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.operational);
}
