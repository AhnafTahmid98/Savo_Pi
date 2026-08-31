// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <string>

#include "savo_supervisor/edge_payload_parser.hpp"

TEST(EdgePayloadParser, ParsesProductionBridgeState)
{
  const std::string payload =
    R"({
    "schema_name":"savo_bridge_state",
    "schema_version":2,
    "process_alive":true,
    "bridge_ready":true,
    "commands_enabled":true,
    "dds_active":true,
    "core_visible":true,
    "edge_visible":true,
    "stop_ready":true,
    "teleop_ready":true,
    "navigation_ready":true,
    "readiness_reason":"bridge_ready"
  })";
  const auto parsed = savo_supervisor::EdgePayloadParser{}.ParseBridgeState(payload);
  EXPECT_TRUE(parsed.valid);
  EXPECT_TRUE(parsed.bridge_ready);
  EXPECT_TRUE(parsed.core_visible);
  EXPECT_EQ(parsed.reason, "bridge_ready");
}

TEST(EdgePayloadParser, RejectsBridgeSchemaMismatch)
{
  const auto parsed = savo_supervisor::EdgePayloadParser{}.ParseBridgeState(
    R"({"schema_name":"other","schema_version":2})");
  EXPECT_FALSE(parsed.valid);
  EXPECT_EQ(parsed.reason, "bridge_schema_invalid");
}

TEST(EdgePayloadParser, ParsesRealSenseStatus)
{
  const std::string payload =
    R"({"ok":true,"color_ok":true,"depth_ok":true,"pointcloud_ok":true,)"
    R"("require_pointcloud":false,"message":"RealSense streams OK"})";
  const auto parsed =
    savo_supervisor::EdgePayloadParser{}.ParseRealSenseStatus(payload);
  EXPECT_TRUE(parsed.valid);
  EXPECT_TRUE(parsed.healthy);
  EXPECT_TRUE(parsed.color_ready);
  EXPECT_TRUE(parsed.depth_ready);
  EXPECT_TRUE(parsed.pointcloud_ready);
}

TEST(EdgePayloadParser, ParsesSpeechReadiness)
{
  const auto ready = savo_supervisor::EdgePayloadParser{}.ParseSpeechReadiness("ready");
  EXPECT_TRUE(ready.valid);
  EXPECT_TRUE(ready.ready);

  const auto waiting =
    savo_supervisor::EdgePayloadParser{}.ParseSpeechReadiness("waiting_for_audio");
  EXPECT_TRUE(waiting.valid);
  EXPECT_FALSE(waiting.ready);
}

TEST(EdgePayloadParser, ParsesVoHealthLevels)
{
  const auto ready = savo_supervisor::EdgePayloadParser{}.ParseVoHealth("ok: tracking");
  EXPECT_TRUE(ready.valid);
  EXPECT_TRUE(ready.ready);

  const auto degraded =
    savo_supervisor::EdgePayloadParser{}.ParseVoHealth("degraded: low features");
  EXPECT_TRUE(degraded.valid);
  EXPECT_TRUE(degraded.degraded);
  EXPECT_FALSE(degraded.ready);

  const auto stale =
    savo_supervisor::EdgePayloadParser{}.ParseVoHealth(
    "stale: visual odometry timeout");
  EXPECT_TRUE(stale.valid);
  EXPECT_EQ(stale.state, "stale");
  EXPECT_FALSE(stale.ready);
  EXPECT_TRUE(stale.degraded);
  EXPECT_EQ(stale.reason, "visual odometry timeout");

  const auto unsupported =
    savo_supervisor::EdgePayloadParser{}.ParseVoHealth("paused: operator request");
  EXPECT_FALSE(unsupported.valid);
  EXPECT_EQ(unsupported.reason, "vo_health_invalid");
}
