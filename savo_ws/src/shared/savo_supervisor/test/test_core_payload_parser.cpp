// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_supervisor/core_payload_parser.hpp"

using savo_supervisor::CorePayloadParser;

TEST(CorePayloadParser, ParsesHealthyBase)
{
  CorePayloadParser parser;
  const auto result =
    parser.ParseBaseState(
    R"({
    "status_level":"OK",
    "backend":{"connected":true},
    "diagnostics":{"last_board_error":""}
  })");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.degraded);
}

TEST(CorePayloadParser, BaseSafetyBlockRemainsReady)
{
  CorePayloadParser parser;
  const auto result =
    parser.ParseBaseState(
    R"({
    "status_level":"BLOCKED",
    "backend":{"connected":true},
    "diagnostics":{"last_board_error":""}
  })");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
}

TEST(CorePayloadParser, BaseStaleCommandIsNormalSafeIdle)
{
  CorePayloadParser parser;
  const auto result =
    parser.ParseBaseState(
    R"({
    "status_level":"STALE",
    "backend":{"connected":true},
    "diagnostics":{"last_board_error":""}
  })");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.degraded);
  EXPECT_EQ(result.reason_code, "base_command_stale_safe_zero");
}

TEST(CorePayloadParser, RejectsDisconnectedBase)
{
  CorePayloadParser parser;
  const auto result =
    parser.ParseBaseState(
    R"({
    "status_level":"ERROR",
    "backend":{"connected":false},
    "diagnostics":{"last_board_error":"i2c_error"}
  })");
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.state, "ERROR");
}

TEST(CorePayloadParser, ControlStaleCommandIsSafeOperationalState)
{
  CorePayloadParser parser;
  const auto result = parser.ParseControlStatus(
    "mode=STOP; source=STOP; reason=stale_zero; stale=true; safety_stop=false");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.degraded);
}

TEST(CorePayloadParser, PerceptionRequiredSensorFailureBlocksReadiness)
{
  CorePayloadParser parser;
  const auto result =
    parser.ParsePerceptionHealth(R"({
    "overall_ok":false,
    "overall_status":"STALE"
  })");
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.state, "STALE");
}

TEST(CorePayloadParser, ParsesHealthyLidar)
{
  CorePayloadParser parser;
  const auto result =
    parser.ParseLidarState(
    R"({
    "status":"OK",
    "hardware_ok":true,
    "scan_ok":true,
    "driver_running":true,
    "last_error":""
  })");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ready);
}

TEST(CorePayloadParser, LowPowerIsReadyButDegraded)
{
  CorePayloadParser parser;
  const auto result = parser.ParsePowerStatus(
    "overall=LOW core=OK edge=OK base=LOW");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
}

TEST(CorePayloadParser, CriticalPowerBlocksReadiness)
{
  CorePayloadParser parser;
  const auto result = parser.ParsePowerHealth(
    "level=ERROR state=CRITICAL reason=critical_power");
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.ready);
}
