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

TEST(CorePayloadParser, ParsesDirectBaseBatteryMeasurement)
{
  CorePayloadParser parser;
  const auto result = parser.ParsePowerSource(
    "Base battery LOW: 6.65 V, SoC 12.4%", "base_battery");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
  EXPECT_EQ(result.reason_code, "base_battery_low");
  ASSERT_TRUE(result.voltage_v.has_value());
  EXPECT_DOUBLE_EQ(result.voltage_v.value(), 6.65);
}

TEST(CorePayloadParser, ParsesDirectUpsJsonMeasurement)
{
  CorePayloadParser parser;
  const auto result = parser.ParsePowerSource(
    R"({"source":"core_ups","state":"OK","ok":true,"voltage_v":4.01})",
    "core_ups");
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.degraded);
  EXPECT_EQ(result.reason_code, "core_ups_operational");
  ASSERT_TRUE(result.voltage_v.has_value());
  EXPECT_DOUBLE_EQ(result.voltage_v.value(), 4.01);
}

TEST(CorePayloadParser, RejectsWrongDirectPowerSource)
{
  CorePayloadParser parser;
  const auto result = parser.ParsePowerSource(
    "Edge UPS OK: 4.02 V, capacity 86.0%", "core_ups");
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.reason_code, "core_ups_source_mismatch");
}

TEST(CorePayloadParser, DirectCriticalSourceFailsClosed)
{
  CorePayloadParser parser;
  const auto result = parser.ParsePowerSource(
    "Core UPS CRITICAL: 3.19 V, capacity 2.0%", "core_ups");
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.reason_code, "core_ups_critical");
}

TEST(CorePayloadParser, RejectsInvalidDirectVoltage)
{
  CorePayloadParser parser;
  const auto result = parser.ParsePowerSource(
    R"({"source":"edge_ups","state":"OK","ok":true,"voltage_v":null})",
    "edge_ups");
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.reason_code, "edge_ups_measurement_invalid");
}

TEST(CorePayloadParser, DirectReadErrorKeepsSourceSpecificReason)
{
  CorePayloadParser parser;
  const auto result = parser.ParsePowerSource(
    "Core UPS error: n/a V, error: i2c read failed", "core_ups");
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.reason_code, "core_ups_error");
  EXPECT_FALSE(result.voltage_v.has_value());
}
