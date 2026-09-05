// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "savo_supervisor/localization_payload_parser.hpp"
#include "savo_supervisor/reason_codes.hpp"

namespace
{

using savo_supervisor::LocalizationPayloadParser;

constexpr char kValidHealth[] =
  R"({
  "schema_version": 1,
  "node": "localization_health_node",
  "state": "OK",
  "ready": true,
  "degraded": false,
  "reason_code": "localization_operational",
  "stamp_s": 100.25,
  "extra_field": {"allowed": true}
})";

constexpr char kValidSummary[] =
  R"({
  "schema_version": 1,
  "state": "DEGRADED",
  "ready": true,
  "degraded": true,
  "reason_code": "localization_degraded",
  "stamp_s": 101.5
})";

constexpr char kValidHeartbeat[] =
  R"({
  "schema_version": 1,
  "node": "localization_health_node",
  "alive": true,
  "state": "OK",
  "ready": true,
  "stamp_s": 102.75
})";

}  // namespace

TEST(LocalizationPayloadParser, ParsesValidHealth)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth(kValidHealth);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.schema_supported);
  EXPECT_EQ(result.state, "OK");
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.degraded);

  EXPECT_EQ(
    result.reason_code,
    "localization_operational");

  ASSERT_TRUE(result.stamp.has_value());
  EXPECT_EQ(result.stamp->sec, 100);
  EXPECT_EQ(result.stamp->nanosec, 250000000u);
}

TEST(LocalizationPayloadParser, ParsesValidSummary)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseSummary(kValidSummary);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.state, "DEGRADED");
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
}

TEST(LocalizationPayloadParser, ParsesValidHeartbeat)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHeartbeat(kValidHeartbeat);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.alive);
  EXPECT_TRUE(result.ready);
  EXPECT_EQ(result.state, "OK");
  EXPECT_TRUE(result.reason_code.empty());
}

TEST(LocalizationPayloadParser, RejectsEmptyPayload)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth("");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.detail, "payload is empty");
}

TEST(LocalizationPayloadParser, RejectsMalformedJson)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth(
    R"({"schema_version":1)");

  EXPECT_FALSE(result.valid);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationMessageInvalid);
}

TEST(LocalizationPayloadParser, RejectsPrefixedJson)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth(
    std::string("garbage ") + kValidHealth);

  EXPECT_FALSE(result.valid);
}

TEST(LocalizationPayloadParser, RejectsJsonArray)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth("[]");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.detail,
    "payload must be a JSON object");
}

TEST(LocalizationPayloadParser, RejectsUnsupportedSchema)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth(
    R"({
      "schema_version": 2,
      "state": "OK",
      "ready": true,
      "degraded": false,
      "reason_code": "localization_operational",
      "stamp_s": 1.0
    })");

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.schema_supported);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationSchemaUnsupported);
}

TEST(LocalizationPayloadParser, RejectsMissingSchema)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth(
    R"({
      "state": "OK",
      "ready": true,
      "degraded": false,
      "reason_code": "localization_operational",
      "stamp_s": 1.0
    })");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.detail,
    "missing required field: schema_version");
}

TEST(LocalizationPayloadParser, RejectsWrongBooleanType)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth(
    R"({
      "schema_version": 1,
      "state": "OK",
      "ready": "true",
      "degraded": false,
      "reason_code": "localization_operational",
      "stamp_s": 1.0
    })");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.detail,
    "field must be a boolean: ready");
}

TEST(LocalizationPayloadParser, RejectsUnknownState)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth(
    R"({
      "schema_version": 1,
      "state": "PERFECT",
      "ready": true,
      "degraded": false,
      "reason_code": "localization_operational",
      "stamp_s": 1.0
    })");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.detail,
    "unsupported localization state: PERFECT");
}

TEST(LocalizationPayloadParser, RejectsEmptyReasonCode)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseSummary(
    R"({
      "schema_version": 1,
      "state": "OK",
      "ready": true,
      "degraded": false,
      "reason_code": "",
      "stamp_s": 1.0
    })");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.detail,
    "field must not be empty: reason_code");
}

TEST(LocalizationPayloadParser, RejectsMissingTimestamp)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHeartbeat(
    R"({
      "schema_version": 1,
      "alive": true,
      "state": "OK",
      "ready": true
    })");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.detail,
    "missing required field: stamp_s");
}

TEST(LocalizationPayloadParser, RejectsNegativeTimestamp)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHeartbeat(
    R"({
      "schema_version": 1,
      "alive": true,
      "state": "OK",
      "ready": true,
      "stamp_s": -0.1
    })");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.detail,
    "timestamp must be non-negative");
}

TEST(LocalizationPayloadParser, RejectsOversizedTimestamp)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHeartbeat(
    R"({
      "schema_version": 1,
      "alive": true,
      "state": "OK",
      "ready": true,
      "stamp_s": 999999999999999999999999
    })");

  EXPECT_FALSE(result.valid);
}

TEST(LocalizationPayloadParser, AcceptsScientificTimestamp)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHeartbeat(
    R"({
      "schema_version": 1,
      "alive": true,
      "state": "OK",
      "ready": true,
      "stamp_s": 1.25e2
    })");

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.stamp.has_value());
  EXPECT_EQ(result.stamp->sec, 125);
  EXPECT_EQ(result.stamp->nanosec, 0u);
}

TEST(LocalizationPayloadParser, RejectsDuplicateTopLevelField)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth(
    R"({
      "schema_version": 1,
      "state": "OK",
      "state": "ERROR",
      "ready": true,
      "degraded": false,
      "reason_code": "localization_operational",
      "stamp_s": 1.0
    })");

  EXPECT_FALSE(result.valid);

  EXPECT_EQ(
    result.detail,
    "duplicate top-level field: state");
}

TEST(LocalizationPayloadParser, HeartbeatAliveFalseIsValidData)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHeartbeat(
    R"({
      "schema_version": 1,
      "alive": false,
      "state": "ERROR",
      "ready": false,
      "reason_code": "localization_error",
      "stamp_s": 1.0
    })");

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.alive);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.state, "ERROR");
}

TEST(LocalizationPayloadParser, AdditionalFieldsAreAllowed)
{
  LocalizationPayloadParser parser(1);

  const auto result =
    parser.ParseHealth(kValidHealth);

  EXPECT_TRUE(result.valid);
}

TEST(LocalizationPayloadParser, AcceptsExtendedRateQualityMetadata)
{
  LocalizationPayloadParser parser(1);

  const auto result = parser.ParseHealth(
    R"({
      "schema_version": 1,
      "node": "localization_health_node",
      "state": "OK",
      "ready": true,
      "degraded": false,
      "reason_code": "localization_operational",
      "stamp_s": 103.0,
      "components": {
        "imu": {
          "rate_hz": 25.0,
          "source_rate_hz": 25.0,
          "receive_rate_hz": 11.7,
          "source_rate_available": true,
          "rate_basis": "producer_successful_publication",
          "rate_quality": "EXCELLENT"
        }
      }
    })");

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.state, "OK");
  EXPECT_TRUE(result.ready);
}

TEST(LocalizationPayloadParser, RateQualityLabelsAreNotTopLevelStates)
{
  LocalizationPayloadParser parser(1);

  for (const char * state : {"GOOD", "EXCELLENT"}) {
    const auto result = parser.ParseHealth(
      std::string(R"({
        "schema_version": 1,
        "state": ")") + state +
      R"(",
        "ready": true,
        "degraded": false,
        "reason_code": "localization_operational",
        "stamp_s": 104.0
      })");
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(
      result.detail,
      std::string("unsupported localization state: ") + state);
  }
}
