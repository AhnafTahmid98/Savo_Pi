// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "savo_localization/localization_health_core.hpp"

namespace
{

using savo_localization::LocalizationHealthCore;
using savo_localization::LocalizationHealthInputs;
using savo_localization::LocalizationHealthState;
using savo_localization::ProducerRateTracker;
using savo_localization::RateAccountingTracker;
using savo_localization::RateQuality;
using savo_localization::SourceHealthObservation;
using savo_localization::TransformHealthObservation;

SourceHealthObservation healthy_source(
  std::string name,
  const bool required = true)
{
  SourceHealthObservation source;
  source.name = std::move(name);
  source.enabled = true;
  source.required = required;
  source.received = true;
  source.fresh = true;
  source.data_valid = true;
  source.frame_valid = true;
  source.timestamp_valid = true;
  source.rate_valid = true;
  source.age_s = 0.01;
  source.rate_hz = 30.0;
  source.source_rate_hz = 30.0;
  source.receive_rate_hz = 30.0;
  source.source_rate_available = true;
  source.rate_basis = "source_header";
  source.rate_quality = "EXCELLENT";
  return source;
}

TransformHealthObservation healthy_transform(std::string name)
{
  TransformHealthObservation transform;
  transform.name = std::move(name);
  transform.required = true;
  transform.available = true;
  transform.fresh = true;
  transform.age_s = 0.01;
  return transform;
}

LocalizationHealthInputs healthy_inputs()
{
  LocalizationHealthInputs inputs;
  inputs.startup_age_s = 5.0;
  inputs.startup_grace_s = 3.0;
  inputs.imu = healthy_source("imu");
  inputs.wheel_odom = healthy_source("wheel_odom");
  inputs.filtered_odom = healthy_source("filtered_odom");
  inputs.vo_odom.enabled = false;
  inputs.odom_to_base = healthy_transform("odom_to_base_footprint_tf");
  inputs.base_to_imu = healthy_transform("base_footprint_to_imu_tf");
  return inputs;
}

TEST(LocalizationHealthCoreTest, ConvertsAllStatesToStableStrings)
{
  const std::vector<std::pair<LocalizationHealthState, std::string>> expected{
    {LocalizationHealthState::kUnknown, "UNKNOWN"},
    {LocalizationHealthState::kInitializing, "INITIALIZING"},
    {LocalizationHealthState::kOk, "OK"},
    {LocalizationHealthState::kDegraded, "DEGRADED"},
    {LocalizationHealthState::kStale, "STALE"},
    {LocalizationHealthState::kError, "ERROR"}
  };

  for (const auto & item : expected) {
    EXPECT_EQ(LocalizationHealthCore::ToString(item.first), item.second);
  }
}

TEST(LocalizationHealthCoreTest, ObservedEkfSeparatesHeaderAndReceiveRates)
{
  RateAccountingTracker tracker;
  constexpr std::int64_t kHeaderPeriodNs = 40000000LL;
  constexpr std::int64_t kReceivePeriodNs = 85000000LL;
  constexpr std::int64_t kBaseNs = 1000000000LL;
  for (std::int64_t index = 0; index < 30; ++index) {
    EXPECT_FALSE(tracker.Record(
        kBaseNs + index * kReceivePeriodNs,
        kBaseNs + index * kHeaderPeriodNs,
        30U));
  }

  const auto rates = tracker.Observe(
    kBaseNs + 29 * kReceivePeriodNs, 25.0, 0.50, 1000000000LL);
  EXPECT_TRUE(rates.source_rate_available);
  EXPECT_TRUE(rates.receive_rate_available);
  EXPECT_NEAR(rates.source_rate_hz, 25.0, 0.01);
  EXPECT_NEAR(rates.receive_rate_hz, 11.7647, 0.01);
  EXPECT_NEAR(rates.validation_rate_hz, 25.0, 0.01);
  EXPECT_TRUE(rates.rate_valid);
  EXPECT_EQ(rates.quality, RateQuality::kExcellent);
}

TEST(LocalizationHealthCoreTest, GenuineLowSourceRateFailsClosed)
{
  RateAccountingTracker tracker;
  constexpr std::int64_t kPeriodNs = 100000000LL;
  constexpr std::int64_t kBaseNs = 1000000000LL;
  for (std::int64_t index = 0; index < 5; ++index) {
    EXPECT_FALSE(tracker.Record(
        kBaseNs + index * kPeriodNs,
        kBaseNs + index * kPeriodNs,
        30U));
  }

  const auto rates = tracker.Observe(
    kBaseNs + 4 * kPeriodNs, 25.0, 0.50, 1000000000LL);
  EXPECT_NEAR(rates.source_rate_hz, 10.0, 0.01);
  EXPECT_FALSE(rates.rate_valid);
  EXPECT_EQ(rates.quality, RateQuality::kBelowMinimum);
}

TEST(LocalizationHealthCoreTest, MissingSourceTimingFallsBackFailClosedToReceiveRate)
{
  RateAccountingTracker tracker;
  constexpr std::int64_t kPeriodNs = 100000000LL;
  constexpr std::int64_t kBaseNs = 1000000000LL;
  for (std::int64_t index = 0; index < 5; ++index) {
    EXPECT_FALSE(tracker.Record(
        kBaseNs + index * kPeriodNs,
        0,
        30U));
  }

  const auto rates = tracker.Observe(
    kBaseNs + 4 * kPeriodNs, 25.0, 0.50, 1000000000LL);
  EXPECT_FALSE(rates.source_rate_available);
  EXPECT_TRUE(rates.receive_rate_available);
  EXPECT_NEAR(rates.validation_rate_hz, 10.0, 0.01);
  EXPECT_FALSE(rates.rate_valid);
}

TEST(LocalizationHealthCoreTest, RateQualityBoundariesAreStableMetadata)
{
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(12.49, 25.0),
    RateQuality::kBelowMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(12.50, 25.0),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(18.75, 25.0),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(22.50, 25.0),
    RateQuality::kExcellent);
  EXPECT_EQ(
    ProducerRateTracker::QualityString(RateQuality::kMinimum),
    "MINIMUM");
  EXPECT_EQ(
    ProducerRateTracker::QualityString(RateQuality::kGood),
    "GOOD");
  EXPECT_EQ(
    ProducerRateTracker::QualityString(RateQuality::kExcellent),
    "EXCELLENT");
}

TEST(LocalizationHealthCoreTest, EstablishedRateUsesTransitionDebounce)
{
  RateAccountingTracker tracker;
  constexpr std::int64_t kBaseNs = 1000000000LL;
  for (std::int64_t index = 0; index < 3; ++index) {
    EXPECT_FALSE(tracker.Record(
        kBaseNs + index * 40000000LL,
        kBaseNs + index * 40000000LL,
        3U));
  }
  EXPECT_TRUE(tracker.Observe(
      kBaseNs + 80000000LL, 25.0, 0.50, 1000000000LL).rate_valid);

  for (std::int64_t index = 1; index <= 3; ++index) {
    EXPECT_FALSE(tracker.Record(
        kBaseNs + 80000000LL + index * 200000000LL,
        kBaseNs + 80000000LL + index * 200000000LL,
        3U));
  }
  const auto pending = tracker.Observe(
    kBaseNs + 680000000LL, 25.0, 0.50, 1000000000LL);
  EXPECT_LT(pending.source_rate_hz, 12.5);
  EXPECT_TRUE(pending.rate_valid);
  EXPECT_FALSE(tracker.Observe(
      kBaseNs + 1680000000LL, 25.0, 0.50, 1000000000LL).rate_valid);
}

TEST(LocalizationHealthCoreTest, HeaderTimestampRegressionIsReported)
{
  RateAccountingTracker tracker;
  EXPECT_FALSE(tracker.Record(1000000000LL, 1000000000LL, 30U));
  EXPECT_FALSE(tracker.Record(1100000000LL, 1100000000LL, 30U));
  EXPECT_TRUE(tracker.Record(1200000000LL, 1050000000LL, 30U));
}

TEST(LocalizationHealthCoreTest, ReportsOkWhenAllRequiredEvidenceIsHealthy)
{
  const LocalizationHealthCore core;
  const auto result = core.Evaluate(healthy_inputs());

  EXPECT_EQ(result.state, LocalizationHealthState::kOk);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.degraded);
  EXPECT_EQ(result.reason_code, "localization_operational");
  EXPECT_TRUE(result.reasons.empty());
}

TEST(LocalizationHealthCoreTest, MissingRequiredInputIsInitializingDuringGrace)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.startup_age_s = 1.0;
  inputs.filtered_odom.received = false;
  inputs.filtered_odom.fresh = false;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kInitializing);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.reason_code, "waiting_for_localization_inputs");
  ASSERT_EQ(result.reasons.size(), 1U);
  EXPECT_NE(result.reasons.front().find("filtered_odom_missing"), std::string::npos);
}

TEST(LocalizationHealthCoreTest, MissingRequiredInputBecomesStaleAfterGrace)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.filtered_odom.received = false;
  inputs.filtered_odom.fresh = false;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kStale);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.reason_code, "required_localization_input_unavailable");
}

TEST(LocalizationHealthCoreTest, StaleRequiredInputBlocksReadiness)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.wheel_odom.fresh = false;
  inputs.wheel_odom.age_s = 1.0;
  inputs.wheel_odom.source_rate_hz = 30.0;
  inputs.wheel_odom.rate_quality = "EXCELLENT";

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kStale);
  EXPECT_FALSE(result.ready);
  ASSERT_EQ(result.reasons.size(), 1U);
  EXPECT_NE(result.reasons.front().find("wheel_odom_stale"), std::string::npos);
}

TEST(LocalizationHealthCoreTest, OptionalVoLossIsDegradedButReady)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.vo_odom.enabled = true;
  inputs.vo_odom.required = false;
  inputs.vo_odom.name = "vo_odom";
  inputs.vo_odom.received = false;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kDegraded);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
  EXPECT_EQ(result.reason_code, "localization_operational_degraded");
}

TEST(LocalizationHealthCoreTest, OptionalStaleVoIsDegradedButReady)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.vo_odom = healthy_source("vo_odom", false);
  inputs.vo_odom.fresh = false;
  inputs.vo_odom.age_s = 1.0;
  inputs.vo_odom.source_rate_hz = 15.0;
  inputs.vo_odom.rate_quality = "EXCELLENT";

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kDegraded);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
  ASSERT_EQ(result.reasons.size(), 1U);
  EXPECT_NE(result.reasons.front().find("vo_odom_stale_optional"), std::string::npos);
}

TEST(LocalizationHealthCoreTest, RequiredVoLossBlocksReadiness)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.vo_odom.enabled = true;
  inputs.vo_odom.required = true;
  inputs.vo_odom.name = "vo_odom";
  inputs.vo_odom.received = false;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kStale);
  EXPECT_FALSE(result.ready);
}

TEST(LocalizationHealthCoreTest, RequiredLowRateBlocksReadiness)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.imu.rate_valid = false;
  inputs.imu.rate_hz = 6.0;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kStale);
  EXPECT_FALSE(result.ready);
  ASSERT_EQ(result.reasons.size(), 1U);
  EXPECT_NE(
    result.reasons.front().find("imu_rate_below_minimum"),
    std::string::npos);
}

TEST(LocalizationHealthCoreTest, OptionalLowRateIsDegradedButReady)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.vo_odom = healthy_source("vo_odom", false);
  inputs.vo_odom.rate_valid = false;
  inputs.vo_odom.rate_hz = 2.0;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kDegraded);
  EXPECT_TRUE(result.ready);
  ASSERT_EQ(result.reasons.size(), 1U);
  EXPECT_NE(result.reasons.front().find("vo_odom_rate_low"), std::string::npos);
}

TEST(LocalizationHealthCoreTest, ImuCalibrationWarningIsDegradedButReady)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.imu.diagnostic_warning = true;
  inputs.imu.detail = "calibration_not_motion_ready";

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kDegraded);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
}

TEST(LocalizationHealthCoreTest, InvalidDataHasPriorityOverStaleness)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.filtered_odom.data_valid = false;
  inputs.filtered_odom.fresh = false;
  inputs.filtered_odom.detail = "nan_pose";

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kError);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.reason_code, "invalid_localization_data");
  EXPECT_NE(result.reasons.front().find("filtered_odom_invalid_data"), std::string::npos);
}

TEST(LocalizationHealthCoreTest, WrongFrameIsError)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.wheel_odom.frame_valid = false;
  inputs.wheel_odom.detail = "expected=odom->base_footprint,actual=odom->base_link";

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kError);
  EXPECT_FALSE(result.ready);
  EXPECT_NE(result.reasons.front().find("wheel_odom_invalid_frame"), std::string::npos);
}

TEST(LocalizationHealthCoreTest, TimestampRegressionIsError)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.imu.timestamp_valid = false;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kError);
  EXPECT_FALSE(result.ready);
  EXPECT_NE(result.reasons.front().find("imu_timestamp_regression"), std::string::npos);
}

TEST(LocalizationHealthCoreTest, DiagnosticErrorIsError)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.imu.diagnostic_error = true;
  inputs.imu.detail = "bno055_system_error";

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kError);
  EXPECT_FALSE(result.ready);
}

TEST(LocalizationHealthCoreTest, MissingOdomTransformBlocksReadiness)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.odom_to_base.available = false;
  inputs.odom_to_base.fresh = false;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kStale);
  EXPECT_FALSE(result.ready);
  EXPECT_NE(
    result.reasons.front().find("odom_to_base_footprint_tf_unavailable"),
    std::string::npos);
}

TEST(LocalizationHealthCoreTest, StaleDynamicTransformBlocksReadiness)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.odom_to_base.fresh = false;
  inputs.odom_to_base.age_s = 2.0;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kStale);
  EXPECT_FALSE(result.ready);
}

TEST(LocalizationHealthCoreTest, MissingStaticImuTransformBlocksReadiness)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.base_to_imu.available = false;
  inputs.base_to_imu.fresh = false;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kStale);
  EXPECT_FALSE(result.ready);
}

TEST(LocalizationHealthCoreTest, PoseJumpIsError)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.filtered_pose_jump_detected = true;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kError);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.reasons.front(), "filtered_odom_pose_jump_detected");
}

TEST(LocalizationHealthCoreTest, YawJumpIsError)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.filtered_yaw_jump_detected = true;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kError);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.reasons.front(), "filtered_odom_yaw_jump_detected");
}

TEST(LocalizationHealthCoreTest, DisabledSourcesDoNotAffectReadiness)
{
  const LocalizationHealthCore core;
  auto inputs = healthy_inputs();
  inputs.imu.enabled = false;
  inputs.imu.required = false;
  inputs.imu.received = false;
  inputs.base_to_imu.required = false;

  const auto result = core.Evaluate(inputs);

  EXPECT_EQ(result.state, LocalizationHealthState::kOk);
  EXPECT_TRUE(result.ready);
}

}  // namespace
