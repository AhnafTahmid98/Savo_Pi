// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "savo_localization/producer_health.hpp"

namespace savo_localization
{
namespace
{

constexpr std::int64_t kMillisecondNs{1000000};
constexpr RateThresholds kImuThresholds{10.0, 15.0, 20.0};
constexpr RateThresholds kWheelThresholds{10.0, 20.0, 25.0};
constexpr RateThresholds kEkfThresholds{10.0, 20.0, 25.0};
constexpr RateThresholds kVoThresholds{5.0, 8.0, 12.0};

ProducerHealthSnapshot healthy_imu_snapshot(
  const ProducerRateObservation & rate)
{
  ProducerHealthSnapshot snapshot;
  snapshot.node = "imu_node";
  snapshot.health_state = "OK";
  snapshot.reason = "IMU healthy";
  snapshot.frame_id = "imu_link";
  snapshot.data_valid = true;
  snapshot.frame_valid = true;
  snapshot.timestamp_valid = true;
  snapshot.hardware_ok = true;
  snapshot.motion_ready = true;
  snapshot.producer_rate_available = rate.available;
  snapshot.producer_rate_hz = rate.rate_hz;
  snapshot.last_success_age_s = rate.last_success_age_s;
  snapshot.rate_quality = std::string(
    ProducerRateTracker::QualityString(rate.quality));
  return snapshot;
}

TEST(ProducerHealthTest, SourceRateStaysTwentyFiveHzWithFiveHzHealthDelivery)
{
  ProducerRateTracker producer;
  ProducerHealthConsumer consumer;

  for (std::int64_t sample = 0; sample <= 50; ++sample) {
    const auto sample_time_ns = sample * 40 * kMillisecondNs;
    EXPECT_FALSE(producer.RecordSuccess(
        sample_time_ns,
        1000000000LL + sample_time_ns,
        30U));
    if (sample % 5 == 0) {
      const auto rate = producer.Observe(sample_time_ns, kImuThresholds);
      consumer.Record(
        sample_time_ns,
        SerializeProducerHealth(healthy_imu_snapshot(rate)),
        30U);
    }
  }

  const auto producer_rate = producer.Observe(2000 * kMillisecondNs, kImuThresholds);
  const auto consumed = consumer.Observe(2000 * kMillisecondNs, 0.5);
  EXPECT_TRUE(producer_rate.available);
  EXPECT_NEAR(producer_rate.rate_hz, 25.0, 1.0e-9);
  EXPECT_TRUE(consumed.fresh);
  EXPECT_NEAR(consumed.snapshot.producer_rate_hz, 25.0, 1.0e-9);
  EXPECT_NEAR(consumed.receive_rate_hz, 5.0, 1.0e-9);
  EXPECT_TRUE(consumer.ObserveRateValid(
      2000 * kMillisecondNs, kImuThresholds, 1000 * kMillisecondNs));
}

TEST(ProducerHealthTest, GenuineRateBelowTenHzFailsImuMinimum)
{
  ProducerRateTracker producer;
  for (std::int64_t sample = 0; sample < 10; ++sample) {
    const auto time_ns = sample * 100100100LL;
    producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
  }

  const auto rate = producer.Observe(900900900LL, kImuThresholds);
  ProducerHealthConsumer consumer;
  consumer.Record(
    900900900LL,
    SerializeProducerHealth(healthy_imu_snapshot(rate)),
    30U);

  EXPECT_NEAR(rate.rate_hz, 9.99, 1.0e-6);
  EXPECT_EQ(rate.quality, RateQuality::kBelowMinimum);
  EXPECT_FALSE(consumer.ObserveRateValid(
      900900900LL, kImuThresholds, 1000 * kMillisecondNs));
}

TEST(ProducerHealthTest, StartupRateIsInvalidUntilThreeSamplesEstablishEvidence)
{
  ProducerRateTracker producer;
  ProducerHealthConsumer consumer;

  producer.RecordSuccess(0, 1000000000LL, 30U);
  auto rate = producer.Observe(0, kImuThresholds);
  EXPECT_FALSE(rate.available);
  EXPECT_EQ(rate.quality, RateQuality::kBelowMinimum);
  consumer.Record(0, SerializeProducerHealth(healthy_imu_snapshot(rate)), 30U);
  EXPECT_FALSE(consumer.ObserveRateValid(0, kImuThresholds, 1000000000LL));

  producer.RecordSuccess(40000000LL, 1040000000LL, 30U);
  rate = producer.Observe(40000000LL, kImuThresholds);
  EXPECT_FALSE(rate.available);
  consumer.Record(
    40000000LL, SerializeProducerHealth(healthy_imu_snapshot(rate)), 30U);
  EXPECT_FALSE(consumer.ObserveRateValid(
      40000000LL, kImuThresholds, 1000000000LL));

  producer.RecordSuccess(80000000LL, 1080000000LL, 30U);
  rate = producer.Observe(80000000LL, kImuThresholds);
  EXPECT_TRUE(rate.available);
  EXPECT_EQ(rate.quality, RateQuality::kExcellent);
  consumer.Record(
    80000000LL, SerializeProducerHealth(healthy_imu_snapshot(rate)), 30U);
  EXPECT_TRUE(consumer.ObserveRateValid(
      80000000LL, kImuThresholds, 1000000000LL));
}

TEST(ProducerHealthTest, OnlyEstablishedLiveLowRateReceivesFailureDebounce)
{
  RateValidityDebouncer debounce;
  EXPECT_FALSE(debounce.Observe(0, false, true, 1000000000LL));
  EXPECT_FALSE(debounce.Observe(1, true, false, 1000000000LL));

  EXPECT_TRUE(debounce.Observe(2, true, true, 1000000000LL));
  EXPECT_TRUE(debounce.Observe(3, true, false, 1000000000LL));
  EXPECT_TRUE(debounce.Observe(999999999LL, true, false, 1000000000LL));
  EXPECT_FALSE(debounce.Observe(1000000003LL, true, false, 1000000000LL));

  EXPECT_FALSE(debounce.Observe(1000000004LL, true, true, 1000000000LL));
  EXPECT_FALSE(debounce.Observe(2000000003LL, true, true, 1000000000LL));
  EXPECT_TRUE(debounce.Observe(2000000004LL, true, true, 1000000000LL));
}

TEST(ProducerHealthTest, ShortRecoverySpikeDoesNotClearEstablishedLowRate)
{
  RateValidityDebouncer debounce;
  constexpr std::int64_t kDebounceNs = 3000000000LL;

  EXPECT_TRUE(debounce.Observe(0, true, true, kDebounceNs));
  EXPECT_TRUE(debounce.Observe(1, true, false, kDebounceNs));
  EXPECT_FALSE(debounce.Observe(kDebounceNs + 1, true, false, kDebounceNs));

  EXPECT_FALSE(debounce.Observe(kDebounceNs + 2, true, true, kDebounceNs));
  EXPECT_FALSE(debounce.Observe(kDebounceNs + 3, true, false, kDebounceNs));
  EXPECT_FALSE(debounce.Observe(kDebounceNs + 4, true, true, kDebounceNs));
  EXPECT_TRUE(debounce.Observe(2 * kDebounceNs + 4, true, true, kDebounceNs));
}

TEST(ProducerHealthTest, StaleHealthCannotBeOverriddenByExcellentOldRate)
{
  ProducerHealthSnapshot snapshot;
  snapshot.node = "imu_node";
  snapshot.health_state = "OK";
  snapshot.reason = "IMU healthy";
  snapshot.frame_id = "imu_link";
  snapshot.data_valid = true;
  snapshot.frame_valid = true;
  snapshot.timestamp_valid = true;
  snapshot.hardware_ok = true;
  snapshot.motion_ready = true;
  snapshot.producer_rate_available = true;
  snapshot.producer_rate_hz = 25.0;
  snapshot.last_success_age_s = 0.01;
  snapshot.rate_quality = "EXCELLENT";

  ProducerHealthConsumer consumer;
  consumer.Record(1000 * kMillisecondNs, SerializeProducerHealth(snapshot), 30U);
  EXPECT_TRUE(consumer.Observe(1200 * kMillisecondNs, 0.5).fresh);

  const auto stale = consumer.Observe(1600 * kMillisecondNs, 0.5);
  EXPECT_FALSE(stale.fresh);
  EXPECT_NEAR(stale.receive_age_s, 0.6, 1.0e-9);
  EXPECT_NEAR(stale.producer_age_s, 0.61, 1.0e-9);
  EXPECT_DOUBLE_EQ(stale.snapshot.producer_rate_hz, 25.0);
}

TEST(ProducerHealthTest, ProducerSuccessAgeAloneFailsFreshness)
{
  ProducerHealthSnapshot snapshot;
  snapshot.node = "imu_node";
  snapshot.health_state = "OK";
  snapshot.reason = "IMU healthy";
  snapshot.frame_id = "imu_link";
  snapshot.data_valid = true;
  snapshot.frame_valid = true;
  snapshot.timestamp_valid = true;
  snapshot.hardware_ok = true;
  snapshot.motion_ready = true;
  snapshot.producer_rate_available = true;
  snapshot.producer_rate_hz = 25.0;
  snapshot.last_success_age_s = 0.49;
  snapshot.rate_quality = "EXCELLENT";

  ProducerHealthConsumer consumer;
  consumer.Record(1000 * kMillisecondNs, SerializeProducerHealth(snapshot), 30U);
  const auto stale = consumer.Observe(1020 * kMillisecondNs, 0.5);

  EXPECT_NEAR(stale.receive_age_s, 0.02, 1.0e-9);
  EXPECT_NEAR(stale.producer_age_s, 0.51, 1.0e-9);
  EXPECT_FALSE(stale.fresh);
}

TEST(ProducerHealthTest, TimestampRegressionRemainsHardEvidence)
{
  ProducerRateTracker producer;
  EXPECT_FALSE(producer.RecordSuccess(100, 1000, 30U));
  EXPECT_FALSE(producer.RecordSuccess(200, 1100, 30U));
  EXPECT_TRUE(producer.RecordSuccess(300, 1050, 30U));
}

TEST(ProducerHealthTest, ExplicitRateQualityBoundariesMatchEachStreamContract)
{
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(9.99, kImuThresholds),
    RateQuality::kBelowMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(10.0, kImuThresholds),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(14.99, kImuThresholds),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(15.0, kImuThresholds),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(19.99, kImuThresholds),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(20.0, kImuThresholds),
    RateQuality::kExcellent);

  for (const auto & thresholds : {kWheelThresholds, kEkfThresholds}) {
    EXPECT_EQ(
      ProducerRateTracker::ClassifyQuality(9.99, thresholds),
      RateQuality::kBelowMinimum);
    EXPECT_EQ(
      ProducerRateTracker::ClassifyQuality(10.0, thresholds),
      RateQuality::kMinimum);
    EXPECT_EQ(
      ProducerRateTracker::ClassifyQuality(19.99, thresholds),
      RateQuality::kMinimum);
    EXPECT_EQ(
      ProducerRateTracker::ClassifyQuality(20.0, thresholds),
      RateQuality::kGood);
    EXPECT_EQ(
      ProducerRateTracker::ClassifyQuality(24.99, thresholds),
      RateQuality::kGood);
    EXPECT_EQ(
      ProducerRateTracker::ClassifyQuality(25.0, thresholds),
      RateQuality::kExcellent);
  }

  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(4.99, kVoThresholds),
    RateQuality::kBelowMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(5.0, kVoThresholds),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(7.99, kVoThresholds),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(8.0, kVoThresholds),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(11.99, kVoThresholds),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(12.0, kVoThresholds),
    RateQuality::kExcellent);
}

TEST(ProducerHealthTest, InvalidRatesFailClassificationClosed)
{
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(
      std::numeric_limits<double>::quiet_NaN(), kImuThresholds),
    RateQuality::kBelowMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(
      std::numeric_limits<double>::infinity(), kImuThresholds),
    RateQuality::kBelowMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(
      -std::numeric_limits<double>::infinity(), kImuThresholds),
    RateQuality::kBelowMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(-1.0, kImuThresholds),
    RateQuality::kBelowMinimum);
}

TEST(ProducerHealthTest, MalformedExplicitThresholdsAreRejected)
{
  EXPECT_NO_THROW(ProducerRateTracker::ValidateThresholds({10.0, 10.0, 10.0}));
  EXPECT_THROW(
    ProducerRateTracker::ValidateThresholds({0.0, 15.0, 20.0}),
    std::invalid_argument);
  EXPECT_THROW(
    ProducerRateTracker::ValidateThresholds({-1.0, 15.0, 20.0}),
    std::invalid_argument);
  EXPECT_THROW(
    ProducerRateTracker::ValidateThresholds({16.0, 15.0, 20.0}),
    std::invalid_argument);
  EXPECT_THROW(
    ProducerRateTracker::ValidateThresholds({10.0, 21.0, 20.0}),
    std::invalid_argument);
  EXPECT_THROW(
    ProducerRateTracker::ValidateThresholds(
      {10.0, std::numeric_limits<double>::quiet_NaN(), 20.0}),
    std::invalid_argument);
  EXPECT_THROW(
    ProducerRateTracker::ValidateThresholds(
      {10.0, 15.0, std::numeric_limits<double>::infinity()}),
    std::invalid_argument);
}

TEST(ProducerHealthTest, InvalidPayloadFailsClosed)
{
  ProducerHealthConsumer consumer;
  consumer.Record(1000, "{not-json", 30U);
  const auto observation = consumer.Observe(1000, 0.5);
  EXPECT_TRUE(observation.received);
  EXPECT_FALSE(observation.payload_valid);
  EXPECT_FALSE(observation.fresh);
}

}  // namespace
}  // namespace savo_localization
