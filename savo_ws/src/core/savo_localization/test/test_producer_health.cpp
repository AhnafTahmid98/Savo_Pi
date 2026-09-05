// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "savo_localization/producer_health.hpp"

namespace savo_localization
{
namespace
{

constexpr std::int64_t kMillisecondNs{1000000};

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
      const auto rate = producer.Observe(sample_time_ns, 25.0);
      consumer.Record(
        sample_time_ns,
        SerializeProducerHealth(healthy_imu_snapshot(rate)),
        30U);
    }
  }

  const auto producer_rate = producer.Observe(2000 * kMillisecondNs, 25.0);
  const auto consumed = consumer.Observe(2000 * kMillisecondNs, 0.5);
  EXPECT_TRUE(producer_rate.available);
  EXPECT_NEAR(producer_rate.rate_hz, 25.0, 1.0e-9);
  EXPECT_TRUE(consumed.fresh);
  EXPECT_NEAR(consumed.snapshot.producer_rate_hz, 25.0, 1.0e-9);
  EXPECT_NEAR(consumed.receive_rate_hz, 5.0, 1.0e-9);
  EXPECT_TRUE(consumer.ObserveRateValid(
      2000 * kMillisecondNs, 25.0, 0.50, 1000 * kMillisecondNs));
}

TEST(ProducerHealthTest, GenuineTenHzProducerFailsExistingImuMinimum)
{
  ProducerRateTracker producer;
  for (std::int64_t sample = 0; sample < 10; ++sample) {
    const auto time_ns = sample * 100 * kMillisecondNs;
    producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
  }

  const auto rate = producer.Observe(900 * kMillisecondNs, 25.0);
  ProducerHealthConsumer consumer;
  consumer.Record(
    900 * kMillisecondNs,
    SerializeProducerHealth(healthy_imu_snapshot(rate)),
    30U);

  EXPECT_NEAR(rate.rate_hz, 10.0, 1.0e-9);
  EXPECT_EQ(rate.quality, RateQuality::kBelowMinimum);
  EXPECT_FALSE(consumer.ObserveRateValid(
      900 * kMillisecondNs, 25.0, 0.50, 1000 * kMillisecondNs));
}

TEST(ProducerHealthTest, StartupRateIsInvalidUntilThreeSamplesEstablishEvidence)
{
  ProducerRateTracker producer;
  ProducerHealthConsumer consumer;

  producer.RecordSuccess(0, 1000000000LL, 30U);
  auto rate = producer.Observe(0, 25.0);
  EXPECT_FALSE(rate.available);
  EXPECT_EQ(rate.quality, RateQuality::kBelowMinimum);
  consumer.Record(0, SerializeProducerHealth(healthy_imu_snapshot(rate)), 30U);
  EXPECT_FALSE(consumer.ObserveRateValid(0, 25.0, 0.50, 1000000000LL));

  producer.RecordSuccess(40000000LL, 1040000000LL, 30U);
  rate = producer.Observe(40000000LL, 25.0);
  EXPECT_FALSE(rate.available);
  consumer.Record(
    40000000LL, SerializeProducerHealth(healthy_imu_snapshot(rate)), 30U);
  EXPECT_FALSE(consumer.ObserveRateValid(
      40000000LL, 25.0, 0.50, 1000000000LL));

  producer.RecordSuccess(80000000LL, 1080000000LL, 30U);
  rate = producer.Observe(80000000LL, 25.0);
  EXPECT_TRUE(rate.available);
  EXPECT_EQ(rate.quality, RateQuality::kExcellent);
  consumer.Record(
    80000000LL, SerializeProducerHealth(healthy_imu_snapshot(rate)), 30U);
  EXPECT_TRUE(consumer.ObserveRateValid(
      80000000LL, 25.0, 0.50, 1000000000LL));
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

  EXPECT_TRUE(debounce.Observe(1000000004LL, true, true, 1000000000LL));
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

TEST(ProducerHealthTest, RateQualityBoundariesMatchContract)
{
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(12.499, 25.0),
    RateQuality::kBelowMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(12.5, 25.0),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(18.75, 25.0),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(22.5, 25.0),
    RateQuality::kExcellent);
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
