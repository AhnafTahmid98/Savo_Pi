// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <cstdint>
#include <iterator>
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
constexpr RateThresholds kEkfThresholds{10.0, 15.0, 20.0};
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
  snapshot.raw_window_rate_hz = rate.raw_window_rate_hz;
  snapshot.last_success_age_s = rate.last_success_age_s;
  snapshot.max_inter_publication_gap_s = rate.max_inter_publication_gap_s;
  snapshot.last_success_monotonic_ns = rate.last_success_monotonic_ns;
  snapshot.rate_window_sample_count = rate.window_sample_count;
  snapshot.isolated_gap_excluded = rate.isolated_gap_excluded;
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

TEST(ProducerHealthTest, HealthyConfiguredProducerRatesRemainValid)
{
  const auto observe_rate = [](
      const std::int64_t period_ns,
      const RateThresholds & thresholds) {
      ProducerRateTracker producer;
      for (std::int64_t sample = 0; sample < 30; ++sample) {
        const auto time_ns = sample * period_ns;
        producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
      }
      return producer.Observe(29 * period_ns, thresholds);
    };

  const auto imu = observe_rate(40 * kMillisecondNs, kImuThresholds);
  const auto wheel = observe_rate(33333333LL, kWheelThresholds);
  const auto ekf = observe_rate(50 * kMillisecondNs, kEkfThresholds);

  EXPECT_NEAR(imu.rate_hz, 25.0, 1.0e-9);
  EXPECT_NEAR(wheel.rate_hz, 30.0, 1.0e-6);
  EXPECT_NEAR(ekf.rate_hz, 20.0, 1.0e-9);
  EXPECT_GE(imu.rate_hz, kImuThresholds.minimum_hz);
  EXPECT_GE(wheel.rate_hz, kWheelThresholds.minimum_hz);
  EXPECT_GE(ekf.rate_hz, kEkfThresholds.minimum_hz);
}

TEST(ProducerHealthTest, HealthObservationJitterDoesNotReplaceProducerRate)
{
  ProducerRateTracker producer;
  ProducerHealthConsumer consumer;
  constexpr std::int64_t kHealthReceiveTimesMs[] = {0, 230, 410, 670, 850, 1090};

  std::size_t next_health = 0U;
  for (std::int64_t sample = 0; sample <= 30; ++sample) {
    const auto sample_time_ns = sample * 40 * kMillisecondNs;
    producer.RecordSuccess(sample_time_ns, 1000000000LL + sample_time_ns, 30U);
    if (sample % 5 == 0 && next_health < std::size(kHealthReceiveTimesMs)) {
      const auto receive_time_ns =
        kHealthReceiveTimesMs[next_health] * kMillisecondNs;
      consumer.Record(
        receive_time_ns,
        SerializeProducerHealth(healthy_imu_snapshot(
            producer.Observe(sample_time_ns, kImuThresholds))),
        30U);
      ++next_health;
    }
  }

  EXPECT_EQ(next_health, std::size(kHealthReceiveTimesMs));

  const auto consumed = consumer.Observe(1200 * kMillisecondNs, 0.5);
  EXPECT_TRUE(consumed.fresh);
  EXPECT_NEAR(consumed.snapshot.producer_rate_hz, 25.0, 1.0e-9);
  EXPECT_LT(consumed.receive_rate_hz, 5.0);
  EXPECT_NEAR(consumed.max_receive_gap_s, 0.26, 1.0e-9);
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

TEST(ProducerHealthTest, IsolatedStaleGapDoesNotPoisonRecoveredSustainedRate)
{
  ProducerRateTracker producer;
  std::int64_t time_ns = 0;

  for (std::int64_t sample = 0; sample < 30; ++sample) {
    producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
    time_ns += 40 * kMillisecondNs;
  }

  time_ns += 1960 * kMillisecondNs;
  producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
  for (std::int64_t sample = 0; sample < 24; ++sample) {
    time_ns += 40 * kMillisecondNs;
    producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
  }

  const auto rate = producer.Observe(time_ns, kImuThresholds);
  EXPECT_TRUE(rate.available);
  EXPECT_NEAR(rate.rate_hz, 25.0, 1.0e-9);
  EXPECT_LT(rate.raw_window_rate_hz, kImuThresholds.minimum_hz);
  EXPECT_NEAR(rate.max_inter_publication_gap_s, 2.0, 1.0e-9);
  EXPECT_TRUE(rate.isolated_gap_excluded);
  EXPECT_EQ(rate.window_sample_count, 30U);
}

TEST(ProducerHealthTest, SustainedLowRateIsNotTreatedAsAnIsolatedGap)
{
  ProducerRateTracker producer;
  constexpr std::int64_t kLowRatePeriodNs = 111111111LL;

  for (std::int64_t sample = 0; sample < 30; ++sample) {
    const auto time_ns = sample * kLowRatePeriodNs;
    producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
  }

  const auto rate = producer.Observe(29 * kLowRatePeriodNs, kImuThresholds);
  EXPECT_TRUE(rate.available);
  EXPECT_NEAR(rate.rate_hz, 9.0, 1.0e-7);
  EXPECT_NEAR(rate.raw_window_rate_hz, 9.0, 1.0e-7);
  EXPECT_FALSE(rate.isolated_gap_excluded);
}

TEST(ProducerHealthTest, RepeatedLongGapsStillFailTheMinimumRate)
{
  ProducerRateTracker producer;
  std::int64_t time_ns = 0;
  for (std::int64_t sample = 0; sample < 30; ++sample) {
    producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
    time_ns += (sample == 8 || sample == 19) ?
      2 * 1000 * kMillisecondNs : 40 * kMillisecondNs;
  }

  const auto rate = producer.Observe(time_ns, kImuThresholds);
  EXPECT_LT(rate.rate_hz, kImuThresholds.minimum_hz);
  EXPECT_FALSE(rate.isolated_gap_excluded);
}

TEST(ProducerHealthTest, TimingDiagnosticsRoundTripInCompactHealthPayload)
{
  ProducerRateTracker producer;
  for (std::int64_t sample = 0; sample < 30; ++sample) {
    const auto time_ns = sample * 40 * kMillisecondNs;
    producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
  }
  auto source = healthy_imu_snapshot(
    producer.Observe(1160 * kMillisecondNs, kImuThresholds));
  source.health_publish_monotonic_ns = 1160 * kMillisecondNs;
  source.health_publish_gap_s = 0.2;
  source.max_health_publish_gap_s = 0.24;

  ProducerHealthSnapshot parsed;
  std::string error;
  ASSERT_TRUE(ParseProducerHealth(SerializeProducerHealth(source), parsed, error));
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(parsed.last_success_monotonic_ns, 1160 * kMillisecondNs);
  EXPECT_EQ(parsed.rate_window_sample_count, 30U);
  EXPECT_NEAR(parsed.raw_window_rate_hz, 25.0, 1.0e-9);
  EXPECT_NEAR(parsed.max_inter_publication_gap_s, 0.04, 1.0e-9);
  EXPECT_NEAR(parsed.health_publish_gap_s, 0.2, 1.0e-9);
  EXPECT_NEAR(parsed.max_health_publish_gap_s, 0.24, 1.0e-9);
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

TEST(ProducerHealthTest, FreshPublicationRecoversAfterStoppedProducer)
{
  ProducerRateTracker producer;
  ProducerHealthConsumer consumer;
  for (std::int64_t sample = 0; sample < 30; ++sample) {
    const auto time_ns = sample * 40 * kMillisecondNs;
    producer.RecordSuccess(time_ns, 1000000000LL + time_ns, 30U);
  }

  auto rate = producer.Observe(1160 * kMillisecondNs, kImuThresholds);
  consumer.Record(
    1160 * kMillisecondNs,
    SerializeProducerHealth(healthy_imu_snapshot(rate)),
    30U);
  EXPECT_FALSE(consumer.Observe(1700 * kMillisecondNs, 0.5).fresh);

  producer.RecordSuccess(
    1740 * kMillisecondNs, 2740 * kMillisecondNs, 30U);
  rate = producer.Observe(1740 * kMillisecondNs, kImuThresholds);
  consumer.Record(
    1740 * kMillisecondNs,
    SerializeProducerHealth(healthy_imu_snapshot(rate)),
    30U);
  EXPECT_TRUE(consumer.Observe(1740 * kMillisecondNs, 0.5).fresh);
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

  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(9.99, kWheelThresholds),
    RateQuality::kBelowMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(10.0, kWheelThresholds),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(19.99, kWheelThresholds),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(20.0, kWheelThresholds),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(24.99, kWheelThresholds),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(25.0, kWheelThresholds),
    RateQuality::kExcellent);

  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(9.9, kEkfThresholds),
    RateQuality::kBelowMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(10.0, kEkfThresholds),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(14.99, kEkfThresholds),
    RateQuality::kMinimum);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(15.0, kEkfThresholds),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(19.99, kEkfThresholds),
    RateQuality::kGood);
  EXPECT_EQ(
    ProducerRateTracker::ClassifyQuality(20.0, kEkfThresholds),
    RateQuality::kExcellent);

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
