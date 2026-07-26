// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "savo_bridge/topic_observation.hpp"

namespace
{

using Observation = savo_bridge::TopicObservation;
using Config = savo_bridge::TopicObservationConfig;
using Duration = Observation::Duration;
using TimePoint = Observation::TimePoint;

[[nodiscard]] Duration seconds_duration(const int value)
{
  return std::chrono::duration_cast<Duration>(
    std::chrono::seconds(value));
}

[[nodiscard]] Duration milliseconds_duration(const int value)
{
  return std::chrono::duration_cast<Duration>(
    std::chrono::milliseconds(value));
}

[[nodiscard]] Config make_config(
  std::string topic_name,
  const Duration stale_after,
  const savo_bridge::TopicRequirement requirement =
  savo_bridge::TopicRequirement::kRequired)
{
  Config config;
  config.topic_name = std::move(topic_name);
  config.requirement = requirement;
  config.stale_after = stale_after;
  return config;
}

TEST(SavoBridgeTopicObservation, DeterministicClassificationContract)
{
  const Duration timeout = seconds_duration(1);

  EXPECT_THROW(
    (void)Observation(make_config("", timeout)),
    std::invalid_argument);

  EXPECT_THROW(
    (void)Observation(make_config("relative/topic", timeout)),
    std::invalid_argument);

  EXPECT_THROW(
    (void)Observation(make_config("/", timeout)),
    std::invalid_argument);

  EXPECT_THROW(
    (void)Observation(make_config("/trailing/", timeout)),
    std::invalid_argument);

  EXPECT_THROW(
    (void)Observation(make_config("/double//slash", timeout)),
    std::invalid_argument);

  EXPECT_THROW(
    (void)Observation(make_config("/contains space", timeout)),
    std::invalid_argument);

  EXPECT_THROW(
    (void)Observation(make_config("/contains-hyphen", timeout)),
    std::invalid_argument);

  EXPECT_THROW(
    (void)Observation(
      make_config("/valid_topic", Duration::zero())),
    std::invalid_argument);

  Observation observation(
    make_config(
      "/savo_control/state",
      timeout,
      savo_bridge::TopicRequirement::kRequired));

  EXPECT_EQ(
    observation.config().topic_name,
    "/savo_control/state");

  EXPECT_EQ(
    observation.config().requirement,
    savo_bridge::TopicRequirement::kRequired);

  EXPECT_EQ(observation.config().stale_after, timeout);

  const TimePoint origin{seconds_duration(10)};

  auto snapshot = observation.snapshot(origin);

  EXPECT_EQ(snapshot.topic_name, "/savo_control/state");
  EXPECT_EQ(
    snapshot.requirement,
    savo_bridge::TopicRequirement::kRequired);
  EXPECT_EQ(
    snapshot.classification,
    savo_bridge::TopicClassification::kNeverObserved);
  EXPECT_FALSE(snapshot.observed);
  EXPECT_EQ(snapshot.accepted_observations, 0U);
  EXPECT_EQ(snapshot.rejected_regressions, 0U);
  EXPECT_EQ(snapshot.age, Duration::zero());
  EXPECT_EQ(snapshot.stale_after, timeout);

  EXPECT_TRUE(observation.observe(origin));

  snapshot = observation.snapshot(
    origin + milliseconds_duration(999));

  EXPECT_EQ(
    snapshot.classification,
    savo_bridge::TopicClassification::kFresh);
  EXPECT_TRUE(snapshot.observed);
  EXPECT_EQ(snapshot.accepted_observations, 1U);
  EXPECT_EQ(snapshot.rejected_regressions, 0U);
  EXPECT_EQ(snapshot.age, milliseconds_duration(999));

  snapshot = observation.snapshot(origin + timeout);

  EXPECT_EQ(
    snapshot.classification,
    savo_bridge::TopicClassification::kStale);
  EXPECT_EQ(snapshot.age, timeout);

  EXPECT_TRUE(
    observation.observe(origin + seconds_duration(2)));

  EXPECT_FALSE(
    observation.observe(
      origin + milliseconds_duration(1500)));

  snapshot = observation.snapshot(
    origin + milliseconds_duration(2100));

  EXPECT_EQ(
    snapshot.classification,
    savo_bridge::TopicClassification::kFresh);
  EXPECT_EQ(snapshot.accepted_observations, 2U);
  EXPECT_EQ(snapshot.rejected_regressions, 1U);
  EXPECT_EQ(snapshot.age, milliseconds_duration(100));

  snapshot = observation.snapshot(
    origin + milliseconds_duration(1900));

  EXPECT_EQ(
    snapshot.classification,
    savo_bridge::TopicClassification::kClockRegression);
  EXPECT_EQ(snapshot.age, Duration::zero());

  EXPECT_EQ(
    savo_bridge::classify_topic(
      savo_bridge::FreshnessState::kNeverObserved),
    savo_bridge::TopicClassification::kNeverObserved);

  EXPECT_EQ(
    savo_bridge::classify_topic(
      savo_bridge::FreshnessState::kFresh),
    savo_bridge::TopicClassification::kFresh);

  EXPECT_EQ(
    savo_bridge::classify_topic(
      savo_bridge::FreshnessState::kStale),
    savo_bridge::TopicClassification::kStale);

  EXPECT_EQ(
    savo_bridge::classify_topic(
      savo_bridge::FreshnessState::kClockRegression),
    savo_bridge::TopicClassification::kClockRegression);

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::TopicRequirement::kRequired),
    std::string_view("required"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::TopicRequirement::kOptional),
    std::string_view("optional"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::TopicClassification::kNeverObserved),
    std::string_view("never_observed"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::TopicClassification::kFresh),
    std::string_view("fresh"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::TopicClassification::kStale),
    std::string_view("stale"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::TopicClassification::kClockRegression),
    std::string_view("clock_regression"));

  observation.reset();
  snapshot = observation.snapshot(origin + seconds_duration(3));

  EXPECT_EQ(
    snapshot.classification,
    savo_bridge::TopicClassification::kNeverObserved);
  EXPECT_FALSE(snapshot.observed);
  EXPECT_EQ(snapshot.accepted_observations, 0U);
  EXPECT_EQ(snapshot.rejected_regressions, 0U);

  Observation optional_observation(
    make_config(
      "/savo_mapping/dashboard",
      seconds_duration(5),
      savo_bridge::TopicRequirement::kOptional));

  const auto optional_snapshot =
    optional_observation.snapshot(origin);

  EXPECT_EQ(
    optional_snapshot.requirement,
    savo_bridge::TopicRequirement::kOptional);
  EXPECT_EQ(
    optional_snapshot.classification,
    savo_bridge::TopicClassification::kNeverObserved);
}

}  // namespace
