// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "savo_bridge/health_evaluator.hpp"

namespace
{

using Evaluator = savo_bridge::HealthEvaluator;
using Snapshot = Evaluator::TopicSnapshot;
using Duration = savo_bridge::TopicObservation::Duration;

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

[[nodiscard]] Snapshot make_snapshot(
  std::string topic_name,
  const savo_bridge::TopicRequirement requirement,
  const savo_bridge::TopicClassification classification)
{
  Snapshot snapshot;
  snapshot.topic_name = std::move(topic_name);
  snapshot.requirement = requirement;
  snapshot.classification = classification;
  snapshot.stale_after = seconds_duration(1);

  if (classification ==
    savo_bridge::TopicClassification::kNeverObserved)
  {
    return snapshot;
  }

  snapshot.observed = true;
  snapshot.accepted_observations = 1U;

  if (classification ==
    savo_bridge::TopicClassification::kFresh)
  {
    snapshot.age = milliseconds_duration(500);
  }

  if (classification ==
    savo_bridge::TopicClassification::kStale)
  {
    snapshot.age = seconds_duration(1);
  }

  return snapshot;
}

TEST(SavoBridgeHealthEvaluator, DeterministicHealthContract)
{
  const Evaluator evaluator;

  auto evaluation = evaluator.evaluate({});

  EXPECT_EQ(
    evaluation.status,
    savo_bridge::HealthStatus::kUnknown);
  EXPECT_EQ(
    evaluation.reason,
    savo_bridge::HealthReason::kNoTopics);
  EXPECT_FALSE(evaluation.required_topics_ready);
  EXPECT_FALSE(evaluation.all_topics_fresh);
  EXPECT_EQ(evaluation.total_topics, 0U);

  const Snapshot required_fresh = make_snapshot(
    "/required/state",
    savo_bridge::TopicRequirement::kRequired,
    savo_bridge::TopicClassification::kFresh);

  const Snapshot optional_fresh = make_snapshot(
    "/optional/state",
    savo_bridge::TopicRequirement::kOptional,
    savo_bridge::TopicClassification::kFresh);

  evaluation = evaluator.evaluate(
    {optional_fresh, required_fresh});

  EXPECT_EQ(
    evaluation.status,
    savo_bridge::HealthStatus::kHealthy);
  EXPECT_EQ(
    evaluation.reason,
    savo_bridge::HealthReason::kAllTopicsFresh);
  EXPECT_TRUE(evaluation.required_topics_ready);
  EXPECT_TRUE(evaluation.all_topics_fresh);
  EXPECT_EQ(evaluation.total_topics, 2U);
  EXPECT_EQ(evaluation.required_topics, 1U);
  EXPECT_EQ(evaluation.optional_topics, 1U);
  EXPECT_EQ(evaluation.fresh_topics, 2U);

  const Snapshot optional_stale = make_snapshot(
    "/optional/z_stale",
    savo_bridge::TopicRequirement::kOptional,
    savo_bridge::TopicClassification::kStale);

  const Snapshot optional_never = make_snapshot(
    "/optional/a_never",
    savo_bridge::TopicRequirement::kOptional,
    savo_bridge::TopicClassification::kNeverObserved);

  evaluation = evaluator.evaluate(
    {optional_stale, required_fresh, optional_never});

  EXPECT_EQ(
    evaluation.status,
    savo_bridge::HealthStatus::kDegraded);
  EXPECT_EQ(
    evaluation.reason,
    savo_bridge::HealthReason::kOptionalTopicUnavailable);
  EXPECT_TRUE(evaluation.required_topics_ready);
  EXPECT_FALSE(evaluation.all_topics_fresh);
  EXPECT_EQ(evaluation.fresh_topics, 1U);
  EXPECT_EQ(evaluation.stale_topics, 1U);
  EXPECT_EQ(evaluation.never_observed_topics, 1U);

  ASSERT_EQ(
    evaluation.optional_unavailable_topics.size(),
    2U);

  EXPECT_EQ(
    evaluation.optional_unavailable_topics.at(0),
    "/optional/a_never");

  EXPECT_EQ(
    evaluation.optional_unavailable_topics.at(1),
    "/optional/z_stale");

  const Snapshot required_stale = make_snapshot(
    "/required/z_stale",
    savo_bridge::TopicRequirement::kRequired,
    savo_bridge::TopicClassification::kStale);

  const Snapshot required_never = make_snapshot(
    "/required/a_never",
    savo_bridge::TopicRequirement::kRequired,
    savo_bridge::TopicClassification::kNeverObserved);

  evaluation = evaluator.evaluate(
    {
      optional_stale,
      required_stale,
      required_never,
    });

  EXPECT_EQ(
    evaluation.status,
    savo_bridge::HealthStatus::kUnhealthy);
  EXPECT_EQ(
    evaluation.reason,
    savo_bridge::HealthReason::kRequiredTopicUnavailable);
  EXPECT_FALSE(evaluation.required_topics_ready);
  EXPECT_FALSE(evaluation.all_topics_fresh);

  ASSERT_EQ(
    evaluation.required_unavailable_topics.size(),
    2U);

  EXPECT_EQ(
    evaluation.required_unavailable_topics.at(0),
    "/required/a_never");

  EXPECT_EQ(
    evaluation.required_unavailable_topics.at(1),
    "/required/z_stale");

  const Snapshot clock_regression = make_snapshot(
    "/optional/clock",
    savo_bridge::TopicRequirement::kOptional,
    savo_bridge::TopicClassification::kClockRegression);

  evaluation = evaluator.evaluate(
    {required_stale, clock_regression});

  EXPECT_EQ(
    evaluation.status,
    savo_bridge::HealthStatus::kUnhealthy);
  EXPECT_EQ(
    evaluation.reason,
    savo_bridge::HealthReason::kClockRegression);
  EXPECT_FALSE(evaluation.required_topics_ready);
  EXPECT_EQ(evaluation.clock_regression_topics, 1U);

  ASSERT_EQ(
    evaluation.clock_regression_topic_names.size(),
    1U);

  EXPECT_EQ(
    evaluation.clock_regression_topic_names.at(0),
    "/optional/clock");

  EXPECT_THROW(
    evaluator.evaluate(
      {required_fresh, required_fresh}),
    std::invalid_argument);

  Snapshot invalid_name = required_fresh;
  invalid_name.topic_name = "relative/topic";

  EXPECT_THROW(
    evaluator.evaluate({invalid_name}),
    std::invalid_argument);

  Snapshot invalid_timeout = required_fresh;
  invalid_timeout.stale_after = Duration::zero();

  EXPECT_THROW(
    evaluator.evaluate({invalid_timeout}),
    std::invalid_argument);

  Snapshot invalid_fresh = required_fresh;
  invalid_fresh.observed = false;

  EXPECT_THROW(
    evaluator.evaluate({invalid_fresh}),
    std::invalid_argument);

  Snapshot invalid_stale = required_stale;
  invalid_stale.age = milliseconds_duration(999);

  EXPECT_THROW(
    evaluator.evaluate({invalid_stale}),
    std::invalid_argument);

  Snapshot invalid_never = required_never;
  invalid_never.accepted_observations = 1U;

  EXPECT_THROW(
    evaluator.evaluate({invalid_never}),
    std::invalid_argument);

  Snapshot invalid_clock = clock_regression;
  invalid_clock.age = milliseconds_duration(1);

  EXPECT_THROW(
    evaluator.evaluate({invalid_clock}),
    std::invalid_argument);

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::HealthStatus::kUnknown),
    std::string_view("unknown"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::HealthStatus::kHealthy),
    std::string_view("healthy"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::HealthStatus::kDegraded),
    std::string_view("degraded"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::HealthStatus::kUnhealthy),
    std::string_view("unhealthy"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::HealthReason::kNoTopics),
    std::string_view("no_topics"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::HealthReason::kAllTopicsFresh),
    std::string_view("all_topics_fresh"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::HealthReason::kOptionalTopicUnavailable),
    std::string_view("optional_topic_unavailable"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::HealthReason::kRequiredTopicUnavailable),
    std::string_view("required_topic_unavailable"));

  EXPECT_EQ(
    savo_bridge::to_string(
      savo_bridge::HealthReason::kClockRegression),
    std::string_view("clock_regression"));
}

}  // namespace
