// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "savo_bridge/observation_config.hpp"

namespace
{

using savo_bridge::ObservationParameterSet;
using savo_bridge::TopicObservation;
using savo_bridge::TopicRequirement;
using savo_bridge::make_observation_configs;

TEST(
  ObservationConfig,
  EmptyParametersProduceEmptyConfiguration)
{
  const ObservationParameterSet parameters;

  const auto result = make_observation_configs(
    parameters,
    {});

  EXPECT_TRUE(result.empty());
}

TEST(
  ObservationConfig,
  BuildsRequiredAndOptionalConfigurations)
{
  ObservationParameterSet parameters;

  parameters.topic_names = {
    "/savo_base/heartbeat",
    "/savo_power/status",
  };

  parameters.requirements = {
    "required",
    "optional",
  };

  parameters.stale_after_ms = {
    1500,
    5000,
  };

  const auto result = make_observation_configs(
    parameters,
    {
      "/savo_bridge/state",
      "/savo_bridge/readiness",
    });

  ASSERT_EQ(result.size(), 2U);

  EXPECT_EQ(
    result.at(0).topic_name,
    "/savo_base/heartbeat");

  EXPECT_EQ(
    result.at(0).requirement,
    TopicRequirement::kRequired);

  EXPECT_EQ(
    std::chrono::duration_cast<
      std::chrono::milliseconds>(
      result.at(0).stale_after).count(),
    1500);

  EXPECT_EQ(
    result.at(1).topic_name,
    "/savo_power/status");

  EXPECT_EQ(
    result.at(1).requirement,
    TopicRequirement::kOptional);

  EXPECT_EQ(
    std::chrono::duration_cast<
      std::chrono::milliseconds>(
      result.at(1).stale_after).count(),
    5000);
}

TEST(
  ObservationConfig,
  RejectsMismatchedRequirementArray)
{
  ObservationParameterSet parameters;

  parameters.topic_names = {
    "/status/one",
  };

  parameters.stale_after_ms = {
    1000,
  };

  EXPECT_THROW(
    make_observation_configs(parameters, {}),
    std::invalid_argument);
}

TEST(
  ObservationConfig,
  RejectsMismatchedThresholdArray)
{
  ObservationParameterSet parameters;

  parameters.topic_names = {
    "/status/one",
  };

  parameters.requirements = {
    "required",
  };

  EXPECT_THROW(
    make_observation_configs(parameters, {}),
    std::invalid_argument);
}

TEST(
  ObservationConfig,
  RejectsUnknownRequirement)
{
  ObservationParameterSet parameters;

  parameters.topic_names = {
    "/status/one",
  };

  parameters.requirements = {
    "critical",
  };

  parameters.stale_after_ms = {
    1000,
  };

  EXPECT_THROW(
    make_observation_configs(parameters, {}),
    std::invalid_argument);
}

TEST(
  ObservationConfig,
  RejectsNonPositiveThreshold)
{
  ObservationParameterSet parameters;

  parameters.topic_names = {
    "/status/one",
  };

  parameters.requirements = {
    "required",
  };

  parameters.stale_after_ms = {
    0,
  };

  EXPECT_THROW(
    make_observation_configs(parameters, {}),
    std::invalid_argument);

  parameters.stale_after_ms = {
    -1,
  };

  EXPECT_THROW(
    make_observation_configs(parameters, {}),
    std::invalid_argument);
}

TEST(
  ObservationConfig,
  RejectsThresholdOutsideClockRange)
{
  ObservationParameterSet parameters;

  parameters.topic_names = {
    "/status/one",
  };

  parameters.requirements = {
    "required",
  };

  parameters.stale_after_ms = {
    std::numeric_limits<std::int64_t>::max(),
  };

  EXPECT_THROW(
    make_observation_configs(parameters, {}),
    std::invalid_argument);
}

TEST(
  ObservationConfig,
  RejectsDuplicateObservationTopics)
{
  ObservationParameterSet parameters;

  parameters.topic_names = {
    "/status/one",
    "/status/one",
  };

  parameters.requirements = {
    "required",
    "optional",
  };

  parameters.stale_after_ms = {
    1000,
    2000,
  };

  EXPECT_THROW(
    make_observation_configs(parameters, {}),
    std::invalid_argument);
}

TEST(
  ObservationConfig,
  RejectsBridgeOwnedObservationTopic)
{
  ObservationParameterSet parameters;

  parameters.topic_names = {
    "/savo_bridge/state",
  };

  parameters.requirements = {
    "required",
  };

  parameters.stale_after_ms = {
    1000,
  };

  EXPECT_THROW(
    make_observation_configs(
      parameters,
    {
      "/savo_bridge/state",
      "/savo_bridge/readiness",
      }),
    std::invalid_argument);
}

TEST(
  ObservationConfig,
  RejectsRelativeObservationTopic)
{
  ObservationParameterSet parameters;

  parameters.topic_names = {
    "status/one",
  };

  parameters.requirements = {
    "required",
  };

  parameters.stale_after_ms = {
    1000,
  };

  EXPECT_THROW(
    make_observation_configs(parameters, {}),
    std::invalid_argument);
}

TEST(
  ObservationConfig,
  RejectsDuplicateForbiddenTopics)
{
  const ObservationParameterSet parameters;

  EXPECT_THROW(
    make_observation_configs(
      parameters,
    {
      "/savo_bridge/state",
      "/savo_bridge/state",
      }),
    std::invalid_argument);
}

TEST(
  ObservationConfig,
  RejectsInvalidForbiddenTopic)
{
  const ObservationParameterSet parameters;

  EXPECT_THROW(
    make_observation_configs(
      parameters,
    {
      "savo_bridge/state",
      }),
    std::invalid_argument);
}

}  // namespace
