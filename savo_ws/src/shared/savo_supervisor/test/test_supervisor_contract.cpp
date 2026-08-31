// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "savo_supervisor/supervisor_policy.hpp"

using savo_supervisor::SupervisorPolicy;

TEST(SupervisorContract, DefaultPolicyIsValid)
{
  SupervisorPolicy policy;

  EXPECT_TRUE(policy.Validate());
  EXPECT_TRUE(policy.ValidationError().empty());
}

TEST(SupervisorContract, RejectsInvalidPublishRate)
{
  SupervisorPolicy policy;
  policy.publish_rate_hz = 0.0;

  EXPECT_FALSE(policy.Validate());

  EXPECT_EQ(
    policy.ValidationError(),
    "publish_rate_hz must be finite and positive");
}

TEST(SupervisorContract, RejectsNonFinitePublishRate)
{
  SupervisorPolicy policy;

  policy.publish_rate_hz =
    std::numeric_limits<double>::infinity();

  EXPECT_FALSE(policy.Validate());
}

TEST(SupervisorContract, RejectsNegativeStartupGrace)
{
  SupervisorPolicy policy;
  policy.startup_grace_s = -1.0;

  EXPECT_FALSE(policy.Validate());

  EXPECT_EQ(
    policy.ValidationError(),
    "startup_grace_s must be finite and non-negative");
}

TEST(SupervisorContract, RejectsDuplicateOutputTopics)
{
  SupervisorPolicy policy;

  policy.events_topic =
    policy.state_summary_topic;

  EXPECT_FALSE(policy.Validate());

  EXPECT_EQ(
    policy.ValidationError(),
    "supervisor output topics must be unique");
}

TEST(SupervisorContract, RejectsMissingLocalizationTopic)
{
  SupervisorPolicy policy;
  policy.localization.summary_topic.clear();

  EXPECT_FALSE(policy.Validate());

  EXPECT_EQ(
    policy.ValidationError(),
    "localization summary_topic must be set");
}

TEST(SupervisorContract, RejectsInvalidTimeout)
{
  SupervisorPolicy policy;
  policy.localization.health_timeout_s = 0.0;

  EXPECT_FALSE(policy.Validate());

  EXPECT_EQ(
    policy.ValidationError(),
    "localization health_timeout_s must be finite and positive");
}

TEST(SupervisorContract, RejectsInvalidConsistencyTransitionGrace)
{
  SupervisorPolicy policy;
  policy.localization.consistency_transition_grace_s = -1.0;

  EXPECT_FALSE(policy.Validate());

  EXPECT_EQ(
    policy.ValidationError(),
    "localization consistency_transition_grace_s must be finite and non-negative");
}

TEST(SupervisorContract, RejectsUnsupportedSchema)
{
  SupervisorPolicy policy;
  policy.localization.expected_schema_version = 2;

  EXPECT_FALSE(policy.Validate());

  EXPECT_EQ(
    policy.ValidationError(),
    "localization expected_schema_version must be 1");
}

TEST(SupervisorContract, RejectsDuplicateLocalizationInputs)
{
  SupervisorPolicy policy;

  policy.localization.summary_topic =
    policy.localization.health_topic;

  EXPECT_FALSE(policy.Validate());

  EXPECT_EQ(
    policy.ValidationError(),
    "localization input topics must be unique");
}

TEST(SupervisorContract, DisabledRequiredComponentIsRejected)
{
  SupervisorPolicy policy;
  policy.localization.enabled = false;

  EXPECT_FALSE(policy.Validate());
  EXPECT_EQ(
    policy.ValidationError(),
    "localization cannot be required while disabled");
}

TEST(SupervisorContract, DisabledOptionalLocalizationNeedsNoInputs)
{
  SupervisorPolicy policy;

  policy.localization.enabled = false;
  policy.localization.required = false;
  policy.localization.health_topic.clear();
  policy.localization.summary_topic.clear();
  policy.localization.heartbeat_topic.clear();
  policy.localization.health_timeout_s = 0.0;
  policy.localization.summary_timeout_s = 0.0;
  policy.localization.heartbeat_timeout_s = 0.0;

  EXPECT_TRUE(policy.Validate());
}
