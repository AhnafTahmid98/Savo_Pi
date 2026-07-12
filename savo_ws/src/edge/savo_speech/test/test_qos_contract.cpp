#include <gtest/gtest.h>

#include <rmw/types.h>

#include "savo_speech/qos_profiles.hpp"

TEST(SpeechQos, StateIsReliableAndLatched)
{
  const auto profile =
    savo_speech::qos::state().get_rmw_qos_profile();

  EXPECT_EQ(profile.depth, 1u);

  EXPECT_EQ(
    profile.reliability,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  EXPECT_EQ(
    profile.durability,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

TEST(SpeechQos, EventsAreReliableAndVolatile)
{
  const auto profile =
    savo_speech::qos::event().get_rmw_qos_profile();

  EXPECT_EQ(profile.depth, 10u);

  EXPECT_EQ(
    profile.reliability,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  EXPECT_EQ(
    profile.durability,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(SpeechQos, HeartbeatIsBestEffort)
{
  const auto profile =
    savo_speech::qos::heartbeat().get_rmw_qos_profile();

  EXPECT_EQ(profile.depth, 5u);

  EXPECT_EQ(
    profile.reliability,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  EXPECT_EQ(
    profile.durability,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}
