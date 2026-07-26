// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <cstddef>

#include "rmw/types.h"
#include "savo_bridge/qos_profiles.hpp"

namespace
{

TEST(SavoBridgeQos, OwnedTopicProfiles)
{
  const auto latched =
    savo_bridge::bridge_latched_qos().get_rmw_qos_profile();

  EXPECT_EQ(latched.depth, static_cast<std::size_t>(1));
  EXPECT_EQ(
    latched.reliability,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(
    latched.durability,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  const auto stream =
    savo_bridge::bridge_stream_qos().get_rmw_qos_profile();

  EXPECT_EQ(stream.depth, static_cast<std::size_t>(10));
  EXPECT_EQ(
    stream.reliability,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(
    stream.durability,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

}  // namespace
