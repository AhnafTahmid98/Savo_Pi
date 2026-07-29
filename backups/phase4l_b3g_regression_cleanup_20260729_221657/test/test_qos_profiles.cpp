#include "savo_mapping/qos_profiles.hpp"

#include <gtest/gtest.h>

TEST(QosProfilesContract, StatusQosIsReliableAndVolatile)
{
  const auto profile =
    savo_mapping::qos::status_qos().get_rmw_qos_profile();

  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, savo_mapping::qos::STATUS_DEPTH);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(QosProfilesContract, StateQosRetainsLatestState)
{
  const auto profile =
    savo_mapping::qos::state_qos().get_rmw_qos_profile();

  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, savo_mapping::qos::STATE_DEPTH);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(
    profile.durability,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

TEST(QosProfilesContract, CommandQosDoesNotReplayOldCommands)
{
  const auto profile =
    savo_mapping::qos::command_qos().get_rmw_qos_profile();

  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, savo_mapping::qos::COMMAND_DEPTH);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(QosProfilesContract, ScanQosUsesSensorDataPolicy)
{
  const auto profile =
    savo_mapping::qos::scan_qos().get_rmw_qos_profile();

  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 5u);
  EXPECT_EQ(
    profile.reliability,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(QosProfilesContract, PointcloudQosUsesSensorDataPolicy)
{
  const auto profile =
    savo_mapping::qos::pointcloud_qos().get_rmw_qos_profile();

  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 5u);
  EXPECT_EQ(
    profile.reliability,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(QosProfilesContract, MapQosRetainsLatestMap)
{
  const auto profile =
    savo_mapping::qos::map_qos().get_rmw_qos_profile();

  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, savo_mapping::qos::MAP_DEPTH);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(
    profile.durability,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

TEST(QosProfilesContract, EventQosIsReliableAndVolatile)
{
  const auto profile =
    savo_mapping::qos::event_qos().get_rmw_qos_profile();

  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, savo_mapping::qos::EVENT_DEPTH);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(QosProfilesContract, ContractDepthsArePositive)
{
  EXPECT_GT(savo_mapping::qos::STATUS_DEPTH, 0u);
  EXPECT_GT(savo_mapping::qos::STATE_DEPTH, 0u);
  EXPECT_GT(savo_mapping::qos::COMMAND_DEPTH, 0u);
  EXPECT_GT(savo_mapping::qos::MAP_DEPTH, 0u);
  EXPECT_GT(savo_mapping::qos::EVENT_DEPTH, 0u);
}
