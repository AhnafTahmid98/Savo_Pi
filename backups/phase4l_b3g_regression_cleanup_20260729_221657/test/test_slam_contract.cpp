#include "savo_mapping/slam_contract.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(SlamContract, UsesAsyncMappingExecutable)
{
  EXPECT_EQ(
    savo_mapping::slam::NODE_NAME,
    "slam_toolbox");

  EXPECT_EQ(
    savo_mapping::slam::ASYNC_EXECUTABLE,
    "async_slam_toolbox_node");

  EXPECT_EQ(
    savo_mapping::slam::MODE_MAPPING,
    "mapping");
}

TEST(SlamContract, UsesRobotSavoFrames)
{
  EXPECT_EQ(savo_mapping::slam::MAP_FRAME, "map");
  EXPECT_EQ(savo_mapping::slam::ODOM_FRAME, "odom");
  EXPECT_EQ(savo_mapping::slam::BASE_FRAME, "base_link");
  EXPECT_EQ(savo_mapping::slam::LIDAR_FRAME, "laser_frame");
}

TEST(SlamContract, UsesOfficialMappingInterfaces)
{
  EXPECT_EQ(savo_mapping::slam::SCAN_TOPIC, "/scan");
  EXPECT_EQ(savo_mapping::slam::MAP_TOPIC, "/map");

  EXPECT_EQ(
    savo_mapping::slam::DYNAMIC_MAP_SERVICE,
    "/slam_toolbox/dynamic_map");

  EXPECT_EQ(
    savo_mapping::slam::PAUSE_MEASUREMENTS_SERVICE,
    "/slam_toolbox/pause_new_measurements");

  EXPECT_EQ(
    savo_mapping::slam::SAVE_MAP_SERVICE,
    "/slam_toolbox/save_map");

  EXPECT_EQ(
    savo_mapping::slam::SERIALIZE_MAP_SERVICE,
    "/slam_toolbox/serialize_map");

  EXPECT_EQ(
    savo_mapping::slam::RESET_SERVICE,
    "/slam_toolbox/reset");
}

TEST(SlamContract, DefaultOwnershipIsSafe)
{
  const savo_mapping::slam::SlamOwnershipContract contract;

  EXPECT_TRUE(
    savo_mapping::slam::validate_slam_ownership(
      contract).empty());
}

TEST(SlamContract, SlamToolboxMustOwnMapToOdom)
{
  savo_mapping::slam::SlamOwnershipContract contract;
  contract.slam_toolbox_owns_map_to_odom = false;

  EXPECT_EQ(
    savo_mapping::slam::validate_slam_ownership(contract),
    "slam_toolbox_must_own_map_to_odom");
}

TEST(SlamContract, LocalizationMustOwnOdomToBase)
{
  savo_mapping::slam::SlamOwnershipContract contract;
  contract.localization_owns_odom_to_base = false;

  EXPECT_EQ(
    savo_mapping::slam::validate_slam_ownership(contract),
    "localization_must_own_odom_to_base");
}

TEST(SlamContract, SupervisorMustRemainMonitorOnly)
{
  savo_mapping::slam::SlamOwnershipContract contract;
  contract.supervisor_is_monitor_only = false;

  EXPECT_EQ(
    savo_mapping::slam::validate_slam_ownership(contract),
    "mapping_supervisor_must_remain_monitor_only");
}

TEST(SlamContract, SessionManagerOwnsSaving)
{
  savo_mapping::slam::SlamOwnershipContract contract;
  contract.session_manager_owns_save_requests = false;

  EXPECT_EQ(
    savo_mapping::slam::validate_slam_ownership(contract),
    "map_session_manager_must_own_save_requests");
}

TEST(SlamContract, RejectsVelocityOwnership)
{
  savo_mapping::slam::SlamOwnershipContract contract;
  contract.mapping_publishes_velocity_commands = true;

  EXPECT_EQ(
    savo_mapping::slam::validate_slam_ownership(contract),
    "savo_mapping_must_not_publish_velocity_commands");
}

TEST(SlamContract, RejectsOdomTfOwnership)
{
  savo_mapping::slam::SlamOwnershipContract contract;
  contract.mapping_publishes_odom_to_base = true;

  EXPECT_EQ(
    savo_mapping::slam::validate_slam_ownership(contract),
    "savo_mapping_must_not_publish_odom_to_base");
}

TEST(SlamContract, NavigationHandoffRemainsDisabled)
{
  savo_mapping::slam::SlamOwnershipContract contract;
  contract.navigation_handoff_enabled = true;

  EXPECT_EQ(
    savo_mapping::slam::validate_slam_ownership(contract),
    "navigation_handoff_must_remain_disabled");
}
