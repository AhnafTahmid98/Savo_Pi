#include "savo_mapping/topic_names.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(TopicNamesContract, CoreInputTopicsAreStable)
{
  EXPECT_EQ(std::string{savo_mapping::topics::SCAN}, "/scan");
  EXPECT_EQ(std::string{savo_mapping::topics::MAP}, "/map");
  EXPECT_EQ(std::string{savo_mapping::topics::MAP_METADATA}, "/map_metadata");
  EXPECT_EQ(std::string{savo_mapping::topics::TF}, "/tf");
  EXPECT_EQ(std::string{savo_mapping::topics::TF_STATIC}, "/tf_static");
  EXPECT_EQ(std::string{savo_mapping::topics::ODOM}, "/odom");
  EXPECT_EQ(std::string{savo_mapping::topics::ODOM_FILTERED}, "/odometry/filtered");
  EXPECT_EQ(std::string{savo_mapping::topics::WHEEL_ODOM}, "/wheel/odom");
}

TEST(TopicNamesContract, EdgeAndSemanticInputTopicsAreStable)
{
  EXPECT_EQ(std::string{savo_mapping::topics::REALSENSE_STATUS}, "/savo_realsense/status");
  EXPECT_EQ(std::string{savo_mapping::topics::DEPTH_MIN_FRONT}, "/depth/min_front_m");
  EXPECT_EQ(std::string{savo_mapping::topics::DEPTH_POINTS}, "/depth/color/points");
  EXPECT_EQ(
    std::string{savo_mapping::topics::HEAD_SEMANTIC_CONFIRMATIONS},
    "/savo_head/semantic_confirmations");
}

TEST(TopicNamesContract, MappingOutputTopicsAreStable)
{
  EXPECT_EQ(std::string{savo_mapping::topics::STATUS}, "/savo_mapping/status");
  EXPECT_EQ(std::string{savo_mapping::topics::READINESS}, "/savo_mapping/readiness");
  EXPECT_EQ(std::string{savo_mapping::topics::MODE}, "/savo_mapping/mode");
  EXPECT_EQ(std::string{savo_mapping::topics::WORKFLOW_PHASE}, "/savo_mapping/workflow_phase");
  EXPECT_EQ(std::string{savo_mapping::topics::SESSION_STATE}, "/savo_mapping/session_state");
  EXPECT_EQ(std::string{savo_mapping::topics::MAP_QUALITY}, "/savo_mapping/map_quality");
  EXPECT_EQ(std::string{savo_mapping::topics::MAP_SAVED}, "/savo_mapping/map_saved");
  EXPECT_EQ(std::string{savo_mapping::topics::EXPLORATION_STATUS}, "/savo_mapping/exploration_status");
  EXPECT_EQ(std::string{savo_mapping::topics::SEMANTIC_EVENTS}, "/savo_mapping/semantic_events");
  EXPECT_EQ(std::string{savo_mapping::topics::DASHBOARD}, "/savo_mapping/dashboard");
}

TEST(TopicNamesContract, MappingCommandTopicsAreStable)
{
  EXPECT_EQ(std::string{savo_mapping::topics::MODE_CMD}, "/savo_mapping/mode_cmd");
  EXPECT_EQ(std::string{savo_mapping::topics::START_SESSION_CMD}, "/savo_mapping/start_session_cmd");
  EXPECT_EQ(std::string{savo_mapping::topics::STOP_SESSION_CMD}, "/savo_mapping/stop_session_cmd");
  EXPECT_EQ(std::string{savo_mapping::topics::SAVE_MAP_CMD}, "/savo_mapping/save_map_cmd");
  EXPECT_EQ(std::string{savo_mapping::topics::CANCEL_SESSION_CMD}, "/savo_mapping/cancel_session_cmd");
  EXPECT_EQ(std::string{savo_mapping::topics::SCAN360_CMD}, "/savo_mapping/scan360_cmd");
}

TEST(TopicNamesContract, ExplorationHandoffTopicsAreStable)
{
  EXPECT_EQ(
    std::string{savo_mapping::topics::EXPLORATION_SELECTED_GOAL},
    "/savo_mapping/exploration/selected_goal");

  EXPECT_EQ(
    std::string{savo_mapping::topics::EXPLORATION_GOAL_STATE},
    "/savo_mapping/exploration_goal/state");

  EXPECT_EQ(
    std::string{savo_mapping::topics::EXPLORATION_GOAL_STATUS},
    "/savo_mapping/exploration_goal/status");

  EXPECT_EQ(
    std::string{savo_mapping::topics::EXPLORATION_GOAL_FEEDBACK},
    "/savo_mapping/exploration_goal/feedback");
}

TEST(TopicNamesContract, NavigationAwarenessAndSafetyTopicsAreStable)
{
  EXPECT_EQ(
    std::string{savo_mapping::topics::NAV_STATUS},
    "/savo_nav/status");

  EXPECT_EQ(std::string{savo_mapping::topics::SAFETY_STOP}, "/safety/stop");
  EXPECT_EQ(std::string{savo_mapping::topics::SAFETY_SLOWDOWN_FACTOR}, "/safety/slowdown_factor");
  EXPECT_EQ(std::string{savo_mapping::topics::CONTROL_MODE_STATE}, "/savo_control/mode_state");
}

TEST(TopicNamesContract, JoinTopicHandlesSlashes)
{
  EXPECT_EQ(savo_mapping::topics::join_topic("/savo_mapping", "status"), "/savo_mapping/status");
  EXPECT_EQ(savo_mapping::topics::join_topic("/savo_mapping/", "status"), "/savo_mapping/status");
  EXPECT_EQ(savo_mapping::topics::join_topic("/savo_mapping", "/status"), "/savo_mapping/status");
  EXPECT_EQ(savo_mapping::topics::join_topic("/savo_mapping/", "/status"), "/savo_mapping/status");

  EXPECT_EQ(savo_mapping::topics::join_topic("", "/scan"), "/scan");
  EXPECT_EQ(savo_mapping::topics::join_topic("/scan", ""), "/scan");
}

TEST(TopicNamesContract, ClassifiesMappingStatusTopics)
{
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::STATUS));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::READINESS));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::MODE));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::WORKFLOW_PHASE));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::SESSION_STATE));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::MAP_QUALITY));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::MAP_SAVED));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::EXPLORATION_STATUS));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::SEMANTIC_EVENTS));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::DASHBOARD));

  EXPECT_FALSE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::SCAN));
  EXPECT_FALSE(savo_mapping::topics::is_mapping_status_topic(savo_mapping::topics::MODE_CMD));
}

TEST(TopicNamesContract, ClassifiesMappingCommandTopics)
{
  EXPECT_TRUE(savo_mapping::topics::is_mapping_command_topic(savo_mapping::topics::MODE_CMD));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_command_topic(savo_mapping::topics::START_SESSION_CMD));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_command_topic(savo_mapping::topics::STOP_SESSION_CMD));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_command_topic(savo_mapping::topics::SAVE_MAP_CMD));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_command_topic(savo_mapping::topics::CANCEL_SESSION_CMD));
  EXPECT_TRUE(savo_mapping::topics::is_mapping_command_topic(savo_mapping::topics::SCAN360_CMD));

  EXPECT_FALSE(savo_mapping::topics::is_mapping_command_topic(savo_mapping::topics::STATUS));
  EXPECT_FALSE(savo_mapping::topics::is_mapping_command_topic(savo_mapping::topics::SCAN));
}

TEST(TopicNamesContract, ClassifiesExternalInputTopics)
{
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::SCAN));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::MAP));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::MAP_METADATA));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::TF));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::TF_STATIC));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::ODOM));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::ODOM_FILTERED));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::WHEEL_ODOM));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::REALSENSE_STATUS));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::DEPTH_MIN_FRONT));
  EXPECT_TRUE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::DEPTH_POINTS));
  EXPECT_TRUE(
    savo_mapping::topics::is_external_input_topic(
      savo_mapping::topics::HEAD_SEMANTIC_CONFIRMATIONS));

  EXPECT_FALSE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::STATUS));
  EXPECT_FALSE(savo_mapping::topics::is_external_input_topic(savo_mapping::topics::SAVE_MAP_CMD));
}

TEST(TopicNamesContract, ClassifiesNavigationAndSafetyTopics)
{
  EXPECT_TRUE(
    savo_mapping::topics::is_navigation_handoff_topic(
      savo_mapping::topics::EXPLORATION_SELECTED_GOAL));

  EXPECT_TRUE(
    savo_mapping::topics::is_navigation_handoff_topic(
      savo_mapping::topics::EXPLORATION_GOAL_STATE));

  EXPECT_TRUE(
    savo_mapping::topics::is_navigation_handoff_topic(
      savo_mapping::topics::EXPLORATION_GOAL_STATUS));

  EXPECT_TRUE(
    savo_mapping::topics::is_navigation_handoff_topic(
      savo_mapping::topics::EXPLORATION_GOAL_FEEDBACK));

  EXPECT_TRUE(
    savo_mapping::topics::is_navigation_handoff_topic(
      savo_mapping::topics::NAV_STATUS));

  EXPECT_TRUE(savo_mapping::topics::is_safety_awareness_topic(savo_mapping::topics::SAFETY_STOP));
  EXPECT_TRUE(
    savo_mapping::topics::is_safety_awareness_topic(
      savo_mapping::topics::SAFETY_SLOWDOWN_FACTOR));
  EXPECT_TRUE(savo_mapping::topics::is_safety_awareness_topic(savo_mapping::topics::CONTROL_MODE_STATE));

  EXPECT_FALSE(savo_mapping::topics::is_navigation_handoff_topic(savo_mapping::topics::SCAN));
  EXPECT_FALSE(savo_mapping::topics::is_safety_awareness_topic(savo_mapping::topics::MAP));
}
