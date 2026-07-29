#include "savo_mapping/topic_names.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#ifndef SAVO_MAPPING_SOURCE_DIR
#error "SAVO_MAPPING_SOURCE_DIR must be defined by CMake"
#endif

namespace
{

namespace fs = std::filesystem;

const fs::path source_root{
  SAVO_MAPPING_SOURCE_DIR};

std::string read_text(
  const fs::path & path)
{
  std::ifstream input(path);

  if (!input.is_open()) {
    throw std::runtime_error(
            "failed to open: " +
            path.string());
  }

  std::ostringstream output;
  output << input.rdbuf();

  return output.str();
}

void expect_contains(
  const std::string & content,
  std::string_view expected)
{
  EXPECT_NE(
    content.find(expected),
    std::string::npos)
    << "missing contract text: "
    << expected;
}

void expect_absent(
  const std::string & content,
  std::string_view forbidden)
{
  EXPECT_EQ(
    content.find(forbidden),
    std::string::npos)
    << "forbidden contract text: "
    << forbidden;
}

}  // namespace

TEST(
  MappingModeManagerContract,
  OwnsAuthorityStateTopics)
{
  const auto source = read_text(
    source_root /
    "src/nodes/"
    "mapping_mode_manager_node.cpp");

  expect_contains(
    source,
    "workflow::evaluate_mode_change");

  expect_contains(
    source,
    "topics::MODE_CMD");

  expect_contains(
    source,
    "topics::START_SESSION_CMD");

  expect_contains(
    source,
    "topics::STOP_SESSION_CMD");

  expect_contains(
    source,
    "topics::CANCEL_SESSION_CMD");

  expect_contains(
    source,
    "topics::READINESS");

  expect_contains(
    source,
    "topics::SAFETY_STOP");

  expect_contains(
    source,
    "topics::EXPLORATION_GOAL_STATE");

  expect_contains(
    source,
    "topics::MODE");

  expect_contains(
    source,
    "topics::EXPLORATION_MODE");

  expect_contains(
    source,
    "topics::WORKFLOW_PHASE");

  expect_contains(
    source,
    "topics::SESSION_STATE");

  expect_contains(
    source,
    "topics::MODE_MANAGER_STATUS");

  expect_contains(
    source,
    "cancel_active_exploration");
}

TEST(
  MappingModeManagerContract,
  HasNoDirectMovementAuthority)
{
  const auto source = read_text(
    source_root /
    "src/nodes/"
    "mapping_mode_manager_node.cpp");

  expect_absent(
    source,
    "/cmd_vel");

  expect_absent(
    source,
    "NavigateToPose");

  expect_absent(
    source,
    "navigate_to_pose");

  expect_absent(
    source,
    "rclcpp_action");

  expect_absent(
    source,
    "/goal_pose");
}

TEST(
  MappingModeManagerContract,
  SupervisorObservesAuthorityState)
{
  const auto source = read_text(
    source_root /
    "src/nodes/"
    "mapping_supervisor_node.cpp");

  expect_contains(
    source,
    "mode_subscription_");

  expect_contains(
    source,
    "exploration_mode_subscription_");

  expect_contains(
    source,
    "workflow_phase_subscription_");

  expect_contains(
    source,
    "session_state_subscription_");

  expect_absent(
    source,
    "mode_publisher_");

  expect_absent(
    source,
    "exploration_mode_publisher_");

  expect_absent(
    source,
    "workflow_phase_publisher_");

  expect_absent(
    source,
    "session_state_publisher_");
}

TEST(
  MappingModeManagerContract,
  ConfigAndTopicRegistryDeclareStableInterfaces)
{
  const auto config = read_text(
    source_root /
    "config/"
    "mapping_mode_manager.yaml");

  const auto topics = read_text(
    source_root /
    "config/topics.yaml");

  expect_contains(
    config,
    "default_autonomous_strategy: "
    "\"frontier\"");

  expect_contains(
    config,
    "initial_mode: \"monitor_only\"");

  expect_contains(
    config,
    "initial_session_state: \"idle\"");

  for (const auto topic : {
    savo_mapping::topics::MODE_CMD,
    savo_mapping::topics::
    START_SESSION_CMD,
    savo_mapping::topics::
    STOP_SESSION_CMD,
    savo_mapping::topics::
    CANCEL_SESSION_CMD,
    savo_mapping::topics::READINESS,
    savo_mapping::topics::SAFETY_STOP,
    savo_mapping::topics::
    EXPLORATION_GOAL_STATE,
    savo_mapping::topics::MODE,
    savo_mapping::topics::
    EXPLORATION_MODE,
    savo_mapping::topics::
    WORKFLOW_PHASE,
    savo_mapping::topics::
    SESSION_STATE,
    savo_mapping::topics::
    MODE_MANAGER_STATUS})
  {
    expect_contains(
      topics,
      std::string{topic});
  }
}

TEST(
  MappingModeManagerContract,
  ProductionLaunchesStartAuthorityBeforeObserver)
{
  for (const auto & filename : {
    "monitor_only.launch.xml",
    "manual_mapping.launch.xml"})
  {
    const auto launch = read_text(
      source_root /
      "launch" /
      filename);

    const auto manager =
      launch.find(
      "exec=\"mapping_mode_manager_node\"");

    const auto supervisor =
      launch.find(
      "exec=\"mapping_supervisor_node\"");

    ASSERT_NE(
      manager,
      std::string::npos)
      << filename;

    ASSERT_NE(
      supervisor,
      std::string::npos)
      << filename;

    EXPECT_LT(
      manager,
      supervisor)
      << filename;

    expect_contains(
      launch,
      "config/"
      "mapping_mode_manager.yaml");
  }
}

TEST(
  MappingModeManagerContract,
  CMakeBuildsInstallsAndTestsManager)
{
  const auto cmake = read_text(
    source_root /
    "CMakeLists.txt");

  expect_contains(
    cmake,
    "add_executable("
    "mapping_mode_manager_node");

  expect_contains(
    cmake,
    "src/nodes/"
    "mapping_mode_manager_node.cpp");

  expect_contains(
    cmake,
    "TARGETS "
    "mapping_mode_manager_node");

  expect_contains(
    cmake,
    "config/"
    "mapping_mode_manager.yaml");

  expect_contains(
    cmake,
    "ament_add_gtest("
    "test_mapping_mode_manager_contract");
}
