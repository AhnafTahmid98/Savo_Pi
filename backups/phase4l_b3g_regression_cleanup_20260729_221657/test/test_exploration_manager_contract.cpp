#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#ifndef SAVO_MAPPING_SOURCE_DIR
#error "SAVO_MAPPING_SOURCE_DIR must be defined by CMake"
#endif

namespace
{

namespace fs = std::filesystem;

const fs::path package_root{
  SAVO_MAPPING_SOURCE_DIR};

std::string read_text(
  const fs::path & path)
{
  std::ifstream input(path);

  if (!input.is_open()) {
    throw std::runtime_error(
            "failed to open file: " +
            path.string());
  }

  std::ostringstream output;
  output << input.rdbuf();

  return output.str();
}

void expect_contains(
  const std::string & content,
  const std::string & expected)
{
  EXPECT_NE(
    content.find(expected),
    std::string::npos)
    << "missing expected text: "
    << expected;
}

void expect_not_contains(
  const std::string & content,
  const std::string & forbidden)
{
  EXPECT_EQ(
    content.find(forbidden),
    std::string::npos)
    << "found forbidden text: "
    << forbidden;
}

}  // namespace

TEST(
  ExplorationManagerContract,
  ManagerUsesDeterministicRuntimeAuthority)
{
  const auto source =
    read_text(
    package_root /
    "src/nodes/exploration_manager_node.cpp");

  expect_contains(
    source,
    "exploration_runtime::evaluate");

  expect_contains(
    source,
    "topics::EXPLORATION_RUNTIME_ENABLED");

  expect_contains(
    source,
    "topics::EXPLORATION_MANAGER_STATUS");

  expect_contains(
    source,
    "exploration::kGoalCancelService");

  expect_contains(
    source,
    "create_client<CancelService>");

  expect_contains(
    source,
    "cancel_client_->async_send_request");

  expect_not_contains(source, "/cmd_vel");
  expect_not_contains(source, "/goal_pose");
  expect_not_contains(source, "rclcpp_action");
  expect_not_contains(source, "NavigateToPose");
  expect_not_contains(source, "navigate_to_pose");
  expect_not_contains(
    source,
    "kExplorationActionName");
}

TEST(
  ExplorationManagerContract,
  ManagerObservesAllAuthorityInputs)
{
  const auto source =
    read_text(
    package_root /
    "src/nodes/exploration_manager_node.cpp");

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

  expect_contains(
    source,
    "readiness_subscription_");

  expect_contains(
    source,
    "safety_stop_subscription_");

  expect_contains(
    source,
    "handoff_state_subscription_");
}

TEST(
  ExplorationManagerContract,
  ProductionConfigurationIsConservative)
{
  const auto config =
    read_text(
    package_root /
    "config/exploration_manager.yaml");

  expect_contains(
    config,
    "evaluation_period_ms: 250");

  expect_contains(
    config,
    "cancel_retry_period_ms: 1000");

  expect_contains(
    config,
    "\"/savo_mapping/exploration/"
    "runtime_enabled\"");

  expect_contains(
    config,
    "\"/savo_mapping/exploration_goal/"
    "cancel\"");

  expect_contains(
    config,
    "\"/safety/stop\"");

  expect_not_contains(config, "/cmd_vel");
  expect_not_contains(config, "/goal_pose");
  expect_not_contains(
    config,
    "navigate_to_pose");
}

TEST(
  ExplorationManagerContract,
  FrontierExplorerUsesRuntimeAuthority)
{
  const auto source =
    read_text(
    package_root /
    "src/nodes/frontier_explorer_node.cpp");

  expect_contains(
    source,
    "std_msgs/msg/bool.hpp");

  expect_contains(
    source,
    "runtime_enabled_subscription_");

  expect_contains(
    source,
    "handle_runtime_enabled");

  expect_contains(
    source,
    "effective_enabled()");

  expect_contains(
    source,
    "topics::EXPLORATION_RUNTIME_ENABLED");

  expect_not_contains(source, "/cmd_vel");
  expect_not_contains(source, "rclcpp_action");
  expect_not_contains(source, "NavigateToPose");
}

TEST(
  ExplorationManagerContract,
  TopicRegistryOwnsRuntimeOutputs)
{
  const auto header =
    read_text(
    package_root /
    "include/savo_mapping/topic_names.hpp");

  const auto source =
    read_text(
    package_root /
    "src/ros/topic_names.cpp");

  const auto config =
    read_text(
    package_root /
    "config/topics.yaml");

  expect_contains(
    header,
    "EXPLORATION_RUNTIME_ENABLED");

  expect_contains(
    header,
    "EXPLORATION_MANAGER_STATUS");

  expect_contains(
    source,
    "topic == "
    "EXPLORATION_RUNTIME_ENABLED");

  expect_contains(
    source,
    "topic == "
    "EXPLORATION_MANAGER_STATUS");

  expect_contains(
    config,
    "exploration_runtime_enabled:");

  expect_contains(
    config,
    "exploration_manager_status:");
}

TEST(
  ExplorationManagerContract,
  LaunchStartsManagerBeforeExplorer)
{
  const auto launch =
    read_text(
    package_root /
    "launch/frontier_mapping.launch.xml");

  const auto manager_position =
    launch.find(
    "exec=\"exploration_manager_node\"");

  const auto explorer_position =
    launch.find(
    "exec=\"frontier_explorer_node\"");

  ASSERT_NE(
    manager_position,
    std::string::npos);

  ASSERT_NE(
    explorer_position,
    std::string::npos);

  EXPECT_LT(
    manager_position,
    explorer_position);

  expect_contains(
    launch,
    "config/exploration_manager.yaml");

  expect_not_contains(launch, "/cmd_vel");
  expect_not_contains(
    launch,
    "navigate_to_pose");
}

TEST(
  ExplorationManagerContract,
  CMakeBuildsAndInstallsD3BFiles)
{
  const auto cmake =
    read_text(
    package_root /
    "CMakeLists.txt");

  expect_contains(
    cmake,
    "add_executable("
    "exploration_manager_node");

  expect_contains(
    cmake,
    "src/nodes/"
    "exploration_manager_node.cpp");

  expect_contains(
    cmake,
    "config/exploration_manager.yaml");

  expect_contains(
    cmake,
    "ament_add_gtest("
    "test_exploration_manager_contract");
}
