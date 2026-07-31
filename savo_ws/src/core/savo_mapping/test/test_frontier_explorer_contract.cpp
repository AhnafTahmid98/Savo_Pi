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
  FrontierExplorerContract,
  SourceUsesOnlyDedicatedGoalHandoff)
{
  const auto source =
    read_text(
    package_root /
    "src/nodes/frontier_explorer_node.cpp");

  expect_contains(
    source,
    "exploration::kSelectedGoalTopic");

  expect_contains(
    source,
    "exploration::kGoalStateTopic");

  expect_contains(
    source,
    "tf_buffer_.lookupTransform");

  expect_contains(
    source,
    "geometry_msgs::msg::PoseStamped");

  expect_contains(
    source,
    "savo_msgs::msg::FrontierExplorationStatus");

  expect_contains(source, "plan_sequence_");
  expect_contains(source, "last_planning_status_");

  expect_not_contains(source, "/cmd_vel");
  expect_not_contains(source, "rclcpp_action");
  expect_not_contains(source, "NavigateToPose");
  expect_not_contains(source, "navigate_to_pose");
  expect_not_contains(
    source,
    "kExplorationActionName");
}

TEST(
  FrontierExplorerContract,
  ConfigurationDefaultsToDisabled)
{
  const auto config =
    read_text(
    package_root /
    "config/frontier_mapping.yaml");

  expect_contains(config, "enabled: false");

  expect_contains(
    config,
    "require_handoff_state: true");

  expect_contains(
    config,
    "map_frame: \"map\"");

  expect_contains(
    config,
    "base_frame: \"base_link\"");

  expect_contains(
    config,
    "\"/savo_mapping/exploration/selected_goal\"");

  expect_contains(
    config,
    "\"/savo_mapping/frontier_explorer/typed_status\"");

  expect_contains(
    config,
    "exhaustion_recheck_period_sec: 2.0");

  expect_not_contains(config, "/cmd_vel");
  expect_not_contains(config, "/goal_pose");
}

TEST(
  FrontierExplorerContract,
  LaunchStartsExplorerThroughHandoffBoundary)
{
  const auto launch =
    read_text(
    package_root /
    "launch/frontier_mapping.launch.xml");

  expect_contains(
    launch,
    "exec=\"frontier_explorer_node\"");

  expect_contains(
    launch,
    "exec=\"exploration_goal_handoff_node\"");

  expect_contains(
    launch,
    "default=\"false\"");

  expect_contains(
    launch,
    "config/frontier_mapping.yaml");

  expect_contains(
    launch,
    "config/exploration_goal_handoff.yaml");

  expect_not_contains(launch, "/cmd_vel");
  expect_not_contains(launch, "navigate_to_pose");
}
