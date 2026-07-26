// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "savo_bridge/bridge_node.hpp"

namespace
{

class RclcppGuard final
{
public:
  RclcppGuard()
  {
    int argument_count = 0;
    char ** arguments = nullptr;

    rclcpp::init(
      argument_count,
      arguments);
  }

  ~RclcppGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  RclcppGuard(const RclcppGuard &) = delete;
  RclcppGuard & operator=(const RclcppGuard &) = delete;
};

class TemporaryDirectory final
{
public:
  TemporaryDirectory()
  {
    const auto identifier =
      std::chrono::steady_clock::now()
      .time_since_epoch()
      .count();

    path_ =
      std::filesystem::temp_directory_path() /
      (
      "savo_bridge_phase2b2_" +
      std::to_string(identifier));

    if (!std::filesystem::create_directory(path_)) {
      throw std::runtime_error(
              "Failed to create temporary test directory");
    }
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory & operator=(
    const TemporaryDirectory &) = delete;

  [[nodiscard]] const std::filesystem::path &
  path() const noexcept
  {
    return path_;
  }

private:
  std::filesystem::path path_;
};

[[nodiscard]] std::string read_file(
  const std::filesystem::path & path)
{
  std::ifstream input(path);

  if (!input.is_open()) {
    return {};
  }

  std::ostringstream content;
  content << input.rdbuf();
  return content.str();
}

TEST(
  GenericTopicObservation,
  PublishesFreshThenStaleRequiredTopic)
{
  using namespace std::chrono_literals;

  const RclcppGuard guard;
  const TemporaryDirectory temporary_directory;

  const std::filesystem::path snapshot_path =
    temporary_directory.path() / "snapshot.json";

  rclcpp::NodeOptions options;

  options.parameter_overrides({
      rclcpp::Parameter(
        "observation_topics",
        std::vector<std::string>{
        "/phase2b2/status"}),
      rclcpp::Parameter(
        "observation_requirements",
        std::vector<std::string>{
        "required"}),
      rclcpp::Parameter(
        "observation_stale_after_ms",
        std::vector<std::int64_t>{
        250}),
      rclcpp::Parameter(
        "snapshot_enabled",
        true),
      rclcpp::Parameter(
        "snapshot_path",
        snapshot_path.string()),
    });

  auto bridge =
    std::make_shared<savo_bridge::BridgeNode>(
    options);

  auto fixture =
    std::make_shared<rclcpp::Node>(
    "phase2b2_fixture");

  auto publisher =
    fixture->create_publisher<std_msgs::msg::String>(
    "/phase2b2/status",
    rclcpp::QoS(10));

  auto timer = fixture->create_wall_timer(
    50ms,
    [publisher]() {
      std_msgs::msg::String message;
      message.data = "alive";
      publisher->publish(message);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(fixture);

  bool fresh_observed = false;
  std::string payload;

  const auto fresh_deadline =
    std::chrono::steady_clock::now() + 6s;

  while (
    std::chrono::steady_clock::now() <
    fresh_deadline)
  {
    executor.spin_some();

    payload = read_file(snapshot_path);

    if (
      payload.find(
        "\"topic_name\":\"/phase2b2/status\"") !=
      std::string::npos &&
      payload.find(
        "\"requirement\":\"required\"") !=
      std::string::npos &&
      payload.find(
        "\"classification\":\"fresh\"") !=
      std::string::npos &&
      payload.find(
        "\"status\":\"healthy\"") !=
      std::string::npos)
    {
      fresh_observed = true;
      break;
    }

    std::this_thread::sleep_for(25ms);
  }

  ASSERT_TRUE(fresh_observed);

  timer->cancel();

  bool stale_observed = false;

  const auto stale_deadline =
    std::chrono::steady_clock::now() + 5s;

  while (
    std::chrono::steady_clock::now() <
    stale_deadline)
  {
    executor.spin_some();

    payload = read_file(snapshot_path);

    if (
      payload.find(
        "\"topic_name\":\"/phase2b2/status\"") !=
      std::string::npos &&
      payload.find(
        "\"classification\":\"stale\"") !=
      std::string::npos &&
      payload.find(
        "\"status\":\"unhealthy\"") !=
      std::string::npos &&
      payload.find(
        "\"reason\":\"required_topic_unavailable\"") !=
      std::string::npos)
    {
      stale_observed = true;
      break;
    }

    std::this_thread::sleep_for(25ms);
  }

  EXPECT_TRUE(stale_observed);

  executor.remove_node(fixture);
  executor.remove_node(bridge);

  timer.reset();
  publisher.reset();
  fixture.reset();
  bridge.reset();
}

}  // namespace
