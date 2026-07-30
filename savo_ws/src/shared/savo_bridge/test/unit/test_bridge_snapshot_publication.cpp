// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

#include <rclcpp/rclcpp.hpp>

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
    const auto now =
      std::chrono::steady_clock::now();

    const auto identifier =
      now.time_since_epoch().count();

    path_ =
      std::filesystem::temp_directory_path() /
      (
      "savo_bridge_phase2a_" +
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
    throw std::runtime_error(
            "Failed to open published snapshot");
  }

  std::ostringstream content;
  content << input.rdbuf();
  return content.str();
}

TEST(
  SavoBridgeSnapshotPublication,
  PublishesCanonicalEmptyObservationSnapshot)
{
  const RclcppGuard guard;
  const TemporaryDirectory temporary_directory;

  const std::filesystem::path snapshot_path =
    temporary_directory.path() / "snapshot.json";

  rclcpp::NodeOptions options;

  options.parameter_overrides({
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

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);

  std::string payload;

  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::seconds(4);

  do {
    executor.spin_some();

    if (std::filesystem::is_regular_file(
        snapshot_path))
    {
      payload = read_file(snapshot_path);

      const auto schema_position =
        payload.find(
        "\"schema_name\":\"savo_bridge_snapshot\"");

      if (schema_position != std::string::npos) {
        break;
      }
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(25));
  } while (
    std::chrono::steady_clock::now() < deadline);

  ASSERT_FALSE(payload.empty());

  EXPECT_NE(
    payload.find(
      "\"schema_name\":\"savo_bridge_snapshot\""),
    std::string::npos);

  EXPECT_NE(
    payload.find("\"schema_version\":2"),
    std::string::npos);

  EXPECT_NE(
    payload.find("\"snapshot_sequence\":"),
    std::string::npos);

  EXPECT_NE(
    payload.find("\"bridge\":{"),
    std::string::npos);

  EXPECT_NE(
    payload.find("\"read_only\":true"),
    std::string::npos);

  EXPECT_NE(
    payload.find("\"commands_enabled\":false"),
    std::string::npos);

  EXPECT_NE(
    payload.find("\"status\":\"unknown\""),
    std::string::npos);

  EXPECT_NE(
    payload.find("\"reason\":\"no_topics\""),
    std::string::npos);

  EXPECT_NE(
    payload.find("\"required_topics_ready\":false"),
    std::string::npos);

  EXPECT_NE(
    payload.find("\"topics\":[]"),
    std::string::npos);

  EXPECT_EQ(payload.back(), '\n');

  executor.remove_node(bridge);
  bridge.reset();
}

}  // namespace
