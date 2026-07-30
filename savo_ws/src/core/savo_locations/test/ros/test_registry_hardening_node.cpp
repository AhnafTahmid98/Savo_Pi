
// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <sqlite3.h>

#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "savo_msgs/srv/approve_location.hpp"
#include "savo_msgs/srv/list_locations.hpp"
#include "savo_msgs/srv/recover_location_storage.hpp"
#include "savo_msgs/srv/register_location_candidate.hpp"

#include "savo_locations/location_registry_node.hpp"
#include "savo_locations/sqlite_repository.hpp"
#include "savo_locations/sqlite_store.hpp"

namespace
{

using namespace std::chrono_literals;

class RclcppGuard
{
public:
  RclcppGuard()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  ~RclcppGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class ExecutorThread
{
public:
  explicit ExecutorThread(
    rclcpp::executors::MultiThreadedExecutor & executor)
  : executor_(executor),
    thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorThread()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::executors::MultiThreadedExecutor & executor_;
  std::thread thread_;
};

std::filesystem::path database_path(
  const std::string & name)
{
  const std::filesystem::path directory{
    SAVO_LOCATIONS_TEST_DB_DIR};

  std::filesystem::create_directories(directory);

  const auto path = directory / name;

  std::filesystem::remove(path);
  std::filesystem::remove(path.string() + "-wal");
  std::filesystem::remove(path.string() + "-shm");

  return path;
}

rclcpp::NodeOptions node_options(
  const std::filesystem::path & path)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("database_path", path.string()),
      rclcpp::Parameter("create_parent_directories", true),
      rclcpp::Parameter("auto_migrate", true),
      rclcpp::Parameter("enable_write_services", true),
      rclcpp::Parameter("status_publish_hz", 20.0),
      rclcpp::Parameter("heartbeat_publish_hz", 20.0),
      rclcpp::Parameter("publish_snapshot", true),
    });
  return options;
}

savo_msgs::msg::LocationCandidate make_candidate()
{
  savo_msgs::msg::LocationCandidate candidate;
  candidate.candidate_id = "candidate-hardening-27";
  candidate.map_id = "campus_main";
  candidate.map_revision = 7U;
  candidate.map_release_id = "campus_main_release_2026_07";
  candidate.tag_family = "tag36h11";
  candidate.tag_id = 27;
  candidate.tag_pose_map.header.frame_id = "map";
  candidate.tag_pose_map.pose.position.x = 12.8;
  candidate.tag_pose_map.pose.position.y = 8.1;
  candidate.tag_pose_map.pose.orientation.w = 1.0;
  candidate.detection_quality = 0.95F;
  candidate.accepted_observations = 8U;
  candidate.position_stddev_m = 0.02F;
  candidate.yaw_stddev_rad = 0.03F;
  candidate.approach_pose_valid = true;
  candidate.approach_pose.header.frame_id = "map";
  candidate.approach_pose.pose.position.x = 12.0;
  candidate.approach_pose.pose.position.y = 7.5;
  candidate.approach_pose.pose.orientation.w = 1.0;
  candidate.suggested_location_id = "A201";
  candidate.suggested_display_name = "Room A201";
  candidate.suggested_aliases = {"East classroom"};
  candidate.suggested_semantic_type = "classroom";
  candidate.source_session_id = "mapping-session-7";
  candidate.source_component = "savo_mapping";
  return candidate;
}

template<typename FutureT>
bool future_ready(FutureT & future)
{
  return future.wait_for(3s) == std::future_status::ready;
}

void execute_sql(
  const std::filesystem::path & path,
  const char * sql)
{
  sqlite3 * database = nullptr;
  ASSERT_EQ(
    sqlite3_open_v2(
      path.c_str(),
      &database,
      SQLITE_OPEN_READWRITE | SQLITE_OPEN_FULLMUTEX,
      nullptr),
    SQLITE_OK);

  char * error = nullptr;
  const int code = sqlite3_exec(
    database, sql, nullptr, nullptr, &error);
  const std::string message =
    error == nullptr ? "" : error;
  sqlite3_free(error);
  sqlite3_close(database);
  ASSERT_EQ(code, SQLITE_OK) << message;
}

void install_event_failure_trigger(
  const std::filesystem::path & path)
{
  execute_sql(
    path,
    "CREATE TRIGGER force_location_event_failure "
    "BEFORE INSERT ON location_events "
    "BEGIN "
    "SELECT RAISE(ABORT, 'forced event failure'); "
    "END;");
}

void remove_event_failure_trigger(
  const std::filesystem::path & path)
{
  execute_sql(
    path,
    "DROP TRIGGER force_location_event_failure;");
}

std::shared_ptr<
  savo_msgs::srv::RegisterLocationCandidate::Response>
register_candidate(
  const std::shared_ptr<rclcpp::Node> & client_node,
  const std::string & actor)
{
  auto client = client_node->create_client<
    savo_msgs::srv::RegisterLocationCandidate>(
      "/savo_locations/candidates/register");
  EXPECT_TRUE(client->wait_for_service(3s));

  auto request = std::make_shared<
    savo_msgs::srv::RegisterLocationCandidate::Request>();
  request->candidate = make_candidate();
  request->actor_id = actor;

  auto future = client->async_send_request(request);
  EXPECT_TRUE(future_ready(future));
  return future.get();
}

}  // namespace

TEST(RegistryHardeningNode, ConcurrentApprovalsCommitExactlyOnce)
{
  RclcppGuard guard;
  const auto path = database_path(
    "loc3p_concurrent_approval.sqlite3");

  auto registry = std::make_shared<
    savo_locations::LocationRegistryNode>(
      node_options(path));
  auto client_node = std::make_shared<rclcpp::Node>(
    "savo_locations_concurrent_client");

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(registry);
  executor.add_node(client_node);
  ExecutorThread spinning(executor);

  const auto registered = register_candidate(
    client_node, "mapping_operator");
  ASSERT_TRUE(registered->registered);

  auto approve_client = client_node->create_client<
    savo_msgs::srv::ApproveLocation>(
      "/savo_locations/candidates/approve");
  ASSERT_TRUE(approve_client->wait_for_service(3s));

  auto request_a = std::make_shared<
    savo_msgs::srv::ApproveLocation::Request>();
  request_a->candidate_id = "candidate-hardening-27";
  request_a->expected_candidate_revision = 1U;
  request_a->actor_id = "operator_a";
  request_a->arrival_confirmation_required = true;

  auto request_b = std::make_shared<
    savo_msgs::srv::ApproveLocation::Request>(*request_a);
  request_b->actor_id = "operator_b";

  auto future_a = approve_client->async_send_request(request_a);
  auto future_b = approve_client->async_send_request(request_b);

  ASSERT_TRUE(future_ready(future_a));
  ASSERT_TRUE(future_ready(future_b));

  const auto response_a = future_a.get();
  const auto response_b = future_b.get();

  const int success_count =
    static_cast<int>(response_a->approved) +
    static_cast<int>(response_b->approved);

  EXPECT_EQ(success_count, 1);
  EXPECT_TRUE(registry->registry_ready());
  EXPECT_TRUE(registry->registry_write_ready());

  savo_locations::SqliteStore store{path.string()};
  ASSERT_TRUE(store.open().success);
  savo_locations::SqliteRepository repository{store};
  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;
  ASSERT_TRUE(repository.bootstrap(&snapshot, &report).success);
  EXPECT_EQ(snapshot.locations.size(), 1U);
  EXPECT_EQ(snapshot.candidates.size(), 1U);
  EXPECT_EQ(report.event_count, 2U);
}

TEST(RegistryHardeningNode, VerifiedRecoveryRestoresWritesAfterRollback)
{
  RclcppGuard guard;
  const auto path = database_path(
    "loc3p_verified_recovery.sqlite3");

  auto registry = std::make_shared<
    savo_locations::LocationRegistryNode>(
      node_options(path));
  auto client_node = std::make_shared<rclcpp::Node>(
    "savo_locations_recovery_client");

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(registry);
  executor.add_node(client_node);
  ExecutorThread spinning(executor);

  const auto registered = register_candidate(
    client_node, "mapping_operator");
  ASSERT_TRUE(registered->registered);
  ASSERT_EQ(registered->stored_candidate.candidate_revision, 1U);

  install_event_failure_trigger(path);

  auto approve_client = client_node->create_client<
    savo_msgs::srv::ApproveLocation>(
      "/savo_locations/candidates/approve");
  ASSERT_TRUE(approve_client->wait_for_service(3s));

  auto approve_request = std::make_shared<
    savo_msgs::srv::ApproveLocation::Request>();
  approve_request->candidate_id = "candidate-hardening-27";
  approve_request->expected_candidate_revision = 1U;
  approve_request->actor_id = "location_operator";
  approve_request->arrival_confirmation_required = true;

  auto failed_future =
    approve_client->async_send_request(approve_request);
  ASSERT_TRUE(future_ready(failed_future));
  const auto failed_response = failed_future.get();

  EXPECT_FALSE(failed_response->approved);
  EXPECT_EQ(
    failed_response->result_code,
    savo_msgs::srv::ApproveLocation::Response::
      RESULT_STORAGE_UNAVAILABLE);
  EXPECT_TRUE(registry->registry_ready());
  EXPECT_FALSE(registry->registry_write_ready());

  auto list_client = client_node->create_client<
    savo_msgs::srv::ListLocations>(
      "/savo_locations/list");
  ASSERT_TRUE(list_client->wait_for_service(3s));
  auto list_future = list_client->async_send_request(
    std::make_shared<
      savo_msgs::srv::ListLocations::Request>());
  ASSERT_TRUE(future_ready(list_future));
  EXPECT_TRUE(list_future.get()->locations.empty());

  remove_event_failure_trigger(path);

  auto recovery_client = client_node->create_client<
    savo_msgs::srv::RecoverLocationStorage>(
      "/savo_locations/storage/recover");
  ASSERT_TRUE(recovery_client->wait_for_service(3s));

  auto recovery_request = std::make_shared<
    savo_msgs::srv::RecoverLocationStorage::Request>();
  recovery_request->actor_id = "maintenance_operator";
  recovery_request->reason = "event trigger removed";

  auto recovery_future =
    recovery_client->async_send_request(recovery_request);
  ASSERT_TRUE(future_ready(recovery_future));
  const auto recovery_response = recovery_future.get();

  ASSERT_TRUE(recovery_response->recovered);
  EXPECT_EQ(recovery_response->candidate_count, 1U);
  EXPECT_EQ(recovery_response->location_count, 0U);
  EXPECT_EQ(recovery_response->event_count, 1U);
  EXPECT_EQ(recovery_response->last_event_sequence, 1U);
  EXPECT_TRUE(registry->registry_write_ready());

  auto retry_future =
    approve_client->async_send_request(approve_request);
  ASSERT_TRUE(future_ready(retry_future));
  const auto retry_response = retry_future.get();

  ASSERT_TRUE(retry_response->approved);
  EXPECT_EQ(retry_response->location.location_id, "A201");
  EXPECT_EQ(retry_response->location.record_revision, 1U);

  savo_locations::SqliteStore store{path.string()};
  ASSERT_TRUE(store.open().success);
  savo_locations::SqliteRepository repository{store};
  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;
  ASSERT_TRUE(repository.bootstrap(&snapshot, &report).success);
  EXPECT_EQ(snapshot.locations.size(), 1U);
  EXPECT_EQ(report.event_count, 2U);
  EXPECT_EQ(report.last_event_sequence, 2U);
}
