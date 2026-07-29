#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "savo_msgs/srv/get_location.hpp"
#include "savo_msgs/srv/list_locations.hpp"
#include "savo_msgs/srv/resolve_location.hpp"

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


savo_locations::PoseData make_pose()
{
  savo_locations::PoseData pose;

  pose.frame_id = "map";
  pose.x = 4.0;
  pose.y = 2.0;
  pose.qw = 1.0;

  return pose;
}


savo_locations::LocationRecordData
make_location()
{
  savo_locations::LocationRecordData record;

  record.state =
    savo_locations::LocationState::
      kApproved;

  record.enabled = true;
  record.record_revision = 1U;

  record.location.location_id = "A201";
  record.location.display_name = "Room A201";

  record.location.aliases = {
    "East classroom",
  };

  record.location.semantic_type =
    "classroom";

  record.location.map.map_id =
    "campus_main";

  record.location.map.map_revision = 7U;

  record.location.approach_pose =
    make_pose();

  record.location.tag.family =
    "tag36h11";

  record.location.tag.id = 27;

  return record;
}


std::filesystem::path database_path()
{
  const std::filesystem::path directory{
    SAVO_LOCATIONS_TEST_DB_DIR};

  std::filesystem::create_directories(
    directory);

  const auto path =
    directory /
    "loc3a_registry_node.sqlite3";

  std::filesystem::remove(path);

  std::filesystem::remove(
    path.string() + "-wal");

  std::filesystem::remove(
    path.string() + "-shm");

  return path;
}


void create_database(
  const std::filesystem::path & path)
{
  savo_locations::SqliteStore store{
    path.string()};

  ASSERT_TRUE(store.open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store.migrate(&status).success);

  savo_locations::SqliteRepository
    repository{store};

  savo_locations::CatalogSnapshot snapshot;

  snapshot.locations.push_back(
    make_location());

  ASSERT_TRUE(
    repository
      .save_snapshot(snapshot)
      .success);
}

}  // namespace


TEST(RegistryNode, BootstrapsAndServesReads)
{
  RclcppGuard guard;

  const auto path = database_path();

  create_database(path);

  rclcpp::NodeOptions options;

  options.parameter_overrides(
    {
      rclcpp::Parameter(
        "database_path",
        path.string()),
      rclcpp::Parameter(
        "create_parent_directories",
        true),
      rclcpp::Parameter(
        "auto_migrate",
        true),
      rclcpp::Parameter(
        "status_publish_hz",
        20.0),
      rclcpp::Parameter(
        "heartbeat_publish_hz",
        20.0),
      rclcpp::Parameter(
        "publish_snapshot",
        true),
    });

  auto registry =
    std::make_shared<
      savo_locations::LocationRegistryNode>(
        options);

  ASSERT_TRUE(
    registry->registry_ready());

  auto client_node =
    std::make_shared<rclcpp::Node>(
      "savo_locations_test_client");

  auto resolve_client =
    client_node->create_client<
      savo_msgs::srv::ResolveLocation>(
        "/savo_locations/resolve");

  auto get_client =
    client_node->create_client<
      savo_msgs::srv::GetLocation>(
        "/savo_locations/get");

  auto list_client =
    client_node->create_client<
      savo_msgs::srv::ListLocations>(
        "/savo_locations/list");

  bool status_received = false;
  bool heartbeat_received = false;

  auto status_subscription =
    client_node->create_subscription<
      std_msgs::msg::String>(
        "/savo_locations/status",
        rclcpp::QoS(
          rclcpp::KeepLast(1))
          .reliable()
          .transient_local(),
        [&status_received](
          const std_msgs::msg::String & message)
        {
          status_received =
            message.data.find(
              "\"ready\":true") !=
            std::string::npos;
        });

  auto heartbeat_subscription =
    client_node->create_subscription<
      std_msgs::msg::UInt64>(
        "/savo_locations/heartbeat",
        rclcpp::QoS(
          rclcpp::KeepLast(10))
          .reliable(),
        [&heartbeat_received](
          const std_msgs::msg::UInt64 & message)
        {
          heartbeat_received =
            message.data > 0U;
        });

  rclcpp::executors::
    SingleThreadedExecutor executor;

  executor.add_node(registry);
  executor.add_node(client_node);

  for (
    int attempt = 0;
    attempt < 100;
    ++attempt)
  {
    executor.spin_some();

    if (
      resolve_client->service_is_ready() &&
      get_client->service_is_ready() &&
      list_client->service_is_ready())
    {
      break;
    }

    rclcpp::sleep_for(10ms);
  }

  ASSERT_TRUE(
    resolve_client->service_is_ready());

  ASSERT_TRUE(
    get_client->service_is_ready());

  ASSERT_TRUE(
    list_client->service_is_ready());

  auto resolve_request =
    std::make_shared<
      savo_msgs::srv::
        ResolveLocation::Request>();

  resolve_request->query =
    "East classroom";

  const auto resolve_future =
    resolve_client->async_send_request(
      resolve_request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      resolve_future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto resolve_response =
    resolve_future.get();

  EXPECT_TRUE(
    resolve_response->resolved);

  EXPECT_EQ(
    resolve_response->result_code,
    savo_msgs::srv::
      ResolveLocation::Response::
        RESULT_RESOLVED);

  EXPECT_EQ(
    resolve_response
      ->location
      .location_id,
    "A201");

  auto get_request =
    std::make_shared<
      savo_msgs::srv::
        GetLocation::Request>();

  get_request->location_id = "A201";

  const auto get_future =
    get_client->async_send_request(
      get_request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      get_future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto get_response =
    get_future.get();

  EXPECT_TRUE(get_response->found);

  EXPECT_EQ(
    get_response->location.location_id,
    "A201");

  auto list_request =
    std::make_shared<
      savo_msgs::srv::
        ListLocations::Request>();

  list_request->map_id =
    "campus_main";

  list_request->map_revision = 7U;
  list_request->enabled_only = true;

  const auto list_future =
    list_client->async_send_request(
      list_request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      list_future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto list_response =
    list_future.get();

  EXPECT_TRUE(list_response->success);

  ASSERT_EQ(
    list_response->locations.size(),
    1U);

  EXPECT_EQ(
    list_response
      ->locations
      .front()
      .location_id,
    "A201");

  for (
    int attempt = 0;
    attempt < 100 &&
    (
      !status_received ||
      !heartbeat_received
    );
    ++attempt)
  {
    executor.spin_some();
    rclcpp::sleep_for(10ms);
  }

  EXPECT_TRUE(status_received);
  EXPECT_TRUE(heartbeat_received);

  static_cast<void>(
    status_subscription);

  static_cast<void>(
    heartbeat_subscription);

  executor.remove_node(client_node);
  executor.remove_node(registry);
}
