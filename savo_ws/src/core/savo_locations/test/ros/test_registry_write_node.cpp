#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <sqlite3.h>

#include "rclcpp/rclcpp.hpp"

#include "savo_msgs/msg/location_candidate.hpp"
#include "savo_msgs/msg/location_event.hpp"
#include "savo_msgs/srv/approve_location.hpp"
#include "savo_msgs/srv/get_location.hpp"
#include "savo_msgs/srv/list_locations.hpp"
#include "savo_msgs/srv/register_location_candidate.hpp"
#include "savo_msgs/srv/set_location_enabled.hpp"

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
  const std::filesystem::path & path,
  const bool enable_writes = true)
{
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
        "enable_write_services",
        enable_writes),
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

  return options;
}


savo_msgs::msg::LocationCandidate
make_candidate()
{
  savo_msgs::msg::LocationCandidate candidate;

  candidate.state =
    savo_msgs::msg::LocationCandidate::STATE_UNKNOWN;

  candidate.candidate_revision = 0U;
  candidate.candidate_id = "candidate-campus-main-27";

  candidate.map_id = "campus_main";
  candidate.map_revision = 7U;
  candidate.map_release_id =
    "campus_main_release_2026_07";

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

  candidate.building = "Main";
  candidate.floor = "2";
  candidate.area = "East wing";
  candidate.source_session_id = "mapping-session-7";
  candidate.source_component = "savo_mapping";

  return candidate;
}


template<typename ClientT>
bool wait_for_service(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::shared_ptr<ClientT> & client)
{
  for (int attempt = 0; attempt < 200; ++attempt) {
    executor.spin_some();

    if (client->service_is_ready()) {
      return true;
    }

    rclcpp::sleep_for(10ms);
  }

  return false;
}


void install_event_failure_trigger(
  const std::filesystem::path & path)
{
  sqlite3 * database = nullptr;

  ASSERT_EQ(
    sqlite3_open_v2(
      path.c_str(),
      &database,
      SQLITE_OPEN_READWRITE |
      SQLITE_OPEN_FULLMUTEX,
      nullptr),
    SQLITE_OK);

  char * error = nullptr;

  const int code = sqlite3_exec(
    database,
    "CREATE TRIGGER force_location_event_failure "
    "BEFORE INSERT ON location_events "
    "BEGIN "
    "SELECT RAISE(ABORT, 'forced event failure'); "
    "END;",
    nullptr,
    nullptr,
    &error);

  const std::string message =
    error == nullptr ? "" : error;

  sqlite3_free(error);
  sqlite3_close(database);

  ASSERT_EQ(code, SQLITE_OK) << message;
}

}  // namespace


TEST(RegistryWriteNode, CommitsWritesPublishesEventsAndRestarts)
{
  RclcppGuard guard;

  const auto path = database_path(
    "loc3b2_write_registry.sqlite3");

  auto registry =
    std::make_shared<
      savo_locations::LocationRegistryNode>(
        node_options(path));

  ASSERT_TRUE(registry->registry_ready());
  ASSERT_TRUE(registry->registry_write_ready());

  auto client_node =
    std::make_shared<rclcpp::Node>(
      "savo_locations_write_test_client");

  auto register_client =
    client_node->create_client<
      savo_msgs::srv::RegisterLocationCandidate>(
        "/savo_locations/candidates/register");

  auto approve_client =
    client_node->create_client<
      savo_msgs::srv::ApproveLocation>(
        "/savo_locations/candidates/approve");

  auto enabled_client =
    client_node->create_client<
      savo_msgs::srv::SetLocationEnabled>(
        "/savo_locations/set_enabled");

  auto get_client =
    client_node->create_client<
      savo_msgs::srv::GetLocation>(
        "/savo_locations/get");

  std::vector<savo_msgs::msg::LocationEvent>
    events;

  auto event_subscription =
    client_node->create_subscription<
      savo_msgs::msg::LocationEvent>(
        "/savo_locations/events",
        rclcpp::QoS(rclcpp::KeepLast(100))
          .reliable(),
        [&events](
          const savo_msgs::msg::LocationEvent & event)
        {
          events.push_back(event);
        });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(registry);
  executor.add_node(client_node);

  ASSERT_TRUE(wait_for_service(executor, register_client));
  ASSERT_TRUE(wait_for_service(executor, approve_client));
  ASSERT_TRUE(wait_for_service(executor, enabled_client));
  ASSERT_TRUE(wait_for_service(executor, get_client));

  auto register_request =
    std::make_shared<
      savo_msgs::srv::RegisterLocationCandidate::Request>();

  register_request->candidate = make_candidate();
  register_request->actor_id = "mapping_operator";

  auto register_future =
    register_client->async_send_request(register_request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      register_future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto register_response =
    register_future.get();

  ASSERT_TRUE(register_response->registered);
  EXPECT_EQ(
    register_response->result_code,
    savo_msgs::srv::RegisterLocationCandidate::
      Response::RESULT_REGISTERED);
  EXPECT_EQ(
    register_response->stored_candidate.candidate_revision,
    1U);

  auto duplicate_future =
    register_client->async_send_request(register_request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      duplicate_future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto duplicate_response =
    duplicate_future.get();

  EXPECT_FALSE(duplicate_response->registered);
  EXPECT_EQ(
    duplicate_response->result_code,
    savo_msgs::srv::RegisterLocationCandidate::
      Response::RESULT_DUPLICATE_CANDIDATE_ID);

  auto approve_request =
    std::make_shared<
      savo_msgs::srv::ApproveLocation::Request>();

  approve_request->candidate_id =
    "candidate-campus-main-27";
  approve_request->expected_candidate_revision = 1U;
  approve_request->actor_id = "location_operator";
  approve_request->arrival_confirmation_required = true;

  auto approve_future =
    approve_client->async_send_request(approve_request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      approve_future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto approve_response = approve_future.get();

  ASSERT_TRUE(approve_response->approved);
  EXPECT_EQ(approve_response->location.location_id, "A201");
  EXPECT_EQ(approve_response->location.record_revision, 1U);
  EXPECT_TRUE(approve_response->location.enabled);

  auto disable_request =
    std::make_shared<
      savo_msgs::srv::SetLocationEnabled::Request>();

  disable_request->location_id = "A201";
  disable_request->expected_record_revision = 1U;
  disable_request->enabled = false;
  disable_request->actor_id = "location_operator";
  disable_request->reason = "temporary maintenance";

  auto disable_future =
    enabled_client->async_send_request(disable_request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      disable_future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto disable_response = disable_future.get();

  ASSERT_TRUE(disable_response->updated);
  EXPECT_FALSE(disable_response->location.enabled);
  EXPECT_EQ(disable_response->location.record_revision, 2U);

  for (
    int attempt = 0;
    attempt < 200 && events.size() < 3U;
    ++attempt)
  {
    executor.spin_some();
    rclcpp::sleep_for(10ms);
  }

  ASSERT_EQ(events.size(), 3U);

  EXPECT_EQ(events[0].event_sequence, 1U);
  EXPECT_EQ(
    events[0].event_type,
    savo_msgs::msg::LocationEvent::
      EVENT_CANDIDATE_REGISTERED);

  EXPECT_EQ(events[1].event_sequence, 2U);
  EXPECT_EQ(
    events[1].event_type,
    savo_msgs::msg::LocationEvent::
      EVENT_LOCATION_APPROVED);

  EXPECT_EQ(events[2].event_sequence, 3U);
  EXPECT_EQ(
    events[2].event_type,
    savo_msgs::msg::LocationEvent::
      EVENT_LOCATION_DISABLED);

  auto get_request =
    std::make_shared<
      savo_msgs::srv::GetLocation::Request>();

  get_request->location_id = "A201";
  get_request->include_disabled = true;

  auto get_future =
    get_client->async_send_request(get_request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      get_future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto get_response = get_future.get();

  ASSERT_TRUE(get_response->found);
  EXPECT_FALSE(get_response->location.enabled);
  EXPECT_EQ(get_response->location.record_revision, 2U);

  static_cast<void>(event_subscription);

  executor.remove_node(client_node);
  executor.remove_node(registry);
  registry.reset();

  savo_locations::SqliteStore store{path.string()};
  ASSERT_TRUE(store.open().success);

  savo_locations::SqliteRepository repository{store};
  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository.bootstrap(&snapshot, &report).success);

  ASSERT_EQ(snapshot.locations.size(), 1U);
  ASSERT_EQ(snapshot.candidates.size(), 1U);
  EXPECT_FALSE(snapshot.locations.front().enabled);
  EXPECT_EQ(snapshot.locations.front().record_revision, 2U);
  EXPECT_EQ(report.event_count, 3U);
  EXPECT_EQ(report.last_event_sequence, 3U);

  ASSERT_TRUE(store.close().success);

  auto restarted =
    std::make_shared<
      savo_locations::LocationRegistryNode>(
        node_options(path));

  EXPECT_TRUE(restarted->registry_ready());
  EXPECT_TRUE(restarted->registry_write_ready());
}


TEST(RegistryWriteNode, EventFailureRollsBackAndDisablesWrites)
{
  RclcppGuard guard;

  const auto path = database_path(
    "loc3b2_write_degraded.sqlite3");

  auto registry =
    std::make_shared<
      savo_locations::LocationRegistryNode>(
        node_options(path));

  ASSERT_TRUE(registry->registry_ready());
  ASSERT_TRUE(registry->registry_write_ready());

  install_event_failure_trigger(path);

  auto client_node =
    std::make_shared<rclcpp::Node>(
      "savo_locations_degraded_test_client");

  auto register_client =
    client_node->create_client<
      savo_msgs::srv::RegisterLocationCandidate>(
        "/savo_locations/candidates/register");

  auto list_client =
    client_node->create_client<
      savo_msgs::srv::ListLocations>(
        "/savo_locations/list");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(registry);
  executor.add_node(client_node);

  ASSERT_TRUE(wait_for_service(executor, register_client));
  ASSERT_TRUE(wait_for_service(executor, list_client));

  auto request =
    std::make_shared<
      savo_msgs::srv::RegisterLocationCandidate::Request>();

  request->candidate = make_candidate();
  request->actor_id = "mapping_operator";

  auto future =
    register_client->async_send_request(request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto response = future.get();

  EXPECT_FALSE(response->registered);
  EXPECT_EQ(
    response->result_code,
    savo_msgs::srv::RegisterLocationCandidate::
      Response::RESULT_STORAGE_UNAVAILABLE);

  EXPECT_TRUE(registry->registry_ready());
  EXPECT_FALSE(registry->registry_write_ready());

  auto list_request =
    std::make_shared<
      savo_msgs::srv::ListLocations::Request>();

  auto list_future =
    list_client->async_send_request(list_request);

  ASSERT_EQ(
    executor.spin_until_future_complete(
      list_future,
      2s),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto list_response = list_future.get();

  EXPECT_TRUE(list_response->success);
  EXPECT_TRUE(list_response->locations.empty());

  executor.remove_node(client_node);
  executor.remove_node(registry);
  registry.reset();

  savo_locations::SqliteStore store{path.string()};
  ASSERT_TRUE(store.open().success);

  savo_locations::SqliteRepository repository{store};
  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository.bootstrap(&snapshot, &report).success);

  EXPECT_TRUE(snapshot.candidates.empty());
  EXPECT_TRUE(snapshot.locations.empty());
  EXPECT_EQ(report.event_count, 0U);
}
