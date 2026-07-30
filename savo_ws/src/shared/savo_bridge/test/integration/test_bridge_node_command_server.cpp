// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "savo_bridge/bridge_node.hpp"

namespace
{

using Json = nlohmann::json;

class RclcppGuard final
{
public:
  RclcppGuard()
  {
    int argument_count = 0;
    char ** arguments = nullptr;
    rclcpp::init(argument_count, arguments);
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
    std::string pattern{
      "/tmp/savo_bridge_node_command_XXXXXX"};
    std::vector<char> writable(
      pattern.begin(), pattern.end());
    writable.push_back('\0');

    const char * const created =
      ::mkdtemp(writable.data());
    if (created == nullptr) {
      throw std::runtime_error(
              "temporary directory creation failed");
    }
    path_ = created;
    if (::chmod(path_.c_str(), 0700) != 0) {
      throw std::runtime_error(
              "temporary directory chmod failed");
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

[[nodiscard]] std::int64_t now_unix_ms()
{
  return std::chrono::duration_cast<
    std::chrono::milliseconds>(
    std::chrono::system_clock::now()
    .time_since_epoch()).count();
}

[[nodiscard]] bool is_unix_socket(
  const std::filesystem::path & path)
{
  struct stat metadata {};
  return ::lstat(path.c_str(), &metadata) == 0 &&
         S_ISSOCK(metadata.st_mode);
}

[[nodiscard]] bool wait_for_socket(
  const std::filesystem::path & path)
{
  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::seconds(2);
  do {
    if (is_unix_socket(path)) {
      return true;
    }
    std::this_thread::sleep_for(
      std::chrono::milliseconds(5));
  } while (
    std::chrono::steady_clock::now() < deadline);
  return false;
}

[[nodiscard]] int connect_client(
  const std::filesystem::path & path)
{
  const std::string path_text = path.string();
  if (path_text.size() >= sizeof(sockaddr_un::sun_path)) {
    throw std::runtime_error("test socket path too long");
  }

  const int descriptor =
    ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (descriptor < 0) {
    throw std::runtime_error("test socket creation failed");
  }

  sockaddr_un address {};
  address.sun_family = AF_UNIX;
  std::memcpy(
    address.sun_path,
    path_text.c_str(),
    path_text.size() + 1U);

  if (::connect(
      descriptor,
      reinterpret_cast<const sockaddr *>(&address),
      sizeof(address)) != 0)
  {
    (void)::close(descriptor);
    throw std::runtime_error("test socket connect failed");
  }

  timeval timeout {};
  timeout.tv_sec = 2;
  (void)::setsockopt(
    descriptor,
    SOL_SOCKET,
    SO_RCVTIMEO,
    &timeout,
    sizeof(timeout));
  return descriptor;
}

[[nodiscard]] std::string socket_exchange(
  const std::filesystem::path & path,
  const std::string & request)
{
  const int descriptor = connect_client(path);
  std::size_t sent{0U};
  while (sent < request.size()) {
    const ssize_t written = ::send(
      descriptor,
      request.data() + sent,
      request.size() - sent,
      MSG_NOSIGNAL);
    if (written <= 0) {
      (void)::close(descriptor);
      throw std::runtime_error("test socket write failed");
    }
    sent += static_cast<std::size_t>(written);
  }

  std::string response;
  char buffer[2048];
  while (true) {
    const ssize_t received =
      ::recv(descriptor, buffer, sizeof(buffer), 0);
    if (received == 0) {
      break;
    }
    if (received < 0) {
      if (errno == ECONNRESET && response.empty()) {
        break;
      }
      (void)::close(descriptor);
      throw std::runtime_error("test socket read failed");
    }
    response.append(
      buffer,
      static_cast<std::size_t>(received));
  }
  (void)::close(descriptor);
  return response;
}

[[nodiscard]] rclcpp::NodeOptions enabled_options(
  const std::filesystem::path & socket_path,
  std::vector<std::int64_t> allowed_uids = {
    static_cast<std::int64_t>(::geteuid())
  })
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("command_server.enabled", true),
      rclcpp::Parameter(
        "command_server.execution_mode",
        "dry_run"),
      rclcpp::Parameter(
        "command_server.socket_path",
        socket_path.string()),
      rclcpp::Parameter(
        "command_server.bridge_instance_id",
        "savo-bridge-integration-test"),
      rclcpp::Parameter(
        "command_server.accept_timeout_ms",
        25),
      rclcpp::Parameter(
        "command_server.client_read_timeout_ms",
        250),
      rclcpp::Parameter(
        "command_server.client_write_timeout_ms",
        250),
      rclcpp::Parameter(
        "command_server.socket_mode",
        384),
      rclcpp::Parameter(
        "command_server.allowed_peer_uids",
        std::move(allowed_uids)),
    });
  return options;
}

[[nodiscard]] rclcpp::NodeOptions live_enabled_options(
  const std::filesystem::path & socket_path,
  const std::string & prefix)
{
  rclcpp::NodeOptions options;

  options.parameter_overrides({
      rclcpp::Parameter("command_server.enabled", true),
      rclcpp::Parameter(
        "command_server.execution_mode",
        "live"),
      rclcpp::Parameter(
        "command_server.socket_path",
        socket_path.string()),
      rclcpp::Parameter(
        "command_server.bridge_instance_id",
        "savo-bridge-live-integration-test"),
      rclcpp::Parameter(
        "command_server.accept_timeout_ms",
        25),
      rclcpp::Parameter(
        "command_server.client_read_timeout_ms",
        500),
      rclcpp::Parameter(
        "command_server.client_write_timeout_ms",
        1000),
      rclcpp::Parameter(
        "command_server.socket_mode",
        384),
      rclcpp::Parameter(
        "command_server.allowed_peer_uids",
        std::vector<std::int64_t>{
        static_cast<std::int64_t>(::geteuid())}),
      rclcpp::Parameter(
        "command_dispatcher.mode_command_topic",
        prefix + "/mode_cmd"),
      rclcpp::Parameter(
        "command_dispatcher.mode_state_topic",
        prefix + "/mode_state"),
      rclcpp::Parameter(
        "command_dispatcher.external_stop_topic",
        prefix + "/external_stop"),
      rclcpp::Parameter(
        "command_dispatcher.safety_stop_topic",
        prefix + "/safety_stop"),
      rclcpp::Parameter(
        "command_dispatcher.manual_velocity_topic",
        prefix + "/cmd_vel_manual"),
      rclcpp::Parameter(
        "command_dispatcher.safe_velocity_topic",
        prefix + "/cmd_vel_safe"),
      rclcpp::Parameter(
        "command_dispatcher.navigation_readiness_topic",
        prefix + "/navigation_readiness"),
      rclcpp::Parameter(
        "command_dispatcher.navigation_action_name",
        prefix + "/navigate_to_pose"),
      rclcpp::Parameter(
        "command_dispatcher.location_resolve_service",
        prefix + "/resolve_location"),
      rclcpp::Parameter(
        "command_dispatcher.active_map_id",
        "integration-map"),
      rclcpp::Parameter(
        "command_dispatcher.active_map_revision",
        1),
      rclcpp::Parameter(
        "command_dispatcher.observed_state_timeout_ms",
        500),
      rclcpp::Parameter(
        "command_dispatcher.stop_confirmation_timeout_ms",
        1000),
    });

  return options;
}

[[nodiscard]] Json command_request(
  const std::string & command_id,
  const std::string & command_type)
{
  const std::int64_t issued_at = now_unix_ms();
  Json request{
    {"protocol_version", 1},
    {"message_type", "command_request"},
    {"command_id", command_id},
    {"command_type", command_type},
    {"source", "savomind"},
    {"origin_agent", "safety_agent"},
    {"priority", "normal"},
    {"issued_at_unix_ms", issued_at},
    {"expires_at_unix_ms", issued_at + 5000},
  };

  if (command_type == "stop") {
    request["priority"] = "emergency";
    request["payload"] = {
      {"reason", "bridge_node_integration_test"},
      {"scope", "all_movement"},
    };
  } else if (command_type == "teleop_nudge") {
    request["payload"] = {
      {"direction", "forward"},
      {"duration_ms", 100},
      {"linear_x_mps", 0.1},
    };
  } else {
    request["payload"] = {
      {"location_id", "integration-location"},
      {"map_id", "integration-map"},
    };
  }
  return request;
}

void expect_dry_run_acknowledgement(
  const std::string & encoded_response,
  const std::string & command_id)
{
  ASSERT_FALSE(encoded_response.empty());
  EXPECT_EQ(encoded_response.back(), '\n');
  EXPECT_EQ(
    std::count(
      encoded_response.begin(),
      encoded_response.end(),
      '\n'),
    1);

  const Json response = Json::parse(
    encoded_response.substr(
      0U, encoded_response.size() - 1U));
  EXPECT_EQ(
    response.at("message_type"),
    "command_acknowledgement");
  EXPECT_EQ(response.at("protocol_version"), 1);
  EXPECT_EQ(response.at("command_id"), command_id);
  EXPECT_EQ(response.at("accepted"), true);
  EXPECT_EQ(response.at("state"), "dry_run");
  EXPECT_EQ(
    response.at("reason"),
    "bridge_command_dry_run_accepted");
  EXPECT_EQ(response.at("execution_mode"), "dry_run");
  EXPECT_EQ(response.at("duplicate"), false);
  EXPECT_EQ(
    response.at("details").at("dispatch_attempted"),
    false);
  EXPECT_EQ(
    response.at("details").at("ros_publications"),
    0);
}

TEST(BridgeNodeCommandServer, DefaultsAreDisabledAndReadOnly)
{
  const RclcppGuard guard;
  ASSERT_FALSE(
    std::filesystem::exists(
      "/run/savo_bridge/command.sock"));

  auto node =
    std::make_shared<savo_bridge::BridgeNode>();
  EXPECT_FALSE(node->command_server_enabled());
  EXPECT_FALSE(node->command_worker_joinable());

  const std::vector<std::string> names{
    "command_server.enabled",
    "command_server.execution_mode",
    "command_server.socket_path",
    "command_server.bridge_instance_id",
    "command_server.max_request_bytes",
    "command_server.max_response_bytes",
    "command_server.accept_timeout_ms",
    "command_server.client_read_timeout_ms",
    "command_server.client_write_timeout_ms",
    "command_server.socket_mode",
    "command_server.command_id_cache_capacity",
    "command_server.allowed_peer_uids",
  };
  for (const std::string & name : names) {
    EXPECT_TRUE(
      node->describe_parameter(name).read_only);
  }

  const auto result = node->set_parameter(
    rclcpp::Parameter(
      "command_server.enabled",
      true));
  EXPECT_FALSE(result.successful);
  EXPECT_FALSE(
    std::filesystem::exists(
      "/run/savo_bridge/command.sock"));
}

TEST(BridgeNodeCommandServer, DisabledInvalidPathIsNeverInspected)
{
  const RclcppGuard guard;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter(
        "command_server.enabled",
        false),
      rclcpp::Parameter(
        "command_server.socket_path",
        "/definitely/not/a/real/parent/command.sock"),
    });

  auto node =
    std::make_shared<savo_bridge::BridgeNode>(
    options);
  EXPECT_FALSE(node->command_worker_joinable());
}

TEST(BridgeNodeCommandServer, EnabledConfigurationIsStrict)
{
  const RclcppGuard guard;

  rclcpp::NodeOptions invalid_mode_options;
  invalid_mode_options.parameter_overrides({
      rclcpp::Parameter("command_server.enabled", true),
      rclcpp::Parameter(
        "command_server.execution_mode",
        "unsafe"),
    });

  EXPECT_THROW(
    std::make_shared<savo_bridge::BridgeNode>(
      invalid_mode_options),
    std::invalid_argument);

  rclcpp::NodeOptions invalid_path_options;
  invalid_path_options.parameter_overrides({
      rclcpp::Parameter("command_server.enabled", true),
      rclcpp::Parameter(
        "command_server.socket_path",
        "/definitely/not/a/real/parent/command.sock"),
    });
  EXPECT_THROW(
    std::make_shared<savo_bridge::BridgeNode>(
      invalid_path_options),
    std::runtime_error);

  rclcpp::NodeOptions duplicate_uids;
  duplicate_uids.parameter_overrides({
      rclcpp::Parameter("command_server.enabled", true),
      rclcpp::Parameter(
        "command_server.allowed_peer_uids",
        std::vector<std::int64_t>{1, 1}),
    });
  EXPECT_THROW(
    std::make_shared<savo_bridge::BridgeNode>(
      duplicate_uids),
    std::invalid_argument);

  rclcpp::NodeOptions negative_uid;
  negative_uid.parameter_overrides({
      rclcpp::Parameter("command_server.enabled", true),
      rclcpp::Parameter(
        "command_server.allowed_peer_uids",
        std::vector<std::int64_t>{-1}),
    });
  EXPECT_THROW(
    std::make_shared<savo_bridge::BridgeNode>(
      negative_uid),
    std::invalid_argument);

  rclcpp::NodeOptions oversized_uid;
  oversized_uid.parameter_overrides({
      rclcpp::Parameter("command_server.enabled", true),
      rclcpp::Parameter(
        "command_server.allowed_peer_uids",
        std::vector<std::int64_t>{4294967296LL}),
    });
  EXPECT_THROW(
    std::make_shared<savo_bridge::BridgeNode>(
      oversized_uid),
    std::invalid_argument);

  rclcpp::NodeOptions numeric_string;
  numeric_string.parameter_overrides({
      rclcpp::Parameter(
        "command_server.max_request_bytes",
        "65536"),
    });
  EXPECT_THROW(
    std::make_shared<savo_bridge::BridgeNode>(
      numeric_string),
    std::exception);
}

TEST(BridgeNodeCommandServer, EnabledNodeOwnsOneWorkerAndSocket)
{
  const RclcppGuard guard;
  const TemporaryDirectory temporary;
  const auto socket_path =
    temporary.path() / "command.sock";

  {
    auto node =
      std::make_shared<savo_bridge::BridgeNode>(
      enabled_options(socket_path));
    ASSERT_TRUE(wait_for_socket(socket_path));
    EXPECT_TRUE(node->command_server_enabled());
    EXPECT_TRUE(node->command_worker_joinable());

    struct stat metadata {};
    ASSERT_EQ(
      ::lstat(socket_path.c_str(), &metadata),
      0);
    EXPECT_TRUE(S_ISSOCK(metadata.st_mode));
    EXPECT_EQ(metadata.st_mode & 0777U, 0600U);
  }
  EXPECT_FALSE(
    std::filesystem::exists(socket_path));
}


TEST(BridgeNodeCommandServer, LiveModeDispatchesStopAndReplaysDuplicate)
{
  const RclcppGuard guard;
  const TemporaryDirectory temporary;

  const auto socket_path =
    temporary.path() / "command.sock";

  const std::string prefix =
    "/savo_bridge_live_command_test";

  auto fixture =
    std::make_shared<rclcpp::Node>(
    "savo_bridge_live_command_fixture");

  auto latched_qos =
    rclcpp::QoS(rclcpp::KeepLast(1));

  latched_qos.reliable();
  latched_qos.transient_local();

  auto reliable_qos =
    rclcpp::QoS(rclcpp::KeepLast(20));

  reliable_qos.reliable();

  auto mode_state_publisher =
    fixture->create_publisher<
    std_msgs::msg::String>(
    prefix + "/mode_state",
    latched_qos);

  auto external_stop_publisher =
    fixture->create_publisher<
    std_msgs::msg::Bool>(
    prefix + "/external_stop",
    reliable_qos);

  auto safety_stop_publisher =
    fixture->create_publisher<
    std_msgs::msg::Bool>(
    prefix + "/safety_stop",
    reliable_qos);

  auto safe_velocity_publisher =
    fixture->create_publisher<
    geometry_msgs::msg::Twist>(
    prefix + "/cmd_vel_safe",
    reliable_qos);

  std::atomic<std::size_t>
    mode_command_count{0U};

  std::atomic<std::size_t>
    external_stop_true_count{0U};

  auto mode_command_subscription =
    fixture->create_subscription<
    std_msgs::msg::String>(
    prefix + "/mode_cmd",
    reliable_qos,
    [&mode_state_publisher,
    &mode_command_count](
      const std_msgs::msg::String::SharedPtr message)
    {
      mode_command_count.fetch_add(1U);
      mode_state_publisher->publish(*message);
    });

  auto external_stop_subscription =
    fixture->create_subscription<
    std_msgs::msg::Bool>(
    prefix + "/external_stop",
    reliable_qos,
    [&external_stop_true_count](
      const std_msgs::msg::Bool::SharedPtr message)
    {
      if (message->data) {
        external_stop_true_count.fetch_add(1U);
      }
    });

  auto observation_timer =
    fixture->create_wall_timer(
    std::chrono::milliseconds(20),
    [&safety_stop_publisher,
    &safe_velocity_publisher]()
    {
      std_msgs::msg::Bool safety;
      safety.data = false;
      safety_stop_publisher->publish(safety);

      geometry_msgs::msg::Twist zero;
      safe_velocity_publisher->publish(zero);
    });

  auto bridge_node =
    std::make_shared<savo_bridge::BridgeNode>(
    live_enabled_options(
      socket_path,
      prefix));

  rclcpp::executors::MultiThreadedExecutor
    executor(
    rclcpp::ExecutorOptions(),
    4U);

  executor.add_node(fixture);
  executor.add_node(bridge_node);

  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    });

  ASSERT_TRUE(wait_for_socket(socket_path));

  std_msgs::msg::String stop_mode;
  stop_mode.data = "STOP";

  std_msgs::msg::Bool external_clear;
  external_clear.data = false;

  for (std::size_t index = 0U; index < 20U; ++index) {
    mode_state_publisher->publish(stop_mode);
    external_stop_publisher->publish(external_clear);
    std::this_thread::sleep_for(
      std::chrono::milliseconds(20));
  }

  const std::string command_id =
    "bridge-node-live-stop-1";

  const std::string encoded_request =
    command_request(
    command_id,
    "stop").dump() + "\n";

  const Json first_response = Json::parse(
    socket_exchange(
      socket_path,
      encoded_request));

  EXPECT_EQ(
    first_response.at("message_type"),
    "command_acknowledgement");

  EXPECT_EQ(
    first_response.at("execution_mode"),
    "live");

  EXPECT_EQ(
    first_response.at("accepted"),
    true);

  EXPECT_EQ(
    first_response.at("state"),
    "accepted");

  EXPECT_EQ(
    first_response.at("reason"),
    "bridge_stop_confirmed");

  EXPECT_EQ(
    first_response.at("duplicate"),
    false);

  EXPECT_EQ(
    first_response.at("details").
    at("dispatch_attempted"),
    true);

  EXPECT_GT(
    first_response.at("details").
    at("ros_publications").get<std::size_t>(),
    0U);

  const Json duplicate_response = Json::parse(
    socket_exchange(
      socket_path,
      encoded_request));

  EXPECT_EQ(
    duplicate_response.at("accepted"),
    true);

  EXPECT_EQ(
    duplicate_response.at("duplicate"),
    true);

  EXPECT_EQ(
    duplicate_response.at("reason"),
    "bridge_stop_confirmed");

  EXPECT_EQ(
    duplicate_response.at("details").
    at("dispatch_attempted"),
    false);

  EXPECT_EQ(
    duplicate_response.at("details").
    at("ros_publications"),
    0);

  EXPECT_GT(mode_command_count.load(), 0U);
  EXPECT_GT(external_stop_true_count.load(), 0U);

  executor.cancel();

  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  executor.remove_node(bridge_node);
  executor.remove_node(fixture);

  bridge_node.reset();
  observation_timer.reset();
  external_stop_subscription.reset();
  mode_command_subscription.reset();
  fixture.reset();

  EXPECT_FALSE(
    std::filesystem::exists(socket_path));
}

TEST(BridgeNodeCommandServer, SemanticCommandsRemainDryRunOnly)
{
  const RclcppGuard guard;
  const TemporaryDirectory temporary;
  const auto socket_path =
    temporary.path() / "command.sock";
  auto node =
    std::make_shared<savo_bridge::BridgeNode>(
    enabled_options(socket_path));
  ASSERT_TRUE(wait_for_socket(socket_path));

  const std::vector<std::string> command_types{
    "stop",
    "teleop_nudge",
    "navigate_to_location",
  };
  for (const std::string & command_type :
    command_types)
  {
    const std::string command_id =
      "bridge-node-" + command_type;
    const std::string request =
      command_request(
      command_id,
      command_type).dump() + "\n";
    expect_dry_run_acknowledgement(
      socket_exchange(socket_path, request),
      command_id);
  }

  for (int index = 0; index < 2; ++index) {
    const std::string command_id =
      "bridge-node-repeated-stop-" +
      std::to_string(index);
    expect_dry_run_acknowledgement(
      socket_exchange(
        socket_path,
        command_request(
          command_id, "stop").dump() + "\n"),
      command_id);
  }
}

TEST(BridgeNodeCommandServer, InvalidClientsDoNotStopWorker)
{
  const RclcppGuard guard;
  const TemporaryDirectory temporary;
  const auto socket_path =
    temporary.path() / "command.sock";
  auto node =
    std::make_shared<savo_bridge::BridgeNode>(
    enabled_options(socket_path));
  ASSERT_TRUE(wait_for_socket(socket_path));

  const Json invalid_response = Json::parse(
    socket_exchange(socket_path, "{}\n"));
  EXPECT_EQ(
    invalid_response.at("message_type"),
    "command_error");

  const std::string first =
    command_request(
    "bridge-node-multiple-1",
    "stop").dump();
  const std::string second =
    command_request(
    "bridge-node-multiple-2",
    "stop").dump();
  const Json multiple_response = Json::parse(
    socket_exchange(
      socket_path,
      first + "\n" + second + "\n"));
  EXPECT_EQ(
    multiple_response.at("message_type"),
    "command_error");

  const std::string command_id =
    "bridge-node-after-invalid";
  expect_dry_run_acknowledgement(
    socket_exchange(
      socket_path,
      command_request(
        command_id, "stop").dump() + "\n"),
    command_id);
  EXPECT_TRUE(node->command_worker_joinable());
}

TEST(BridgeNodeCommandServer, PeerUidPolicyIsEnforced)
{
  const RclcppGuard guard;
  const TemporaryDirectory temporary;
  const auto socket_path =
    temporary.path() / "command.sock";
  const std::int64_t disallowed_uid =
    ::geteuid() == 0 ? 1 : 0;
  auto node =
    std::make_shared<savo_bridge::BridgeNode>(
    enabled_options(
      socket_path,
      {disallowed_uid}));
  ASSERT_TRUE(wait_for_socket(socket_path));

  // Peer credentials are checked immediately after accept and before
  // request bytes are read. Do not race the expected early close by
  // attempting to write a command from the unauthorized client.
  const std::string response =
    socket_exchange(socket_path, "");
  EXPECT_TRUE(response.empty());
  EXPECT_TRUE(node->command_worker_joinable());
}

TEST(BridgeNodeCommandServer, ReplacedSocketPathIsPreserved)
{
  const RclcppGuard guard;
  const TemporaryDirectory temporary;
  const auto socket_path =
    temporary.path() / "command.sock";

  {
    auto node =
      std::make_shared<savo_bridge::BridgeNode>(
      enabled_options(socket_path));
    ASSERT_TRUE(wait_for_socket(socket_path));
    ASSERT_EQ(::unlink(socket_path.c_str()), 0);
    std::ofstream replacement(socket_path);
    ASSERT_TRUE(replacement.is_open());
    replacement << "replacement";
  }

  EXPECT_TRUE(
    std::filesystem::is_regular_file(
      socket_path));
}

TEST(BridgeNodeCommandServer, NoCommandRosInterfacesAreCreated)
{
  const RclcppGuard guard;
  auto node =
    std::make_shared<savo_bridge::BridgeNode>();

  const auto graph =
    node->get_node_graph_interface();
  const auto publishers =
    graph->get_publisher_names_and_types_by_node(
    node->get_name(),
    node->get_namespace());
  const auto subscribers =
    graph->get_subscriber_names_and_types_by_node(
    node->get_name(),
    node->get_namespace());

  const std::vector<std::string_view> forbidden{
    "cmd_vel",
    "teleop",
    "navigate",
    "navigation",
    "goal",
    "stop",
    "cancel",
    "dispatch",
    "motor",
  };
  for (const auto & interfaces :
    {publishers, subscribers})
  {
    for (const auto & entry : interfaces) {
      for (const std::string_view token : forbidden) {
        EXPECT_EQ(
          entry.first.find(token),
          std::string::npos);
      }
    }
  }
}

}  // namespace
