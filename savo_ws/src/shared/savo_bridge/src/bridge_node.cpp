// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/bridge_node.hpp"

#include <sys/types.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/serialized_message.hpp>

#include "savo_bridge/health_evaluator.hpp"
#include "savo_bridge/observation_config.hpp"
#include "savo_bridge/qos_profiles.hpp"
#include "savo_bridge/ros_graph_discovery.hpp"

namespace savo_bridge
{
namespace
{

void append_json_string(
  std::string & output,
  const std::string_view value)
{
  constexpr char hexadecimal[] = "0123456789abcdef";

  output.push_back('"');

  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output += "\\\"";
        break;
      case '\\':
        output += "\\\\";
        break;
      case '\b':
        output += "\\b";
        break;
      case '\f':
        output += "\\f";
        break;
      case '\n':
        output += "\\n";
        break;
      case '\r':
        output += "\\r";
        break;
      case '\t':
        output += "\\t";
        break;
      default:
        if (character < 0x20U) {
          output += "\\u00";
          output.push_back(
            hexadecimal[(character >> 4U) & 0x0FU]);

          output.push_back(
            hexadecimal[character & 0x0FU]);
        } else {
          output.push_back(
            static_cast<char>(character));
        }
        break;
    }
  }

  output.push_back('"');
}

void append_boolean(
  std::string & output,
  const bool value)
{
  output += value ? "true" : "false";
}

template<typename Integer>
void append_integer(
  std::string & output,
  const Integer value)
{
  output += std::to_string(value);
}

void append_string_array(
  std::string & output,
  const std::vector<std::string> & values)
{
  output.push_back('[');

  for (std::size_t index = 0U;
    index < values.size();
    ++index)
  {
    if (index != 0U) {
      output.push_back(',');
    }

    append_json_string(
      output,
      values.at(index));
  }

  output.push_back(']');
}

struct SnapshotPublicationState
{
  bool enabled{false};
  std::string path;

  std::uint64_t sequence{0U};
  std::uint64_t success_count{0U};
  std::uint64_t failure_count{0U};

  std::size_t bytes_written{0U};

  bool replaced_existing{false};
  bool last_write_ok{false};

  std::string error;
};

struct ObservationRuntimeState
{
  std::size_t configured{0U};
  std::size_t active{0U};

  std::vector<std::string> active_topics;
  std::vector<std::string> error_topics;

  std::string runtime_error;
};

[[nodiscard]] std::string make_state_json(
  const BridgeRuntimeSnapshot & bridge,
  const GraphEvidence & evidence,
  const std::string & graph_error,
  const SnapshotPublicationState & snapshot,
  const ObservationRuntimeState & observations,
  const HealthEvaluation & health)
{
  std::string output;
  output.reserve(2048U);

  output += "{\"schema_name\":\"savo_bridge_state\"";
  output += ",\"schema_version\":2";
  output += ",\"owner\":";
  append_json_string(output, bridge.owner);
  output += ",\"instance_id\":";
  append_json_string(output, bridge.instance_id);
  output += ",\"phase\":\"production_runtime\"";
  output += ",\"read_only\":";
  append_boolean(output, bridge.read_only);
  output += ",\"commands_enabled\":";
  append_boolean(output, bridge.commands_enabled);
  output += ",\"process_alive\":";
  append_boolean(output, bridge.process_alive);
  output += ",\"bridge_ready\":";
  append_boolean(output, bridge.bridge_ready);
  output += ",\"validated\":";
  append_boolean(output, bridge.validated);
  output += ",\"dispatch_enabled\":";
  append_boolean(output, bridge.dispatch_enabled);
  output += ",\"navigation_bridge_validated\":";
  append_boolean(output, bridge.navigation_bridge_validated);
  output += ",\"block_navigation\":";
  append_boolean(output, bridge.block_navigation);
  output += ",\"readiness_reason\":";
  append_json_string(output, bridge.readiness_reason);

  output += ",\"dds_active\":";
  append_boolean(output, evidence.dds_active);

  output += ",\"core_evidence_configured\":";
  append_boolean(
    output,
    evidence.core_evidence_configured);

  output += ",\"edge_evidence_configured\":";
  append_boolean(
    output,
    evidence.edge_evidence_configured);

  output += ",\"core_visible\":";
  append_boolean(output, evidence.core_visible);

  output += ",\"edge_visible\":";
  append_boolean(output, evidence.edge_visible);


  output += ",\"discovered_nodes\":";
  append_integer(output, evidence.discovered_nodes);

  output += ",\"discovered_topics\":";
  append_integer(output, evidence.discovered_topics);

  output += ",\"external_nodes\":";
  append_integer(output, evidence.external_nodes);

  output += ",\"external_topics\":";
  append_integer(output, evidence.external_topics);

  output += ",\"visible_owned_topics\":";
  append_string_array(
    output,
    evidence.visible_owned_topics);

  output += ",\"missing_owned_topics\":";
  append_string_array(
    output,
    evidence.missing_owned_topics);

  output += ",\"matched_core_nodes\":";
  append_string_array(
    output,
    evidence.matched_core_nodes);

  output += ",\"matched_core_topics\":";
  append_string_array(
    output,
    evidence.matched_core_topics);

  output += ",\"matched_edge_nodes\":";
  append_string_array(
    output,
    evidence.matched_edge_nodes);

  output += ",\"matched_edge_topics\":";
  append_string_array(
    output,
    evidence.matched_edge_topics);

  output += ",\"command_server_enabled\":";
  append_boolean(output, bridge.command_server_enabled);
  output += ",\"command_execution_mode\":";
  append_json_string(output, bridge.command_execution_mode);
  output += ",\"command_worker_fatal\":";
  append_boolean(output, bridge.command_worker_fatal);
  output += ",\"command_last_status\":";
  append_json_string(output, bridge.command_last_status);
  output += ",\"stop_ready\":";
  append_boolean(output, bridge.stop_ready);
  output += ",\"teleop_ready\":";
  append_boolean(output, bridge.teleop_ready);
  output += ",\"navigation_ready\":";
  append_boolean(output, bridge.navigation_ready);

  output += ",\"graph_error\":";
  append_json_string(output, graph_error);

  output += ",\"observation_topics_configured\":";
  append_integer(output, observations.configured);

  output += ",\"observation_subscriptions_active\":";
  append_integer(output, observations.active);

  output += ",\"observation_active_topics\":";
  append_string_array(
    output,
    observations.active_topics);

  output += ",\"observation_error_topics\":";
  append_string_array(
    output,
    observations.error_topics);

  output += ",\"observation_runtime_error\":";
  append_json_string(
    output,
    observations.runtime_error);

  output += ",\"observation_health_status\":";
  append_json_string(
    output,
    to_string(health.status));

  output += ",\"observation_health_reason\":";
  append_json_string(
    output,
    to_string(health.reason));

  output += ",\"required_topics_ready\":";
  append_boolean(
    output,
    health.required_topics_ready);

  output += ",\"all_topics_fresh\":";
  append_boolean(
    output,
    health.all_topics_fresh);

  output += ",\"snapshot_enabled\":";
  append_boolean(output, snapshot.enabled);

  output += ",\"snapshot_path\":";
  append_json_string(output, snapshot.path);

  output += ",\"snapshot_sequence\":";
  append_integer(output, snapshot.sequence);

  output += ",\"snapshot_success_count\":";
  append_integer(output, snapshot.success_count);

  output += ",\"snapshot_failure_count\":";
  append_integer(output, snapshot.failure_count);

  output += ",\"snapshot_bytes_written\":";
  append_integer(output, snapshot.bytes_written);

  output += ",\"snapshot_replaced_existing\":";
  append_boolean(
    output,
    snapshot.replaced_existing);

  output += ",\"snapshot_last_write_ok\":";
  append_boolean(
    output,
    snapshot.last_write_ok);

  output += ",\"snapshot_error\":";
  append_json_string(output, snapshot.error);

  output.push_back('}');
  return output;
}

[[nodiscard]] diagnostic_msgs::msg::KeyValue make_key_value(
  std::string key,
  std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

[[nodiscard]] std::string boolean_string(
  const bool value)
{
  return value ? "true" : "false";
}

[[nodiscard]] rclcpp::QoS observation_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));

  qos.best_effort();
  qos.durability_volatile();

  return qos;
}

template<typename Value>
[[nodiscard]] Value declare_read_only_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const Value & default_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return node.declare_parameter<Value>(
    name,
    default_value,
    descriptor);
}

[[nodiscard]] bool is_fatal_command_server_status(
  const CommandServerStatus status) noexcept
{
  return status == CommandServerStatus::NotRunning ||
         status == CommandServerStatus::AcceptError ||
         status == CommandServerStatus::SocketError;
}

[[nodiscard]] bool observation_is_fresh(
  const bool observed,
  const std::int64_t age_ms,
  const std::int64_t timeout_ms) noexcept
{
  return observed &&
         age_ms >= 0 &&
         timeout_ms > 0 &&
         age_ms <= timeout_ms;
}

[[nodiscard]] std::string uppercase_ascii(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](const unsigned char character) {
      return static_cast<char>(std::toupper(character));
    });

  return value;
}

}  // namespace

BridgeNode::BridgeNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node(
    "savo_bridge_node",
    "/savo_bridge",
    options)
{
  configure_command_server();

  readiness_publisher_ =
    create_publisher<std_msgs::msg::Bool>(
    "/savo_bridge/readiness",
    bridge_latched_qos());

  state_publisher_ =
    create_publisher<std_msgs::msg::String>(
    "/savo_bridge/state",
    bridge_latched_qos());

  heartbeat_publisher_ =
    create_publisher<std_msgs::msg::UInt64>(
    "/savo_bridge/heartbeat",
    bridge_stream_qos());

  diagnostics_publisher_ =
    create_publisher<
    diagnostic_msgs::msg::DiagnosticArray>(
    "/savo_bridge/diagnostics",
    bridge_stream_qos());

  graph_config_.local_node_fully_qualified_name =
    get_fully_qualified_name();

  graph_config_.owned_topic_names = {
    "/savo_bridge/diagnostics",
    "/savo_bridge/heartbeat",
    "/savo_bridge/readiness",
    "/savo_bridge/state",
  };

  graph_config_.core_node_names =
    declare_parameter<std::vector<std::string>>(
    "core_evidence_nodes",
    std::vector<std::string>{});

  graph_config_.core_topic_names =
    declare_parameter<std::vector<std::string>>(
    "core_evidence_topics",
    std::vector<std::string>{});

  graph_config_.edge_node_names =
    declare_parameter<std::vector<std::string>>(
    "edge_evidence_nodes",
    std::vector<std::string>{});

  graph_config_.edge_topic_names =
    declare_parameter<std::vector<std::string>>(
    "edge_evidence_topics",
    std::vector<std::string>{});

  ObservationParameterSet observation_parameters;

  observation_parameters.topic_names =
    declare_parameter<std::vector<std::string>>(
    "observation_topics",
    std::vector<std::string>{});

  observation_parameters.requirements =
    declare_parameter<std::vector<std::string>>(
    "observation_requirements",
    std::vector<std::string>{});

  observation_parameters.stale_after_ms =
    declare_parameter<std::vector<std::int64_t>>(
    "observation_stale_after_ms",
    std::vector<std::int64_t>{});

  const std::vector<TopicObservationConfig>
  observation_configs = make_observation_configs(
    observation_parameters,
    graph_config_.owned_topic_names);

  observation_runtimes_.reserve(
    observation_configs.size());

  for (const TopicObservationConfig & config :
    observation_configs)
  {
    ObservationRuntime runtime;
    runtime.topic_name = config.topic_name;

    runtime.observation =
      std::make_unique<TopicObservation>(config);

    runtime.error = "topic_type_unresolved";

    observation_runtimes_.push_back(
      std::move(runtime));
  }

  snapshot_enabled_ =
    declare_parameter<bool>(
    "snapshot_enabled",
    false);

  snapshot_path_ =
    declare_parameter<std::string>(
    "snapshot_path",
    "/run/savo_bridge/snapshot.json");

  snapshot_writer_ =
    std::make_unique<AtomicSnapshotWriter>(
    snapshot_path_);

  status_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      publish_status();
    });

  start_command_server();

  RCLCPP_INFO(
    get_logger(),
    "savo_bridge topic observation started; "
    "configured_topics=%zu snapshot_enabled=%s "
    "command_mode=%s",
    observation_runtimes_.size(),
    snapshot_enabled_ ? "true" : "false",
    command_execution_mode_.c_str());
}

BridgeNode::~BridgeNode()
{
  command_stop_requested_.store(
    true,
    std::memory_order_release);

  if (command_server_) {
    (void)command_server_->stop();
  }

  if (command_worker_.joinable()) {
    command_worker_.join();
  }

  command_server_.reset();

  if (ros_command_dispatcher_) {
    ros_command_dispatcher_->shutdown();
    ros_command_dispatcher_.reset();
  }
}

bool BridgeNode::command_server_enabled() const noexcept
{
  return command_server_enabled_;
}

bool BridgeNode::command_worker_joinable() const noexcept
{
  return command_worker_.joinable();
}

void BridgeNode::configure_command_server()
{
  command_server_enabled_ =
    declare_read_only_parameter<bool>(
    *this,
    "command_server.enabled",
    false);

  const std::string execution_mode =
    declare_read_only_parameter<std::string>(
    *this,
    "command_server.execution_mode",
    "dry_run");

  const std::string socket_path =
    declare_read_only_parameter<std::string>(
    *this,
    "command_server.socket_path",
    "/run/savo_bridge/command.sock");

  const std::string bridge_instance_id =
    declare_read_only_parameter<std::string>(
    *this,
    "command_server.bridge_instance_id",
    "savo-bridge-native");

  const std::int64_t max_request_bytes =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_server.max_request_bytes",
    65536);

  const std::int64_t max_response_bytes =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_server.max_response_bytes",
    65536);

  const std::int64_t accept_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_server.accept_timeout_ms",
    250);

  const std::int64_t client_read_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_server.client_read_timeout_ms",
    1000);

  const std::int64_t client_write_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_server.client_write_timeout_ms",
    1000);

  const std::int64_t socket_mode =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_server.socket_mode",
    432);

  const std::int64_t command_id_cache_capacity =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_server.command_id_cache_capacity",
    1024);

  const std::int64_t socket_gid =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_server.socket_gid",
    -1);

  const std::vector<std::int64_t> allowed_peer_uids =
    declare_read_only_parameter<std::vector<std::int64_t>>(
    *this,
    "command_server.allowed_peer_uids",
    std::vector<std::int64_t>{});

  command_execution_mode_ =
    command_server_enabled_ ?
    execution_mode :
    "disabled";

  if (!command_server_enabled_) {
    command_last_status_.store(
      CommandServerStatus::Disabled,
      std::memory_order_release);
    return;
  }

  const auto invalid_configuration = [this]() {
      RCLCPP_ERROR(
        get_logger(),
        "command server configuration rejected");
      throw std::invalid_argument(
              "command server configuration invalid");
    };

  if (
    (
      execution_mode != "dry_run" &&
      execution_mode != "live"
    ) ||
    max_request_bytes <= 0 ||
    max_response_bytes <= 0 ||
    command_id_cache_capacity <= 0 ||
    static_cast<std::uintmax_t>(max_request_bytes) >
    std::numeric_limits<std::size_t>::max() ||
    static_cast<std::uintmax_t>(max_response_bytes) >
    std::numeric_limits<std::size_t>::max() ||
    static_cast<std::uintmax_t>(
      command_id_cache_capacity) >
    std::numeric_limits<std::size_t>::max() ||
    socket_mode < 0 ||
    static_cast<std::uintmax_t>(socket_mode) >
    std::numeric_limits<std::uint32_t>::max() ||
    socket_gid < -1 ||
    (
      socket_gid >= 0 &&
      static_cast<std::uintmax_t>(socket_gid) >
      std::numeric_limits<gid_t>::max()
    ))
  {
    invalid_configuration();
  }

  CommandServerConfig config;
  config.enabled = true;
  config.socket_path = socket_path;
  config.bridge_instance_id = bridge_instance_id;
  config.execution_mode = execution_mode;
  config.max_request_bytes =
    static_cast<std::size_t>(max_request_bytes);
  config.max_response_bytes =
    static_cast<std::size_t>(max_response_bytes);
  config.command_id_cache_capacity =
    static_cast<std::size_t>(
    command_id_cache_capacity);
  config.accept_timeout_ms = accept_timeout_ms;
  config.client_read_timeout_ms =
    client_read_timeout_ms;
  config.client_write_timeout_ms =
    client_write_timeout_ms;
  config.socket_mode =
    static_cast<std::uint32_t>(socket_mode);

  if (socket_gid >= 0) {
    config.socket_gid =
      static_cast<std::uint32_t>(socket_gid);
  }

  if (!allowed_peer_uids.empty()) {
    std::set<std::uint32_t> unique_uids;
    config.allowed_peer_uids.clear();
    config.allowed_peer_uids.reserve(
      allowed_peer_uids.size());

    for (const std::int64_t uid : allowed_peer_uids) {
      if (
        uid < 0 ||
        static_cast<std::uintmax_t>(uid) >
        std::numeric_limits<uid_t>::max())
      {
        invalid_configuration();
      }

      const auto converted =
        static_cast<std::uint32_t>(uid);

      if (!unique_uids.insert(converted).second) {
        invalid_configuration();
      }

      config.allowed_peer_uids.push_back(converted);
    }
  }

  command_server_config_ =
    std::make_unique<CommandServerConfig>(
    std::move(config));

  if (execution_mode != "live") {
    return;
  }

  RosCommandDispatcherConfig dispatcher_config;

  dispatcher_config.mode_command_topic =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.mode_command_topic",
    dispatcher_config.mode_command_topic);

  dispatcher_config.mode_state_topic =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.mode_state_topic",
    dispatcher_config.mode_state_topic);

  dispatcher_config.external_stop_topic =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.external_stop_topic",
    dispatcher_config.external_stop_topic);

  dispatcher_config.safety_stop_topic =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.safety_stop_topic",
    dispatcher_config.safety_stop_topic);

  dispatcher_config.manual_velocity_topic =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.manual_velocity_topic",
    dispatcher_config.manual_velocity_topic);

  dispatcher_config.safe_velocity_topic =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.safe_velocity_topic",
    dispatcher_config.safe_velocity_topic);

  dispatcher_config.navigation_readiness_topic =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.navigation_readiness_topic",
    dispatcher_config.navigation_readiness_topic);

  dispatcher_config.map_context_status_topic =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.map_context_status_topic",
    dispatcher_config.map_context_status_topic);

  dispatcher_config.navigation_action_name =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.navigation_action_name",
    dispatcher_config.navigation_action_name);

  dispatcher_config.location_resolve_service =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.location_resolve_service",
    dispatcher_config.location_resolve_service);

  dispatcher_config.mapping_action_name =
    declare_read_only_parameter<std::string>(
    *this, "command_dispatcher.mapping_action_name",
    dispatcher_config.mapping_action_name);
  dispatcher_config.mapping_control_service =
    declare_read_only_parameter<std::string>(
    *this, "command_dispatcher.mapping_control_service",
    dispatcher_config.mapping_control_service);
  dispatcher_config.mapping_status_topic =
    declare_read_only_parameter<std::string>(
    *this, "command_dispatcher.mapping_status_topic",
    dispatcher_config.mapping_status_topic);
  dispatcher_config.supervisor_state_topic =
    declare_read_only_parameter<std::string>(
    *this, "command_dispatcher.supervisor_state_topic",
    dispatcher_config.supervisor_state_topic);

  dispatcher_config.active_map_id =
    declare_read_only_parameter<std::string>(
    *this,
    "command_dispatcher.active_map_id",
    dispatcher_config.active_map_id);

  const std::int64_t active_map_revision =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.active_map_revision",
    0);

  dispatcher_config.require_active_map_context =
    declare_read_only_parameter<bool>(
    *this,
    "command_dispatcher.require_active_map_context",
    dispatcher_config.require_active_map_context);

  dispatcher_config.observed_state_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.observed_state_timeout_ms",
    dispatcher_config.observed_state_timeout_ms);

  dispatcher_config.mode_transition_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.mode_transition_timeout_ms",
    dispatcher_config.mode_transition_timeout_ms);

  dispatcher_config.stop_confirmation_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.stop_confirmation_timeout_ms",
    dispatcher_config.stop_confirmation_timeout_ms);

  dispatcher_config.location_service_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.location_service_timeout_ms",
    dispatcher_config.location_service_timeout_ms);

  dispatcher_config.navigation_server_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.navigation_server_timeout_ms",
    dispatcher_config.navigation_server_timeout_ms);

  dispatcher_config.navigation_goal_response_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.navigation_goal_response_timeout_ms",
    dispatcher_config.navigation_goal_response_timeout_ms);

  dispatcher_config.navigation_execution_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.navigation_execution_timeout_ms",
    dispatcher_config.navigation_execution_timeout_ms);

  dispatcher_config.teleop_cancel_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.teleop_cancel_timeout_ms",
    dispatcher_config.teleop_cancel_timeout_ms);

  dispatcher_config.navigation_cancel_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this,
    "command_dispatcher.navigation_cancel_timeout_ms",
    dispatcher_config.navigation_cancel_timeout_ms);

  dispatcher_config.mapping_server_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this, "command_dispatcher.mapping_server_timeout_ms",
    dispatcher_config.mapping_server_timeout_ms);
  dispatcher_config.mapping_control_timeout_ms =
    declare_read_only_parameter<std::int64_t>(
    *this, "command_dispatcher.mapping_control_timeout_ms",
    dispatcher_config.mapping_control_timeout_ms);

  if (
    active_map_revision < 0 ||
    static_cast<std::uintmax_t>(
      active_map_revision) >
    std::numeric_limits<std::uint32_t>::max())
  {
    invalid_configuration();
  }

  dispatcher_config.active_map_revision =
    static_cast<std::uint32_t>(
    active_map_revision);

  ros_command_dispatcher_config_ =
    std::make_unique<RosCommandDispatcherConfig>(
    std::move(dispatcher_config));
}


void BridgeNode::start_command_server()
{
  if (!command_server_enabled_) {
    return;
  }

  CommandServer::Dispatcher dispatcher;

  if (command_execution_mode_ == "live") {
    if (!ros_command_dispatcher_config_) {
      throw std::runtime_error(
              "live command dispatcher configuration missing");
    }

    ros_command_dispatcher_ =
      std::make_unique<RosCommandDispatcher>(
      *this,
      *ros_command_dispatcher_config_);

    dispatcher =
      [this](const ValidatedCommand & command)
      {
        return ros_command_dispatcher_->dispatch(
          command);
      };
  }

  command_server_ =
    std::make_unique<CommandServer>(
    *command_server_config_,
    CommandServer::Clock{},
    std::move(dispatcher));

  const CommandServerResult start_result =
    command_server_->start();
  command_last_status_.store(
    start_result.status,
    std::memory_order_release);

  if (start_result.status != CommandServerStatus::Started) {
    command_server_.reset();
    RCLCPP_ERROR(
      get_logger(),
      "command server startup failed: %s",
      start_result.reason.c_str());
    throw std::runtime_error(
            "command server startup failed");
  }

  command_stop_requested_.store(
    false,
    std::memory_order_release);

  try {
    command_worker_ =
      std::thread(
      &BridgeNode::run_command_worker,
      this);
  } catch (const std::system_error &) {
    (void)command_server_->stop();
    command_server_.reset();
    throw std::runtime_error(
            "command server worker startup failed");
  }

  RCLCPP_INFO(
    get_logger(),
    "command server enabled; execution_mode=%s",
    command_execution_mode_.c_str());
}

void BridgeNode::run_command_worker() noexcept
{
  try {
    while (!command_stop_requested_.load(
        std::memory_order_acquire))
    {
      const CommandServerResult result =
        command_server_->serve_one();

      command_last_status_.store(
        result.status,
        std::memory_order_release);

      if (result.status ==
        CommandServerStatus::AcceptTimeout)
      {
        continue;
      }

      if (
        result.status ==
        CommandServerStatus::DryRunAcknowledged ||
        result.status ==
        CommandServerStatus::CommandAcknowledged)
      {
        command_accepted_count_.fetch_add(
          1U,
          std::memory_order_relaxed);
      } else {
        command_rejected_count_.fetch_add(
          1U,
          std::memory_order_relaxed);
      }

      if (is_fatal_command_server_status(
          result.status))
      {
        command_worker_fatal_.store(
          true,
          std::memory_order_release);
        return;
      }
    }
  } catch (...) {
    command_worker_fatal_.store(
      true,
      std::memory_order_release);
  }
}

void BridgeNode::refresh_observation_subscriptions()
{
  const auto graph_interface =
    get_node_graph_interface();

  const auto topic_names_and_types =
    graph_interface->get_topic_names_and_types();

  for (ObservationRuntime & runtime :
    observation_runtimes_)
  {
    const auto iterator =
      topic_names_and_types.find(runtime.topic_name);

    if (iterator == topic_names_and_types.end()) {
      if (!runtime.subscription) {
        runtime.error = "topic_type_unresolved";
      }

      continue;
    }

    std::vector<std::string> topic_types =
      iterator->second;

    std::sort(
      topic_types.begin(),
      topic_types.end());

    topic_types.erase(
      std::unique(
        topic_types.begin(),
        topic_types.end()),
      topic_types.end());

    if (topic_types.size() != 1U) {
      runtime.subscription.reset();
      runtime.topic_type.clear();

      runtime.error =
        topic_types.empty() ?
        "topic_type_unresolved" :
        "topic_type_ambiguous";

      continue;
    }

    const std::string & topic_type =
      topic_types.front();

    if (topic_type.empty()) {
      runtime.subscription.reset();
      runtime.topic_type.clear();
      runtime.error = "topic_type_empty";
      continue;
    }

    if (
      runtime.subscription &&
      runtime.topic_type == topic_type)
    {
      runtime.error.clear();
      continue;
    }

    runtime.subscription.reset();
    runtime.topic_type.clear();

    TopicObservation * const observation =
      runtime.observation.get();

    std::function<void(
        std::shared_ptr<rclcpp::SerializedMessage>)>
    callback =
      [observation](
      std::shared_ptr<rclcpp::SerializedMessage>)
      {
        (void)observation->observe(
          TopicObservation::Clock::now());
      };

    try {
      runtime.subscription =
        create_generic_subscription(
        runtime.topic_name,
        topic_type,
        observation_qos(),
        std::move(callback));

      runtime.topic_type = topic_type;
      runtime.error.clear();
    } catch (const std::exception & exception) {
      runtime.subscription.reset();
      runtime.topic_type.clear();

      runtime.error =
        std::string("subscription_creation_failed:") +
        exception.what();
    }
  }
}

std::vector<TopicObservation::Snapshot>
BridgeNode::collect_observation_snapshots(
  const TopicObservation::TimePoint evaluated_at) const
{
  std::vector<TopicObservation::Snapshot> result;

  result.reserve(observation_runtimes_.size());

  for (const ObservationRuntime & runtime :
    observation_runtimes_)
  {
    result.push_back(
      runtime.observation->snapshot(evaluated_at));
  }

  return result;
}

void BridgeNode::publish_runtime_snapshot(
  const BridgeRuntimeSnapshot & bridge_runtime,
  const std::vector<TopicObservation::Snapshot> &
  topic_snapshots)
{
  snapshot_last_write_ok_ = false;
  snapshot_replaced_existing_ = false;
  snapshot_bytes_written_ = 0U;
  snapshot_error_.clear();

  if (!snapshot_enabled_) {
    return;
  }

  SnapshotDocument document;
  document.sequence = snapshot_sequence_ + 1U;
  document.bridge = bridge_runtime;
  document.topics = topic_snapshots;

  try {
    const SnapshotWriteResult result =
      snapshot_writer_->write(document);

    snapshot_sequence_ = document.sequence;
    ++snapshot_success_count_;

    snapshot_bytes_written_ = result.bytes_written;
    snapshot_replaced_existing_ =
      result.replaced_existing;

    snapshot_last_write_ok_ = true;
  } catch (const std::exception & exception) {
    ++snapshot_failure_count_;
    snapshot_error_ = exception.what();
  }
}

void BridgeNode::publish_status()
{
  GraphEvidence evidence;
  std::string graph_error;
  std::string observation_runtime_error;

  try {
    refresh_observation_subscriptions();
  } catch (const std::exception & exception) {
    observation_runtime_error = exception.what();
  }

  try {
    const GraphSnapshot snapshot =
      collect_ros_graph_snapshot(*this);

    const GraphEvidenceEvaluator evaluator;

    evidence = evaluator.evaluate(
      graph_config_,
      snapshot);
  } catch (const std::exception & exception) {
    graph_error = exception.what();
  }

  const TopicObservation::TimePoint evaluated_at =
    TopicObservation::Clock::now();

  const std::vector<TopicObservation::Snapshot>
  topic_snapshots =
    collect_observation_snapshots(evaluated_at);

  const HealthEvaluator health_evaluator;

  const HealthEvaluation health =
    health_evaluator.evaluate(topic_snapshots);

  ObservationRuntimeState observation_state;
  observation_state.configured =
    observation_runtimes_.size();

  observation_state.runtime_error =
    observation_runtime_error;

  for (const ObservationRuntime & runtime :
    observation_runtimes_)
  {
    if (runtime.subscription) {
      ++observation_state.active;

      observation_state.active_topics.push_back(
        runtime.topic_name);
    }

    if (!runtime.error.empty()) {
      observation_state.error_topics.push_back(
        runtime.topic_name);
    }
  }

  const bool subscriptions_complete =
    observation_state.configured > 0U &&
    observation_state.active == observation_state.configured &&
    observation_runtime_error.empty();

  RosCommandDispatcherSnapshot dispatcher_snapshot;
  const bool dispatcher_available =
    static_cast<bool>(ros_command_dispatcher_);

  if (dispatcher_available) {
    dispatcher_snapshot =
      ros_command_dispatcher_->snapshot();
  }

  BridgeRuntimeSnapshot bridge_runtime;

  if (command_server_config_) {
    bridge_runtime.instance_id =
      command_server_config_->bridge_instance_id;
  }

  bridge_runtime.command_server_enabled =
    command_server_enabled_;
  bridge_runtime.command_execution_mode =
    command_execution_mode_;
  bridge_runtime.command_worker_fatal =
    command_worker_fatal_.load(
    std::memory_order_acquire);
  bridge_runtime.command_last_status =
    to_string(
    command_last_status_.load(
      std::memory_order_acquire));
  bridge_runtime.command_accepted_count =
    command_accepted_count_.load(
    std::memory_order_relaxed);
  bridge_runtime.command_rejected_count =
    command_rejected_count_.load(
    std::memory_order_relaxed);

  bridge_runtime.dds_active = evidence.dds_active;
  bridge_runtime.core_visible = evidence.core_visible;
  bridge_runtime.edge_visible = evidence.edge_visible;
  bridge_runtime.observation_subscriptions_complete =
    subscriptions_complete;
  bridge_runtime.required_topics_ready =
    health.required_topics_ready;
  bridge_runtime.all_topics_fresh =
    health.all_topics_fresh;

  bridge_runtime.location_service_configured =
    ros_command_dispatcher_config_ &&
    !ros_command_dispatcher_config_->
    location_resolve_service.empty();

  bridge_runtime.active_map_context_configured =
    ros_command_dispatcher_config_ &&
    (
    !ros_command_dispatcher_config_->
    require_active_map_context ||
    (
      !ros_command_dispatcher_config_->active_map_id.empty() &&
      ros_command_dispatcher_config_->active_map_revision > 0U
    ) ||
    (
      dispatcher_snapshot.map_context_synchronized &&
      observation_is_fresh(
        dispatcher_snapshot.map_context_observed,
        dispatcher_snapshot.map_context_age_ms,
        ros_command_dispatcher_config_->observed_state_timeout_ms) &&
      !dispatcher_snapshot.active_map_id.empty() &&
      dispatcher_snapshot.active_map_revision > 0U
    )
    );

  bridge_runtime.command_active =
    dispatcher_snapshot.command_active;
  bridge_runtime.teleop_active =
    dispatcher_snapshot.teleop_active;
  bridge_runtime.navigation_goal_active =
    dispatcher_snapshot.navigation_goal_active;
  bridge_runtime.active_command_id =
    dispatcher_snapshot.active_command_id;
  bridge_runtime.active_command_type =
    dispatcher_snapshot.active_command_type;
  bridge_runtime.last_terminal_command_id =
    dispatcher_snapshot.last_terminal_command_id;
  bridge_runtime.mode_state_observed =
    dispatcher_snapshot.mode_state_observed;
  bridge_runtime.mode_state = dispatcher_snapshot.mode_state;
  bridge_runtime.mode_state_age_ms =
    dispatcher_snapshot.mode_state_age_ms;
  bridge_runtime.external_stop_state_known =
    dispatcher_snapshot.external_stop_observed;
  bridge_runtime.external_stop_active =
    dispatcher_snapshot.external_stop_active;
  bridge_runtime.external_stop_age_ms =
    dispatcher_snapshot.external_stop_age_ms;
  bridge_runtime.safety_stop_state_known =
    dispatcher_snapshot.safety_stop_observed;
  bridge_runtime.safety_stop_active =
    dispatcher_snapshot.safety_stop_active;
  bridge_runtime.safety_stop_age_ms =
    dispatcher_snapshot.safety_stop_age_ms;
  bridge_runtime.safe_velocity_state_known =
    dispatcher_snapshot.safe_velocity_observed;
  bridge_runtime.safe_velocity_zero =
    dispatcher_snapshot.safe_velocity_zero;
  bridge_runtime.safe_velocity_age_ms =
    dispatcher_snapshot.safe_velocity_age_ms;
  bridge_runtime.navigation_readiness_observed =
    dispatcher_snapshot.navigation_readiness_observed;
  bridge_runtime.navigation_readiness =
    dispatcher_snapshot.navigation_readiness;
  bridge_runtime.navigation_readiness_age_ms =
    dispatcher_snapshot.navigation_readiness_age_ms;
  bridge_runtime.dispatcher_accepted_command_count =
    dispatcher_snapshot.accepted_command_count;
  bridge_runtime.dispatcher_rejected_command_count =
    dispatcher_snapshot.rejected_command_count;
  bridge_runtime.dispatcher_ros_publication_count =
    dispatcher_snapshot.ros_publication_count;
  bridge_runtime.dispatcher_last_reason =
    dispatcher_snapshot.last_reason;

  const bool live_transport_healthy =
    command_server_enabled_ &&
    command_execution_mode_ == "live" &&
    !bridge_runtime.command_worker_fatal &&
    dispatcher_available &&
    !dispatcher_snapshot.shutdown_requested;

  bridge_runtime.stop_ready = live_transport_healthy;

  if (ros_command_dispatcher_config_) {
    const std::int64_t freshness_timeout_ms =
      ros_command_dispatcher_config_->
      observed_state_timeout_ms;

    const bool common_runtime_fresh =
      observation_is_fresh(
        dispatcher_snapshot.mode_state_observed,
        dispatcher_snapshot.mode_state_age_ms,
        freshness_timeout_ms) &&
      observation_is_fresh(
        dispatcher_snapshot.external_stop_observed,
        dispatcher_snapshot.external_stop_age_ms,
        freshness_timeout_ms) &&
      observation_is_fresh(
        dispatcher_snapshot.safety_stop_observed,
        dispatcher_snapshot.safety_stop_age_ms,
        freshness_timeout_ms) &&
      observation_is_fresh(
        dispatcher_snapshot.safe_velocity_observed,
        dispatcher_snapshot.safe_velocity_age_ms,
        freshness_timeout_ms);

    bridge_runtime.teleop_ready =
      live_transport_healthy &&
      !dispatcher_snapshot.command_active &&
      common_runtime_fresh &&
      !dispatcher_snapshot.external_stop_active &&
      !dispatcher_snapshot.safety_stop_active &&
      dispatcher_snapshot.safe_velocity_zero;

    bridge_runtime.navigation_ready =
      bridge_runtime.teleop_ready &&
      observation_is_fresh(
        dispatcher_snapshot.navigation_readiness_observed,
        dispatcher_snapshot.navigation_readiness_age_ms,
        freshness_timeout_ms) &&
      dispatcher_snapshot.navigation_readiness ==
      uppercase_ascii(
        ros_command_dispatcher_config_->navigation_ready_state);
  }

  // The bridge's local node plus all bridge-owned topics are already required
  // by dds_active. An additional Edge selector is optional; requiring Edge
  // bringup readiness here would create Bridge -> Readiness -> Bridge.
  const bool selectors_configured =
    evidence.core_evidence_configured;

  const bool edge_presence_ready =
    !evidence.edge_evidence_configured ||
    evidence.edge_visible;

  const bool production_dependencies_ready =
    graph_error.empty() &&
    observation_runtime_error.empty() &&
    evidence.dds_active &&
    selectors_configured &&
    evidence.core_visible &&
    edge_presence_ready &&
    subscriptions_complete &&
    health.required_topics_ready &&
    snapshot_enabled_ &&
    bridge_runtime.location_service_configured &&
    bridge_runtime.active_map_context_configured;

  bridge_runtime.bridge_ready =
    live_transport_healthy &&
    production_dependencies_ready;
  bridge_runtime.validated = bridge_runtime.bridge_ready;
  bridge_runtime.dispatch_enabled = bridge_runtime.bridge_ready;
  bridge_runtime.commands_enabled = bridge_runtime.bridge_ready;
  bridge_runtime.read_only = !bridge_runtime.commands_enabled;
  bridge_runtime.navigation_bridge_validated =
    bridge_runtime.bridge_ready &&
    bridge_runtime.navigation_ready;
  bridge_runtime.block_navigation =
    !bridge_runtime.navigation_bridge_validated;

  if (!graph_error.empty()) {
    bridge_runtime.readiness_reason =
      "graph_discovery_failed";
  } else if (!observation_runtime_error.empty()) {
    bridge_runtime.readiness_reason =
      "observation_runtime_failed";
  } else if (!live_transport_healthy) {
    bridge_runtime.readiness_reason =
      "live_command_transport_not_ready";
  } else if (!snapshot_enabled_) {
    bridge_runtime.readiness_reason =
      "snapshot_publication_disabled";
  } else if (!evidence.dds_active) {
    bridge_runtime.readiness_reason =
      "local_dds_evidence_missing";
  } else if (!selectors_configured) {
    bridge_runtime.readiness_reason =
      "graph_evidence_selectors_unconfigured";
  } else if (!evidence.core_visible || !edge_presence_ready) {
    bridge_runtime.readiness_reason =
      "graph_evidence_incomplete";
  } else if (!subscriptions_complete) {
    bridge_runtime.readiness_reason =
      "observation_subscriptions_incomplete";
  } else if (!health.required_topics_ready) {
    bridge_runtime.readiness_reason =
      "required_topic_unavailable";
  } else if (!bridge_runtime.location_service_configured) {
    bridge_runtime.readiness_reason =
      "location_service_unconfigured";
  } else if (!bridge_runtime.active_map_context_configured) {
    bridge_runtime.readiness_reason =
      "active_map_context_unconfigured";
  } else {
    bridge_runtime.readiness_reason =
      "bridge_ready";
  }

  last_bridge_runtime_ = bridge_runtime;
  publish_runtime_snapshot(
    bridge_runtime,
    topic_snapshots);

  const SnapshotPublicationState snapshot_state{
    snapshot_enabled_,
    snapshot_path_,
    snapshot_sequence_,
    snapshot_success_count_,
    snapshot_failure_count_,
    snapshot_bytes_written_,
    snapshot_replaced_existing_,
    snapshot_last_write_ok_,
    snapshot_error_,
  };

  std_msgs::msg::Bool readiness_message;
  readiness_message.data =
    bridge_runtime.bridge_ready &&
    snapshot_last_write_ok_;
  readiness_publisher_->publish(readiness_message);

  std_msgs::msg::String state_message;

  state_message.data =
    make_state_json(
    bridge_runtime,
    evidence,
    graph_error,
    snapshot_state,
    observation_state,
    health);

  state_publisher_->publish(state_message);

  std_msgs::msg::UInt64 heartbeat_message;
  heartbeat_message.data = ++heartbeat_sequence_;
  heartbeat_publisher_->publish(heartbeat_message);

  diagnostic_msgs::msg::DiagnosticArray diagnostics_message;
  diagnostics_message.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/savo_bridge/runtime";
  status.hardware_id = "savo-edge";

  const bool evidence_complete =
    evidence.core_visible &&
    edge_presence_ready;

  const bool snapshot_failed =
    snapshot_enabled_ &&
    !snapshot_last_write_ok_;

  const bool subscriptions_incomplete =
    observation_state.active <
    observation_state.configured;

  if (!graph_error.empty()) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    status.message = "graph_discovery_failed";
  } else if (!observation_runtime_error.empty()) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    status.message = "observation_runtime_failed";
  } else if (snapshot_failed) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    status.message = "snapshot_publication_failed";
  } else if (!evidence.dds_active) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    status.message = "local_dds_evidence_missing";
  } else if (!selectors_configured) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message =
      "graph_evidence_selectors_unconfigured";
  } else if (!evidence_complete) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message = "graph_evidence_incomplete";
  } else if (observation_state.configured == 0U) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message = "observation_topics_unconfigured";
  } else if (subscriptions_incomplete) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message =
      "observation_subscriptions_incomplete";
  } else if (health.status == HealthStatus::kUnhealthy) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    status.message = "observation_health_unhealthy";
  } else if (health.status == HealthStatus::kDegraded) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message = "observation_health_degraded";
  } else if (!snapshot_enabled_) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message = "snapshot_publication_disabled";
  } else if (!bridge_runtime.bridge_ready) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message = bridge_runtime.readiness_reason;
  } else {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::OK;

    status.message = "bridge_ready";
  }

  status.values.push_back(
    make_key_value(
      "read_only",
      boolean_string(bridge_runtime.read_only)));

  status.values.push_back(
    make_key_value(
      "commands_enabled",
      boolean_string(bridge_runtime.commands_enabled)));

  status.values.push_back(
    make_key_value(
      "bridge_ready",
      boolean_string(bridge_runtime.bridge_ready)));

  status.values.push_back(
    make_key_value(
      "readiness_reason",
      bridge_runtime.readiness_reason));

  status.values.push_back(
    make_key_value(
      "command_execution_mode",
      bridge_runtime.command_execution_mode));

  status.values.push_back(
    make_key_value(
      "command_last_status",
      bridge_runtime.command_last_status));

  status.values.push_back(
    make_key_value(
      "stop_ready",
      boolean_string(bridge_runtime.stop_ready)));

  status.values.push_back(
    make_key_value(
      "teleop_ready",
      boolean_string(bridge_runtime.teleop_ready)));

  status.values.push_back(
    make_key_value(
      "navigation_ready",
      boolean_string(bridge_runtime.navigation_ready)));

  status.values.push_back(
    make_key_value(
      "dds_active",
      boolean_string(evidence.dds_active)));

  status.values.push_back(
    make_key_value(
      "core_visible",
      boolean_string(evidence.core_visible)));

  status.values.push_back(
    make_key_value(
      "edge_visible",
      boolean_string(evidence.edge_visible)));

  status.values.push_back(
    make_key_value(
      "observation_topics_configured",
      std::to_string(observation_state.configured)));

  status.values.push_back(
    make_key_value(
      "observation_subscriptions_active",
      std::to_string(observation_state.active)));

  status.values.push_back(
    make_key_value(
      "observation_health_status",
      std::string(to_string(health.status))));

  status.values.push_back(
    make_key_value(
      "observation_health_reason",
      std::string(to_string(health.reason))));

  status.values.push_back(
    make_key_value(
      "required_topics_ready",
      boolean_string(health.required_topics_ready)));

  status.values.push_back(
    make_key_value(
      "all_topics_fresh",
      boolean_string(health.all_topics_fresh)));

  status.values.push_back(
    make_key_value(
      "snapshot_enabled",
      boolean_string(snapshot_enabled_)));

  status.values.push_back(
    make_key_value(
      "snapshot_path",
      snapshot_path_));

  status.values.push_back(
    make_key_value(
      "snapshot_last_write_ok",
      boolean_string(snapshot_last_write_ok_)));

  status.values.push_back(
    make_key_value(
      "snapshot_sequence",
      std::to_string(snapshot_sequence_)));

  status.values.push_back(
    make_key_value(
      "snapshot_success_count",
      std::to_string(snapshot_success_count_)));

  status.values.push_back(
    make_key_value(
      "snapshot_failure_count",
      std::to_string(snapshot_failure_count_)));

  status.values.push_back(
    make_key_value(
      "snapshot_bytes_written",
      std::to_string(snapshot_bytes_written_)));

  status.values.push_back(
    make_key_value(
      "snapshot_error",
      snapshot_error_));

  status.values.push_back(
    make_key_value(
      "observation_runtime_error",
      observation_runtime_error));

  status.values.push_back(
    make_key_value(
      "graph_error",
      graph_error));

  diagnostics_message.status.push_back(
    std::move(status));

  diagnostics_publisher_->publish(
    diagnostics_message);
}

}  // namespace savo_bridge
