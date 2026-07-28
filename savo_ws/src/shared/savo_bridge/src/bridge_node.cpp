// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/bridge_node.hpp"

#include <sys/types.h>

#include <algorithm>
#include <chrono>
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
  const GraphEvidence & evidence,
  const std::string & graph_error,
  const SnapshotPublicationState & snapshot,
  const ObservationRuntimeState & observations,
  const HealthEvaluation & health)
{
  std::string output;
  output.reserve(2048U);

  output += "{\"schema_name\":\"savo_bridge_state\"";
  output += ",\"schema_version\":1";
  output += ",\"owner\":\"savo_bridge\"";
  output += ",\"phase\":\"topic_observation\"";
  output += ",\"read_only\":true";
  output += ",\"commands_enabled\":false";
  output += ",\"process_alive\":true";

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

  output += ",\"bridge_ready\":false";

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
    "savo_bridge Phase 2B-2 topic observation started; "
    "configured_topics=%zu snapshot_enabled=%s "
    "commands remain disabled",
    observation_runtimes_.size(),
    snapshot_enabled_ ? "true" : "false");
}

BridgeNode::~BridgeNode()
{
  command_stop_requested_.store(
    true,
    std::memory_order_release);

  if (command_worker_.joinable()) {
    command_worker_.join();
  }

  if (command_server_) {
    (void)command_server_->stop();
    command_server_.reset();
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

  const std::vector<std::int64_t> allowed_peer_uids =
    declare_read_only_parameter<std::vector<std::int64_t>>(
    *this,
    "command_server.allowed_peer_uids",
    std::vector<std::int64_t>{});

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

  if (execution_mode != "dry_run" ||
    max_request_bytes <= 0 ||
    max_response_bytes <= 0 ||
    static_cast<std::uintmax_t>(max_request_bytes) >
    std::numeric_limits<std::size_t>::max() ||
    static_cast<std::uintmax_t>(max_response_bytes) >
    std::numeric_limits<std::size_t>::max() ||
    socket_mode < 0 ||
    static_cast<std::uintmax_t>(socket_mode) >
    std::numeric_limits<std::uint32_t>::max())
  {
    invalid_configuration();
  }

  CommandServerConfig config;
  config.enabled = true;
  config.socket_path = socket_path;
  config.bridge_instance_id = bridge_instance_id;
  config.max_request_bytes =
    static_cast<std::size_t>(max_request_bytes);
  config.max_response_bytes =
    static_cast<std::size_t>(max_response_bytes);
  config.accept_timeout_ms = accept_timeout_ms;
  config.client_read_timeout_ms =
    client_read_timeout_ms;
  config.client_write_timeout_ms =
    client_write_timeout_ms;
  config.socket_mode =
    static_cast<std::uint32_t>(socket_mode);

  if (!allowed_peer_uids.empty()) {
    std::set<std::uint32_t> unique_uids;
    config.allowed_peer_uids.clear();
    config.allowed_peer_uids.reserve(
      allowed_peer_uids.size());

    for (const std::int64_t uid : allowed_peer_uids) {
      if (uid < 0 ||
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
}

void BridgeNode::start_command_server()
{
  if (!command_server_enabled_) {
    return;
  }

  command_server_ =
    std::make_unique<CommandServer>(
    *command_server_config_);

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
    "command server enabled in dry-run mode");
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

      if (result.status ==
        CommandServerStatus::DryRunAcknowledged)
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

  publish_runtime_snapshot(topic_snapshots);

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
  readiness_message.data = false;
  readiness_publisher_->publish(readiness_message);

  std_msgs::msg::String state_message;

  state_message.data =
    make_state_json(
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
  status.name = "/savo_bridge/topic_observation";
  status.hardware_id = "savo-edge";

  const bool selectors_configured =
    evidence.core_evidence_configured &&
    evidence.edge_evidence_configured;

  const bool evidence_complete =
    evidence.core_visible &&
    evidence.edge_visible;

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
  } else {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message =
      "observation_active_readiness_not_enabled";
  }

  status.values.push_back(
    make_key_value(
      "read_only",
      "true"));

  status.values.push_back(
    make_key_value(
      "commands_enabled",
      "false"));

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
