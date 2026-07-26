// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/bridge_node.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

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

[[nodiscard]] std::string make_state_json(
  const GraphEvidence & evidence,
  const std::string & graph_error,
  const SnapshotPublicationState & snapshot)
{
  std::string output;
  output.reserve(1536U);

  output += "{\"schema_name\":\"savo_bridge_state\"";
  output += ",\"schema_version\":1";
  output += ",\"owner\":\"savo_bridge\"";
  output += ",\"phase\":\"snapshot_publication\"";
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

}  // namespace

BridgeNode::BridgeNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node(
    "savo_bridge_node",
    "/savo_bridge",
    options)
{
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

  RCLCPP_INFO(
    get_logger(),
    "savo_bridge Phase 2A snapshot publication started; "
    "enabled=%s path=%s commands remain disabled",
    snapshot_enabled_ ? "true" : "false",
    snapshot_path_.c_str());
}

void BridgeNode::publish_runtime_snapshot()
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

  publish_runtime_snapshot();

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
    snapshot_state);

  state_publisher_->publish(state_message);

  std_msgs::msg::UInt64 heartbeat_message;
  heartbeat_message.data = ++heartbeat_sequence_;
  heartbeat_publisher_->publish(heartbeat_message);

  diagnostic_msgs::msg::DiagnosticArray diagnostics_message;
  diagnostics_message.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/savo_bridge/runtime_snapshot";
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

  if (!graph_error.empty()) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    status.message = "graph_discovery_failed";
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
  } else if (!snapshot_enabled_) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message = "snapshot_publication_disabled";
  } else {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message =
      "snapshot_publication_active_observation_not_started";
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
      "snapshot_replaced_existing",
      boolean_string(snapshot_replaced_existing_)));

  status.values.push_back(
    make_key_value(
      "snapshot_error",
      snapshot_error_));

  status.values.push_back(
    make_key_value(
      "discovered_nodes",
      std::to_string(evidence.discovered_nodes)));

  status.values.push_back(
    make_key_value(
      "discovered_topics",
      std::to_string(evidence.discovered_topics)));

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
