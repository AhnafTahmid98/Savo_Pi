// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__BRIDGE_NODE_HPP_
#define SAVO_BRIDGE__BRIDGE_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include "savo_bridge/graph_evidence.hpp"
#include "savo_bridge/snapshot_writer.hpp"

namespace savo_bridge
{

class BridgeNode final : public rclcpp::Node
{
public:
  explicit BridgeNode(
    const rclcpp::NodeOptions & options =
    rclcpp::NodeOptions());

private:
  void publish_status();
  void publish_runtime_snapshot();

  GraphEvidenceConfig graph_config_;

  bool snapshot_enabled_{false};
  std::string snapshot_path_;
  std::unique_ptr<AtomicSnapshotWriter> snapshot_writer_;

  std::uint64_t snapshot_sequence_{0U};
  std::uint64_t snapshot_success_count_{0U};
  std::uint64_t snapshot_failure_count_{0U};

  std::size_t snapshot_bytes_written_{0U};

  bool snapshot_replaced_existing_{false};
  bool snapshot_last_write_ok_{false};

  std::string snapshot_error_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    readiness_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr
    heartbeat_publisher_;

  rclcpp::Publisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;

  rclcpp::TimerBase::SharedPtr status_timer_;

  std::uint64_t heartbeat_sequence_{0U};
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__BRIDGE_NODE_HPP_
