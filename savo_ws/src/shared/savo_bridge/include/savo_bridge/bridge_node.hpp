// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__BRIDGE_NODE_HPP_
#define SAVO_BRIDGE__BRIDGE_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include "savo_bridge/command_server.hpp"
#include "savo_bridge/graph_evidence.hpp"
#include "savo_bridge/ros_command_dispatcher.hpp"
#include "savo_bridge/snapshot_writer.hpp"
#include "savo_bridge/topic_observation.hpp"

namespace savo_bridge
{

class BridgeNode final : public rclcpp::Node
{
public:
  explicit BridgeNode(
    const rclcpp::NodeOptions & options =
    rclcpp::NodeOptions());
  ~BridgeNode() override;

  [[nodiscard]] bool command_server_enabled() const noexcept;
  [[nodiscard]] bool command_worker_joinable() const noexcept;

private:
  struct ObservationRuntime
  {
    std::string topic_name;
    std::unique_ptr<TopicObservation> observation;
    std::string topic_type;
    std::shared_ptr<rclcpp::GenericSubscription> subscription;
    std::string error;
  };

  void publish_status();
  void refresh_observation_subscriptions();
  void configure_command_server();
  void start_command_server();
  void run_command_worker() noexcept;

  void publish_runtime_snapshot(
    const BridgeRuntimeSnapshot & bridge_runtime,
    const std::vector<TopicObservation::Snapshot> &
    topic_snapshots);

  [[nodiscard]] std::vector<TopicObservation::Snapshot>
  collect_observation_snapshots(
    TopicObservation::TimePoint evaluated_at) const;

  GraphEvidenceConfig graph_config_;

  std::vector<ObservationRuntime> observation_runtimes_;

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

  bool command_server_enabled_{false};
  std::string command_execution_mode_{"disabled"};

  std::unique_ptr<CommandServerConfig> command_server_config_;

  std::unique_ptr<RosCommandDispatcherConfig>
  ros_command_dispatcher_config_;

  std::unique_ptr<RosCommandDispatcher>
  ros_command_dispatcher_;

  std::unique_ptr<CommandServer> command_server_;
  std::thread command_worker_;
  std::atomic<bool> command_stop_requested_{false};
  std::atomic<CommandServerStatus> command_last_status_{
    CommandServerStatus::Disabled};
  std::atomic<std::uint64_t> command_accepted_count_{0U};
  std::atomic<std::uint64_t> command_rejected_count_{0U};
  std::atomic<bool> command_worker_fatal_{false};

  BridgeRuntimeSnapshot last_bridge_runtime_;
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__BRIDGE_NODE_HPP_
