// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__SNAPSHOT_WRITER_HPP_
#define SAVO_BRIDGE__SNAPSHOT_WRITER_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "savo_bridge/topic_observation.hpp"

namespace savo_bridge
{

struct BridgeRuntimeSnapshot
{
  std::string owner{"savo_bridge"};
  std::string instance_id{"savo-bridge"};

  bool process_alive{true};
  bool read_only{true};
  bool commands_enabled{false};
  bool bridge_ready{false};
  bool validated{false};
  bool dispatch_enabled{false};
  bool navigation_bridge_validated{false};
  bool block_navigation{true};

  std::string readiness_reason{"bridge_not_configured"};

  bool command_server_enabled{false};
  std::string command_execution_mode{"disabled"};
  bool command_worker_fatal{false};
  std::string command_last_status{"disabled"};
  std::uint64_t command_accepted_count{0U};
  std::uint64_t command_rejected_count{0U};

  bool stop_ready{false};
  bool teleop_ready{false};
  bool navigation_ready{false};

  bool dds_active{false};
  bool core_visible{false};
  bool edge_visible{false};
  bool observation_subscriptions_complete{false};
  bool required_topics_ready{false};
  bool all_topics_fresh{false};
  bool location_service_configured{false};
  bool active_map_context_configured{false};

  bool command_active{false};
  bool teleop_active{false};
  bool navigation_goal_active{false};
  std::string active_command_id;
  std::string active_command_type;
  std::string last_terminal_command_id;

  bool mode_state_observed{false};
  std::string mode_state;
  std::int64_t mode_state_age_ms{-1};

  bool external_stop_state_known{false};
  bool external_stop_active{false};
  std::int64_t external_stop_age_ms{-1};

  bool safety_stop_state_known{false};
  bool safety_stop_active{false};
  std::int64_t safety_stop_age_ms{-1};

  bool safe_velocity_state_known{false};
  bool safe_velocity_zero{false};
  std::int64_t safe_velocity_age_ms{-1};

  bool navigation_readiness_observed{false};
  std::string navigation_readiness;
  std::int64_t navigation_readiness_age_ms{-1};

  std::uint64_t dispatcher_accepted_command_count{0U};
  std::uint64_t dispatcher_rejected_command_count{0U};
  std::uint64_t dispatcher_ros_publication_count{0U};
  std::string dispatcher_last_reason{
    "ros_command_dispatcher_not_started"};
};

struct SnapshotDocument
{
  std::uint64_t sequence{0U};
  BridgeRuntimeSnapshot bridge;
  std::vector<TopicObservation::Snapshot> topics;
};

[[nodiscard]] std::string serialize_snapshot(
  const SnapshotDocument & document);

struct SnapshotWriteResult
{
  std::size_t bytes_written{0U};
  bool replaced_existing{false};
};

class AtomicSnapshotWriter final
{
public:
  explicit AtomicSnapshotWriter(std::filesystem::path target_path);

  [[nodiscard]] SnapshotWriteResult write(
    const SnapshotDocument & document) const;

  [[nodiscard]] const std::filesystem::path &
  target_path() const noexcept;

private:
  [[nodiscard]] static std::filesystem::path validate_target_path(
    std::filesystem::path target_path);

  const std::filesystem::path target_path_;
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__SNAPSHOT_WRITER_HPP_
