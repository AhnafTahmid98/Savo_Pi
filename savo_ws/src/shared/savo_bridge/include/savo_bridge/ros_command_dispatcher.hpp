// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__ROS_COMMAND_DISPATCHER_HPP_
#define SAVO_BRIDGE__ROS_COMMAND_DISPATCHER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "savo_bridge/command_server.hpp"

namespace rclcpp
{

class Node;

}  // namespace rclcpp

namespace savo_bridge
{

// ROS-facing command authority owned exclusively by the native bridge.
//
// Authority boundaries:
//
// - STOP publishes only through savo_control.
// - Teleoperation publishes only /cmd_vel_manual.
// - Navigation calls only the guarded public savo_nav action.
// - Named locations are resolved through savo_locations.
// - /cmd_vel_safe, /cmd_vel, motor drivers and raw Nav2 actions are forbidden.
struct RosCommandDispatcherConfig
{
  // savo_control command and observed-state interfaces.
  std::string mode_command_topic{
    "/savo_control/mode_cmd"};

  std::string mode_state_topic{
    "/savo_control/mode_state"};

  std::string external_stop_topic{
    "/savo_control/external_stop"};

  std::string safety_stop_topic{
    "/safety/stop"};

  std::string manual_velocity_topic{
    "/cmd_vel_manual"};

  // Observation only. The dispatcher must never publish this topic.
  std::string safe_velocity_topic{
    "/cmd_vel_safe"};

  // savo_nav interfaces.
  std::string navigation_readiness_topic{
    "/savo_nav/readiness"};

  std::string navigation_action_name{
    "/savo_nav/navigation/navigate_to_pose"};

  // This must be supplied from the authoritative savo_locations
  // runtime contract. An empty value fails navigation closed.
  std::string location_resolve_service{};

  // Canonical state values.
  std::string stop_mode{"STOP"};
  std::string manual_mode{"MANUAL"};
  std::string navigation_mode{"NAV"};
  std::string navigation_ready_state{"ready"};
  std::string map_frame{"map"};

  // Active saved-map context. Navigation fails closed when required
  // and these values are unavailable.
  std::string active_map_id{};
  std::uint32_t active_map_revision{0U};
  bool require_active_map_context{true};

  // Freshness and acknowledgement bounds.
  std::int64_t observed_state_timeout_ms{1000};
  std::int64_t mode_transition_timeout_ms{1500};
  std::int64_t stop_confirmation_timeout_ms{2000};
  std::int64_t location_service_timeout_ms{1500};
  std::int64_t navigation_server_timeout_ms{1500};
  std::int64_t navigation_goal_response_timeout_ms{2000};
  std::int64_t navigation_execution_timeout_ms{300000};
  std::int64_t teleop_cancel_timeout_ms{2000};
  std::int64_t navigation_cancel_timeout_ms{2000};

  // Teleoperation execution bounds. Protocol parsing already applies
  // the same or stricter limits; the dispatcher validates again.
  double maximum_linear_speed_mps{0.18};
  double maximum_angular_speed_radps{0.65};
  std::int64_t maximum_teleop_duration_ms{1000};
  std::int64_t teleop_publish_period_ms{50};

  // STOP and teleop completion produce repeated zero commands.
  std::size_t final_zero_publication_count{3U};

  // All six geometry_msgs/Twist components must satisfy this bound
  // before the safe velocity is considered zero.
  double safe_zero_epsilon{1.0e-4};
};

struct RosCommandDispatcherSnapshot
{
  bool shutdown_requested{false};

  bool command_active{false};
  bool teleop_active{false};
  bool teleop_cancel_requested{false};
  bool navigation_goal_active{false};
  bool navigation_cancel_requested{false};

  std::string active_command_id{};
  std::string active_command_type{};
  std::string last_terminal_command_id{};

  bool mode_state_observed{false};
  std::string mode_state{};
  std::int64_t mode_state_age_ms{-1};

  bool external_stop_observed{false};
  bool external_stop_active{false};
  std::int64_t external_stop_age_ms{-1};

  bool safety_stop_observed{false};
  bool safety_stop_active{false};
  std::int64_t safety_stop_age_ms{-1};

  bool safe_velocity_observed{false};
  bool safe_velocity_zero{false};
  std::int64_t safe_velocity_age_ms{-1};

  bool navigation_readiness_observed{false};
  std::string navigation_readiness{};
  std::int64_t navigation_readiness_age_ms{-1};

  std::uint64_t accepted_command_count{0U};
  std::uint64_t rejected_command_count{0U};
  std::uint64_t ros_publication_count{0U};

  std::string last_reason{
    "ros_command_dispatcher_not_started"};
};

class RosCommandDispatcher
{
public:
  explicit RosCommandDispatcher(
    rclcpp::Node & node,
    RosCommandDispatcherConfig config);

  ~RosCommandDispatcher();

  RosCommandDispatcher(
    const RosCommandDispatcher &) = delete;

  RosCommandDispatcher & operator=(
    const RosCommandDispatcher &) = delete;

  RosCommandDispatcher(
    RosCommandDispatcher &&) = delete;

  RosCommandDispatcher & operator=(
    RosCommandDispatcher &&) = delete;

  // Performs deterministic validation and dispatch for one already
  // protocol-validated bridge command.
  //
  // STOP returns accepted only after downstream STOP confirmation.
  // Teleop and navigation return accepted only after their bounded
  // admission sequence succeeds.
  [[nodiscard]] CommandDispatchResult dispatch(
    const ValidatedCommand & command);

  [[nodiscard]] RosCommandDispatcherSnapshot snapshot() const;

  // Cancels active bridge-owned work, publishes final teleop zero
  // commands and prevents future non-STOP dispatch.
  void shutdown() noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__ROS_COMMAND_DISPATCHER_HPP_
