// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_msgs/action/confirm_april_tag.hpp"
#include "savo_msgs/action/execute_coverage_path.hpp"
#include "savo_msgs/action/rotate_to_heading.hpp"
#include "savo_msgs/action/run_autonomous_mapping.hpp"
#include "savo_msgs/srv/authorize_location_operation.hpp"
#include "savo_msgs/srv/authorize_operation.hpp"
#include "savo_msgs/srv/manage_system_state.hpp"
#include "savo_msgs/srv/update_map_context.hpp"
#include "savo_supervisor/component_status.hpp"
#include "savo_supervisor/core_payload_parser.hpp"
#include "savo_supervisor/edge_payload_parser.hpp"
#include "savo_supervisor/edge_supervision.hpp"
#include "savo_supervisor/localization_payload_parser.hpp"
#include "savo_supervisor/location_authorization_policy.hpp"
#include "savo_supervisor/mission_authority.hpp"
#include "savo_supervisor/mission_payload_parser.hpp"
#include "savo_supervisor/reason_codes.hpp"
#include "savo_supervisor/supervisor_policy.hpp"
#include "savo_supervisor/supervisor_state.hpp"
#include "savo_supervisor/system_authority.hpp"
#include "savo_supervisor/system_state_store.hpp"
#include "savo_supervisor/transition_tracker.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include <nlohmann/json.hpp>

namespace svo = savo_supervisor;

namespace
{

enum class Channel {kHealth, kSummary, kHeartbeat};

svo::ComponentStatus initialize_component(const svo::ComponentConfig & config)
{
  svo::ComponentStatus status;
  status.config = config;
  if (!config.enabled) {
    status.health_tracker.mark_disabled();
    status.summary_tracker.mark_disabled();
    status.heartbeat_tracker.mark_disabled();
  }
  return status;
}

void apply_payload(
  svo::ComponentStatus & status,
  Channel channel,
  const svo::ParsedCorePayload & parsed,
  const rclcpp::Time & receive_time)
{
  switch (channel) {
    case Channel::kHealth:
      status.health_valid = parsed.valid;
      status.health_state = parsed.state;
      status.health_ready = parsed.ready;
      status.health_degraded = parsed.degraded;
      status.health_reason_code = parsed.reason_code;
      status.health_tracker.observe_message(
        receive_time, parsed.stamp, !parsed.valid, parsed.detail);
      break;
    case Channel::kSummary:
      status.summary_valid = parsed.valid;
      status.summary_state = parsed.state;
      status.summary_ready = parsed.ready;
      status.summary_degraded = parsed.degraded;
      status.summary_reason_code = parsed.reason_code;
      status.summary_tracker.observe_message(
        receive_time, parsed.stamp, !parsed.valid, parsed.detail);
      break;
    case Channel::kHeartbeat:
      status.heartbeat_valid = parsed.valid;
      status.heartbeat_state = parsed.state;
      status.heartbeat_alive = parsed.alive;
      status.heartbeat_ready = parsed.ready;
      status.heartbeat_degraded = parsed.degraded;
      status.heartbeat_reason_code = parsed.reason_code;
      status.heartbeat_tracker.observe_message(
        receive_time, parsed.stamp, !parsed.valid, parsed.detail);
      break;
  }
}

void append_key_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}

void append_aggregate_diagnostic(
  diagnostic_msgs::msg::DiagnosticArray & array,
  const svo::SupervisorState & state)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "savo_supervisor/aggregate";
  status.hardware_id = "robot_savo_supervisor";
  status.level = state.health == svo::AggregateHealth::ERROR ?
    diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    state.health == svo::AggregateHealth::OK ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = state.reason_code.empty() ? "supervisor_state_unknown" : state.reason_code;

  append_key_value(status, "lifecycle", svo::ToString(state.lifecycle));
  append_key_value(status, "operating_mode", svo::ToString(state.operating_mode));
  append_key_value(status, "health", svo::ToString(state.health));
  append_key_value(status, "safety", svo::ToString(state.safety));
  append_key_value(status, "ready", state.ready ? "true" : "false");
  append_key_value(status, "degraded", state.degraded ? "true" : "false");
  append_key_value(status, "reason_code", state.reason_code);
  append_key_value(
    status, "core_health_ready",
    state.capabilities.core_health_ready ? "true" : "false");
  append_key_value(
    status, "core_safety_ready",
    state.capabilities.core_safety_ready ? "true" : "false");
  append_key_value(
    status, "core_motion_ready",
    state.capabilities.core_motion_ready ? "true" : "false");
  append_key_value(
    status, "can_manual_drive",
    state.capabilities.can_manual_drive ? "true" : "false");
  append_key_value(status, "can_rotate", state.capabilities.can_rotate ? "true" : "false");
  append_key_value(
    status, "can_start_geometric_mapping",
    state.capabilities.can_start_geometric_mapping ? "true" : "false");
  array.status.push_back(status);
}

void append_component_diagnostic(
  diagnostic_msgs::msg::DiagnosticArray & array,
  const svo::ComponentSummary & summary)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "savo_supervisor/" + summary.name;
  status.hardware_id = "robot_savo_supervisor";
  const bool required_unavailable = summary.enabled && summary.required && !summary.ready &&
    summary.state != svo::ComponentState::INITIALIZING;
  status.level = required_unavailable || summary.state == svo::ComponentState::ERROR ?
    diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    (summary.state == svo::ComponentState::OK || summary.state == svo::ComponentState::DISABLED) ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = summary.reason_code.empty() ? "component_state_unknown" : summary.reason_code;

  append_key_value(status, "enabled", summary.enabled ? "true" : "false");
  append_key_value(status, "required", summary.required ? "true" : "false");
  append_key_value(status, "received", summary.received ? "true" : "false");
  append_key_value(status, "state", svo::ToString(summary.state));
  append_key_value(status, "ready", summary.ready ? "true" : "false");
  append_key_value(status, "degraded", summary.degraded ? "true" : "false");
  append_key_value(status, "reason_code", summary.reason_code);
  append_key_value(status, "last_message_age_s", std::to_string(summary.last_message_age_s));
  append_key_value(status, "timeout_s", std::to_string(summary.timeout_s));
  append_key_value(
    status, "malformed_message_count",
    std::to_string(summary.malformed_message_count));
  append_key_value(status, "recovery_count", std::to_string(summary.recovery_count));
  append_key_value(status, "detail", summary.detail);
  array.status.push_back(status);
}

void append_edge_diagnostic(
  diagnostic_msgs::msg::DiagnosticArray & array,
  const svo::EdgeComponentSummary & component)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "savo_supervisor/edge/" + component.name;
  status.hardware_id = "robot_savo_edge";
  status.level = component.ready ?
    (component.degraded ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK) :
    (component.required_for_startup ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status.message = component.reason;
  append_key_value(status, "enabled", component.enabled ? "true" : "false");
  append_key_value(
    status, "required_for_startup", component.required_for_startup ? "true" : "false");
  append_key_value(status, "received", component.received ? "true" : "false");
  append_key_value(status, "fresh", component.fresh ? "true" : "false");
  append_key_value(status, "valid", component.valid ? "true" : "false");
  append_key_value(status, "ready", component.ready ? "true" : "false");
  append_key_value(status, "state", svo::ToString(component.state));
  array.status.push_back(status);
}

void append_system_diagnostic(
  diagnostic_msgs::msg::DiagnosticArray & array,
  const svo::SystemAuthoritySnapshot & system)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "savo_supervisor/system_authority";
  status.hardware_id = "robot_savo_supervisor";
  status.level = system.fault_latched ?
    diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    system.armed && !system.shutdown_requested ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = system.reason;
  append_key_value(status, "state", svo::ToString(system.state));
  append_key_value(status, "startup_ready", system.startup_ready ? "true" : "false");
  append_key_value(status, "system_armed", system.armed ? "true" : "false");
  append_key_value(status, "fault_latched", system.fault_latched ? "true" : "false");
  append_key_value(
    status, "shutdown_requested", system.shutdown_requested ? "true" : "false");
  append_key_value(
    status, "remote_commands_ready", system.remote_commands_ready ? "true" : "false");
  append_key_value(status, "system_generation", std::to_string(system.generation));
  append_key_value(status, "last_actor", system.last_actor);
  array.status.push_back(status);
}

std::optional<svo::LocationOperation> location_operation_from_ros(std::uint8_t operation)
{
  using Service = savo_msgs::srv::AuthorizeLocationOperation;
  switch (operation) {
    case Service::Request::OP_REGISTER_LOCATION_CANDIDATE:
      return svo::LocationOperation::kRegisterCandidate;
    case Service::Request::OP_APPROVE_LOCATION: return svo::LocationOperation::kApproveLocation;
    case Service::Request::OP_NAVIGATE_TO_LOCATION:
      return svo::LocationOperation::kNavigateToLocation;
    case Service::Request::OP_CONFIRM_LOCATION_ARRIVAL:
      return svo::LocationOperation::kConfirmArrival;
    case Service::Request::OP_REJECT_LOCATION_CANDIDATE:
      return svo::LocationOperation::kRejectLocationCandidate;
    default: return std::nullopt;
  }
}

std::uint8_t location_authorization_code_to_ros(svo::LocationAuthorizationCode code)
{
  using Response = savo_msgs::srv::AuthorizeLocationOperation::Response;
  switch (code) {
    case svo::LocationAuthorizationCode::kAuthorized: return Response::RESULT_AUTHORIZED;
    case svo::LocationAuthorizationCode::kInvalidRequest: return Response::RESULT_INVALID_REQUEST;
    case svo::LocationAuthorizationCode::kSupervisorNotReady:
      return Response::RESULT_SUPERVISOR_NOT_READY;
    case svo::LocationAuthorizationCode::kHealthBlocked: return Response::RESULT_HEALTH_BLOCKED;
    case svo::LocationAuthorizationCode::kSafetyBlocked: return Response::RESULT_SAFETY_BLOCKED;
    case svo::LocationAuthorizationCode::kMapContextBlocked:
      return Response::RESULT_MAP_CONTEXT_BLOCKED;
    case svo::LocationAuthorizationCode::kOperationDisabled:
      return Response::RESULT_OPERATION_DISABLED;
  }
  return Response::RESULT_INTERNAL_ERROR;
}

std::optional<svo::MissionOperation> mission_operation_from_ros(std::uint8_t operation)
{
  using Request = savo_msgs::srv::AuthorizeOperation::Request;
  switch (operation) {
    case Request::OP_ENTER_MANUAL_CONTROL: return svo::MissionOperation::kManualControl;
    case Request::OP_START_MANUAL_MAPPING: return svo::MissionOperation::kManualMapping;
    case Request::OP_START_AUTONOMOUS_MAPPING:
      return svo::MissionOperation::kAutonomousMapping;
    case Request::OP_RUN_SCAN360: return svo::MissionOperation::kScan360;
    case Request::OP_RUN_COVERAGE: return svo::MissionOperation::kCoverage;
    case Request::OP_NAVIGATE_TO_POSE: return svo::MissionOperation::kNavigateToPose;
    case Request::OP_NAVIGATE_TO_LOCATION:
      return svo::MissionOperation::kNavigateToLocation;
    case Request::OP_REGISTER_LOCATION: return svo::MissionOperation::kRegisterLocation;
    case Request::OP_REVIEW_LOCATION: return svo::MissionOperation::kReviewLocation;
    case Request::OP_CONFIRM_ARRIVAL: return svo::MissionOperation::kConfirmArrival;
    default: return std::nullopt;
  }
}

std::optional<svo::AuthorityCommand> authority_command_from_ros(std::uint8_t command)
{
  using Request = savo_msgs::srv::AuthorizeOperation::Request;
  switch (command) {
    case Request::COMMAND_CHECK: return svo::AuthorityCommand::kCheck;
    case Request::COMMAND_ACQUIRE: return svo::AuthorityCommand::kAcquire;
    case Request::COMMAND_RELEASE: return svo::AuthorityCommand::kRelease;
    case Request::COMMAND_PAUSE: return svo::AuthorityCommand::kPause;
    case Request::COMMAND_RESUME: return svo::AuthorityCommand::kResume;
    default: return std::nullopt;
  }
}

std::uint8_t mission_authorization_code_to_ros(svo::MissionAuthorizationCode code)
{
  using Response = savo_msgs::srv::AuthorizeOperation::Response;
  switch (code) {
    case svo::MissionAuthorizationCode::kAuthorized: return Response::RESULT_AUTHORIZED;
    case svo::MissionAuthorizationCode::kInvalidRequest: return Response::RESULT_INVALID_REQUEST;
    case svo::MissionAuthorizationCode::kNotReady: return Response::RESULT_NOT_READY;
    case svo::MissionAuthorizationCode::kHealthBlocked: return Response::RESULT_HEALTH_BLOCKED;
    case svo::MissionAuthorizationCode::kSafetyBlocked: return Response::RESULT_SAFETY_BLOCKED;
    case svo::MissionAuthorizationCode::kDependencyBlocked:
      return Response::RESULT_DEPENDENCY_BLOCKED;
    case svo::MissionAuthorizationCode::kMapContextBlocked:
      return Response::RESULT_MAP_CONTEXT_BLOCKED;
    case svo::MissionAuthorizationCode::kOperationConflict:
      return Response::RESULT_OPERATION_CONFLICT;
    case svo::MissionAuthorizationCode::kOwnershipMismatch:
      return Response::RESULT_OWNERSHIP_MISMATCH;
    case svo::MissionAuthorizationCode::kOperationDisabled:
      return Response::RESULT_OPERATION_DISABLED;
  }
  return Response::RESULT_INTERNAL_ERROR;
}

std::optional<svo::SystemCommand> system_command_from_ros(std::uint8_t command)
{
  using Request = savo_msgs::srv::ManageSystemState::Request;
  switch (command) {
    case Request::COMMAND_STATUS: return svo::SystemCommand::kStatus;
    case Request::COMMAND_ARM: return svo::SystemCommand::kArm;
    case Request::COMMAND_DISARM: return svo::SystemCommand::kDisarm;
    case Request::COMMAND_BEGIN_SHUTDOWN: return svo::SystemCommand::kBeginShutdown;
    case Request::COMMAND_CLEAR_FAULT_LATCH: return svo::SystemCommand::kClearFaultLatch;
    default: return std::nullopt;
  }
}

std::uint8_t system_authority_code_to_ros(svo::SystemAuthorityCode code)
{
  using Response = savo_msgs::srv::ManageSystemState::Response;
  switch (code) {
    case svo::SystemAuthorityCode::kAccepted: return Response::RESULT_ACCEPTED;
    case svo::SystemAuthorityCode::kInvalidRequest: return Response::RESULT_INVALID_REQUEST;
    case svo::SystemAuthorityCode::kNotReady: return Response::RESULT_NOT_READY;
    case svo::SystemAuthorityCode::kAlreadyInState: return Response::RESULT_ALREADY_IN_STATE;
    case svo::SystemAuthorityCode::kFaultLatched: return Response::RESULT_FAULT_LATCHED;
    case svo::SystemAuthorityCode::kUnsafeToClear: return Response::RESULT_UNSAFE_TO_CLEAR;
    case svo::SystemAuthorityCode::kGenerationMismatch:
      return Response::RESULT_GENERATION_MISMATCH;
  }
  return Response::RESULT_INTERNAL_ERROR;
}

std::uint8_t mission_operation_to_ros(svo::MissionOperation operation)
{
  using Request = savo_msgs::srv::AuthorizeOperation::Request;
  switch (operation) {
    case svo::MissionOperation::kNone: return Request::OP_NONE;
    case svo::MissionOperation::kManualControl: return Request::OP_ENTER_MANUAL_CONTROL;
    case svo::MissionOperation::kManualMapping: return Request::OP_START_MANUAL_MAPPING;
    case svo::MissionOperation::kAutonomousMapping:
      return Request::OP_START_AUTONOMOUS_MAPPING;
    case svo::MissionOperation::kScan360: return Request::OP_RUN_SCAN360;
    case svo::MissionOperation::kCoverage: return Request::OP_RUN_COVERAGE;
    case svo::MissionOperation::kNavigateToPose: return Request::OP_NAVIGATE_TO_POSE;
    case svo::MissionOperation::kNavigateToLocation:
      return Request::OP_NAVIGATE_TO_LOCATION;
    case svo::MissionOperation::kRegisterLocation: return Request::OP_REGISTER_LOCATION;
    case svo::MissionOperation::kReviewLocation: return Request::OP_REVIEW_LOCATION;
    case svo::MissionOperation::kConfirmArrival: return Request::OP_CONFIRM_ARRIVAL;
  }
  return Request::OP_NONE;
}

}  // namespace

class SupervisorNode : public rclcpp::Node
{
public:
  SupervisorNode()
  : Node("savo_supervisor_node")
  {
    declare_global_parameters();
    declare_component_parameters("base", svo::SupervisorPolicy::DefaultBaseConfig());
    declare_component_parameters("control", svo::SupervisorPolicy::DefaultControlConfig());
    declare_component_parameters("perception", svo::SupervisorPolicy::DefaultPerceptionConfig());
    declare_component_parameters("lidar", svo::SupervisorPolicy::DefaultLidarConfig());
    declare_component_parameters(
      "localization", svo::SupervisorPolicy::DefaultLocalizationConfig());
    declare_component_parameters("power", svo::SupervisorPolicy::DefaultPowerConfig());
    load_policy();
    system_state_store_ = svo::SystemStateStore(system_state_path_);
    const auto persistent_state = system_state_store_.Load();
    const bool persistent_state_invalid =
      system_state_store_.enabled() &&
      persistent_state.error != "persistent_state_not_found";

    if (persistent_state.valid) {
      system_authority_.RestoreFaultLatch(
        persistent_state.fault_latched, persistent_state.generation, persistent_state.reason);
    } else if (persistent_state_invalid) {
      system_authority_.RestoreFaultLatch(
        true, 1U, "persistent_state_invalid:" + persistent_state.error);
      RCLCPP_ERROR(
        get_logger(), "Persistent supervisor state is invalid: %s",
        persistent_state.error.c_str());
    }

    if (!policy_.Validate()) {
      RCLCPP_FATAL(
        get_logger(), "Invalid supervisor policy: %s",
        policy_.ValidationError().c_str());
      throw std::runtime_error("invalid supervisor policy");
    }

    base_status_ = initialize_component(policy_.base);
    control_status_ = initialize_component(policy_.control);
    perception_status_ = initialize_component(policy_.perception);
    lidar_status_ = initialize_component(policy_.lidar);
    localization_status_ = initialize_component(policy_.localization);
    power_status_ = initialize_component(policy_.power);
    localization_parser_ = svo::LocalizationPayloadParser(
      policy_.localization.expected_schema_version);

    state_publisher_ = create_publisher<std_msgs::msg::String>(
      policy_.state_summary_topic, rclcpp::QoS(1).transient_local().reliable());
    heartbeat_publisher_ = create_publisher<std_msgs::msg::String>(
      policy_.heartbeat_topic, rclcpp::QoS(1).reliable());
    health_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      policy_.health_topic, rclcpp::QoS(1).reliable());
    events_publisher_ = create_publisher<std_msgs::msg::String>(
      policy_.events_topic, rclcpp::QoS(10).reliable());
    system_ready_publisher_ = create_publisher<std_msgs::msg::Bool>(
      system_ready_topic_, rclcpp::QoS(1).transient_local().reliable());
    remote_commands_ready_publisher_ = create_publisher<std_msgs::msg::Bool>(
      remote_commands_ready_topic_, rclcpp::QoS(1).transient_local().reliable());
    shutdown_requested_publisher_ = create_publisher<std_msgs::msg::Bool>(
      shutdown_requested_topic_, rclcpp::QoS(1).transient_local().reliable());

    location_authorization_service_ = create_service<savo_msgs::srv::AuthorizeLocationOperation>(
      location_authorization_service_name_,
      std::bind(&SupervisorNode::on_location_authorization, this,
        std::placeholders::_1, std::placeholders::_2));
    operation_authorization_service_ = create_service<savo_msgs::srv::AuthorizeOperation>(
      operation_authorization_service_name_,
      std::bind(&SupervisorNode::on_operation_authorization, this,
        std::placeholders::_1, std::placeholders::_2));
    map_context_service_ = create_service<savo_msgs::srv::UpdateMapContext>(
      map_context_service_name_,
      std::bind(&SupervisorNode::on_map_context_update, this,
        std::placeholders::_1, std::placeholders::_2));
    system_state_service_ = create_service<savo_msgs::srv::ManageSystemState>(
      system_state_service_name_,
      std::bind(&SupervisorNode::on_system_state_request, this,
        std::placeholders::_1, std::placeholders::_2));

    create_component_subscriptions();
    create_safety_subscriptions();
    create_mission_subscriptions();
    create_edge_subscriptions();
    create_action_clients();

    const auto period = std::chrono::duration<double>(1.0 / policy_.publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SupervisorNode::timer_callback, this));
    startup_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Phase 3 supervisor started: core, mission, edge and startup authority");
  }

private:
  void declare_global_parameters()
  {
    declare_parameter<double>("publish_rate_hz", 2.0);
    declare_parameter<double>("startup_grace_s", 3.0);
    declare_parameter<std::string>("state_summary_topic", "/savo_supervisor/state_summary");
    declare_parameter<std::string>("heartbeat_topic", "/savo_supervisor/heartbeat");
    declare_parameter<std::string>("health_topic", "/savo_supervisor/health");
    declare_parameter<std::string>("events_topic", "/savo_supervisor/events");
    declare_parameter<bool>("capabilities.manual_drive_requires_lidar", false);
    declare_parameter<bool>("capabilities.rotate_requires_lidar", false);
    declare_parameter<bool>("capabilities.geometric_mapping_requires_nominal_power", true);
    declare_parameter<std::string>("safety.stop_topic", "/safety/stop");
    declare_parameter<std::string>("safety.slowdown_topic", "/safety/slowdown_factor");
    declare_parameter<double>("safety.stop_timeout_s", 1.0);
    declare_parameter<double>("safety.slowdown_timeout_s", 1.0);

    declare_parameter<std::string>(
      "location_authorization.service_name", "/savo_supervisor/authorize_location_operation");
    declare_parameter<bool>("location_authorization.allow_registration", true);
    declare_parameter<bool>("location_authorization.allow_approval", true);
    declare_parameter<bool>("location_authorization.allow_rejection", true);
    declare_parameter<bool>("location_authorization.allow_navigation", true);
    declare_parameter<bool>("location_authorization.allow_arrival_confirmation", true);
    declare_parameter<bool>("location_authorization.allow_degraded_non_motion", true);
    declare_parameter<bool>("location_authorization.allow_degraded_motion", false);
    declare_parameter<bool>("location_authorization.require_known_safety_for_motion", true);

    declare_parameter<std::string>(
      "mission_authorization.service_name", "/savo_supervisor/authorize_operation");
    declare_parameter<std::string>(
      "map_context.service_name", "/savo_supervisor/update_map_context");
    declare_parameter<bool>("mission_authorization.allow_manual_control", true);
    declare_parameter<bool>("mission_authorization.allow_manual_mapping", true);
    declare_parameter<bool>("mission_authorization.allow_autonomous_mapping", true);
    declare_parameter<bool>("mission_authorization.allow_scan360", true);
    declare_parameter<bool>("mission_authorization.allow_coverage", true);
    declare_parameter<bool>("mission_authorization.allow_navigation", true);
    declare_parameter<bool>("mission_authorization.allow_location_registration", true);
    declare_parameter<bool>("mission_authorization.allow_location_review", true);
    declare_parameter<bool>("mission_authorization.allow_arrival_confirmation", true);
    declare_parameter<bool>(
      "mission_authorization.require_semantic_autonomous_mapping", true);
    declare_parameter<bool>(
      "mission_authorization.require_approved_release_for_navigation", true);

    declare_parameter<std::string>("mission.mapping_status_topic", "/savo_mapping/status");
    declare_parameter<double>("mission.mapping_status_timeout_s", 2.0);
    declare_parameter<std::string>("mission.navigation_status_topic", "/savo_nav/status");
    declare_parameter<std::string>("mission.navigation_heartbeat_topic", "/savo_nav/heartbeat");
    declare_parameter<double>("mission.navigation_status_timeout_s", 1.5);
    declare_parameter<double>("mission.navigation_heartbeat_timeout_s", 2.5);
    declare_parameter<std::string>("mission.head_status_topic", "/savo_head/status");
    declare_parameter<double>("mission.head_status_timeout_s", 1.5);
    declare_parameter<std::string>("mission.locations_status_topic", "/savo_locations/status");
    declare_parameter<std::string>(
      "mission.locations_heartbeat_topic", "/savo_locations/heartbeat");
    declare_parameter<double>("mission.locations_status_timeout_s", 2.0);
    declare_parameter<double>("mission.locations_heartbeat_timeout_s", 2.0);

    declare_parameter<std::string>(
      "mission.autonomous_mapping_action", "/savo_mapping/autonomous/run");
    declare_parameter<std::string>(
      "mission.rotate_to_heading_action", "/savo_control/rotate_to_heading");
    declare_parameter<std::string>(
      "mission.coverage_action", "/savo_nav/coverage/execute_path");
    declare_parameter<std::string>(
      "mission.apriltag_confirmation_action", "/savo_head/apriltag/confirm");

    declare_parameter<std::string>(
      "system_authority.service_name", "/savo_supervisor/manage_system_state");
    declare_parameter<bool>("system_authority.auto_arm", false);
    declare_parameter<bool>("system_authority.latch_core_faults_after_arm", true);
    declare_parameter<std::string>(
      "system_authority.state_path", "/var/lib/robot_savo/supervisor/system_state.json");
    declare_parameter<std::string>(
      "system_authority.system_ready_topic", "/savo_supervisor/system_ready");
    declare_parameter<std::string>(
      "system_authority.remote_commands_ready_topic",
      "/savo_supervisor/remote_commands_ready");
    declare_parameter<std::string>(
      "system_authority.shutdown_requested_topic",
      "/savo_supervisor/shutdown_requested");
    declare_parameter<std::vector<std::string>>(
      "system_authority.allowed_actor_prefixes",
      std::vector<std::string>{"system_operator", "savo_bringup", "operator_app"});

    declare_parameter<bool>("edge.bridge.enabled", true);
    declare_parameter<bool>("edge.bridge.required_for_startup", true);
    declare_parameter<std::string>("edge.bridge.state_topic", "/savo_bridge/state");
    declare_parameter<std::string>("edge.bridge.readiness_topic", "/savo_bridge/readiness");
    declare_parameter<std::string>("edge.bridge.heartbeat_topic", "/savo_bridge/heartbeat");
    declare_parameter<double>("edge.bridge.state_timeout_s", 2.0);
    declare_parameter<double>("edge.bridge.heartbeat_timeout_s", 2.5);

    declare_parameter<bool>("edge.realsense.enabled", true);
    declare_parameter<bool>("edge.realsense.required_for_startup", false);
    declare_parameter<std::string>("edge.realsense.status_topic", "/realsense/status");
    declare_parameter<double>("edge.realsense.status_timeout_s", 2.0);

    declare_parameter<bool>("edge.speech.enabled", true);
    declare_parameter<bool>("edge.speech.required_for_startup", false);
    declare_parameter<std::string>("edge.speech.readiness_topic", "/savo_speech/readiness");
    declare_parameter<std::string>("edge.speech.heartbeat_topic", "/savo_speech/heartbeat");
    declare_parameter<double>("edge.speech.readiness_timeout_s", 3.0);
    declare_parameter<double>("edge.speech.heartbeat_timeout_s", 3.0);

    declare_parameter<bool>("edge.vo.enabled", true);
    declare_parameter<bool>("edge.vo.required_for_startup", false);
    declare_parameter<std::string>("edge.vo.health_topic", "/vo/health");
    declare_parameter<double>("edge.vo.health_timeout_s", 2.0);

    declare_parameter<bool>("edge.ui.enabled", true);
    declare_parameter<bool>("edge.ui.required_for_startup", false);
    declare_parameter<std::vector<std::string>>(
      "edge.ui.expected_nodes", std::vector<std::string>{"/savo_ui_node"});
    declare_parameter<std::vector<std::string>>(
      "edge.remote_actor_prefixes",
      std::vector<std::string>{"savo_bridge", "savomind", "operator_app", "savo_speech"});
  }

  void declare_component_parameters(
    const std::string & prefix,
    const svo::ComponentConfig & defaults)
  {
    declare_parameter<bool>(prefix + ".enabled", defaults.enabled);
    declare_parameter<bool>(prefix + ".required", defaults.required);
    declare_parameter<std::string>(prefix + ".health_topic", defaults.health_topic);
    declare_parameter<std::string>(prefix + ".summary_topic", defaults.summary_topic);
    declare_parameter<std::string>(prefix + ".heartbeat_topic", defaults.heartbeat_topic);
    declare_parameter<double>(prefix + ".health_timeout_s", defaults.health_timeout_s);
    declare_parameter<double>(prefix + ".summary_timeout_s", defaults.summary_timeout_s);
    declare_parameter<double>(prefix + ".heartbeat_timeout_s", defaults.heartbeat_timeout_s);
    declare_parameter<int>(prefix + ".expected_schema_version", defaults.expected_schema_version);
  }

  svo::ComponentConfig load_component(
    const std::string & prefix,
    svo::ComponentConfig config) const
  {
    config.enabled = get_parameter(prefix + ".enabled").as_bool();
    config.required = get_parameter(prefix + ".required").as_bool();
    config.health_topic = get_parameter(prefix + ".health_topic").as_string();
    config.summary_topic = get_parameter(prefix + ".summary_topic").as_string();
    config.heartbeat_topic = get_parameter(prefix + ".heartbeat_topic").as_string();
    config.health_timeout_s = get_parameter(prefix + ".health_timeout_s").as_double();
    config.summary_timeout_s = get_parameter(prefix + ".summary_timeout_s").as_double();
    config.heartbeat_timeout_s = get_parameter(prefix + ".heartbeat_timeout_s").as_double();
    config.expected_schema_version = static_cast<int>(
      get_parameter(prefix + ".expected_schema_version").as_int());
    return config;
  }

  void load_policy()
  {
    policy_.publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    policy_.startup_grace_s = get_parameter("startup_grace_s").as_double();
    policy_.state_summary_topic = get_parameter("state_summary_topic").as_string();
    policy_.heartbeat_topic = get_parameter("heartbeat_topic").as_string();
    policy_.health_topic = get_parameter("health_topic").as_string();
    policy_.events_topic = get_parameter("events_topic").as_string();
    policy_.manual_drive_requires_lidar =
      get_parameter("capabilities.manual_drive_requires_lidar").as_bool();
    policy_.rotate_requires_lidar = get_parameter("capabilities.rotate_requires_lidar").as_bool();
    policy_.geometric_mapping_requires_nominal_power =
      get_parameter("capabilities.geometric_mapping_requires_nominal_power").as_bool();

    policy_.base = load_component("base", svo::SupervisorPolicy::DefaultBaseConfig());
    policy_.control = load_component("control", svo::SupervisorPolicy::DefaultControlConfig());
    policy_.perception = load_component(
      "perception", svo::SupervisorPolicy::DefaultPerceptionConfig());
    policy_.lidar = load_component("lidar", svo::SupervisorPolicy::DefaultLidarConfig());
    policy_.localization = load_component(
      "localization", svo::SupervisorPolicy::DefaultLocalizationConfig());
    policy_.power = load_component("power", svo::SupervisorPolicy::DefaultPowerConfig());

    safety_stop_topic_ = get_parameter("safety.stop_topic").as_string();
    safety_slowdown_topic_ = get_parameter("safety.slowdown_topic").as_string();
    safety_stop_timeout_s_ = get_parameter("safety.stop_timeout_s").as_double();
    safety_slowdown_timeout_s_ = get_parameter("safety.slowdown_timeout_s").as_double();
    if (safety_stop_topic_.empty() || safety_slowdown_topic_.empty() ||
      !std::isfinite(safety_stop_timeout_s_) || safety_stop_timeout_s_ <= 0.0 ||
      !std::isfinite(safety_slowdown_timeout_s_) || safety_slowdown_timeout_s_ <= 0.0)
    {
      throw std::runtime_error("invalid safety observation configuration");
    }

    location_authorization_service_name_ =
      get_parameter("location_authorization.service_name").as_string();
    location_authorization_policy_.allow_registration =
      get_parameter("location_authorization.allow_registration").as_bool();
    location_authorization_policy_.allow_approval =
      get_parameter("location_authorization.allow_approval").as_bool();
    location_authorization_policy_.allow_rejection =
      get_parameter("location_authorization.allow_rejection").as_bool();
    location_authorization_policy_.allow_navigation =
      get_parameter("location_authorization.allow_navigation").as_bool();
    location_authorization_policy_.allow_arrival_confirmation =
      get_parameter("location_authorization.allow_arrival_confirmation").as_bool();
    location_authorization_policy_.allow_degraded_non_motion =
      get_parameter("location_authorization.allow_degraded_non_motion").as_bool();
    location_authorization_policy_.allow_degraded_motion =
      get_parameter("location_authorization.allow_degraded_motion").as_bool();
    location_authorization_policy_.require_known_safety_for_motion =
      get_parameter("location_authorization.require_known_safety_for_motion").as_bool();
    location_authorization_evaluator_ =
      svo::LocationAuthorizationEvaluator(location_authorization_policy_);

    operation_authorization_service_name_ =
      get_parameter("mission_authorization.service_name").as_string();
    map_context_service_name_ = get_parameter("map_context.service_name").as_string();
    mission_authority_policy_.allow_manual_control =
      get_parameter("mission_authorization.allow_manual_control").as_bool();
    mission_authority_policy_.allow_manual_mapping =
      get_parameter("mission_authorization.allow_manual_mapping").as_bool();
    mission_authority_policy_.allow_autonomous_mapping =
      get_parameter("mission_authorization.allow_autonomous_mapping").as_bool();
    mission_authority_policy_.allow_scan360 =
      get_parameter("mission_authorization.allow_scan360").as_bool();
    mission_authority_policy_.allow_coverage =
      get_parameter("mission_authorization.allow_coverage").as_bool();
    mission_authority_policy_.allow_navigation =
      get_parameter("mission_authorization.allow_navigation").as_bool();
    mission_authority_policy_.allow_location_registration =
      get_parameter("mission_authorization.allow_location_registration").as_bool();
    mission_authority_policy_.allow_location_review =
      get_parameter("mission_authorization.allow_location_review").as_bool();
    mission_authority_policy_.allow_arrival_confirmation =
      get_parameter("mission_authorization.allow_arrival_confirmation").as_bool();
    mission_authority_policy_.require_semantic_autonomous_mapping =
      get_parameter("mission_authorization.require_semantic_autonomous_mapping").as_bool();
    mission_authority_policy_.require_approved_release_for_navigation =
      get_parameter("mission_authorization.require_approved_release_for_navigation").as_bool();
    mission_authority_ = svo::MissionAuthority(mission_authority_policy_);

    mapping_status_topic_ = get_parameter("mission.mapping_status_topic").as_string();
    mapping_status_timeout_s_ = get_parameter("mission.mapping_status_timeout_s").as_double();
    navigation_status_topic_ = get_parameter("mission.navigation_status_topic").as_string();
    navigation_heartbeat_topic_ =
      get_parameter("mission.navigation_heartbeat_topic").as_string();
    navigation_status_timeout_s_ =
      get_parameter("mission.navigation_status_timeout_s").as_double();
    navigation_heartbeat_timeout_s_ =
      get_parameter("mission.navigation_heartbeat_timeout_s").as_double();
    head_status_topic_ = get_parameter("mission.head_status_topic").as_string();
    head_status_timeout_s_ = get_parameter("mission.head_status_timeout_s").as_double();
    locations_status_topic_ = get_parameter("mission.locations_status_topic").as_string();
    locations_heartbeat_topic_ =
      get_parameter("mission.locations_heartbeat_topic").as_string();
    locations_status_timeout_s_ =
      get_parameter("mission.locations_status_timeout_s").as_double();
    locations_heartbeat_timeout_s_ =
      get_parameter("mission.locations_heartbeat_timeout_s").as_double();
    autonomous_mapping_action_name_ =
      get_parameter("mission.autonomous_mapping_action").as_string();
    rotate_to_heading_action_name_ =
      get_parameter("mission.rotate_to_heading_action").as_string();
    coverage_action_name_ = get_parameter("mission.coverage_action").as_string();
    apriltag_confirmation_action_name_ =
      get_parameter("mission.apriltag_confirmation_action").as_string();

    system_state_service_name_ =
      get_parameter("system_authority.service_name").as_string();
    system_authority_policy_.auto_arm =
      get_parameter("system_authority.auto_arm").as_bool();
    system_authority_policy_.latch_core_faults_after_arm =
      get_parameter("system_authority.latch_core_faults_after_arm").as_bool();
    system_authority_ = svo::SystemAuthority(system_authority_policy_);
    system_state_path_ = get_parameter("system_authority.state_path").as_string();
    system_ready_topic_ =
      get_parameter("system_authority.system_ready_topic").as_string();
    remote_commands_ready_topic_ =
      get_parameter("system_authority.remote_commands_ready_topic").as_string();
    shutdown_requested_topic_ =
      get_parameter("system_authority.shutdown_requested_topic").as_string();
    system_actor_prefixes_ =
      get_parameter("system_authority.allowed_actor_prefixes").as_string_array();

    edge_policy_.bridge_enabled = get_parameter("edge.bridge.enabled").as_bool();
    edge_policy_.bridge_required_for_startup =
      get_parameter("edge.bridge.required_for_startup").as_bool();
    edge_policy_.realsense_enabled = get_parameter("edge.realsense.enabled").as_bool();
    edge_policy_.realsense_required_for_startup =
      get_parameter("edge.realsense.required_for_startup").as_bool();
    edge_policy_.speech_enabled = get_parameter("edge.speech.enabled").as_bool();
    edge_policy_.speech_required_for_startup =
      get_parameter("edge.speech.required_for_startup").as_bool();
    edge_policy_.vo_enabled = get_parameter("edge.vo.enabled").as_bool();
    edge_policy_.vo_required_for_startup =
      get_parameter("edge.vo.required_for_startup").as_bool();
    edge_policy_.ui_enabled = get_parameter("edge.ui.enabled").as_bool();
    edge_policy_.ui_required_for_startup =
      get_parameter("edge.ui.required_for_startup").as_bool();
    edge_supervision_ = svo::EdgeSupervision(edge_policy_);

    bridge_state_topic_ = get_parameter("edge.bridge.state_topic").as_string();
    bridge_readiness_topic_ = get_parameter("edge.bridge.readiness_topic").as_string();
    bridge_heartbeat_topic_ = get_parameter("edge.bridge.heartbeat_topic").as_string();
    bridge_state_timeout_s_ = get_parameter("edge.bridge.state_timeout_s").as_double();
    bridge_heartbeat_timeout_s_ =
      get_parameter("edge.bridge.heartbeat_timeout_s").as_double();
    realsense_status_topic_ = get_parameter("edge.realsense.status_topic").as_string();
    realsense_status_timeout_s_ =
      get_parameter("edge.realsense.status_timeout_s").as_double();
    speech_readiness_topic_ = get_parameter("edge.speech.readiness_topic").as_string();
    speech_heartbeat_topic_ = get_parameter("edge.speech.heartbeat_topic").as_string();
    speech_readiness_timeout_s_ =
      get_parameter("edge.speech.readiness_timeout_s").as_double();
    speech_heartbeat_timeout_s_ =
      get_parameter("edge.speech.heartbeat_timeout_s").as_double();
    vo_health_topic_ = get_parameter("edge.vo.health_topic").as_string();
    vo_health_timeout_s_ = get_parameter("edge.vo.health_timeout_s").as_double();
    ui_expected_nodes_ = get_parameter("edge.ui.expected_nodes").as_string_array();
    remote_actor_prefixes_ = get_parameter("edge.remote_actor_prefixes").as_string_array();
    validate_phase3_configuration();
  }

  void validate_phase3_configuration() const
  {
    const auto valid_timeout = [](const bool enabled, const double timeout_s) {
        return !enabled || (std::isfinite(timeout_s) && timeout_s > 0.0);
      };
    const auto valid_topic = [](const bool enabled, const std::string & topic) {
        return !enabled || !topic.empty();
      };
    const bool required_component_disabled =
      (edge_policy_.bridge_required_for_startup && !edge_policy_.bridge_enabled) ||
      (edge_policy_.realsense_required_for_startup && !edge_policy_.realsense_enabled) ||
      (edge_policy_.speech_required_for_startup && !edge_policy_.speech_enabled) ||
      (edge_policy_.vo_required_for_startup && !edge_policy_.vo_enabled) ||
      (edge_policy_.ui_required_for_startup && !edge_policy_.ui_enabled);
    if (required_component_disabled) {
      throw std::runtime_error("required edge component cannot be disabled");
    }
    if (system_state_service_name_.empty() || system_ready_topic_.empty() ||
      remote_commands_ready_topic_.empty() || shutdown_requested_topic_.empty() ||
      system_actor_prefixes_.empty())
    {
      throw std::runtime_error("invalid system authority topic or service");
    }
    const std::unordered_set<std::string> output_topics{
      policy_.state_summary_topic,
      policy_.heartbeat_topic,
      policy_.health_topic,
      policy_.events_topic,
      system_ready_topic_,
      remote_commands_ready_topic_,
      shutdown_requested_topic_};
    if (output_topics.size() != 7U) {
      throw std::runtime_error("duplicate supervisor output topic");
    }
    const bool topics_valid =
      valid_topic(edge_policy_.bridge_enabled, bridge_state_topic_) &&
      valid_topic(edge_policy_.bridge_enabled, bridge_readiness_topic_) &&
      valid_topic(edge_policy_.bridge_enabled, bridge_heartbeat_topic_) &&
      valid_topic(edge_policy_.realsense_enabled, realsense_status_topic_) &&
      valid_topic(edge_policy_.speech_enabled, speech_readiness_topic_) &&
      valid_topic(edge_policy_.speech_enabled, speech_heartbeat_topic_) &&
      valid_topic(edge_policy_.vo_enabled, vo_health_topic_) &&
      (!edge_policy_.ui_enabled || !ui_expected_nodes_.empty());
    const bool timeouts_valid =
      valid_timeout(edge_policy_.bridge_enabled, bridge_state_timeout_s_) &&
      valid_timeout(edge_policy_.bridge_enabled, bridge_heartbeat_timeout_s_) &&
      valid_timeout(edge_policy_.realsense_enabled, realsense_status_timeout_s_) &&
      valid_timeout(edge_policy_.speech_enabled, speech_readiness_timeout_s_) &&
      valid_timeout(edge_policy_.speech_enabled, speech_heartbeat_timeout_s_) &&
      valid_timeout(edge_policy_.vo_enabled, vo_health_timeout_s_);
    if (!topics_valid || !timeouts_valid) {
      throw std::runtime_error("invalid edge supervision configuration");
    }
  }

  template<typename Callback>
  void subscribe_string(const std::string & topic, Callback callback)
  {
    if (topic.empty()) {
      return;
    }
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(10).reliable(), callback));
  }

  void create_component_subscriptions()
  {
    if (policy_.base.enabled) {
      subscribe_string(
        policy_.base.summary_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            base_status_, Channel::kSummary,
            core_parser_.ParseBaseState(msg->data), now());
        });
    }
    if (policy_.control.enabled) {
      subscribe_string(
        policy_.control.health_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            control_status_, Channel::kHealth,
            core_parser_.ParseControlStatus(msg->data), now());
        });
      subscribe_string(
        policy_.control.summary_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            control_status_, Channel::kSummary,
            core_parser_.ParseControlStatus(msg->data), now());
        });
      subscribe_string(
        policy_.control.heartbeat_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            control_status_, Channel::kHeartbeat,
            core_parser_.ParseControlStatus(msg->data), now());
        });
    }
    if (policy_.perception.enabled) {
      subscribe_string(
        policy_.perception.health_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            perception_status_, Channel::kHealth,
            core_parser_.ParsePerceptionHealth(msg->data), now());
        });
      subscribe_string(
        policy_.perception.summary_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            perception_status_, Channel::kSummary,
            core_parser_.ParsePerceptionSafetyState(msg->data), now());
        });
      subscribe_string(
        policy_.perception.heartbeat_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            perception_status_, Channel::kHeartbeat,
            core_parser_.ParsePerceptionHeartbeat(msg->data), now());
        });
    }
    if (policy_.lidar.enabled) {
      subscribe_string(
        policy_.lidar.summary_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            lidar_status_, Channel::kSummary,
            core_parser_.ParseLidarState(msg->data), now());
        });
      subscribe_string(
        policy_.lidar.heartbeat_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            lidar_status_, Channel::kHeartbeat,
            core_parser_.ParseLidarHeartbeat(msg->data), now());
        });
    }
    if (policy_.localization.enabled) {
      subscribe_localization_inputs();
    }
    if (policy_.power.enabled) {
      subscribe_string(
        policy_.power.health_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            power_status_, Channel::kHealth,
            core_parser_.ParsePowerHealth(msg->data), now());
        });
      subscribe_string(
        policy_.power.summary_topic,
        [this](std_msgs::msg::String::SharedPtr msg) {
          apply_payload(
            power_status_, Channel::kSummary,
            core_parser_.ParsePowerStatus(msg->data), now());
        });
    }
  }

  static svo::ParsedCorePayload normalize_localization_payload(
    const svo::ParsedLocalizationPayload & parsed,
    bool alive,
    bool degraded)
  {
    svo::ParsedCorePayload normalized;
    normalized.valid = parsed.valid;
    normalized.state = parsed.state;
    normalized.ready = parsed.ready;
    normalized.degraded = degraded;
    normalized.alive = alive;
    normalized.reason_code = parsed.reason_code;
    normalized.stamp = parsed.stamp;
    normalized.detail = parsed.detail;
    return normalized;
  }

  void subscribe_localization_inputs()
  {
    subscribe_string(
      policy_.localization.health_topic,
      [this](std_msgs::msg::String::SharedPtr msg) {
        const auto parsed = localization_parser_.ParseHealth(msg->data);
        apply_payload(
          localization_status_, Channel::kHealth,
          normalize_localization_payload(parsed, true, parsed.degraded), now());
      });
    subscribe_string(
      policy_.localization.summary_topic,
      [this](std_msgs::msg::String::SharedPtr msg) {
        const auto parsed = localization_parser_.ParseSummary(msg->data);
        apply_payload(
          localization_status_, Channel::kSummary,
          normalize_localization_payload(parsed, true, parsed.degraded), now());
      });
    subscribe_string(
      policy_.localization.heartbeat_topic,
      [this](std_msgs::msg::String::SharedPtr msg) {
        const auto parsed = localization_parser_.ParseHeartbeat(msg->data);
        apply_payload(
          localization_status_, Channel::kHeartbeat,
          normalize_localization_payload(parsed, parsed.alive, false), now());
      });
  }

  void create_safety_subscriptions()
  {
    subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_, rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::Bool::SharedPtr msg) {
          safety_stop_active_ = msg->data;
          safety_stop_valid_ = true;
          safety_stop_tracker_.observe_message(now(), std::nullopt, false, "");
      }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::Float32>(
      safety_slowdown_topic_, rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::Float32::SharedPtr msg) {
          const double value = static_cast<double>(msg->data);
          const bool valid = std::isfinite(value) && value >= 0.0 && value <= 1.0;
          safety_slowdown_valid_ = valid;
          if (valid) {safety_slowdown_factor_ = value;}
          safety_slowdown_tracker_.observe_message(
          now(), std::nullopt, !valid, valid ? "" : "slowdown factor outside [0,1]");
      }));
  }

  void create_mission_subscriptions()
  {
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
      mapping_status_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](std_msgs::msg::String::SharedPtr msg) {
          mapping_observation_ = mission_parser_.ParseMappingStatus(msg->data);
          mapping_status_tracker_.observe_message(
          now(), std::nullopt, !mapping_observation_.valid, mapping_observation_.reason);
      }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
      navigation_status_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](std_msgs::msg::String::SharedPtr msg) {
          navigation_observation_ = mission_parser_.ParseNavigationStatus(msg->data);
          navigation_status_tracker_.observe_message(
          now(), std::nullopt, !navigation_observation_.valid,
          navigation_observation_.reason);
      }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::UInt64>(
      navigation_heartbeat_topic_, rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::UInt64::SharedPtr) {
          navigation_heartbeat_tracker_.observe_message(now(), std::nullopt, false, "");
      }));
    subscriptions_.push_back(create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      head_status_topic_, rclcpp::QoS(10).reliable(),
        [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
          head_observation_ = mission_parser_.ParseHeadStatus(*msg);
          head_status_tracker_.observe_message(
          now(), msg->header.stamp, !head_observation_.valid, head_observation_.reason);
      }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
      locations_status_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](std_msgs::msg::String::SharedPtr msg) {
          locations_observation_ = mission_parser_.ParseLocationsStatus(msg->data);
          locations_status_tracker_.observe_message(
          now(), std::nullopt, !locations_observation_.valid,
          locations_observation_.reason);
      }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::UInt64>(
      locations_heartbeat_topic_, rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::UInt64::SharedPtr) {
          locations_heartbeat_tracker_.observe_message(now(), std::nullopt, false, "");
      }));
  }

  void create_edge_subscriptions()
  {
    if (edge_policy_.bridge_enabled) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        bridge_state_topic_, rclcpp::QoS(1).transient_local().reliable(),
          [this](std_msgs::msg::String::SharedPtr msg) {
            bridge_observation_ = edge_parser_.ParseBridgeState(msg->data);
            bridge_state_tracker_.observe_message(
            now(), std::nullopt, !bridge_observation_.valid, bridge_observation_.reason);
        }));
      subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
        bridge_readiness_topic_, rclcpp::QoS(1).transient_local().reliable(),
          [this](std_msgs::msg::Bool::SharedPtr msg) {
            bridge_readiness_asserted_ = msg->data;
            bridge_readiness_tracker_.observe_message(now(), std::nullopt, false, "");
        }));
      subscriptions_.push_back(create_subscription<std_msgs::msg::UInt64>(
        bridge_heartbeat_topic_, rclcpp::QoS(10).reliable(),
          [this](std_msgs::msg::UInt64::SharedPtr) {
            bridge_heartbeat_tracker_.observe_message(now(), std::nullopt, false, "");
        }));
    }
    if (edge_policy_.realsense_enabled) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        realsense_status_topic_, rclcpp::QoS(1).transient_local().reliable(),
          [this](std_msgs::msg::String::SharedPtr msg) {
            realsense_observation_ = edge_parser_.ParseRealSenseStatus(msg->data);
            realsense_status_tracker_.observe_message(
            now(), std::nullopt, !realsense_observation_.valid,
            realsense_observation_.reason);
        }));
    }
    if (edge_policy_.speech_enabled) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        speech_readiness_topic_, rclcpp::QoS(1).transient_local().reliable(),
          [this](std_msgs::msg::String::SharedPtr msg) {
            speech_observation_ = edge_parser_.ParseSpeechReadiness(msg->data);
            speech_readiness_tracker_.observe_message(
            now(), std::nullopt, !speech_observation_.valid, speech_observation_.reason);
        }));
      subscriptions_.push_back(create_subscription<std_msgs::msg::UInt64>(
        speech_heartbeat_topic_, rclcpp::QoS(10).reliable(),
          [this](std_msgs::msg::UInt64::SharedPtr) {
            speech_heartbeat_tracker_.observe_message(now(), std::nullopt, false, "");
        }));
    }
    if (edge_policy_.vo_enabled) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        vo_health_topic_, rclcpp::QoS(1).transient_local().reliable(),
          [this](std_msgs::msg::String::SharedPtr msg) {
            vo_observation_ = edge_parser_.ParseVoHealth(msg->data);
            vo_health_tracker_.observe_message(
            now(), std::nullopt, !vo_observation_.valid, vo_observation_.reason);
        }));
    }
  }

  bool ui_graph_visible()
  {
    if (!edge_policy_.ui_enabled) {
      return true;
    }
    std::unordered_set<std::string> visible;
    for (const auto & item :
      get_node_graph_interface()->get_node_names_and_namespaces())
    {
      const auto & name = item.first;
      const auto & node_namespace = item.second;
      const std::string full_name = node_namespace == "/" ?
        "/" + name : node_namespace + "/" + name;
      visible.insert(full_name);
    }
    return std::any_of(
      ui_expected_nodes_.begin(), ui_expected_nodes_.end(),
      [&visible](const std::string & expected) {
        return visible.count(expected) > 0U;
      });
  }

  bool is_remote_actor(const std::string & actor_id) const
  {
    return std::any_of(
      remote_actor_prefixes_.begin(), remote_actor_prefixes_.end(),
      [&actor_id](const std::string & prefix) {
        return !prefix.empty() && actor_id.rfind(prefix, 0U) == 0U;
      });
  }

  bool is_system_actor(const std::string & actor_id) const
  {
    return std::any_of(
      system_actor_prefixes_.begin(), system_actor_prefixes_.end(),
      [&actor_id](const std::string & prefix) {
        return !prefix.empty() && actor_id.rfind(prefix, 0U) == 0U;
      });
  }

  svo::OperatingMode effective_operating_mode(
    const svo::SupervisorState & core) const noexcept
  {
    if (latest_system_snapshot_.shutdown_requested) {
      return svo::OperatingMode::SHUTTING_DOWN;
    }
    if (latest_system_snapshot_.fault_latched) {
      return svo::OperatingMode::ERROR;
    }
    if (!latest_system_snapshot_.armed) {
      return latest_system_snapshot_.state == svo::SystemAuthorityState::kBooting ?
             svo::OperatingMode::BOOTING : svo::OperatingMode::STOP;
    }
    return svo::ModeForAuthority(core, mission_authority_.state());
  }

  svo::EdgeSupervisionState evaluate_edge_state(const rclcpp::Time & evaluation_time)
  {
    auto bridge = bridge_observation_;
    const auto bridge_state = bridge_state_tracker_.snapshot(
      evaluation_time, bridge_state_timeout_s_);
    const auto bridge_readiness = bridge_readiness_tracker_.snapshot(
      evaluation_time, bridge_state_timeout_s_);
    const auto bridge_heartbeat = bridge_heartbeat_tracker_.snapshot(
      evaluation_time, bridge_heartbeat_timeout_s_);
    bridge.fresh = bridge_state.received && !bridge_state.stale && bridge_state.valid &&
      bridge_readiness.received && !bridge_readiness.stale && bridge_readiness.valid;
    bridge.readiness_asserted = bridge_readiness_asserted_;
    bridge.heartbeat_fresh = bridge_heartbeat.received && !bridge_heartbeat.stale &&
      bridge_heartbeat.valid;
    if (!bridge_state.received) {
      bridge.reason = "bridge_state_missing";
    } else if (bridge_state.stale) {
      bridge.reason = "bridge_state_stale";
    } else if (!bridge_state.valid) {
      bridge.reason = bridge_observation_.reason;
    } else if (!bridge_readiness.received) {
      bridge.reason = "bridge_readiness_missing";
    } else if (bridge_readiness.stale) {
      bridge.reason = "bridge_readiness_stale";
    } else if (!bridge_readiness.valid) {
      bridge.reason = "bridge_readiness_invalid";
    } else if (!bridge.readiness_asserted) {
      bridge.reason = "bridge_readiness_false";
    } else if (!bridge_heartbeat.received) {
      bridge.reason = "bridge_heartbeat_missing";
    } else if (bridge_heartbeat.stale) {
      bridge.reason = "bridge_heartbeat_stale";
    } else if (!bridge_heartbeat.valid) {
      bridge.reason = "bridge_heartbeat_invalid";
    }

    auto realsense = realsense_observation_;
    const auto realsense_status = realsense_status_tracker_.snapshot(
      evaluation_time, realsense_status_timeout_s_);
    realsense.fresh = realsense_status.received && !realsense_status.stale &&
      realsense_status.valid;
    if (!realsense_status.received) {
      realsense.reason = "realsense_status_missing";
    } else if (realsense_status.stale) {
      realsense.reason = "realsense_status_stale";
    } else if (!realsense_status.valid) {
      realsense.reason = realsense_observation_.reason;
    }

    auto speech = speech_observation_;
    const auto speech_readiness = speech_readiness_tracker_.snapshot(
      evaluation_time, speech_readiness_timeout_s_);
    const auto speech_heartbeat = speech_heartbeat_tracker_.snapshot(
      evaluation_time, speech_heartbeat_timeout_s_);
    speech.fresh = speech_readiness.received && !speech_readiness.stale &&
      speech_readiness.valid;
    speech.heartbeat_fresh = speech_heartbeat.received && !speech_heartbeat.stale &&
      speech_heartbeat.valid;
    if (!speech_readiness.received) {
      speech.reason = "speech_readiness_missing";
    } else if (speech_readiness.stale) {
      speech.reason = "speech_readiness_stale";
    } else if (!speech_readiness.valid) {
      speech.reason = speech_observation_.reason;
    } else if (!speech_heartbeat.received) {
      speech.reason = "speech_heartbeat_missing";
    } else if (speech_heartbeat.stale) {
      speech.reason = "speech_heartbeat_stale";
    } else if (!speech_heartbeat.valid) {
      speech.reason = "speech_heartbeat_invalid";
    }

    auto vo = vo_observation_;
    const auto vo_health = vo_health_tracker_.snapshot(evaluation_time, vo_health_timeout_s_);
    vo.fresh = vo_health.received && !vo_health.stale && vo_health.valid;
    if (!vo_health.received) {
      vo.reason = "vo_health_missing";
    } else if (vo_health.stale) {
      vo.reason = "vo_health_stale";
    } else if (!vo_health.valid) {
      vo.reason = vo_observation_.reason;
    }

    svo::UiObservation ui;
    ui.enabled = edge_policy_.ui_enabled;
    ui.graph_visible = ui_graph_visible();
    ui.reason = ui.graph_visible ? "ui_node_visible" : "ui_node_not_visible";
    return edge_supervision_.Evaluate(bridge, realsense, speech, vo, ui);
  }

  svo::SystemDependencySnapshot make_system_dependencies(
    const svo::SupervisorState & core,
    const svo::EdgeSupervisionState & edge) const
  {
    svo::SystemDependencySnapshot dependencies;
    dependencies.core_ready = core.lifecycle == svo::Lifecycle::RUNNING &&
      core.ready && core.capabilities.core_health_ready && core.capabilities.core_safety_ready;
    dependencies.core_faulted = core.lifecycle == svo::Lifecycle::FAULTED ||
      core.health == svo::AggregateHealth::ERROR;
    dependencies.safety_known = core.safety != svo::SafetyObservation::UNKNOWN;
    dependencies.startup_dependencies_ready = edge.capabilities.edge_startup_ready;
    dependencies.degraded = core.degraded || edge.degraded;
    dependencies.remote_commands_ready = edge.capabilities.remote_command_path_ready;
    dependencies.mission_idle = mission_authority_.state().state == svo::OperationState::kIdle;
    return dependencies;
  }

  void persist_system_state()
  {
    if (!system_state_store_.enabled()) {
      return;
    }
    std::string error;
    if (!system_state_store_.Save(
        latest_system_snapshot_.fault_latched, latest_system_snapshot_.generation,
        latest_system_snapshot_.reason, error))
    {
      RCLCPP_ERROR(get_logger(), "Failed to persist supervisor state: %s", error.c_str());
    }
  }

  void publish_system_event(const std::string & event_type, const std::string & reason)
  {
    nlohmann::json json{
      {"schema_version", 1},
      {"node", "savo_supervisor"},
      {"event_type", event_type},
      {"system_state", svo::ToString(latest_system_snapshot_.state)},
      {"system_generation", latest_system_snapshot_.generation},
      {"system_armed", latest_system_snapshot_.armed},
      {"fault_latched", latest_system_snapshot_.fault_latched},
      {"shutdown_requested", latest_system_snapshot_.shutdown_requested},
      {"reason", reason},
      {"stamp_s", now().seconds()}
    };
    std_msgs::msg::String event;
    event.data = json.dump();
    events_publisher_->publish(event);
  }

  void create_action_clients()
  {
    autonomous_mapping_client_ =
      rclcpp_action::create_client<savo_msgs::action::RunAutonomousMapping>(
      this, autonomous_mapping_action_name_);
    rotate_to_heading_client_ =
      rclcpp_action::create_client<savo_msgs::action::RotateToHeading>(
      this, rotate_to_heading_action_name_);
    coverage_client_ =
      rclcpp_action::create_client<savo_msgs::action::ExecuteCoveragePath>(
      this, coverage_action_name_);
    apriltag_confirmation_client_ =
      rclcpp_action::create_client<savo_msgs::action::ConfirmAprilTag>(
      this, apriltag_confirmation_action_name_);
  }

  svo::MissionDependencySnapshot evaluate_mission_dependencies(
    const svo::SupervisorState & core,
    const rclcpp::Time & evaluation_time)
  {
    svo::MissionDependencySnapshot dependencies;
    dependencies.core = core;
    dependencies.map_context = map_context_;

    const auto mapping_freshness = mapping_status_tracker_.snapshot(
      evaluation_time, mapping_status_timeout_s_);
    dependencies.mapping = mapping_observation_;
    dependencies.mapping.fresh = mapping_freshness.received &&
      !mapping_freshness.stale && mapping_freshness.valid;

    const auto navigation_status_freshness = navigation_status_tracker_.snapshot(
      evaluation_time, navigation_status_timeout_s_);
    const auto navigation_heartbeat_freshness = navigation_heartbeat_tracker_.snapshot(
      evaluation_time, navigation_heartbeat_timeout_s_);
    dependencies.navigation = navigation_observation_;
    dependencies.navigation.fresh = navigation_status_freshness.received &&
      !navigation_status_freshness.stale && navigation_status_freshness.valid &&
      navigation_heartbeat_freshness.received && !navigation_heartbeat_freshness.stale &&
      navigation_heartbeat_freshness.valid;

    const auto head_freshness = head_status_tracker_.snapshot(
      evaluation_time, head_status_timeout_s_);
    dependencies.head = head_observation_;
    dependencies.head.fresh = head_freshness.received && !head_freshness.stale &&
      head_freshness.valid;

    const auto locations_status_freshness = locations_status_tracker_.snapshot(
      evaluation_time, locations_status_timeout_s_);
    const auto locations_heartbeat_freshness = locations_heartbeat_tracker_.snapshot(
      evaluation_time, locations_heartbeat_timeout_s_);
    dependencies.locations = locations_observation_;
    dependencies.locations.fresh = locations_status_freshness.received &&
      !locations_status_freshness.stale && locations_status_freshness.valid &&
      locations_heartbeat_freshness.received && !locations_heartbeat_freshness.stale &&
      locations_heartbeat_freshness.valid;

    dependencies.endpoints.autonomous_mapping_action =
      autonomous_mapping_client_->wait_for_action_server(std::chrono::seconds(0));
    dependencies.endpoints.rotate_to_heading_action =
      rotate_to_heading_client_->wait_for_action_server(std::chrono::seconds(0));
    dependencies.endpoints.coverage_action =
      coverage_client_->wait_for_action_server(std::chrono::seconds(0));
    dependencies.endpoints.apriltag_confirmation_action =
      apriltag_confirmation_client_->wait_for_action_server(std::chrono::seconds(0));
    return dependencies;
  }

  std::string extended_state_json(
    const svo::SupervisorState & state,
    const svo::MissionDependencySnapshot & dependencies,
    const rclcpp::Time & stamp) const
  {
    auto json = nlohmann::json::parse(policy_.CompactStateJson(state, stamp));
    const auto capabilities = mission_authority_.EvaluateCapabilities(dependencies);
    const auto & authority = mission_authority_.state();
    json["mission"] = {
      {"active_operation", svo::ToString(authority.operation)},
      {"operation_state", svo::ToString(authority.state)},
      {"active_request_id", authority.request_id},
      {"active_actor_id", authority.actor_id},
      {"map_id", authority.map_id},
      {"map_revision", authority.map_revision},
      {"map_release_id", authority.map_release_id},
      {"semantic_required", authority.semantic_required},
      {"remote_origin", authority.remote_origin},
      {"authority_generation", authority.generation},
      {"reason", authority.reason}
    };
    json["mission_capabilities"] = {
      {"mapping_available", capabilities.mapping_available},
      {"navigation_ready", capabilities.navigation_ready},
      {"head_ready", capabilities.head_ready},
      {"locations_read_ready", capabilities.locations_read_ready},
      {"locations_write_ready", capabilities.locations_write_ready},
      {"semantic_mapping_ready", capabilities.semantic_mapping_ready},
      {"can_start_manual_mapping", capabilities.can_start_manual_mapping},
      {"can_start_autonomous_mapping", capabilities.can_start_autonomous_mapping},
      {"can_run_scan360", capabilities.can_run_scan360},
      {"can_run_coverage", capabilities.can_run_coverage},
      {"can_navigate", capabilities.can_navigate},
      {"can_register_location", capabilities.can_register_location},
      {"can_review_location", capabilities.can_review_location},
      {"can_confirm_arrival", capabilities.can_confirm_arrival}
    };
    json["map_context"] = {
      {"type", svo::ToString(map_context_.type)},
      {"map_id", map_context_.map_id},
      {"map_revision", map_context_.map_revision},
      {"map_release_id", map_context_.map_release_id},
      {"mapping_session_id", map_context_.mapping_session_id},
      {"approved", map_context_.approved},
      {"generation", map_context_.generation}
    };
    json["edge_capabilities"] = {
      {"edge_health_ready", latest_edge_state_.capabilities.edge_health_ready},
      {"edge_startup_ready", latest_edge_state_.capabilities.edge_startup_ready},
      {"core_edge_link_ready", latest_edge_state_.capabilities.core_edge_link_ready},
      {"bridge_ready", latest_edge_state_.capabilities.bridge_ready},
      {"remote_command_path_ready",
        latest_edge_state_.capabilities.remote_command_path_ready},
      {"speech_ready", latest_edge_state_.capabilities.speech_ready},
      {"realsense_ready", latest_edge_state_.capabilities.realsense_ready},
      {"vo_ready", latest_edge_state_.capabilities.vo_ready},
      {"ui_ready", latest_edge_state_.capabilities.ui_ready},
      {"degraded", latest_edge_state_.degraded},
      {"reason", latest_edge_state_.reason}
    };
    json["edge_components"] = nlohmann::json::array();
    for (const auto & component : latest_edge_state_.components) {
      json["edge_components"].push_back({
        {"name", component.name},
        {"enabled", component.enabled},
        {"required_for_startup", component.required_for_startup},
        {"received", component.received},
        {"fresh", component.fresh},
        {"valid", component.valid},
        {"ready", component.ready},
        {"degraded", component.degraded},
        {"state", svo::ToString(component.state)},
        {"reason", component.reason}
      });
    }
    json["system_authority"] = {
      {"state", svo::ToString(latest_system_snapshot_.state)},
      {"startup_ready", latest_system_snapshot_.startup_ready},
      {"system_armed", latest_system_snapshot_.armed},
      {"fault_latched", latest_system_snapshot_.fault_latched},
      {"shutdown_requested", latest_system_snapshot_.shutdown_requested},
      {"remote_commands_ready", latest_system_snapshot_.remote_commands_ready},
      {"system_generation", latest_system_snapshot_.generation},
      {"reason", latest_system_snapshot_.reason},
      {"last_actor", latest_system_snapshot_.last_actor},
      {"persistent_state_path", system_state_store_.path()}
    };
    return json.dump();
  }

  void publish_mission_event(const std::string & event_type, const std::string & reason)
  {
    nlohmann::json json{
      {"schema_version", 1},
      {"node", "savo_supervisor"},
      {"event_type", event_type},
      {"active_operation", svo::ToString(mission_authority_.state().operation)},
      {"operation_state", svo::ToString(mission_authority_.state().state)},
      {"authority_generation", mission_authority_.state().generation},
      {"reason", reason},
      {"stamp_s", now().seconds()}
    };
    std_msgs::msg::String event;
    event.data = json.dump();
    events_publisher_->publish(event);
  }

  svo::SafetySummary evaluate_safety(const rclcpp::Time & evaluation_time) const
  {
    svo::SafetySummary summary;
    const auto stop = safety_stop_tracker_.snapshot(evaluation_time, safety_stop_timeout_s_);
    const auto slowdown = safety_slowdown_tracker_.snapshot(
      evaluation_time, safety_slowdown_timeout_s_);
    summary.stop_received = stop.received;
    summary.slowdown_received = slowdown.received;
    summary.stop_fresh = stop.received && !stop.stale && stop.valid && safety_stop_valid_;
    summary.slowdown_fresh = slowdown.received && !slowdown.stale &&
      slowdown.valid && safety_slowdown_valid_;
    summary.stop_active = safety_stop_active_;
    summary.slowdown_factor = safety_slowdown_factor_;
    summary.last_message_age_s = std::max(
      stop.received && std::isfinite(stop.age_s) ? stop.age_s : -1.0,
      slowdown.received && std::isfinite(slowdown.age_s) ? slowdown.age_s : -1.0);

    if (!summary.stop_fresh || !summary.slowdown_fresh) {
      summary.observation = svo::SafetyObservation::UNKNOWN;
      summary.ready = false;
      summary.reason_code = !stop.received ? "safety_stop_missing" :
        stop.stale ? "safety_stop_stale" :
        !slowdown.received ? "safety_slowdown_missing" :
        slowdown.stale ? "safety_slowdown_stale" : "safety_message_invalid";
      return summary;
    }
    summary.ready = true;
    if (summary.stop_active) {
      summary.observation = svo::SafetyObservation::STOPPED;
      summary.reason_code = svo::reason::kSafetyStop;
    } else if (summary.slowdown_factor < 0.999) {
      summary.observation = svo::SafetyObservation::SLOWDOWN;
      summary.reason_code = svo::reason::kSafetySlowdown;
    } else {
      summary.observation = svo::SafetyObservation::CLEAR;
      summary.reason_code = svo::reason::kSafetyClear;
    }
    return summary;
  }

  svo::SupervisorState evaluate_state(const rclcpp::Time & evaluation_time)
  {
    const double startup_age_s = (evaluation_time - startup_time_).seconds();
    std::vector<svo::ComponentSummary> summaries;
    summaries.reserve(6U);
    summaries.push_back(policy_.EvaluateComponent(base_status_, evaluation_time, startup_age_s));
    summaries.push_back(policy_.EvaluateComponent(control_status_, evaluation_time, startup_age_s));
    summaries.push_back(policy_.EvaluateComponent(
      perception_status_, evaluation_time, startup_age_s));
    summaries.push_back(policy_.EvaluateComponent(lidar_status_, evaluation_time, startup_age_s));
    summaries.push_back(policy_.EvaluateComponent(
      localization_status_, evaluation_time, startup_age_s));
    summaries.push_back(policy_.EvaluateComponent(power_status_, evaluation_time, startup_age_s));
    const auto core_state = policy_.EvaluateSupervisor(
      summaries, evaluate_safety(evaluation_time), evaluation_time, startup_age_s);

    latest_edge_state_ = evaluate_edge_state(evaluation_time);
    latest_system_dependencies_ = make_system_dependencies(core_state, latest_edge_state_);
    const auto previous_system_snapshot = latest_system_snapshot_;
    const bool system_changed = system_authority_.Update(latest_system_dependencies_);
    latest_system_snapshot_ = system_authority_.snapshot(latest_system_dependencies_);
    const bool system_generation_changed =
      latest_system_snapshot_.generation != previous_system_snapshot.generation;
    const bool system_observation_changed =
      latest_system_snapshot_.state != previous_system_snapshot.state ||
      latest_system_snapshot_.startup_ready != previous_system_snapshot.startup_ready ||
      latest_system_snapshot_.remote_commands_ready !=
      previous_system_snapshot.remote_commands_ready;
    if (system_changed || system_generation_changed) {
      persist_system_state();
    }
    if (system_changed || system_generation_changed || system_observation_changed) {
      publish_system_event("system_authority_changed", latest_system_snapshot_.reason);
    }

    latest_dependencies_ = evaluate_mission_dependencies(core_state, evaluation_time);
    latest_dependencies_.system.armed = latest_system_snapshot_.armed;
    latest_dependencies_.system.fault_latched = latest_system_snapshot_.fault_latched;
    latest_dependencies_.system.shutdown_requested = latest_system_snapshot_.shutdown_requested;
    latest_dependencies_.system.remote_commands_ready =
      latest_system_snapshot_.remote_commands_ready;
    if (mission_authority_.Revalidate(latest_dependencies_)) {
      publish_mission_event("operation_authorization_revoked", mission_authority_.state().reason);
    }

    auto state = core_state;
    if (state.health == svo::AggregateHealth::OK && latest_edge_state_.degraded) {
      state.health = svo::AggregateHealth::DEGRADED;
      state.degraded = true;
      state.reason_code = latest_edge_state_.reason;
    }
    if (latest_system_snapshot_.shutdown_requested) {
      state.lifecycle = svo::Lifecycle::STOPPING;
      state.operating_mode = svo::OperatingMode::SHUTTING_DOWN;
      state.ready = false;
      state.reason_code = "controlled_shutdown_requested";
    } else if (latest_system_snapshot_.fault_latched) {
      state.lifecycle = svo::Lifecycle::FAULTED;
      state.operating_mode = svo::OperatingMode::ERROR;
      state.health = svo::AggregateHealth::ERROR;
      state.ready = false;
      state.degraded = false;
      state.reason_code = "system_fault_latched";
    } else if (!latest_system_snapshot_.armed) {
      state.operating_mode =
        latest_system_snapshot_.state == svo::SystemAuthorityState::kBooting ?
        svo::OperatingMode::BOOTING : svo::OperatingMode::STOP;
    } else {
      state.operating_mode = svo::ModeForAuthority(state, mission_authority_.state());
    }
    latest_dependencies_.core = state;
    return state;
  }

  void timer_callback()
  {
    const auto evaluation_time = now();
    const auto state = evaluate_state(evaluation_time);
    publish_observations(state, evaluation_time);
    maybe_publish_event(state, evaluation_time);
  }

  void publish_observations(const svo::SupervisorState & state, const rclcpp::Time & stamp)
  {
    std_msgs::msg::String state_msg;
    state_msg.data = extended_state_json(state, latest_dependencies_, stamp);
    state_publisher_->publish(state_msg);

    std_msgs::msg::Bool system_ready;
    system_ready.data = latest_system_snapshot_.armed &&
      !latest_system_snapshot_.fault_latched &&
      !latest_system_snapshot_.shutdown_requested;
    system_ready_publisher_->publish(system_ready);

    std_msgs::msg::Bool remote_commands_ready;
    remote_commands_ready.data = latest_system_snapshot_.remote_commands_ready;
    remote_commands_ready_publisher_->publish(remote_commands_ready);

    std_msgs::msg::Bool shutdown_requested;
    shutdown_requested.data = latest_system_snapshot_.shutdown_requested;
    shutdown_requested_publisher_->publish(shutdown_requested);

    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = policy_.CompactHeartbeatJson(state, stamp);
    heartbeat_publisher_->publish(heartbeat_msg);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    append_aggregate_diagnostic(array, state);
    for (const auto & component : state.component_summaries) {
      append_component_diagnostic(array, component);
    }
    for (const auto & component : latest_edge_state_.components) {
      append_edge_diagnostic(array, component);
    }
    append_system_diagnostic(array, latest_system_snapshot_);
    health_publisher_->publish(array);
  }

  void maybe_publish_event(const svo::SupervisorState & state, const rclcpp::Time & stamp)
  {
    const auto transition = transition_tracker_.Observe(state);
    if (!transition.has_value()) {return;}
    std_msgs::msg::String event;
    event.data = svo::CompactTransitionJson(*transition, stamp);
    events_publisher_->publish(event);

    std::ostringstream message;
    message << "supervisor event=" << svo::ToString(transition->type)
            << " lifecycle=" << svo::ToString(state.lifecycle)
            << " health=" << svo::ToString(state.health)
            << " safety=" << svo::ToString(state.safety)
            << " ready=" << (state.ready ? "true" : "false")
            << " reason=" << state.reason_code;
    if (transition->has_component) {
      message << " component=" << transition->component_name
              << " state=" << svo::ToString(transition->previous_component_state)
              << "->" << svo::ToString(transition->current_component_state);
    }
    if (state.health == svo::AggregateHealth::ERROR) {
      RCLCPP_ERROR(get_logger(), "%s", message.str().c_str());
    } else if (state.health == svo::AggregateHealth::DEGRADED) {
      RCLCPP_WARN(get_logger(), "%s", message.str().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", message.str().c_str());
    }
  }

  void on_location_authorization(
    const std::shared_ptr<savo_msgs::srv::AuthorizeLocationOperation::Request> request,
    std::shared_ptr<savo_msgs::srv::AuthorizeLocationOperation::Response> response)
  {
    const auto operation = location_operation_from_ros(request->operation);
    const auto evaluation_time = now();
    const auto state = evaluate_state(evaluation_time);
    response->evaluated_at = evaluation_time;
    response->supervisor_lifecycle = svo::ToString(state.lifecycle);
    response->supervisor_health = svo::ToString(state.health);
    response->supervisor_safety = svo::ToString(state.safety);

    if (!operation.has_value()) {
      response->authorized = false;
      response->result_code =
        savo_msgs::srv::AuthorizeLocationOperation::Response::RESULT_INVALID_REQUEST;
      response->reason = "unsupported_location_operation";
      return;
    }

    svo::LocationAuthorizationRequest domain_request;
    domain_request.operation = *operation;
    domain_request.request_id = request->request_id;
    domain_request.actor_id = request->actor_id;
    domain_request.candidate_id = request->candidate_id;
    domain_request.location_id = request->location_id;
    domain_request.map_id = request->map_id;
    domain_request.map_revision = request->map_revision;
    domain_request.motion_required = request->motion_required ||
      *operation == svo::LocationOperation::kNavigateToLocation;

    svo::MissionAuthorizationRequest mission_request;
    mission_request.command = svo::AuthorityCommand::kCheck;
    mission_request.request_id = request->request_id;
    mission_request.actor_id = request->actor_id;
    mission_request.map_id = request->map_id;
    mission_request.map_revision = request->map_revision;
    mission_request.motion_required = domain_request.motion_required;
    mission_request.remote_origin = is_remote_actor(request->actor_id);
    switch (*operation) {
      case svo::LocationOperation::kRegisterCandidate:
        mission_request.operation = svo::MissionOperation::kRegisterLocation;
        break;
      case svo::LocationOperation::kApproveLocation:
      case svo::LocationOperation::kRejectLocationCandidate:
        mission_request.operation = svo::MissionOperation::kReviewLocation;
        break;
      case svo::LocationOperation::kNavigateToLocation:
        mission_request.operation = svo::MissionOperation::kNavigateToLocation;
        break;
      case svo::LocationOperation::kConfirmArrival:
        mission_request.operation = svo::MissionOperation::kConfirmArrival;
        break;
    }
    const auto mission_decision = mission_authority_.Handle(
      mission_request, latest_dependencies_);
    if (!mission_decision.authorized) {
      response->authorized = false;
      response->result_code =
        mission_decision.code == svo::MissionAuthorizationCode::kSafetyBlocked ?
        savo_msgs::srv::AuthorizeLocationOperation::Response::RESULT_SAFETY_BLOCKED :
        mission_decision.code == svo::MissionAuthorizationCode::kMapContextBlocked ?
        savo_msgs::srv::AuthorizeLocationOperation::Response::RESULT_MAP_CONTEXT_BLOCKED :
        savo_msgs::srv::AuthorizeLocationOperation::Response::RESULT_HEALTH_BLOCKED;
      response->reason = mission_decision.reason;
      return;
    }

    const auto decision = location_authorization_evaluator_.Evaluate(domain_request, state);
    response->authorized = decision.authorized;
    response->result_code = location_authorization_code_to_ros(decision.code);
    response->reason = decision.reason;
  }

  void fill_operation_response(
    const svo::MissionAuthorizationDecision & decision,
    const rclcpp::Time & evaluation_time,
    savo_msgs::srv::AuthorizeOperation::Response & response) const
  {
    response.authorized = decision.authorized;
    response.result_code = mission_authorization_code_to_ros(decision.code);
    response.reason = decision.reason;
    response.supervisor_mode = svo::ToString(latest_dependencies_.core.operating_mode);
    response.operation_state = svo::ToString(mission_authority_.state().state);
    response.active_operation = mission_operation_to_ros(mission_authority_.state().operation);
    response.active_request_id = mission_authority_.state().request_id;
    response.authority_generation = mission_authority_.state().generation;
    response.evaluated_at = evaluation_time;
  }

  void on_operation_authorization(
    const std::shared_ptr<savo_msgs::srv::AuthorizeOperation::Request> request,
    std::shared_ptr<savo_msgs::srv::AuthorizeOperation::Response> response)
  {
    const auto evaluation_time = now();
    (void)evaluate_state(evaluation_time);
    const auto command = authority_command_from_ros(request->command);
    auto operation = mission_operation_from_ros(request->operation);
    if (!command.has_value()) {
      fill_operation_response(
        {false, svo::MissionAuthorizationCode::kInvalidRequest,
          "unsupported_authority_command"}, evaluation_time, *response);
      return;
    }
    if (!operation.has_value() &&
      (*command == svo::AuthorityCommand::kRelease ||
      *command == svo::AuthorityCommand::kPause ||
      *command == svo::AuthorityCommand::kResume))
    {
      operation = mission_authority_.state().operation;
    }
    if (!operation.has_value()) {
      fill_operation_response(
        {false, svo::MissionAuthorizationCode::kInvalidRequest,
          "unsupported_mission_operation"}, evaluation_time, *response);
      return;
    }

    svo::MissionAuthorizationRequest domain;
    domain.command = *command;
    domain.operation = *operation;
    domain.request_id = request->request_id;
    domain.actor_id = request->actor_id;
    domain.map_id = request->map_id;
    domain.map_revision = request->map_revision;
    domain.map_release_id = request->map_release_id;
    domain.require_semantic = request->require_semantic;
    domain.motion_required = request->motion_required || svo::IsExclusiveOperation(*operation);
    domain.remote_origin = is_remote_actor(request->actor_id);
    domain.expected_generation = request->expected_generation;

    const auto previous_generation = mission_authority_.state().generation;
    const auto decision = mission_authority_.Handle(domain, latest_dependencies_);
    if (decision.authorized && mission_authority_.state().generation != previous_generation) {
      publish_mission_event("mission_authority_changed", decision.reason);
    }
    if (decision.authorized && *command == svo::AuthorityCommand::kAcquire &&
      (*operation == svo::MissionOperation::kManualMapping ||
      *operation == svo::MissionOperation::kAutonomousMapping) && !request->map_id.empty())
    {
      map_context_.type = svo::MapContextType::kLiveMapping;
      map_context_.map_id = request->map_id;
      map_context_.map_revision = request->map_revision;
      map_context_.map_release_id.clear();
      map_context_.mapping_session_id = request->request_id;
      map_context_.approved = false;
      ++map_context_.generation;
      latest_dependencies_.map_context = map_context_;
    }
    auto state = latest_dependencies_.core;
    state.operating_mode = effective_operating_mode(state);
    latest_dependencies_.core = state;
    fill_operation_response(decision, evaluation_time, *response);
    response->supervisor_mode = svo::ToString(state.operating_mode);
  }

  void on_system_state_request(
    const std::shared_ptr<savo_msgs::srv::ManageSystemState::Request> request,
    std::shared_ptr<savo_msgs::srv::ManageSystemState::Response> response)
  {
    const auto evaluation_time = now();
    (void)evaluate_state(evaluation_time);
    const auto command = system_command_from_ros(request->command);
    const bool unauthorized_system_actor =
      command.has_value() &&
      *command != svo::SystemCommand::kStatus &&
      !is_system_actor(request->actor_id);

    if (!command.has_value()) {
      response->accepted = false;
      response->result_code =
        savo_msgs::srv::ManageSystemState::Response::RESULT_INVALID_REQUEST;
      response->reason = "unsupported_system_command";
    } else if (unauthorized_system_actor) {
      response->accepted = false;
      response->result_code =
        savo_msgs::srv::ManageSystemState::Response::RESULT_INVALID_REQUEST;
      response->reason = "system_actor_not_authorized";
    } else {
      svo::SystemAuthorityRequest domain;
      domain.command = *command;
      domain.request_id = request->request_id;
      domain.actor_id = request->actor_id;
      domain.reason = request->reason;
      domain.expected_generation = request->expected_generation;
      const auto previous_generation = latest_system_snapshot_.generation;
      const auto decision = system_authority_.Handle(domain, latest_system_dependencies_);
      latest_system_snapshot_ = system_authority_.snapshot(latest_system_dependencies_);
      response->accepted = decision.accepted;
      response->result_code = system_authority_code_to_ros(decision.code);
      response->reason = decision.reason;
      if (decision.accepted && latest_system_snapshot_.generation != previous_generation) {
        persist_system_state();
        publish_system_event("system_authority_changed", decision.reason);
      }
      latest_dependencies_.system.armed = latest_system_snapshot_.armed;
      latest_dependencies_.system.fault_latched = latest_system_snapshot_.fault_latched;
      latest_dependencies_.system.shutdown_requested =
        latest_system_snapshot_.shutdown_requested;
      latest_dependencies_.system.remote_commands_ready =
        latest_system_snapshot_.remote_commands_ready;
      if (mission_authority_.Revalidate(latest_dependencies_)) {
        publish_mission_event(
          "operation_authorization_revoked", mission_authority_.state().reason);
      }
    }
    response->system_state = svo::ToString(latest_system_snapshot_.state);
    response->startup_ready = latest_system_snapshot_.startup_ready;
    response->system_armed = latest_system_snapshot_.armed;
    response->fault_latched = latest_system_snapshot_.fault_latched;
    response->shutdown_requested = latest_system_snapshot_.shutdown_requested;
    response->remote_commands_ready = latest_system_snapshot_.remote_commands_ready;
    response->system_generation = latest_system_snapshot_.generation;
    response->evaluated_at = evaluation_time;
  }

  void on_map_context_update(
    const std::shared_ptr<savo_msgs::srv::UpdateMapContext::Request> request,
    std::shared_ptr<savo_msgs::srv::UpdateMapContext::Response> response)
  {
    using Service = savo_msgs::srv::UpdateMapContext;
    response->evaluated_at = now();

    const auto active_operation = mission_authority_.state();
    const bool navigation_active =
      active_operation.state == svo::OperationState::kActive &&
      (active_operation.operation == svo::MissionOperation::kNavigateToPose ||
      active_operation.operation == svo::MissionOperation::kNavigateToLocation);

    const bool set_live_mapping =
      request->command == Service::Request::COMMAND_SET_LIVE_MAPPING &&
      !request->map_id.empty();

    const bool set_saved_release =
      request->command == Service::Request::COMMAND_SET_SAVED_RELEASE &&
      !request->map_id.empty() &&
      request->map_revision > 0U &&
      !request->map_release_id.empty();

    if (request->request_id.empty() || request->actor_id.empty()) {
      response->updated = false;
      response->result_code = Service::Response::RESULT_INVALID_REQUEST;
      response->reason = "map_context_identity_required";
    } else if (navigation_active) {
      response->updated = false;
      response->result_code = Service::Response::RESULT_CONFLICT;
      response->reason = "cannot_change_map_context_during_navigation";
    } else if (request->command == Service::Request::COMMAND_CLEAR) {
      const auto generation = map_context_.generation + 1U;
      map_context_ = svo::ActiveMapContext{};
      map_context_.generation = generation;
      response->updated = true;
      response->result_code = Service::Response::RESULT_UPDATED;
      response->reason = "map_context_cleared";
    } else if (set_live_mapping) {
      map_context_.type = svo::MapContextType::kLiveMapping;
      map_context_.map_id = request->map_id;
      map_context_.map_revision = request->map_revision;
      map_context_.map_release_id.clear();
      map_context_.mapping_session_id = request->mapping_session_id;
      map_context_.approved = false;
      ++map_context_.generation;
      response->updated = true;
      response->result_code = Service::Response::RESULT_UPDATED;
      response->reason = "live_mapping_context_updated";
    } else if (set_saved_release) {
      map_context_.type = svo::MapContextType::kSavedRelease;
      map_context_.map_id = request->map_id;
      map_context_.map_revision = request->map_revision;
      map_context_.map_release_id = request->map_release_id;
      map_context_.mapping_session_id = request->mapping_session_id;
      map_context_.approved = request->approved;
      ++map_context_.generation;
      response->updated = true;
      response->result_code = Service::Response::RESULT_UPDATED;
      response->reason = "saved_release_context_updated";
    } else {
      response->updated = false;
      response->result_code = Service::Response::RESULT_INVALID_REQUEST;
      response->reason = "invalid_map_context_update";
    }
    latest_dependencies_.map_context = map_context_;
    response->context_type = svo::ToString(map_context_.type);
    response->active_map_id = map_context_.map_id;
    response->active_map_revision = map_context_.map_revision;
    response->active_map_release_id = map_context_.map_release_id;
    response->context_generation = map_context_.generation;
    if (response->updated) {
      publish_mission_event("map_context_changed", response->reason);
    }
  }

  svo::SupervisorPolicy policy_{};
  svo::CorePayloadParser core_parser_{};
  svo::LocalizationPayloadParser localization_parser_{1};
  svo::MissionPayloadParser mission_parser_{};
  svo::EdgePayloadParser edge_parser_{};
  svo::EdgeSupervisionPolicy edge_policy_{};
  svo::EdgeSupervision edge_supervision_{};
  svo::EdgeSupervisionState latest_edge_state_{};
  svo::SystemAuthorityPolicy system_authority_policy_{};
  svo::SystemAuthority system_authority_{};
  svo::SystemDependencySnapshot latest_system_dependencies_{};
  svo::SystemAuthoritySnapshot latest_system_snapshot_{};
  svo::SystemStateStore system_state_store_{};
  svo::MissionAuthorityPolicy mission_authority_policy_{};
  svo::MissionAuthority mission_authority_{};
  svo::MissionDependencySnapshot latest_dependencies_{};
  svo::ActiveMapContext map_context_{};

  svo::ComponentStatus base_status_{};
  svo::ComponentStatus control_status_{};
  svo::ComponentStatus perception_status_{};
  svo::ComponentStatus lidar_status_{};
  svo::ComponentStatus localization_status_{};
  svo::ComponentStatus power_status_{};

  std::string safety_stop_topic_;
  std::string safety_slowdown_topic_;
  double safety_stop_timeout_s_{1.0};
  double safety_slowdown_timeout_s_{1.0};
  svo::FreshnessTracker safety_stop_tracker_{};
  svo::FreshnessTracker safety_slowdown_tracker_{};
  bool safety_stop_valid_{false};
  bool safety_slowdown_valid_{false};
  bool safety_stop_active_{false};
  double safety_slowdown_factor_{1.0};

  std::string location_authorization_service_name_;
  svo::LocationAuthorizationPolicy location_authorization_policy_{};
  svo::LocationAuthorizationEvaluator location_authorization_evaluator_{};

  std::string operation_authorization_service_name_;
  std::string map_context_service_name_;
  std::string system_state_service_name_;
  std::string system_state_path_;
  std::string system_ready_topic_;
  std::string remote_commands_ready_topic_;
  std::string shutdown_requested_topic_;
  std::string mapping_status_topic_;
  std::string navigation_status_topic_;
  std::string navigation_heartbeat_topic_;
  std::string head_status_topic_;
  std::string locations_status_topic_;
  std::string locations_heartbeat_topic_;
  std::string autonomous_mapping_action_name_;
  std::string rotate_to_heading_action_name_;
  std::string coverage_action_name_;
  std::string apriltag_confirmation_action_name_;
  double mapping_status_timeout_s_{2.0};
  double navigation_status_timeout_s_{1.5};
  double navigation_heartbeat_timeout_s_{2.5};
  double head_status_timeout_s_{1.5};
  double locations_status_timeout_s_{2.0};
  double locations_heartbeat_timeout_s_{2.0};
  svo::FreshnessTracker mapping_status_tracker_{};
  svo::FreshnessTracker navigation_status_tracker_{};
  svo::FreshnessTracker navigation_heartbeat_tracker_{};
  svo::FreshnessTracker head_status_tracker_{};
  svo::FreshnessTracker locations_status_tracker_{};
  svo::FreshnessTracker locations_heartbeat_tracker_{};
  svo::MappingObservation mapping_observation_{};
  svo::NavigationObservation navigation_observation_{};
  svo::HeadObservation head_observation_{};
  svo::LocationsObservation locations_observation_{};

  std::string bridge_state_topic_;
  std::string bridge_readiness_topic_;
  std::string bridge_heartbeat_topic_;
  std::string realsense_status_topic_;
  std::string speech_readiness_topic_;
  std::string speech_heartbeat_topic_;
  std::string vo_health_topic_;
  double bridge_state_timeout_s_{2.0};
  double bridge_heartbeat_timeout_s_{2.5};
  double realsense_status_timeout_s_{2.0};
  double speech_readiness_timeout_s_{3.0};
  double speech_heartbeat_timeout_s_{3.0};
  double vo_health_timeout_s_{2.0};
  std::vector<std::string> ui_expected_nodes_{};
  std::vector<std::string> remote_actor_prefixes_{};
  std::vector<std::string> system_actor_prefixes_{};
  svo::FreshnessTracker bridge_state_tracker_{};
  svo::FreshnessTracker bridge_readiness_tracker_{};
  svo::FreshnessTracker bridge_heartbeat_tracker_{};
  svo::FreshnessTracker realsense_status_tracker_{};
  svo::FreshnessTracker speech_readiness_tracker_{};
  svo::FreshnessTracker speech_heartbeat_tracker_{};
  svo::FreshnessTracker vo_health_tracker_{};
  svo::BridgeObservation bridge_observation_{};
  svo::RealSenseObservation realsense_observation_{};
  svo::VoiceObservation speech_observation_{};
  svo::VisualOdometryObservation vo_observation_{};
  bool bridge_readiness_asserted_{false};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr system_ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remote_commands_ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdown_requested_publisher_;
  rclcpp::Service<savo_msgs::srv::AuthorizeLocationOperation>::SharedPtr
    location_authorization_service_;
  rclcpp::Service<savo_msgs::srv::AuthorizeOperation>::SharedPtr
    operation_authorization_service_;
  rclcpp::Service<savo_msgs::srv::UpdateMapContext>::SharedPtr map_context_service_;
  rclcpp::Service<savo_msgs::srv::ManageSystemState>::SharedPtr system_state_service_;
  rclcpp_action::Client<savo_msgs::action::RunAutonomousMapping>::SharedPtr
    autonomous_mapping_client_;
  rclcpp_action::Client<savo_msgs::action::RotateToHeading>::SharedPtr
    rotate_to_heading_client_;
  rclcpp_action::Client<savo_msgs::action::ExecuteCoveragePath>::SharedPtr coverage_client_;
  rclcpp_action::Client<savo_msgs::action::ConfirmAprilTag>::SharedPtr
    apriltag_confirmation_client_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time startup_time_;
  svo::TransitionTracker transition_tracker_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
