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
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "savo_msgs/srv/authorize_location_operation.hpp"
#include "savo_supervisor/component_status.hpp"
#include "savo_supervisor/core_payload_parser.hpp"
#include "savo_supervisor/localization_payload_parser.hpp"
#include "savo_supervisor/location_authorization_policy.hpp"
#include "savo_supervisor/reason_codes.hpp"
#include "savo_supervisor/supervisor_policy.hpp"
#include "savo_supervisor/supervisor_state.hpp"
#include "savo_supervisor/transition_tracker.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

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

    location_authorization_service_ = create_service<savo_msgs::srv::AuthorizeLocationOperation>(
      location_authorization_service_name_,
      std::bind(&SupervisorNode::on_location_authorization, this,
        std::placeholders::_1, std::placeholders::_2));

    create_component_subscriptions();
    create_safety_subscriptions();

    const auto period = std::chrono::duration<double>(1.0 / policy_.publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SupervisorNode::timer_callback, this));
    startup_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Phase 1 core supervisor started: base/control/perception/lidar/localization/power");
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
  }

  template<typename Callback>
  void subscribe_string(const std::string & topic, Callback callback)
  {
    if (topic.empty()) {return;}
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

  svo::SupervisorState evaluate_state(const rclcpp::Time & evaluation_time) const
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
    return policy_.EvaluateSupervisor(
      summaries, evaluate_safety(evaluation_time), evaluation_time,
      startup_age_s);
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
    state_msg.data = policy_.CompactStateJson(state, stamp);
    state_publisher_->publish(state_msg);

    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = policy_.CompactHeartbeatJson(state, stamp);
    heartbeat_publisher_->publish(heartbeat_msg);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    append_aggregate_diagnostic(array, state);
    for (const auto & component : state.component_summaries) {
      append_component_diagnostic(array, component);
    }
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

    const auto decision = location_authorization_evaluator_.Evaluate(domain_request, state);
    response->authorized = decision.authorized;
    response->result_code = location_authorization_code_to_ros(decision.code);
    response->reason = decision.reason;
  }

  svo::SupervisorPolicy policy_{};
  svo::CorePayloadParser core_parser_{};
  svo::LocalizationPayloadParser localization_parser_{1};

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

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_publisher_;
  rclcpp::Service<savo_msgs::srv::AuthorizeLocationOperation>::SharedPtr
    location_authorization_service_;
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
