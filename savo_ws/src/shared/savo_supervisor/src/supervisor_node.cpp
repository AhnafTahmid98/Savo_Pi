// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <iomanip>
#include <sstream>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "savo_msgs/srv/authorize_location_operation.hpp"
#include "savo_supervisor/component_status.hpp"
#include "savo_supervisor/freshness_tracker.hpp"
#include "savo_supervisor/localization_payload_parser.hpp"
#include "savo_supervisor/location_authorization_policy.hpp"
#include "savo_supervisor/reason_codes.hpp"
#include "savo_supervisor/supervisor_policy.hpp"
#include "savo_supervisor/supervisor_state.hpp"
#include "savo_supervisor/transition_tracker.hpp"
#include "std_msgs/msg/string.hpp"

namespace svo = savo_supervisor;

namespace
{

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

  switch (state.health) {
    case svo::AggregateHealth::OK:
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      break;

    case svo::AggregateHealth::DEGRADED:
    case svo::AggregateHealth::UNKNOWN:
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;

    case svo::AggregateHealth::ERROR:
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      break;
  }

  status.message = state.reason_code.empty() ?
    "supervisor_state_unknown" :
    state.reason_code;

  append_key_value(
    status,
    "lifecycle",
    svo::ToString(state.lifecycle));

  append_key_value(
    status,
    "operating_mode",
    svo::ToString(state.operating_mode));

  append_key_value(
    status,
    "health",
    svo::ToString(state.health));

  append_key_value(
    status,
    "safety",
    svo::ToString(state.safety));

  append_key_value(
    status,
    "ready",
    state.ready ? "true" : "false");

  append_key_value(
    status,
    "degraded",
    state.degraded ? "true" : "false");

  append_key_value(
    status,
    "reason_code",
    state.reason_code);

  append_key_value(
    status,
    "component_count",
    std::to_string(
      state.component_summaries.size()));

  array.status.push_back(status);
}

void append_component_diagnostic(
  diagnostic_msgs::msg::DiagnosticArray & array,
  const svo::ComponentSummary & summary)
{
  diagnostic_msgs::msg::DiagnosticStatus status;

  status.name =
    "savo_supervisor/" + summary.name;

  status.hardware_id =
    "robot_savo_supervisor";

  switch (summary.state) {
    case svo::ComponentState::OK:
    case svo::ComponentState::DISABLED:
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      break;

    case svo::ComponentState::UNKNOWN:
    case svo::ComponentState::INITIALIZING:
    case svo::ComponentState::DEGRADED:
    case svo::ComponentState::STALE:
    case svo::ComponentState::INVALID:
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;

    case svo::ComponentState::ERROR:
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      break;
  }

  status.message = summary.reason_code.empty() ?
    "component_state_unknown" :
    summary.reason_code;

  append_key_value(
    status,
    "enabled",
    summary.enabled ? "true" : "false");

  append_key_value(
    status,
    "required",
    summary.required ? "true" : "false");

  append_key_value(
    status,
    "received",
    summary.received ? "true" : "false");

  append_key_value(
    status,
    "state",
    svo::ToString(summary.state));

  append_key_value(
    status,
    "ready",
    summary.ready ? "true" : "false");

  append_key_value(
    status,
    "degraded",
    summary.degraded ? "true" : "false");

  append_key_value(
    status,
    "health_valid",
    summary.health_valid ? "true" : "false");

  append_key_value(
    status,
    "summary_valid",
    summary.summary_valid ? "true" : "false");

  append_key_value(
    status,
    "heartbeat_valid",
    summary.heartbeat_valid ? "true" : "false");

  append_key_value(
    status,
    "reason_code",
    summary.reason_code);

  append_key_value(
    status,
    "last_message_age_s",
    std::to_string(
      summary.last_message_age_s));

  append_key_value(
    status,
    "timeout_s",
    std::to_string(summary.timeout_s));

  append_key_value(
    status,
    "malformed_message_count",
    std::to_string(
      summary.malformed_message_count));

  append_key_value(
    status,
    "recovery_count",
    std::to_string(
      summary.recovery_count));

  append_key_value(
    status,
    "detail",
    summary.detail);

  array.status.push_back(status);
}


std::optional<svo::LocationOperation> location_operation_from_ros(
  const std::uint8_t operation)
{
  using Service = savo_msgs::srv::AuthorizeLocationOperation;

  switch (operation) {
    case Service::Request::OP_REGISTER_LOCATION_CANDIDATE:
      return svo::LocationOperation::kRegisterCandidate;
    case Service::Request::OP_APPROVE_LOCATION:
      return svo::LocationOperation::kApproveLocation;
    case Service::Request::OP_NAVIGATE_TO_LOCATION:
      return svo::LocationOperation::kNavigateToLocation;
    case Service::Request::OP_CONFIRM_LOCATION_ARRIVAL:
      return svo::LocationOperation::kConfirmArrival;
    case Service::Request::OP_REJECT_LOCATION_CANDIDATE:
      return svo::LocationOperation::kRejectLocationCandidate;
    default:
      return std::nullopt;
  }
}

std::uint8_t location_authorization_code_to_ros(
  const svo::LocationAuthorizationCode code)
{
  using Response = savo_msgs::srv::AuthorizeLocationOperation::Response;

  switch (code) {
    case svo::LocationAuthorizationCode::kAuthorized:
      return Response::RESULT_AUTHORIZED;
    case svo::LocationAuthorizationCode::kInvalidRequest:
      return Response::RESULT_INVALID_REQUEST;
    case svo::LocationAuthorizationCode::kSupervisorNotReady:
      return Response::RESULT_SUPERVISOR_NOT_READY;
    case svo::LocationAuthorizationCode::kHealthBlocked:
      return Response::RESULT_HEALTH_BLOCKED;
    case svo::LocationAuthorizationCode::kSafetyBlocked:
      return Response::RESULT_SAFETY_BLOCKED;
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
    declare_parameter<double>("publish_rate_hz", 2.0);
    declare_parameter<double>("startup_grace_s", 3.0);
    declare_parameter<std::string>("state_summary_topic", "/savo_supervisor/state_summary");
    declare_parameter<std::string>("heartbeat_topic", "/savo_supervisor/heartbeat");
    declare_parameter<std::string>("health_topic", "/savo_supervisor/health");
    declare_parameter<std::string>("events_topic", "/savo_supervisor/events");
    declare_parameter<std::string>(
      "location_authorization.service_name",
      "/savo_supervisor/authorize_location_operation");
    declare_parameter<bool>("location_authorization.allow_registration", true);
    declare_parameter<bool>("location_authorization.allow_approval", true);
    declare_parameter<bool>("location_authorization.allow_rejection", true);
    declare_parameter<bool>("location_authorization.allow_navigation", true);
    declare_parameter<bool>("location_authorization.allow_arrival_confirmation", true);
    declare_parameter<bool>("location_authorization.allow_degraded_non_motion", true);
    declare_parameter<bool>("location_authorization.allow_degraded_motion", false);
    declare_parameter<bool>(
      "location_authorization.require_known_safety_for_motion", false);

    declare_parameter<bool>("localization.enabled", true);
    declare_parameter<bool>("localization.required", true);
    declare_parameter<std::string>("localization.health_topic", "/savo_localization/health");
    declare_parameter<std::string>("localization.summary_topic",
      "/savo_localization/state_summary");
    declare_parameter<std::string>("localization.heartbeat_topic", "/savo_localization/heartbeat");
    declare_parameter<double>("localization.health_timeout_s", 1.5);
    declare_parameter<double>("localization.summary_timeout_s", 1.5);
    declare_parameter<double>("localization.heartbeat_timeout_s", 2.5);
    declare_parameter<int>("localization.expected_schema_version", 1);

    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    startup_grace_s_ = get_parameter("startup_grace_s").as_double();
    state_summary_topic_ = get_parameter("state_summary_topic").as_string();
    heartbeat_topic_ = get_parameter("heartbeat_topic").as_string();
    health_topic_ = get_parameter("health_topic").as_string();
    events_topic_ = get_parameter("events_topic").as_string();
    location_authorization_service_name_ = get_parameter(
      "location_authorization.service_name").as_string();
    location_authorization_policy_.allow_registration = get_parameter(
      "location_authorization.allow_registration").as_bool();
    location_authorization_policy_.allow_approval = get_parameter(
      "location_authorization.allow_approval").as_bool();
    location_authorization_policy_.allow_rejection = get_parameter(
      "location_authorization.allow_rejection").as_bool();
    location_authorization_policy_.allow_navigation = get_parameter(
      "location_authorization.allow_navigation").as_bool();
    location_authorization_policy_.allow_arrival_confirmation = get_parameter(
      "location_authorization.allow_arrival_confirmation").as_bool();
    location_authorization_policy_.allow_degraded_non_motion = get_parameter(
      "location_authorization.allow_degraded_non_motion").as_bool();
    location_authorization_policy_.allow_degraded_motion = get_parameter(
      "location_authorization.allow_degraded_motion").as_bool();
    location_authorization_policy_.require_known_safety_for_motion = get_parameter(
      "location_authorization.require_known_safety_for_motion").as_bool();
    location_authorization_evaluator_ =
      svo::LocationAuthorizationEvaluator(location_authorization_policy_);

    localization_config_.enabled = get_parameter("localization.enabled").as_bool();
    localization_config_.required = get_parameter("localization.required").as_bool();
    localization_config_.health_topic = get_parameter("localization.health_topic").as_string();
    localization_config_.summary_topic = get_parameter("localization.summary_topic").as_string();
    localization_config_.heartbeat_topic =
      get_parameter("localization.heartbeat_topic").as_string();
    localization_config_.health_timeout_s =
      get_parameter("localization.health_timeout_s").as_double();
    localization_config_.summary_timeout_s =
      get_parameter("localization.summary_timeout_s").as_double();
    localization_config_.heartbeat_timeout_s =
      get_parameter("localization.heartbeat_timeout_s").as_double();
    localization_config_.expected_schema_version =
      get_parameter("localization.expected_schema_version").as_int();
    localization_config_.name = "localization";

    // Keep the runtime policy synchronized with the parameters loaded by this node.
    policy_.publish_rate_hz = publish_rate_hz_;
    policy_.startup_grace_s = startup_grace_s_;
    policy_.state_summary_topic = state_summary_topic_;
    policy_.heartbeat_topic = heartbeat_topic_;
    policy_.health_topic = health_topic_;
    policy_.events_topic = events_topic_;
    policy_.localization = localization_config_;

    policy_.publish_rate_hz = publish_rate_hz_;
    policy_.startup_grace_s = startup_grace_s_;
    policy_.state_summary_topic = state_summary_topic_;
    policy_.heartbeat_topic = heartbeat_topic_;
    policy_.health_topic = health_topic_;
    policy_.events_topic = events_topic_;
    policy_.localization = localization_config_;

    localization_parser_ =
      svo::LocalizationPayloadParser(
        localization_config_.expected_schema_version);

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "publish_rate_hz must be positive");
      throw std::runtime_error("invalid publish_rate_hz");
    }
    if (startup_grace_s_ < 0.0) {
      RCLCPP_FATAL(get_logger(), "startup_grace_s must be non-negative");
      throw std::runtime_error("invalid startup_grace_s");
    }
    if (state_summary_topic_.empty() || heartbeat_topic_.empty() || health_topic_.empty() ||
      events_topic_.empty() || location_authorization_service_name_.empty())
    {
      RCLCPP_FATAL(get_logger(), "supervisor output topics must be non-empty");
      throw std::runtime_error("invalid output topic");
    }
    if (localization_config_.enabled) {
      if (localization_config_.health_topic.empty() ||
        localization_config_.summary_topic.empty() ||
        localization_config_.heartbeat_topic.empty())
      {
        RCLCPP_FATAL(
          get_logger(),
          "All localization topic parameters must be set when localization is enabled");
        throw std::runtime_error("invalid localization input topics");
      }

      if (!std::isfinite(localization_config_.health_timeout_s) ||
        !std::isfinite(localization_config_.summary_timeout_s) ||
        !std::isfinite(localization_config_.heartbeat_timeout_s) ||
        localization_config_.health_timeout_s <= 0.0 ||
        localization_config_.summary_timeout_s <= 0.0 ||
        localization_config_.heartbeat_timeout_s <= 0.0)
      {
        RCLCPP_FATAL(
          get_logger(),
          "Localization timeout parameters must be finite and positive");
        throw std::runtime_error("invalid localization timeouts");
      }

      if (localization_config_.expected_schema_version != 1) {
        RCLCPP_FATAL(
          get_logger(),
          "Only localization schema version 1 is supported");
        throw std::runtime_error("unsupported localization schema version");
      }
    }

    if (!policy_.Validate()) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid supervisor policy: %s",
        policy_.ValidationError().c_str());
      throw std::runtime_error("invalid supervisor policy");
    }

    localization_status_ = initialize_component(localization_config_);
    if (!localization_config_.enabled) {
      localization_status_.health_tracker.mark_disabled();
      localization_status_.summary_tracker.mark_disabled();
      localization_status_.heartbeat_tracker.mark_disabled();
    }

    state_publisher_ = create_publisher<std_msgs::msg::String>(state_summary_topic_,
      rclcpp::QoS(1).transient_local().reliable());
    heartbeat_publisher_ = create_publisher<std_msgs::msg::String>(heartbeat_topic_,
      rclcpp::QoS(1).reliable());
    health_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(health_topic_,
      rclcpp::QoS(1).reliable());
    events_publisher_ =
      create_publisher<std_msgs::msg::String>(
      events_topic_,
      rclcpp::QoS(10).reliable());

    location_authorization_service_ = create_service<
      savo_msgs::srv::AuthorizeLocationOperation>(
      location_authorization_service_name_,
      std::bind(
        &SupervisorNode::on_location_authorization,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 /
      publish_rate_hz_)),
      std::bind(&SupervisorNode::timer_callback, this)
    );

    if (localization_config_.enabled) {
      const auto latched_status_qos =
        rclcpp::QoS(1).reliable().transient_local();

      const auto heartbeat_qos =
        rclcpp::QoS(10).reliable();

      health_subscription_ = create_subscription<std_msgs::msg::String>(
        localization_config_.health_topic,
        latched_status_qos,
        std::bind(
          &SupervisorNode::on_localization_health,
          this,
          std::placeholders::_1));

      summary_subscription_ = create_subscription<std_msgs::msg::String>(
        localization_config_.summary_topic,
        latched_status_qos,
        std::bind(
          &SupervisorNode::on_localization_summary,
          this,
          std::placeholders::_1));

      heartbeat_subscription_ = create_subscription<std_msgs::msg::String>(
        localization_config_.heartbeat_topic,
        heartbeat_qos,
        std::bind(
          &SupervisorNode::on_localization_heartbeat,
          this,
          std::placeholders::_1));
    }

    startup_time_ = now();
  }

private:
  void timer_callback()
  {
    const auto now = this->now();
    const auto startup_age_s = (now - startup_time_).seconds();
    const auto summary = policy_.EvaluateComponent(localization_status_, now, startup_age_s);
    const auto state = policy_.EvaluateSupervisor(summary, now, startup_age_s);
    publish_observations(state, now);
    maybe_publish_event(state, now);
  }

  void publish_observations(
    const svo::SupervisorState & state,
    const rclcpp::Time & now)
  {
    std_msgs::msg::String state_msg;
    state_msg.data =
      policy_.CompactStateJson(state, now);

    state_publisher_->publish(state_msg);

    /*
     * The supervisor heartbeat is always published, including
     * while required components are stale, invalid, or erroneous.
     */
    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data =
      policy_.CompactHeartbeatJson(state, now);

    heartbeat_publisher_->publish(heartbeat_msg);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;

    append_aggregate_diagnostic(array, state);

    for (const auto & component :
      state.component_summaries)
    {
      append_component_diagnostic(
        array,
        component);
    }

    health_publisher_->publish(array);
  }

  void maybe_publish_event(
    const svo::SupervisorState & state,
    const rclcpp::Time & now)
  {
    const auto transition =
      transition_tracker_.Observe(state);

    if (!transition.has_value()) {
      return;
    }

    std_msgs::msg::String event;

    event.data =
      svo::CompactTransitionJson(
      transition.value(),
      now);

    events_publisher_->publish(event);

    log_transition(transition.value());
  }

  void log_transition(
    const svo::SupervisorTransition & transition)
  {
    std::ostringstream message;

    message
      << "supervisor event="
      << svo::ToString(transition.type)
      << " lifecycle="
      << svo::ToString(
        transition.current.lifecycle)
      << " health="
      << svo::ToString(
        transition.current.health)
      << " ready="
      << (
      transition.current.ready ?
      "true" :
      "false")
      << " reason="
      << transition.current.reason_code;

    if (transition.has_previous) {
      message
        << " previous_lifecycle="
        << svo::ToString(
          transition.previous.lifecycle)
        << " previous_health="
        << svo::ToString(
          transition.previous.health)
        << " previous_ready="
        << (
        transition.previous.ready ?
        "true" :
        "false");
    }

    if (transition.has_component) {
      message
        << " component="
        << transition.component_name
        << " component_state="
        << svo::ToString(
          transition.previous_component_state)
        << "->"
        << svo::ToString(
          transition.current_component_state);
    }

    switch (transition.type) {
      case svo::SupervisorEventType::COMPONENT_ERROR:
        RCLCPP_ERROR(
          get_logger(),
          "%s",
          message.str().c_str());
        break;

      case svo::SupervisorEventType::COMPONENT_STALE:
      case svo::SupervisorEventType::COMPONENT_INVALID:
        RCLCPP_WARN(
          get_logger(),
          "%s",
          message.str().c_str());
        break;

      default:
        if (
          transition.current.health ==
          svo::AggregateHealth::ERROR)
        {
          RCLCPP_WARN(
            get_logger(),
            "%s",
            message.str().c_str());
        } else {
          RCLCPP_INFO(
            get_logger(),
            "%s",
            message.str().c_str());
        }
        break;
    }
  }

  void on_location_authorization(
    const std::shared_ptr<
      savo_msgs::srv::AuthorizeLocationOperation::Request> request,
    std::shared_ptr<
      savo_msgs::srv::AuthorizeLocationOperation::Response> response)
  {
    const auto operation = location_operation_from_ros(request->operation);
    const auto evaluation_time = now();
    response->evaluated_at = evaluation_time;

    const auto startup_age_s = (evaluation_time - startup_time_).seconds();
    const auto summary = policy_.EvaluateComponent(
      localization_status_, evaluation_time, startup_age_s);
    const auto state = policy_.EvaluateSupervisor(
      summary, evaluation_time, startup_age_s);

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
    domain_request.operation = operation.value();
    domain_request.request_id = request->request_id;
    domain_request.actor_id = request->actor_id;
    domain_request.candidate_id = request->candidate_id;
    domain_request.location_id = request->location_id;
    domain_request.map_id = request->map_id;
    domain_request.map_revision = request->map_revision;
    domain_request.motion_required =
      request->motion_required ||
      operation.value() == svo::LocationOperation::kNavigateToLocation;

    const auto decision = location_authorization_evaluator_.Evaluate(
      domain_request, state);
    response->authorized = decision.authorized;
    response->result_code = location_authorization_code_to_ros(decision.code);
    response->reason = decision.reason;
  }

  void on_localization_health(
    const std_msgs::msg::String::SharedPtr msg)
  {
    const auto receive_time = this->now();

    const auto parsed =
      localization_parser_.ParseHealth(msg->data);

    localization_status_.health_valid =
      parsed.valid;

    localization_status_.health_state =
      parsed.state;

    localization_status_.health_ready =
      parsed.ready;

    localization_status_.health_degraded =
      parsed.degraded;

    localization_status_.health_reason_code =
      parsed.reason_code;

    localization_status_.health_tracker.observe_message(
      receive_time,
      parsed.stamp,
      !parsed.valid,
      parsed.detail);
  }

  void on_localization_summary(
    const std_msgs::msg::String::SharedPtr msg)
  {
    const auto receive_time = this->now();

    const auto parsed =
      localization_parser_.ParseSummary(msg->data);

    localization_status_.summary_valid =
      parsed.valid;

    localization_status_.summary_state =
      parsed.state;

    localization_status_.summary_ready =
      parsed.ready;

    localization_status_.summary_degraded =
      parsed.degraded;

    localization_status_.summary_reason_code =
      parsed.reason_code;

    localization_status_.summary_tracker.observe_message(
      receive_time,
      parsed.stamp,
      !parsed.valid,
      parsed.detail);
  }

  void on_localization_heartbeat(
    const std_msgs::msg::String::SharedPtr msg)
  {
    const auto receive_time = this->now();

    const auto parsed =
      localization_parser_.ParseHeartbeat(msg->data);

    localization_status_.heartbeat_valid =
      parsed.valid;

    localization_status_.heartbeat_state =
      parsed.state;

    localization_status_.heartbeat_alive =
      parsed.alive;

    localization_status_.heartbeat_ready =
      parsed.ready;

    localization_status_.heartbeat_reason_code =
      parsed.reason_code;

    localization_status_.heartbeat_tracker.observe_message(
      receive_time,
      parsed.stamp,
      !parsed.valid,
      parsed.detail);
  }

  double publish_rate_hz_;
  double startup_grace_s_;
  std::string state_summary_topic_;
  std::string heartbeat_topic_;
  std::string health_topic_;
  std::string events_topic_;
  std::string location_authorization_service_name_;

  svo::ComponentConfig localization_config_;
  svo::ComponentStatus localization_status_;

  svo::LocalizationPayloadParser localization_parser_{1};
  svo::SupervisorPolicy policy_;
  svo::LocationAuthorizationPolicy location_authorization_policy_{};
  svo::LocationAuthorizationEvaluator location_authorization_evaluator_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_publisher_;
  rclcpp::Service<savo_msgs::srv::AuthorizeLocationOperation>::SharedPtr
    location_authorization_service_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr summary_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time startup_time_;
  svo::TransitionTracker transition_tracker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SupervisorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
