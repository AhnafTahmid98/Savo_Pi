// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/supervisor_policy.hpp"
#include "savo_supervisor/component_status.hpp"
#include "savo_supervisor/reason_codes.hpp"
#include "savo_supervisor/supervisor_state.hpp"
#include "savo_supervisor/freshness_tracker.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <regex>
#include <sstream>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace svo = savo_supervisor;

namespace
{

constexpr double kInvalidTimeout = -1.0;


std::optional<int> parse_int_field(const std::string & payload, const std::string & key)
{
  const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+)");
  std::smatch match;
  if (std::regex_search(payload, match, re) && match.size() >= 2) {
    try {
      return std::stoi(match[1].str());
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<double> parse_double_field(const std::string & payload, const std::string & key)
{
  const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  std::smatch match;
  if (std::regex_search(payload, match, re) && match.size() >= 2) {
    try {
      return std::stod(match[1].str());
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<bool> parse_bool_field(const std::string & payload, const std::string & key)
{
  const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(true|false)");
  std::smatch match;
  if (std::regex_search(payload, match, re) && match.size() >= 2) {
    return match[1].str() == "true";
  }
  return std::nullopt;
}

std::optional<std::string> parse_string_field(
  const std::string & payload,
  const std::string & key)
{
  const std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch match;
  if (std::regex_search(payload, match, re) && match.size() >= 2) {
    return match[1].str();
  }
  return std::nullopt;
}

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

void append_diagnostic_section(
  diagnostic_msgs::msg::DiagnosticArray & array,
  const svo::ComponentSummary & summary)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "savo_supervisor/" + summary.name;
  status.hardware_id = "robot_savo_supervisor";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (summary.state == svo::ComponentState::STALE || summary.state == svo::ComponentState::INVALID) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else if (summary.state == svo::ComponentState::ERROR) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  status.message = summary.reason_code;
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "state";
    kv.value = svo::ToString(summary.state);
    status.values.push_back(kv);
  }
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "ready";
    kv.value = summary.ready ? "true" : "false";
    status.values.push_back(kv);
  }
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "degraded";
    kv.value = summary.degraded ? "true" : "false";
    status.values.push_back(kv);
  }
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "last_message_age_s";
    kv.value = std::to_string(summary.last_message_age_s);
    status.values.push_back(kv);
  }
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "timeout_s";
    kv.value = std::to_string(summary.timeout_s);
    status.values.push_back(kv);
  }
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "malformed_message_count";
    kv.value = std::to_string(summary.malformed_message_count);
    status.values.push_back(kv);
  }
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "recovery_count";
    kv.value = std::to_string(summary.recovery_count);
    status.values.push_back(kv);
  }
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "detail";
    kv.value = summary.detail;
    status.values.push_back(kv);
  }
  array.status.push_back(status);
}

} // namespace

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

    declare_parameter<bool>("localization.enabled", true);
    declare_parameter<bool>("localization.required", true);
    declare_parameter<std::string>("localization.health_topic", "/savo_localization/health");
    declare_parameter<std::string>("localization.summary_topic", "/savo_localization/state_summary");
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

    localization_config_.enabled = get_parameter("localization.enabled").as_bool();
    localization_config_.required = get_parameter("localization.required").as_bool();
    localization_config_.health_topic = get_parameter("localization.health_topic").as_string();
    localization_config_.summary_topic = get_parameter("localization.summary_topic").as_string();
    localization_config_.heartbeat_topic = get_parameter("localization.heartbeat_topic").as_string();
    localization_config_.health_timeout_s = get_parameter("localization.health_timeout_s").as_double();
    localization_config_.summary_timeout_s = get_parameter("localization.summary_timeout_s").as_double();
    localization_config_.heartbeat_timeout_s = get_parameter("localization.heartbeat_timeout_s").as_double();
    localization_config_.expected_schema_version = get_parameter("localization.expected_schema_version").as_int();
    localization_config_.name = "localization";

    policy_.publish_rate_hz = publish_rate_hz_;
    policy_.startup_grace_s = startup_grace_s_;
    policy_.state_summary_topic = state_summary_topic_;
    policy_.heartbeat_topic = heartbeat_topic_;
    policy_.health_topic = health_topic_;
    policy_.events_topic = events_topic_;
    policy_.localization = localization_config_;

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "publish_rate_hz must be positive");
      throw std::runtime_error("invalid publish_rate_hz");
    }
    if (startup_grace_s_ < 0.0) {
      RCLCPP_FATAL(get_logger(), "startup_grace_s must be non-negative");
      throw std::runtime_error("invalid startup_grace_s");
    }
    if (state_summary_topic_.empty() || heartbeat_topic_.empty() || health_topic_.empty() || events_topic_.empty()) {
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

    state_publisher_ = create_publisher<std_msgs::msg::String>(state_summary_topic_, rclcpp::QoS(1).transient_local().reliable());
    heartbeat_publisher_ = create_publisher<std_msgs::msg::String>(heartbeat_topic_, rclcpp::QoS(1).reliable());
    health_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(health_topic_, rclcpp::QoS(1).reliable());
    events_publisher_ = create_publisher<std_msgs::msg::String>(events_topic_, rclcpp::QoS(1).reliable());

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&SupervisorNode::timer_callback, this)
    );

    if (localization_config_.enabled) {
      const auto localization_qos =
        rclcpp::QoS(1).reliable().transient_local();

      health_subscription_ = create_subscription<std_msgs::msg::String>(
        localization_config_.health_topic,
        localization_qos,
        std::bind(
          &SupervisorNode::on_localization_health,
          this,
          std::placeholders::_1));

      summary_subscription_ = create_subscription<std_msgs::msg::String>(
        localization_config_.summary_topic,
        localization_qos,
        std::bind(
          &SupervisorNode::on_localization_summary,
          this,
          std::placeholders::_1));

      heartbeat_subscription_ = create_subscription<std_msgs::msg::String>(
        localization_config_.heartbeat_topic,
        localization_qos,
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

  void publish_observations(const svo::SupervisorState & state, const rclcpp::Time & now)
  {
    std_msgs::msg::String state_msg;
    state_msg.data = policy_.CompactStateJson(state, now);
    state_publisher_->publish(state_msg);

    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = policy_.CompactHeartbeatJson(state, now);
    heartbeat_publisher_->publish(heartbeat_msg);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;
    append_diagnostic_section(array, state.component_summaries[0]);
    health_publisher_->publish(array);
  }

  void maybe_publish_event(const svo::SupervisorState & state, const rclcpp::Time & now)
  {
    const bool state_changed = state.lifecycle != last_state_.lifecycle ||
      state.health != last_state_.health ||
      state.ready != last_state_.ready ||
      state.reason_code != last_state_.reason_code;
    if (!state_changed) {
      return;
    }

    std_msgs::msg::String event;
    std::ostringstream output;
    output << '{';
    output << "\"schema_version\":1,";
    output << "\"node\":\"savo_supervisor\",";
    output << "\"timestamp_s\":" << now.seconds() << ',';
    output << "\"lifecycle\":\"" << svo::ToString(state.lifecycle) << "\",";
    output << "\"health\":\"" << svo::ToString(state.health) << "\",";
    output << "\"ready\":" << (state.ready ? "true" : "false") << ',';
    output << "\"reason_code\":\"" << state.reason_code << "\"";
    output << '}';
    event.data = output.str();
    events_publisher_->publish(event);
    last_state_ = state;
  }

  void on_localization_health(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto now = this->now();
    const auto parsed = parse_localization_health(msg->data);
    localization_status_.health_valid = parsed.valid;
    localization_status_.health_state = parsed.state;
    localization_status_.health_ready = parsed.ready;
    localization_status_.health_degraded = parsed.degraded;
    localization_status_.health_reason_code = parsed.reason_code;
    localization_status_.health_tracker.observe_message(now, parsed.stamp, !parsed.valid, parsed.detail);
  }

  void on_localization_summary(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto now = this->now();
    const auto parsed = parse_localization_summary(msg->data);
    localization_status_.summary_valid = parsed.valid;
    localization_status_.summary_state = parsed.state;
    localization_status_.summary_ready = parsed.ready;
    localization_status_.summary_degraded = parsed.degraded;
    localization_status_.summary_reason_code = parsed.reason_code;
    localization_status_.summary_tracker.observe_message(now, parsed.stamp, !parsed.valid, parsed.detail);
  }

  void on_localization_heartbeat(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto now = this->now();
    const auto parsed = parse_localization_heartbeat(msg->data);
    localization_status_.heartbeat_valid = parsed.valid;
    localization_status_.heartbeat_alive = parsed.alive;
    localization_status_.heartbeat_ready = parsed.ready;
    localization_status_.heartbeat_reason_code = parsed.reason_code;
    localization_status_.heartbeat_tracker.observe_message(now, parsed.stamp, !parsed.valid, parsed.detail);
  }

  struct ParsedPayload
  {
    bool valid = false;
    std::string state;
    bool ready = false;
    bool degraded = false;
    bool alive = false;
    std::string reason_code;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::optional<builtin_interfaces::msg::Time> stamp_opt;
    std::string detail;
  };

  ParsedPayload parse_localization_health(const std::string & payload)
  {
    ParsedPayload parsed;
    parsed.valid = true;
    parsed.detail = "";

    const auto schema_version = parse_int_field(payload, "schema_version");
    if (!schema_version.has_value() || schema_version.value() != localization_config_.expected_schema_version) {
      parsed.valid = false;
      parsed.reason_code = savo_supervisor::reason::kLocalizationSchemaUnsupported;
      return parsed;
    }

    auto state = parse_string_field(payload, "state");
    if (!state.has_value()) {
      parsed.valid = false;
      parsed.reason_code = savo_supervisor::reason::kLocalizationMessageInvalid;
      return parsed;
    }
    parsed.state = state.value();

    const auto ready = parse_bool_field(payload, "ready");
    const auto degraded = parse_bool_field(payload, "degraded");
    parsed.ready = ready.value_or(false);
    parsed.degraded = degraded.value_or(false);

    const auto reason_code = parse_string_field(payload, "reason_code");
    parsed.reason_code = reason_code.value_or(savo_supervisor::reason::kLocalizationMessageInvalid);

    const auto stamp_val = parse_double_field(payload, "stamp_s");
    if (stamp_val.has_value()) {
      const double sec = stamp_val.value();
      if (!std::isfinite(sec) || sec < 0.0) {
        parsed.valid = false;
        parsed.detail = "invalid timestamp";
      } else {
        const auto sec_int = static_cast<int64_t>(std::floor(sec));
        builtin_interfaces::msg::Time stamp;
        stamp.sec = sec_int;
        stamp.nanosec = static_cast<uint32_t>((sec - sec_int) * 1e9);
        parsed.stamp_opt = stamp;
        parsed.stamp = rclcpp::Time(stamp.sec, stamp.nanosec, RCL_ROS_TIME);
      }
    }

    if (!parsed.ready) {
      parsed.reason_code = parsed.reason_code.empty() ? savo_supervisor::reason::kLocalizationNotReady : parsed.reason_code;
    }
    return parsed;
  }

  ParsedPayload parse_localization_summary(const std::string & payload)
  {
    ParsedPayload parsed;
    parsed.valid = true;
    parsed.detail = "";

    const auto schema_version = parse_int_field(payload, "schema_version");
    if (!schema_version.has_value() || schema_version.value() != localization_config_.expected_schema_version) {
      parsed.valid = false;
      parsed.reason_code = savo_supervisor::reason::kLocalizationSchemaUnsupported;
      return parsed;
    }

    auto state = parse_string_field(payload, "state");
    if (!state.has_value()) {
      parsed.valid = false;
      parsed.reason_code = savo_supervisor::reason::kLocalizationMessageInvalid;
      return parsed;
    }
    parsed.state = state.value();

    const auto ready = parse_bool_field(payload, "ready");
    parsed.ready = ready.value_or(false);
    const auto reason_code = parse_string_field(payload, "reason_code");
    parsed.reason_code = reason_code.value_or(savo_supervisor::reason::kLocalizationMessageInvalid);

    const auto stamp_val = parse_double_field(payload, "stamp_s");
    if (stamp_val.has_value()) {
      const double sec = stamp_val.value();
      if (!std::isfinite(sec) || sec < 0.0) {
        parsed.valid = false;
        parsed.detail = "invalid timestamp";
      } else {
        const auto sec_int = static_cast<int64_t>(std::floor(sec));
        builtin_interfaces::msg::Time stamp;
        stamp.sec = sec_int;
        stamp.nanosec = static_cast<uint32_t>((sec - sec_int) * 1e9);
        parsed.stamp_opt = stamp;
        parsed.stamp = rclcpp::Time(stamp.sec, stamp.nanosec, RCL_ROS_TIME);
      }
    }

    return parsed;
  }

  ParsedPayload parse_localization_heartbeat(const std::string & payload)
  {
    ParsedPayload parsed;
    parsed.valid = true;
    parsed.detail = "";

    const auto schema_version = parse_int_field(payload, "schema_version");
    if (!schema_version.has_value() || schema_version.value() != localization_config_.expected_schema_version) {
      parsed.valid = false;
      parsed.reason_code = savo_supervisor::reason::kLocalizationSchemaUnsupported;
      return parsed;
    }

    const auto alive = parse_bool_field(payload, "alive");
    parsed.alive = alive.value_or(false);
    parsed.ready = parsed.alive;
    if (!parsed.alive) {
      parsed.reason_code = savo_supervisor::reason::kLocalizationHeartbeatMissing;
      parsed.valid = false;
    }

    const auto state = parse_string_field(payload, "state");
    parsed.state = state.value_or("UNKNOWN");

    const auto ready = parse_bool_field(payload, "ready");
    parsed.ready = ready.value_or(false);

    const auto reason_code = parse_string_field(payload, "reason_code");
    parsed.reason_code = reason_code.value_or(savo_supervisor::reason::kLocalizationMessageInvalid);

    const auto stamp_val = parse_double_field(payload, "stamp_s");
    if (stamp_val.has_value()) {
      const double sec = stamp_val.value();
      if (!std::isfinite(sec) || sec < 0.0) {
        parsed.valid = false;
        parsed.detail = "invalid timestamp";
      } else {
        const auto sec_int = static_cast<int64_t>(std::floor(sec));
        builtin_interfaces::msg::Time stamp;
        stamp.sec = sec_int;
        stamp.nanosec = static_cast<uint32_t>((sec - sec_int) * 1e9);
        parsed.stamp_opt = stamp;
        parsed.stamp = rclcpp::Time(stamp.sec, stamp.nanosec, RCL_ROS_TIME);
      }
    }

    return parsed;
  }

  double publish_rate_hz_;
  double startup_grace_s_;
  std::string state_summary_topic_;
  std::string heartbeat_topic_;
  std::string health_topic_;
  std::string events_topic_;

  svo::ComponentConfig localization_config_;
  svo::ComponentStatus localization_status_;
  svo::SupervisorPolicy policy_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr summary_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time startup_time_;
  svo::SupervisorState last_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SupervisorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
