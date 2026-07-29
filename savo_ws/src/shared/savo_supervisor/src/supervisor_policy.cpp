// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/supervisor_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>

#include "savo_supervisor/reason_codes.hpp"

namespace savo_supervisor
{
namespace
{

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

double finite_age_or_negative_one(
  const FreshnessSnapshot & snapshot)
{
  if (!snapshot.received ||
    !std::isfinite(snapshot.age_s))
  {
    return -1.0;
  }

  return std::max(0.0, snapshot.age_s);
}

double maximum_received_age(
  const FreshnessSnapshot & health,
  const FreshnessSnapshot & summary,
  const FreshnessSnapshot & heartbeat)
{
  return std::max({
        finite_age_or_negative_one(health),
        finite_age_or_negative_one(summary),
        finite_age_or_negative_one(heartbeat)});
}

std::size_t total_malformed_count(
  const FreshnessSnapshot & health,
  const FreshnessSnapshot & summary,
  const FreshnessSnapshot & heartbeat)
{
  return
    health.malformed_count +
    summary.malformed_count +
    heartbeat.malformed_count;
}

std::size_t total_recovery_count(
  const FreshnessSnapshot & health,
  const FreshnessSnapshot & summary,
  const FreshnessSnapshot & heartbeat)
{
  return
    health.recovery_count +
    summary.recovery_count +
    heartbeat.recovery_count;
}

bool has_schema_error(const ComponentStatus & status)
{
  return
    status.health_reason_code ==
    reason::kLocalizationSchemaUnsupported ||
    status.summary_reason_code ==
    reason::kLocalizationSchemaUnsupported ||
    status.heartbeat_reason_code ==
    reason::kLocalizationSchemaUnsupported;
}

bool localization_states_consistent(
  const ComponentStatus & status)
{
  if (status.health_state != status.summary_state) {
    return false;
  }

  if (status.health_ready != status.summary_ready) {
    return false;
  }

  if (
    status.health_degraded !=
    status.summary_degraded)
  {
    return false;
  }

  if (status.heartbeat_ready != status.health_ready) {
    return false;
  }

  if (
    !status.heartbeat_state.empty() &&
    status.heartbeat_state != "UNKNOWN" &&
    status.heartbeat_state != status.health_state)
  {
    return false;
  }

  return true;
}

void set_component_result(
  ComponentSummary & summary,
  ComponentState state,
  bool ready,
  bool degraded,
  const std::string & reason_code,
  const std::string & detail)
{
  summary.state = state;
  summary.ready = ready;
  summary.degraded = degraded;
  summary.reason_code = reason_code;
  summary.detail = detail;
}

bool has_duplicate_strings(
  const std::array<std::string, 4> & values)
{
  std::set<std::string> unique;

  for (const auto & value : values) {
    unique.insert(value);
  }

  return unique.size() != values.size();
}

}  // namespace

SupervisorPolicy::SupervisorPolicy()
: localization(DefaultLocalizationConfig())
{
}

bool SupervisorPolicy::Validate() const
{
  return ValidationError().empty();
}

std::string SupervisorPolicy::ValidationError() const
{
  if (!finite_positive(publish_rate_hz)) {
    return "publish_rate_hz must be finite and positive";
  }

  if (!finite_nonnegative(startup_grace_s)) {
    return "startup_grace_s must be finite and non-negative";
  }

  if (state_summary_topic.empty()) {
    return "state_summary_topic must be set";
  }

  if (heartbeat_topic.empty()) {
    return "heartbeat_topic must be set";
  }

  if (health_topic.empty()) {
    return "health_topic must be set";
  }

  if (events_topic.empty()) {
    return "events_topic must be set";
  }

  const std::array<std::string, 4> output_topics{
    state_summary_topic,
    heartbeat_topic,
    health_topic,
    events_topic};

  if (has_duplicate_strings(output_topics)) {
    return "supervisor output topics must be unique";
  }

  if (!localization.enabled) {
    return {};
  }

  if (localization.name.empty()) {
    return "localization component name must be set";
  }

  if (localization.health_topic.empty()) {
    return "localization health_topic must be set";
  }

  if (localization.summary_topic.empty()) {
    return "localization summary_topic must be set";
  }

  if (localization.heartbeat_topic.empty()) {
    return "localization heartbeat_topic must be set";
  }

  if (!finite_positive(localization.health_timeout_s)) {
    return "localization health_timeout_s must be finite and positive";
  }

  if (!finite_positive(localization.summary_timeout_s)) {
    return "localization summary_timeout_s must be finite and positive";
  }

  if (!finite_positive(localization.heartbeat_timeout_s)) {
    return "localization heartbeat_timeout_s must be finite and positive";
  }

  if (localization.expected_schema_version != 1) {
    return "localization expected_schema_version must be 1";
  }

  const std::set<std::string> localization_inputs{
    localization.health_topic,
    localization.summary_topic,
    localization.heartbeat_topic};

  if (localization_inputs.size() != 3U) {
    return "localization input topics must be unique";
  }

  for (const auto & output_topic : output_topics) {
    if (localization_inputs.count(output_topic) != 0U) {
      return "supervisor output topics must not match localization input topics";
    }
  }

  return {};
}

ComponentConfig SupervisorPolicy::DefaultLocalizationConfig()
{
  ComponentConfig config;

  config.name = "localization";
  config.enabled = true;
  config.required = true;

  config.health_topic =
    "/savo_localization/health";

  config.summary_topic =
    "/savo_localization/state_summary";

  config.heartbeat_topic =
    "/savo_localization/heartbeat";

  config.health_timeout_s = 1.5;
  config.summary_timeout_s = 1.5;
  config.heartbeat_timeout_s = 2.5;

  config.expected_schema_version = 1;

  return config;
}

ComponentSummary SupervisorPolicy::EvaluateComponent(
  const ComponentStatus & status,
  const rclcpp::Time & now,
  const double startup_age_s) const
{
  ComponentSummary result;

  result.name = status.config.name;
  result.enabled = status.config.enabled;
  result.required = status.config.required;

  if (!status.config.enabled) {
    set_component_result(
      result,
      ComponentState::DISABLED,
      true,
      false,
      reason::kComponentDisabled,
      "component disabled by configuration");

    result.received = false;
    result.last_message_age_s = -1.0;
    result.timeout_s = 0.0;

    return result;
  }

  const auto health_snapshot =
    status.health_tracker.snapshot(
      now,
      status.config.health_timeout_s);

  const auto summary_snapshot =
    status.summary_tracker.snapshot(
      now,
      status.config.summary_timeout_s);

  const auto heartbeat_snapshot =
    status.heartbeat_tracker.snapshot(
      now,
      status.config.heartbeat_timeout_s);

  result.health_valid = status.health_valid;
  result.summary_valid = status.summary_valid;
  result.heartbeat_valid = status.heartbeat_valid;

  result.received =
    health_snapshot.received &&
    summary_snapshot.received &&
    heartbeat_snapshot.received;

  result.last_message_age_s =
    maximum_received_age(
      health_snapshot,
      summary_snapshot,
      heartbeat_snapshot);

  result.timeout_s = std::max({
      status.config.health_timeout_s,
      status.config.summary_timeout_s,
      status.config.heartbeat_timeout_s});

  result.malformed_message_count =
    total_malformed_count(
      health_snapshot,
      summary_snapshot,
      heartbeat_snapshot);

  result.recovery_count =
    total_recovery_count(
      health_snapshot,
      summary_snapshot,
      heartbeat_snapshot);

  const bool within_startup_grace =
    finite_nonnegative(startup_age_s) &&
    startup_age_s < startup_grace_s;

  if (!health_snapshot.received) {
    if (within_startup_grace) {
      set_component_result(
        result,
        ComponentState::INITIALIZING,
        false,
        false,
        reason::kLocalizationInitializing,
        "waiting for localization health");
    } else {
      set_component_result(
        result,
        ComponentState::STALE,
        false,
        false,
        reason::kLocalizationHealthMissing,
        health_snapshot.detail);
    }

    return result;
  }

  if (!summary_snapshot.received) {
    if (within_startup_grace) {
      set_component_result(
        result,
        ComponentState::INITIALIZING,
        false,
        false,
        reason::kLocalizationInitializing,
        "waiting for localization state summary");
    } else {
      set_component_result(
        result,
        ComponentState::STALE,
        false,
        false,
        reason::kLocalizationSummaryMissing,
        summary_snapshot.detail);
    }

    return result;
  }

  if (!heartbeat_snapshot.received) {
    if (within_startup_grace) {
      set_component_result(
        result,
        ComponentState::INITIALIZING,
        false,
        false,
        reason::kLocalizationInitializing,
        "waiting for localization heartbeat");
    } else {
      set_component_result(
        result,
        ComponentState::STALE,
        false,
        false,
        reason::kLocalizationHeartbeatMissing,
        heartbeat_snapshot.detail);
    }

    return result;
  }

  if (
    health_snapshot.time_regression ||
    summary_snapshot.time_regression ||
    heartbeat_snapshot.time_regression)
  {
    set_component_result(
      result,
      ComponentState::INVALID,
      false,
      false,
      reason::kRosTimeRegressionDetected,
      "localization message time regressed");

    return result;
  }

  if (has_schema_error(status)) {
    set_component_result(
      result,
      ComponentState::INVALID,
      false,
      false,
      reason::kLocalizationSchemaUnsupported,
      "localization schema version is unsupported");

    return result;
  }

  if (
    !status.health_valid ||
    !status.summary_valid ||
    !status.heartbeat_valid ||
    health_snapshot.malformed ||
    summary_snapshot.malformed ||
    heartbeat_snapshot.malformed ||
    health_snapshot.timestamp_fault ||
    summary_snapshot.timestamp_fault ||
    heartbeat_snapshot.timestamp_fault)
  {
    set_component_result(
      result,
      ComponentState::INVALID,
      false,
      false,
      reason::kLocalizationMessageInvalid,
      "localization contract message is invalid");

    return result;
  }

  if (health_snapshot.stale) {
    set_component_result(
      result,
      ComponentState::STALE,
      false,
      false,
      reason::kLocalizationHealthStale,
      health_snapshot.detail);

    return result;
  }

  if (summary_snapshot.stale) {
    set_component_result(
      result,
      ComponentState::STALE,
      false,
      false,
      reason::kLocalizationSummaryStale,
      summary_snapshot.detail);

    return result;
  }

  if (heartbeat_snapshot.stale) {
    set_component_result(
      result,
      ComponentState::STALE,
      false,
      false,
      reason::kLocalizationHeartbeatStale,
      heartbeat_snapshot.detail);

    return result;
  }

  if (!status.heartbeat_alive) {
    set_component_result(
      result,
      ComponentState::ERROR,
      false,
      false,
      reason::kLocalizationHeartbeatNotAlive,
      "localization heartbeat reports alive=false");

    return result;
  }

  if (!localization_states_consistent(status)) {
    set_component_result(
      result,
      ComponentState::INVALID,
      false,
      false,
      reason::kLocalizationStateInconsistent,
      "health, summary, and heartbeat disagree");

    return result;
  }

  if (status.health_state == "INITIALIZING") {
    set_component_result(
      result,
      ComponentState::INITIALIZING,
      false,
      false,
      reason::kLocalizationInitializing,
      status.health_reason_code);

    return result;
  }

  if (
    status.health_state == "OK" &&
    status.health_ready &&
    !status.health_degraded)
  {
    set_component_result(
      result,
      ComponentState::OK,
      true,
      false,
      reason::kLocalizationOperational,
      status.health_reason_code);

    return result;
  }

  if (
    status.health_state == "DEGRADED" &&
    status.health_ready)
  {
    set_component_result(
      result,
      ComponentState::DEGRADED,
      true,
      true,
      reason::kLocalizationDegraded,
      status.health_reason_code);

    return result;
  }

  if (status.health_state == "STALE") {
    set_component_result(
      result,
      ComponentState::STALE,
      false,
      false,
      status.health_reason_code.empty() ?
      reason::kLocalizationNotReady :
      status.health_reason_code,
      "localization reports STALE");

    return result;
  }

  if (status.health_state == "ERROR") {
    set_component_result(
      result,
      ComponentState::ERROR,
      false,
      false,
      reason::kLocalizationError,
      status.health_reason_code);

    return result;
  }

  if (!status.health_ready) {
    set_component_result(
      result,
      ComponentState::ERROR,
      false,
      false,
      reason::kLocalizationNotReady,
      status.health_reason_code);

    return result;
  }

  set_component_result(
    result,
    ComponentState::INVALID,
    false,
    false,
    reason::kLocalizationMessageInvalid,
    "unsupported localization state value");

  return result;
}

SupervisorState SupervisorPolicy::EvaluateSupervisor(
  const ComponentSummary & localization_summary,
  const rclcpp::Time & now,
  const double startup_age_s) const
{
  (void)now;

  SupervisorState state;

  state.lifecycle = Lifecycle::STARTING;
  state.operating_mode = OperatingMode::STOP;
  state.health = AggregateHealth::UNKNOWN;
  state.safety = SafetyObservation::UNKNOWN;
  state.ready = false;
  state.degraded = false;
  state.reason_code = reason::kSupervisorStarting;

  state.component_summaries.push_back(
    localization_summary);

  if (!localization_summary.required) {
    state.lifecycle = Lifecycle::RUNNING;
    state.ready = true;

    if (
      localization_summary.state ==
      ComponentState::DISABLED ||
      localization_summary.state ==
      ComponentState::OK)
    {
      state.health = AggregateHealth::OK;
      state.degraded = false;
      state.reason_code =
        reason::kSupervisorOperational;
    } else {
      state.health = AggregateHealth::DEGRADED;
      state.degraded = true;
      state.reason_code =
        localization_summary.reason_code;
    }

    return state;
  }

  if (
    localization_summary.state ==
    ComponentState::INITIALIZING)
  {
    return state;
  }

  const bool within_startup_grace =
    finite_nonnegative(startup_age_s) &&
    startup_age_s < startup_grace_s;

  if (
    within_startup_grace &&
    !localization_summary.ready)
  {
    return state;
  }

  if (
    localization_summary.state ==
    ComponentState::OK &&
    localization_summary.ready)
  {
    state.lifecycle = Lifecycle::RUNNING;
    state.health = AggregateHealth::OK;
    state.ready = true;
    state.degraded = false;
    state.reason_code =
      reason::kSupervisorOperational;

    return state;
  }

  if (
    localization_summary.state ==
    ComponentState::DEGRADED &&
    localization_summary.ready)
  {
    state.lifecycle = Lifecycle::RUNNING;
    state.health = AggregateHealth::DEGRADED;
    state.ready = true;
    state.degraded = true;
    state.reason_code =
      localization_summary.reason_code;

    return state;
  }

  state.lifecycle = Lifecycle::FAULTED;
  state.health = AggregateHealth::ERROR;
  state.ready = false;
  state.degraded = false;

  state.reason_code =
    localization_summary.reason_code.empty() ?
    reason::kRequiredComponentUnavailable :
    localization_summary.reason_code;

  return state;
}

std::string SupervisorPolicy::CompactStateJson(
  const SupervisorState & state,
  const rclcpp::Time & now) const
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(6);

  output << '{';
  output << "\"schema_version\":1,";
  output << "\"node\":\"savo_supervisor\",";
  output << "\"lifecycle\":\""
         << ToString(state.lifecycle) << "\",";
  output << "\"operating_mode\":\""
         << ToString(state.operating_mode) << "\",";
  output << "\"health\":\""
         << ToString(state.health) << "\",";
  output << "\"safety\":\""
         << ToString(state.safety) << "\",";
  output << "\"ready\":"
         << (state.ready ? "true" : "false") << ',';
  output << "\"degraded\":"
         << (state.degraded ? "true" : "false") << ',';
  output << "\"reason_code\":\""
         << state.reason_code << "\",";
  output << "\"stamp_s\":"
         << now.seconds() << ',';
  output << "\"components\":{";

  for (
    std::size_t index = 0;
    index < state.component_summaries.size();
    ++index)
  {
    const auto & component =
      state.component_summaries[index];

    if (index > 0U) {
      output << ',';
    }

    output << "\"" << component.name << "\":{";
    output << "\"enabled\":"
           << (component.enabled ? "true" : "false")
           << ',';
    output << "\"required\":"
           << (component.required ? "true" : "false")
           << ',';
    output << "\"received\":"
           << (component.received ? "true" : "false")
           << ',';
    output << "\"ready\":"
           << (component.ready ? "true" : "false")
           << ',';
    output << "\"degraded\":"
           << (component.degraded ? "true" : "false")
           << ',';
    output << "\"state\":\""
           << ToString(component.state) << "\",";
    output << "\"reason_code\":\""
           << component.reason_code << "\",";
    output << "\"last_message_age_s\":"
           << component.last_message_age_s << ',';
    output << "\"timeout_s\":"
           << component.timeout_s << ',';
    output << "\"malformed_message_count\":"
           << component.malformed_message_count << ',';
    output << "\"recovery_count\":"
           << component.recovery_count;
    output << '}';
  }

  output << '}';
  output << '}';

  return output.str();
}

std::string SupervisorPolicy::CompactHeartbeatJson(
  const SupervisorState & state,
  const rclcpp::Time & now) const
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(6);

  output << '{';
  output << "\"schema_version\":1,";
  output << "\"node\":\"savo_supervisor\",";
  output << "\"alive\":true,";
  output << "\"ready\":"
         << (state.ready ? "true" : "false") << ',';
  output << "\"lifecycle\":\""
         << ToString(state.lifecycle) << "\",";
  output << "\"health\":\""
         << ToString(state.health) << "\",";
  output << "\"stamp_s\":"
         << now.seconds();
  output << '}';

  return output.str();
}

}  // namespace savo_supervisor
