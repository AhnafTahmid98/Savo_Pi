// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/reason_codes.hpp"
#include "savo_supervisor/supervisor_policy.hpp"

#include <cmath>
#include <sstream>

namespace savo_supervisor
{

SupervisorPolicy::SupervisorPolicy()
{
}

bool SupervisorPolicy::Validate() const
{
  if (publish_rate_hz <= 0.0) {
    return false;
  }
  if (startup_grace_s < 0.0) {
    return false;
  }
  if (state_summary_topic.empty() || heartbeat_topic.empty() || health_topic.empty() || events_topic.empty()) {
    return false;
  }
  return true;
}

std::string SupervisorPolicy::ValidationError() const
{
  if (publish_rate_hz <= 0.0) {
    return "publish_rate_hz must be positive";
  }
  if (startup_grace_s < 0.0) {
    return "startup_grace_s must be non-negative";
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
  return {};
}

ComponentConfig SupervisorPolicy::DefaultLocalizationConfig()
{
  ComponentConfig config;
  config.name = "localization";
  config.enabled = true;
  config.required = true;
  config.health_topic = "/savo_localization/health";
  config.summary_topic = "/savo_localization/state_summary";
  config.heartbeat_topic = "/savo_localization/heartbeat";
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
  (void)startup_age_s;
  ComponentSummary summary;
  summary.name = status.config.name;
  summary.enabled = status.config.enabled;
  summary.required = status.config.required;

  if (!status.config.enabled) {
    summary.state = ComponentState::DISABLED;
    summary.ready = true;
    summary.degraded = false;
    summary.reason_code = "disabled";
    summary.last_message_age_s = 0.0;
    summary.timeout_s = 0.0;
    return summary;
  }

  summary.health_valid = status.health_valid;
  summary.summary_valid = status.summary_valid;
  summary.heartbeat_valid = status.heartbeat_valid;

  const auto health_snapshot = status.health_tracker.snapshot(now, status.config.health_timeout_s);
  const auto summary_snapshot = status.summary_tracker.snapshot(now, status.config.summary_timeout_s);
  const auto heartbeat_snapshot = status.heartbeat_tracker.snapshot(now, status.config.heartbeat_timeout_s);

  summary.last_message_age_s = std::max({health_snapshot.age_s, summary_snapshot.age_s, heartbeat_snapshot.age_s});
  summary.timeout_s = std::max({status.config.health_timeout_s, status.config.summary_timeout_s, status.config.heartbeat_timeout_s});

  if (!status.health_valid || !status.summary_valid || !status.heartbeat_valid) {
    summary.state = ComponentState::INVALID;
    summary.ready = false;
    summary.degraded = false;
    summary.reason_code = reason::kLocalizationMessageInvalid;
    return summary;
  }

  if (!health_snapshot.received) {
    summary.state = ComponentState::STALE;
    summary.ready = false;
    summary.degraded = false;
    summary.reason_code = reason::kLocalizationHealthMissing;
    return summary;
  }

  if (health_snapshot.stale || summary_snapshot.stale || heartbeat_snapshot.stale) {
    summary.state = ComponentState::STALE;
    summary.ready = false;
    summary.degraded = false;
    if (health_snapshot.stale) {
      summary.reason_code = reason::kLocalizationHealthStale;
    } else if (heartbeat_snapshot.stale) {
      summary.reason_code = reason::kLocalizationHeartbeatStale;
    } else {
      summary.reason_code = reason::kLocalizationHeartbeatMissing;
    }
    return summary;
  }

  if (!status.health_ready) {
    summary.state = ComponentState::ERROR;
    summary.ready = false;
    summary.degraded = false;
    summary.reason_code = reason::kLocalizationNotReady;
    return summary;
  }

  if (status.health_state == "INITIALIZING") {
    summary.state = ComponentState::INITIALIZING;
    summary.ready = false;
    summary.degraded = false;
    summary.reason_code = reason::kLocalizationInitializing;
    return summary;
  }

  if (status.health_state == "OK" && status.health_ready) {
    summary.state = ComponentState::OK;
    summary.ready = true;
    summary.degraded = false;
    summary.reason_code = reason::kLocalizationOperational;
    return summary;
  }

  if (status.health_state == "DEGRADED" && status.health_ready) {
    summary.state = ComponentState::DEGRADED;
    summary.ready = true;
    summary.degraded = true;
    summary.reason_code = reason::kLocalizationDegraded;
    return summary;
  }

  summary.state = ComponentState::ERROR;
  summary.ready = false;
  summary.degraded = false;
  summary.reason_code = reason::kLocalizationError;
  return summary;
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

  if (startup_age_s >= startup_grace_s) {
    if (!localization_summary.required) {
      if (localization_summary.state == ComponentState::DEGRADED && localization_summary.ready) {
        state.lifecycle = Lifecycle::RUNNING;
        state.health = AggregateHealth::DEGRADED;
        state.ready = true;
        state.degraded = true;
        state.reason_code = localization_summary.reason_code;
      } else {
        state.lifecycle = Lifecycle::RUNNING;
        state.health = AggregateHealth::OK;
        state.ready = true;
        state.degraded = false;
        state.reason_code = reason::kSupervisorOperational;
      }
    } else if (localization_summary.state == ComponentState::OK && localization_summary.ready) {
      state.lifecycle = Lifecycle::RUNNING;
      state.health = AggregateHealth::OK;
      state.ready = true;
      state.degraded = false;
      state.reason_code = reason::kSupervisorOperational;
    } else if (localization_summary.state == ComponentState::DEGRADED && localization_summary.ready) {
      state.lifecycle = Lifecycle::RUNNING;
      state.health = AggregateHealth::DEGRADED;
      state.ready = true;
      state.degraded = true;
      state.reason_code = localization_summary.reason_code;
    } else {
      state.lifecycle = Lifecycle::FAULTED;
      state.health = AggregateHealth::ERROR;
      state.ready = false;
      state.degraded = localization_summary.degraded;
      state.reason_code = localization_summary.reason_code;
    }
  }

  state.component_summaries.push_back(localization_summary);
  return state;
}

std::string SupervisorPolicy::CompactStateJson(
  const SupervisorState & state,
  const rclcpp::Time & now) const
{
  std::ostringstream output;
  output << '{';
  output << "\"schema_version\":1,";
  output << "\"node\":\"savo_supervisor\",";
  output << "\"lifecycle\":\"" << ToString(state.lifecycle) << "\",";
  output << "\"operating_mode\":\"" << ToString(state.operating_mode) << "\",";
  output << "\"health\":\"" << ToString(state.health) << "\",";
  output << "\"safety\":\"" << ToString(state.safety) << "\",";
  output << "\"ready\":" << (state.ready ? "true" : "false") << ',';
  output << "\"degraded\":" << (state.degraded ? "true" : "false") << ',';
  output << "\"reason_code\":\"" << state.reason_code << "\",";
  output << "\"stamp_s\":" << now.seconds() << ',';
  output << "\"components\":{";
  for (size_t i = 0; i < state.component_summaries.size(); ++i) {
    const auto & component = state.component_summaries[i];
    if (i > 0) {
      output << ',';
    }
    output << "\"" << component.name << "\":{";
    output << "\"enabled\":" << (component.enabled ? "true" : "false") << ',';
    output << "\"required\":" << (component.required ? "true" : "false") << ',';
    output << "\"received\":" << (component.received ? "true" : "false") << ',';
    output << "\"ready\":" << (component.ready ? "true" : "false") << ',';
    output << "\"degraded\":" << (component.degraded ? "true" : "false") << ',';
    output << "\"state\":\"" << ToString(component.state) << "\",";
    output << "\"reason_code\":\"" << component.reason_code << "\",";
    output << "\"last_message_age_s\":" << component.last_message_age_s << ',';
    output << "\"timeout_s\":" << component.timeout_s << ',';
    output << "\"malformed_message_count\":" << component.malformed_message_count << ',';
    output << "\"recovery_count\":" << component.recovery_count;
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
  output << '{';
  output << "\"schema_version\":1,";
  output << "\"node\":\"savo_supervisor\",";
  output << "\"alive\":true,";
  output << "\"ready\":" << (state.ready ? "true" : "false") << ',';
  output << "\"lifecycle\":\"" << ToString(state.lifecycle) << "\",";
  output << "\"health\":\"" << ToString(state.health) << "\",";
  output << "\"stamp_s\":" << now.seconds();
  output << '}';
  return output.str();
}

} // namespace savo_supervisor
