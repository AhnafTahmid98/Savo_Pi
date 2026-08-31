// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/supervisor_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "savo_supervisor/reason_codes.hpp"

namespace savo_supervisor
{
namespace
{
using Json = nlohmann::json;

enum class Channel {kHealth, kSummary, kHeartbeat};

bool finite_positive(double value) {return std::isfinite(value) && value > 0.0;}
bool finite_nonnegative(double value) {return std::isfinite(value) && value >= 0.0;}

bool channel_required(const ComponentConfig & config, Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return config.health_required;
    case Channel::kSummary: return config.summary_required;
    case Channel::kHeartbeat: return config.heartbeat_required;
  }
  return false;
}

const std::string & channel_topic(const ComponentConfig & config, Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return config.health_topic;
    case Channel::kSummary: return config.summary_topic;
    case Channel::kHeartbeat: return config.heartbeat_topic;
  }
  return config.health_topic;
}

double channel_timeout(const ComponentConfig & config, Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return config.health_timeout_s;
    case Channel::kSummary: return config.summary_timeout_s;
    case Channel::kHeartbeat: return config.heartbeat_timeout_s;
  }
  return 0.0;
}

const FreshnessTracker & channel_tracker(const ComponentStatus & status, Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return status.health_tracker;
    case Channel::kSummary: return status.summary_tracker;
    case Channel::kHeartbeat: return status.heartbeat_tracker;
  }
  return status.health_tracker;
}

bool channel_valid(const ComponentStatus & status, Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return status.health_valid;
    case Channel::kSummary: return status.summary_valid;
    case Channel::kHeartbeat: return status.heartbeat_valid;
  }
  return false;
}

const std::string & channel_state(const ComponentStatus & status, Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return status.health_state;
    case Channel::kSummary: return status.summary_state;
    case Channel::kHeartbeat: return status.heartbeat_state;
  }
  return status.health_state;
}

bool channel_ready(const ComponentStatus & status, Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return status.health_ready;
    case Channel::kSummary: return status.summary_ready;
    case Channel::kHeartbeat: return status.heartbeat_ready;
  }
  return false;
}

bool channel_degraded(const ComponentStatus & status, Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return status.health_degraded;
    case Channel::kSummary: return status.summary_degraded;
    case Channel::kHeartbeat: return status.heartbeat_degraded;
  }
  return false;
}

const std::string & channel_reason(const ComponentStatus & status, Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return status.health_reason_code;
    case Channel::kSummary: return status.summary_reason_code;
    case Channel::kHeartbeat: return status.heartbeat_reason_code;
  }
  return status.health_reason_code;
}

std::string channel_name(Channel channel)
{
  switch (channel) {
    case Channel::kHealth: return "health";
    case Channel::kSummary: return "summary";
    case Channel::kHeartbeat: return "heartbeat";
  }
  return "unknown";
}

std::string component_reason(
  const ComponentConfig & config,
  Channel channel,
  const std::string & condition)
{
  if (config.name == "localization") {
    if (condition == "initializing") {return reason::kLocalizationInitializing;}
    if (condition == "missing" && channel == Channel::kHealth) {
      return reason::kLocalizationHealthMissing;
    }
    if (condition == "missing" && channel == Channel::kSummary) {
      return reason::kLocalizationSummaryMissing;
    }
    if (condition == "missing" && channel == Channel::kHeartbeat) {
      return reason::kLocalizationHeartbeatMissing;
    }
    if (condition == "stale" && channel == Channel::kHealth) {
      return reason::kLocalizationHealthStale;
    }
    if (condition == "stale" && channel == Channel::kSummary) {
      return reason::kLocalizationSummaryStale;
    }
    if (condition == "stale" && channel == Channel::kHeartbeat) {
      return reason::kLocalizationHeartbeatStale;
    }
    if (condition == "invalid") {return reason::kLocalizationMessageInvalid;}
    if (condition == "not_alive") {return reason::kLocalizationHeartbeatNotAlive;}
    if (condition == "not_ready") {return reason::kLocalizationNotReady;}
    if (condition == "error") {return reason::kLocalizationError;}
    if (condition == "operational") {return reason::kLocalizationOperational;}
    if (condition == "degraded") {return reason::kLocalizationDegraded;}
  }
  return config.name + "_" + channel_name(channel) + "_" + condition;
}

void set_component_result(
  ComponentSummary & summary,
  ComponentState state,
  bool ready,
  bool degraded,
  std::string reason_code,
  std::string detail)
{
  summary.state = state;
  summary.ready = ready;
  summary.degraded = degraded;
  summary.reason_code = std::move(reason_code);
  summary.detail = std::move(detail);
}

bool states_consistent(const ComponentStatus & status)
{
  const std::array<Channel, 3> channels{
    Channel::kHealth, Channel::kSummary, Channel::kHeartbeat};
  std::string state;
  bool ready = false;
  bool degraded = false;
  bool initialized = false;
  for (const auto channel : channels) {
    if (!channel_required(status.config, channel)) {continue;}
    if (!initialized) {
      state = channel_state(status, channel);
      ready = channel_ready(status, channel);
      degraded = channel_degraded(status, channel);
      initialized = true;
      continue;
    }
    const auto observed_state = channel_state(status, channel);
    if (channel == Channel::kHeartbeat && (observed_state.empty() || observed_state == "UNKNOWN")) {
      if (channel_ready(status, channel) != ready) {return false;}
      continue;
    }
    if (observed_state != state || channel_ready(status, channel) != ready) {
      return false;
    }
    if (channel != Channel::kHeartbeat && channel_degraded(status, channel) != degraded) {
      return false;
    }
  }
  return true;
}

bool semantically_compatible_operational_transition(
  const ComponentStatus & status)
{
  const std::array<Channel, 3> channels{
    Channel::kHealth, Channel::kSummary, Channel::kHeartbeat};
  for (const auto channel : channels) {
    if (!channel_required(status.config, channel)) {continue;}
    const auto & state = channel_state(status, channel);
    if ((state != "OK" && state != "DEGRADED") || !channel_ready(status, channel)) {
      return false;
    }
    if (channel != Channel::kHeartbeat &&
      channel_degraded(status, channel) != (state == "DEGRADED"))
    {
      return false;
    }
  }
  return true;
}

bool required_component_ready(const std::vector<ComponentSummary> & summaries)
{
  return std::all_of(summaries.begin(), summaries.end(), [](const ComponentSummary & item) {
             return !item.required || item.ready;
  });
}

const ComponentSummary * find_component(
  const std::vector<ComponentSummary> & summaries,
  const std::string & name)
{
  const auto iterator = std::find_if(
    summaries.begin(), summaries.end(), [&name](const auto & item) {
      return item.name == name;
    });
  return iterator == summaries.end() ? nullptr : &(*iterator);
}

bool component_ready(
  const std::vector<ComponentSummary> & summaries,
  const std::string & name)
{
  const auto * item = find_component(summaries, name);
  return item != nullptr && item->enabled && item->ready;
}

bool component_nominal(
  const std::vector<ComponentSummary> & summaries,
  const std::string & name)
{
  const auto * item = find_component(summaries, name);
  return item != nullptr && item->ready && !item->degraded && item->state == ComponentState::OK;
}

std::string validate_component(const ComponentConfig & config)
{
  if (config.required && !config.enabled) {
    return config.name + " cannot be required while disabled";
  }
  if (!config.enabled) {return {};}
  if (config.name.empty()) {return "component name must be set";}
  const std::array<Channel, 3> channels{
    Channel::kHealth, Channel::kSummary, Channel::kHeartbeat};
  std::set<std::string> topics;
  for (const auto channel : channels) {
    if (!channel_required(config, channel)) {continue;}
    if (channel_topic(config, channel).empty()) {
      return config.name + " " + channel_name(channel) + "_topic must be set";
    }
    if (!finite_positive(channel_timeout(config, channel))) {
      return config.name + " " + channel_name(channel) +
             "_timeout_s must be finite and positive";
    }
    if (!topics.insert(channel_topic(config, channel)).second) {
      return config.name + " input topics must be unique";
    }
  }
  if (topics.empty()) {return config.name + " must require at least one input channel";}
  if (config.enforce_consistency &&
    !finite_nonnegative(config.consistency_transition_grace_s))
  {
    return config.name + " consistency_transition_grace_s must be finite and non-negative";
  }
  return {};
}

ComponentConfig make_config(
  std::string name,
  std::string health_topic,
  std::string summary_topic,
  std::string heartbeat_topic,
  bool health_required,
  bool summary_required,
  bool heartbeat_required,
  double health_timeout,
  double summary_timeout,
  double heartbeat_timeout,
  bool enforce_consistency = false)
{
  ComponentConfig config;
  config.name = std::move(name);
  config.enabled = true;
  config.required = true;
  config.health_topic = std::move(health_topic);
  config.summary_topic = std::move(summary_topic);
  config.heartbeat_topic = std::move(heartbeat_topic);
  config.health_required = health_required;
  config.summary_required = summary_required;
  config.heartbeat_required = heartbeat_required;
  config.health_timeout_s = health_timeout;
  config.summary_timeout_s = summary_timeout;
  config.heartbeat_timeout_s = heartbeat_timeout;
  config.enforce_consistency = enforce_consistency;
  return config;
}

}  // namespace

SupervisorPolicy::SupervisorPolicy()
: base(DefaultBaseConfig()),
  control(DefaultControlConfig()),
  perception(DefaultPerceptionConfig()),
  lidar(DefaultLidarConfig()),
  localization(DefaultLocalizationConfig()),
  power(DefaultPowerConfig())
{
}

bool SupervisorPolicy::Validate() const {return ValidationError().empty();}

std::string SupervisorPolicy::ValidationError() const
{
  if (!finite_positive(publish_rate_hz)) {return "publish_rate_hz must be finite and positive";}
  if (!finite_nonnegative(startup_grace_s)) {
    return "startup_grace_s must be finite and non-negative";
  }
  const std::array<std::string, 4> outputs{
    state_summary_topic, heartbeat_topic, health_topic, events_topic};
  if (std::any_of(outputs.begin(), outputs.end(), [](const auto & topic) {return topic.empty();})) {
    return "supervisor output topics must be set";
  }
  if (std::set<std::string>(outputs.begin(), outputs.end()).size() != outputs.size()) {
    return "supervisor output topics must be unique";
  }

  const std::array<const ComponentConfig *, 6> components{
    &base, &control, &perception, &lidar, &localization, &power};
  for (const auto * config : components) {
    const auto error = validate_component(*config);
    if (!error.empty()) {return error;}
    if (config->name == "localization" && config->expected_schema_version != 1) {
      return "localization expected_schema_version must be 1";
    }
  }

  for (const auto * config : components) {
    for (const auto & output : outputs) {
      if ((config->health_required && config->health_topic == output) ||
        (config->summary_required && config->summary_topic == output) ||
        (config->heartbeat_required && config->heartbeat_topic == output))
      {
        return "supervisor output topics must not match component input topics";
      }
    }
  }
  return {};
}

ComponentConfig SupervisorPolicy::DefaultBaseConfig()
{
  return make_config("base", "", "/savo_base/base_state", "", false, true, false, 1.5, 1.5, 1.5);
}

ComponentConfig SupervisorPolicy::DefaultControlConfig()
{
  return make_config(
    "control", "/savo_control/control_status", "/savo_control/twist_mux/status",
    "/savo_control/cmd_vel_shaper/status", true, true, true, 1.0, 1.0, 1.0);
}

ComponentConfig SupervisorPolicy::DefaultPerceptionConfig()
{
  return make_config(
    "perception", "/savo_perception/range_health", "/savo_perception/safety_state",
    "/savo_perception/heartbeat", true, true, true, 1.5, 1.0, 2.5);
}

ComponentConfig SupervisorPolicy::DefaultLidarConfig()
{
  return make_config(
    "lidar", "", "/savo_lidar/state", "/savo_lidar/heartbeat",
    false, true, true, 1.5, 2.0, 3.0);
}

ComponentConfig SupervisorPolicy::DefaultLocalizationConfig()
{
  auto config = make_config(
    "localization", "/savo_localization/health", "/savo_localization/state_summary",
    "/savo_localization/heartbeat", true, true, true, 1.5, 1.5, 2.5, true);
  config.expected_schema_version = 1;
  config.consistency_transition_grace_s = 1.5;
  return config;
}

ComponentConfig SupervisorPolicy::DefaultPowerConfig()
{
  return make_config(
    "power", "/savo_power/health", "/savo_power/status", "",
    true, true, false, 3.0, 3.0, 3.0);
}

ComponentSummary SupervisorPolicy::EvaluateComponent(
  ComponentStatus & status,
  const rclcpp::Time & now,
  double startup_age_s) const
{
  ComponentSummary result;
  result.name = status.config.name;
  result.enabled = status.config.enabled;
  result.required = status.config.required;

  if (!status.config.enabled) {
    set_component_result(result, ComponentState::DISABLED, true, false,
      reason::kComponentDisabled, "component disabled by configuration");
    return result;
  }

  const bool within_grace = finite_nonnegative(startup_age_s) && startup_age_s < startup_grace_s;
  const std::array<Channel, 3> channels{
    Channel::kHealth, Channel::kSummary, Channel::kHeartbeat};
  bool all_received = true;
  bool any_degraded = false;
  bool any_initializing = false;
  double maximum_age = -1.0;
  double maximum_timeout = 0.0;
  result.health_valid = !status.config.health_required;
  result.summary_valid = !status.config.summary_required;
  result.heartbeat_valid = !status.config.heartbeat_required;

  for (const auto channel : channels) {
    if (!channel_required(status.config, channel)) {continue;}
    const auto snapshot = channel_tracker(status, channel).snapshot(
      now, channel_timeout(status.config, channel));
    all_received = all_received && snapshot.received;
    if (snapshot.received && std::isfinite(snapshot.age_s)) {
      maximum_age = std::max(maximum_age, snapshot.age_s);
    }
    maximum_timeout = std::max(maximum_timeout, channel_timeout(status.config, channel));
    result.malformed_message_count += snapshot.malformed_count;
    result.recovery_count += snapshot.recovery_count;
    result.received = all_received;
    result.last_message_age_s = maximum_age;
    result.timeout_s = maximum_timeout;
    if (channel == Channel::kHealth) {result.health_valid = status.health_valid;}
    if (channel == Channel::kSummary) {result.summary_valid = status.summary_valid;}
    if (channel == Channel::kHeartbeat) {result.heartbeat_valid = status.heartbeat_valid;}

    if (!snapshot.received) {
      status.consistency_mismatch_since.reset();
      set_component_result(result,
        within_grace ? ComponentState::INITIALIZING : ComponentState::STALE,
        false, false,
        component_reason(status.config, channel, within_grace ? "initializing" : "missing"),
        within_grace ?
        "waiting for " + status.config.name + " " + channel_name(channel) :
        snapshot.detail);
      result.received = false;
      result.last_message_age_s = maximum_age;
      result.timeout_s = maximum_timeout;
      return result;
    }
    if (snapshot.time_regression || snapshot.timestamp_fault) {
      status.consistency_mismatch_since.reset();
      set_component_result(result, ComponentState::INVALID, false, false,
        reason::kRosTimeRegressionDetected, status.config.name + " message time regressed");
      return result;
    }
    if (!channel_valid(status, channel) || snapshot.malformed) {
      status.consistency_mismatch_since.reset();
      const auto & observed_reason = channel_reason(status, channel);
      set_component_result(result, ComponentState::INVALID, false, false,
        observed_reason.empty() ?
        component_reason(status.config, channel, "invalid") : observed_reason,
        snapshot.detail.empty() ?
        status.config.name + " contract message is invalid" : snapshot.detail);
      return result;
    }
    if (snapshot.stale) {
      status.consistency_mismatch_since.reset();
      set_component_result(result, ComponentState::STALE, false, false,
        component_reason(status.config, channel, "stale"), snapshot.detail);
      return result;
    }
    if (channel == Channel::kHeartbeat && !status.heartbeat_alive) {
      status.consistency_mismatch_since.reset();
      set_component_result(result, ComponentState::ERROR, false, false,
        component_reason(status.config, channel, "not_alive"), "heartbeat reports alive=false");
      return result;
    }
  }

  result.received = all_received;
  result.last_message_age_s = maximum_age;
  result.timeout_s = maximum_timeout;
  result.health_valid = !status.config.health_required || status.health_valid;
  result.summary_valid = !status.config.summary_required || status.summary_valid;
  result.heartbeat_valid = !status.config.heartbeat_required || status.heartbeat_valid;

  if (status.config.enforce_consistency) {
    if (states_consistent(status)) {
      status.consistency_mismatch_since.reset();
    } else if (semantically_compatible_operational_transition(status)) {
      if (!status.consistency_mismatch_since.has_value()) {
        status.consistency_mismatch_since = now;
      }
      const double mismatch_age_s =
        (now - status.consistency_mismatch_since.value()).seconds();
      if (mismatch_age_s > status.config.consistency_transition_grace_s) {
        set_component_result(result, ComponentState::INVALID, false, false,
          reason::kLocalizationStateInconsistent,
          "compatible localization transition exceeded coherency grace");
        return result;
      }
    } else {
      status.consistency_mismatch_since.reset();
      set_component_result(result, ComponentState::INVALID, false, false,
        reason::kLocalizationStateInconsistent, "required component channels disagree");
      return result;
    }
  }

  for (const auto channel : channels) {
    if (!channel_required(status.config, channel)) {continue;}
    const auto & observed_state = channel_state(status, channel);
    if (observed_state == "ERROR") {
      set_component_result(result, ComponentState::ERROR, false, false,
        channel_reason(status, channel).empty() ?
        component_reason(status.config, channel, "error") :
        channel_reason(status, channel),
        status.config.name + " reports ERROR");
      return result;
    }
    if (observed_state == "STALE") {
      set_component_result(result, ComponentState::STALE, false, false,
        channel_reason(status, channel).empty() ?
        component_reason(status.config, channel, "stale") :
        channel_reason(status, channel),
        status.config.name + " reports STALE");
      return result;
    }
    if (observed_state == "INITIALIZING") {any_initializing = true;}
    if (!channel_ready(status, channel) && observed_state != "INITIALIZING") {
      set_component_result(result, ComponentState::ERROR, false, false,
        channel_reason(status, channel).empty() ?
        component_reason(status.config, channel, "not_ready") :
        channel_reason(status, channel),
        status.config.name + " reports ready=false");
      return result;
    }
    any_degraded = any_degraded || channel_degraded(status, channel) ||
      observed_state == "DEGRADED";
  }

  if (any_initializing) {
    set_component_result(result, ComponentState::INITIALIZING, false, false,
      component_reason(status.config, Channel::kHealth, "initializing"),
      status.config.name + " is initializing");
    return result;
  }
  if (any_degraded) {
    set_component_result(result, ComponentState::DEGRADED, true, true,
      component_reason(status.config, Channel::kHealth, "degraded"),
      status.config.name + " is operational with restrictions");
    return result;
  }
  set_component_result(result, ComponentState::OK, true, false,
    component_reason(status.config, Channel::kHealth, "operational"),
    status.config.name + " is operational");
  return result;
}

SupervisorState SupervisorPolicy::EvaluateSupervisor(
  const std::vector<ComponentSummary> & summaries,
  const SafetySummary & safety,
  const rclcpp::Time & now,
  double startup_age_s) const
{
  (void)now;
  SupervisorState state;
  state.component_summaries = summaries;
  state.safety_summary = safety;
  state.safety = safety.observation;
  state.reason_code = reason::kSupervisorStarting;

  const bool within_grace = finite_nonnegative(startup_age_s) && startup_age_s < startup_grace_s;
  const auto unavailable = std::find_if(
    summaries.begin(), summaries.end(), [](const ComponentSummary & item) {
      return item.required && !item.ready &&
             item.state != ComponentState::INITIALIZING;
    });
  const bool initializing = std::any_of(
    summaries.begin(), summaries.end(), [](const ComponentSummary & item) {
      return item.required && item.state == ComponentState::INITIALIZING;
    });

  if (initializing || (within_grace && (!required_component_ready(summaries) || !safety.ready))) {
    state.lifecycle = Lifecycle::STARTING;
    state.health = AggregateHealth::UNKNOWN;
    return state;
  }

  if (unavailable != summaries.end()) {
    state.lifecycle = Lifecycle::FAULTED;
    state.health = AggregateHealth::ERROR;
    state.reason_code = unavailable->reason_code.empty() ?
      reason::kRequiredComponentUnavailable : unavailable->reason_code;
    return state;
  }

  if (!safety.ready) {
    state.lifecycle = Lifecycle::FAULTED;
    state.health = AggregateHealth::ERROR;
    state.reason_code = safety.reason_code.empty() ?
      reason::kSafetyStateUnknown : safety.reason_code;
    return state;
  }

  state.lifecycle = Lifecycle::RUNNING;
  state.ready = true;
  const bool any_component_degraded = std::any_of(
    summaries.begin(), summaries.end(), [](const ComponentSummary & item) {
      return item.enabled && item.degraded;
    });
  const bool optional_fault = std::any_of(
    summaries.begin(), summaries.end(), [](const ComponentSummary & item) {
      return item.enabled && !item.required && !item.ready;
    });
  const bool safety_restricts = safety.observation == SafetyObservation::STOPPED ||
    safety.observation == SafetyObservation::SLOWDOWN;

  state.degraded = any_component_degraded || optional_fault || safety_restricts;
  state.health = state.degraded ? AggregateHealth::DEGRADED : AggregateHealth::OK;
  state.reason_code = safety.observation == SafetyObservation::STOPPED ? reason::kSafetyStop :
    safety.observation == SafetyObservation::SLOWDOWN ? reason::kSafetySlowdown :
    state.degraded ? reason::kSupervisorDegraded : reason::kSupervisorOperational;

  const bool base_ready = component_ready(summaries, "base");
  const bool control_ready = component_ready(summaries, "control");
  const bool base_motion_available = component_nominal(summaries, "base");
  const bool control_motion_available = component_nominal(summaries, "control");
  const bool perception_ready = component_ready(summaries, "perception");
  const bool localization_ready = component_ready(summaries, "localization");
  const bool power_ready = component_ready(summaries, "power");
  const bool lidar_ready = component_ready(summaries, "lidar");
  const bool motion_safety = safety.observation == SafetyObservation::CLEAR ||
    (safety.observation == SafetyObservation::SLOWDOWN && safety.slowdown_factor > 0.0);

  state.capabilities.core_health_ready = required_component_ready(summaries);
  state.capabilities.core_safety_ready = safety.ready;
  state.capabilities.core_motion_ready = base_ready && control_ready &&
    base_motion_available && control_motion_available && perception_ready &&
    localization_ready && power_ready && motion_safety;
  state.capabilities.can_manual_drive = state.capabilities.core_motion_ready &&
    (!manual_drive_requires_lidar || lidar_ready);
  state.capabilities.can_rotate = state.capabilities.core_motion_ready &&
    (!rotate_requires_lidar || lidar_ready);
  state.capabilities.can_start_geometric_mapping = state.capabilities.core_motion_ready &&
    lidar_ready && component_nominal(summaries, "lidar") &&
    (!geometric_mapping_requires_nominal_power || component_nominal(summaries, "power"));
  return state;
}

SupervisorState SupervisorPolicy::EvaluateSupervisor(
  const ComponentSummary & localization_summary,
  const rclcpp::Time & now,
  double startup_age_s) const
{
  SafetySummary compatibility_safety;
  compatibility_safety.observation = SafetyObservation::CLEAR;
  compatibility_safety.ready = true;
  compatibility_safety.stop_received = true;
  compatibility_safety.slowdown_received = true;
  compatibility_safety.stop_fresh = true;
  compatibility_safety.slowdown_fresh = true;
  compatibility_safety.reason_code = reason::kSafetyClear;
  return EvaluateSupervisor({localization_summary}, compatibility_safety, now, startup_age_s);
}

std::string SupervisorPolicy::CompactStateJson(
  const SupervisorState & state,
  const rclcpp::Time & now) const
{
  Json components = Json::object();
  for (const auto & component : state.component_summaries) {
    components[component.name] = {
      {"enabled", component.enabled}, {"required", component.required},
      {"received", component.received}, {"ready", component.ready},
      {"degraded", component.degraded}, {"state", ToString(component.state)},
      {"reason_code", component.reason_code}, {"detail", component.detail},
      {"last_message_age_s", component.last_message_age_s},
      {"timeout_s", component.timeout_s},
      {"malformed_message_count", component.malformed_message_count},
      {"recovery_count", component.recovery_count}
    };
  }

  Json output{
    {"schema_version", 2}, {"node", "savo_supervisor"},
    {"lifecycle", ToString(state.lifecycle)}, {"operating_mode", ToString(state.operating_mode)},
    {"health", ToString(state.health)}, {"safety", ToString(state.safety)},
    {"ready", state.ready}, {"degraded", state.degraded},
    {"reason_code", state.reason_code}, {"stamp_s", now.seconds()},
    {"safety_detail", {
        {"ready", state.safety_summary.ready}, {"reason_code", state.safety_summary.reason_code},
        {"stop_received", state.safety_summary.stop_received},
        {"slowdown_received", state.safety_summary.slowdown_received},
        {"stop_fresh", state.safety_summary.stop_fresh},
        {"slowdown_fresh", state.safety_summary.slowdown_fresh},
        {"stop_active", state.safety_summary.stop_active},
        {"slowdown_factor", state.safety_summary.slowdown_factor},
        {"last_message_age_s", state.safety_summary.last_message_age_s}
      }},
    {"capabilities", {
        {"core_health_ready", state.capabilities.core_health_ready},
        {"core_safety_ready", state.capabilities.core_safety_ready},
        {"core_motion_ready", state.capabilities.core_motion_ready},
        {"can_manual_drive", state.capabilities.can_manual_drive},
        {"can_rotate", state.capabilities.can_rotate},
        {"can_start_geometric_mapping", state.capabilities.can_start_geometric_mapping}
      }},
    {"components", components}
  };
  return output.dump();
}

std::string SupervisorPolicy::CompactHeartbeatJson(
  const SupervisorState & state,
  const rclcpp::Time & now) const
{
  return Json{
    {"schema_version", 2}, {"node", "savo_supervisor"}, {"alive", true},
    {"ready", state.ready}, {"lifecycle", ToString(state.lifecycle)},
    {"health", ToString(state.health)}, {"safety", ToString(state.safety)},
    {"core_motion_ready", state.capabilities.core_motion_ready},
    {"stamp_s", now.seconds()}
  }.dump();
}

}  // namespace savo_supervisor
