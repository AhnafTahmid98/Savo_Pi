// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/transition_tracker.hpp"

#include <algorithm>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

namespace savo_supervisor
{
namespace
{

using Json = nlohmann::json;

bool is_fault_state(ComponentState state)
{
  return
    state == ComponentState::STALE ||
    state == ComponentState::INVALID ||
    state == ComponentState::ERROR;
}

bool is_operational_state(ComponentState state)
{
  return
    state == ComponentState::OK ||
    state == ComponentState::DEGRADED;
}

const ComponentSummary * find_component(
  const SupervisorState & state,
  const std::string & name)
{
  const auto iterator = std::find_if(
    state.component_summaries.begin(),
    state.component_summaries.end(),
    [&name](const ComponentSummary & component) {
      return component.name == name;
    });

  if (iterator == state.component_summaries.end()) {
    return nullptr;
  }

  return &(*iterator);
}

struct ComponentChange
{
  bool changed = false;
  std::string name;

  ComponentState previous_state =
    ComponentState::UNKNOWN;

  ComponentState current_state =
    ComponentState::UNKNOWN;

  std::size_t previous_malformed_count = 0;
  std::size_t current_malformed_count = 0;

  std::size_t previous_recovery_count = 0;
  std::size_t current_recovery_count = 0;
};

ComponentChange find_component_change(
  const SupervisorState & previous,
  const SupervisorState & current)
{
  for (const auto & current_component :
    current.component_summaries)
  {
    const auto * previous_component =
      find_component(previous, current_component.name);

    if (previous_component == nullptr) {
      ComponentChange change;
      change.changed = true;
      change.name = current_component.name;
      change.current_state = current_component.state;
      change.current_malformed_count =
        current_component.malformed_message_count;
      change.current_recovery_count =
        current_component.recovery_count;
      return change;
    }

    /*
     * A reason-code change alone is an aggregate reason transition,
     * not a second component-state transition. Component changes are
     * reserved for state/readiness/degradation and stream counters.
     */
    const bool changed =
      previous_component->state != current_component.state ||
      previous_component->ready != current_component.ready ||
      previous_component->degraded != current_component.degraded ||
      previous_component->malformed_message_count !=
      current_component.malformed_message_count ||
      previous_component->recovery_count !=
      current_component.recovery_count;

    if (changed) {
      ComponentChange change;
      change.changed = true;
      change.name = current_component.name;
      change.previous_state = previous_component->state;
      change.current_state = current_component.state;
      change.previous_malformed_count =
        previous_component->malformed_message_count;
      change.current_malformed_count =
        current_component.malformed_message_count;
      change.previous_recovery_count =
        previous_component->recovery_count;
      change.current_recovery_count =
        current_component.recovery_count;
      return change;
    }
  }

  return {};
}

SupervisorEventType classify_component_change(
  const ComponentChange & change)
{
  if (
    change.current_recovery_count >
    change.previous_recovery_count)
  {
    return SupervisorEventType::COMPONENT_RECOVERED;
  }

  if (
    change.current_malformed_count >
    change.previous_malformed_count)
  {
    return SupervisorEventType::COMPONENT_INVALID;
  }

  if (change.current_state == ComponentState::STALE) {
    return SupervisorEventType::COMPONENT_STALE;
  }

  if (change.current_state == ComponentState::INVALID) {
    return SupervisorEventType::COMPONENT_INVALID;
  }

  if (change.current_state == ComponentState::ERROR) {
    return SupervisorEventType::COMPONENT_ERROR;
  }

  if (
    is_fault_state(change.previous_state) &&
    is_operational_state(change.current_state))
  {
    return SupervisorEventType::COMPONENT_RECOVERED;
  }

  return SupervisorEventType::COMPONENT_STATE_CHANGED;
}

Json aggregate_state_json(const SupervisorState & state)
{
  return Json{
    {"lifecycle", ToString(state.lifecycle)},
    {"operating_mode", ToString(state.operating_mode)},
    {"health", ToString(state.health)},
    {"safety", ToString(state.safety)},
    {"ready", state.ready},
    {"degraded", state.degraded},
    {"reason_code", state.reason_code}
  };
}

}  // namespace

std::optional<SupervisorTransition>
TransitionTracker::Observe(
  const SupervisorState & current)
{
  SupervisorTransition transition;
  transition.current = current;

  if (!previous_.has_value()) {
    transition.type =
      SupervisorEventType::SUPERVISOR_STARTED;

    transition.has_previous = false;

    previous_ = current;
    return transition;
  }

  transition.has_previous = true;
  transition.previous = previous_.value();

  if (
    !previous_->ready &&
    current.ready &&
    previous_->lifecycle == Lifecycle::STARTING &&
    current.lifecycle == Lifecycle::RUNNING)
  {
    transition.type =
      SupervisorEventType::STARTUP_COMPLETED;

    previous_ = current;
    return transition;
  }

  const auto component_change =
    find_component_change(
      previous_.value(),
      current);

  if (component_change.changed) {
    transition.type =
      classify_component_change(component_change);

    transition.has_component = true;
    transition.component_name =
      component_change.name;

    transition.previous_component_state =
      component_change.previous_state;

    transition.current_component_state =
      component_change.current_state;

    previous_ = current;
    return transition;
  }

  if (previous_->ready != current.ready) {
    transition.type =
      SupervisorEventType::READINESS_CHANGED;

    previous_ = current;
    return transition;
  }

  if (previous_->health != current.health) {
    transition.type =
      SupervisorEventType::AGGREGATE_HEALTH_CHANGED;

    previous_ = current;
    return transition;
  }

  if (previous_->lifecycle != current.lifecycle) {
    transition.type =
      SupervisorEventType::LIFECYCLE_CHANGED;

    previous_ = current;
    return transition;
  }

  if (previous_->degraded != current.degraded) {
    transition.type =
      SupervisorEventType::DEGRADED_CHANGED;

    previous_ = current;
    return transition;
  }

  if (
    previous_->operating_mode !=
    current.operating_mode)
  {
    transition.type =
      SupervisorEventType::OPERATING_MODE_CHANGED;

    previous_ = current;
    return transition;
  }

  if (previous_->safety != current.safety) {
    transition.type =
      SupervisorEventType::SAFETY_CHANGED;

    previous_ = current;
    return transition;
  }

  if (
    previous_->reason_code !=
    current.reason_code)
  {
    transition.type =
      SupervisorEventType::REASON_CHANGED;

    previous_ = current;
    return transition;
  }

  previous_ = current;
  return std::nullopt;
}

void TransitionTracker::Reset()
{
  previous_.reset();
}

const char * ToString(SupervisorEventType value)
{
  switch (value) {
    case SupervisorEventType::SUPERVISOR_STARTED:
      return "supervisor_started";

    case SupervisorEventType::STARTUP_COMPLETED:
      return "startup_completed";

    case SupervisorEventType::COMPONENT_STALE:
      return "component_stale";

    case SupervisorEventType::COMPONENT_INVALID:
      return "component_invalid";

    case SupervisorEventType::COMPONENT_ERROR:
      return "component_error";

    case SupervisorEventType::COMPONENT_RECOVERED:
      return "component_recovered";

    case SupervisorEventType::COMPONENT_STATE_CHANGED:
      return "component_state_changed";

    case SupervisorEventType::READINESS_CHANGED:
      return "readiness_changed";

    case SupervisorEventType::AGGREGATE_HEALTH_CHANGED:
      return "aggregate_health_changed";

    case SupervisorEventType::LIFECYCLE_CHANGED:
      return "lifecycle_changed";

    case SupervisorEventType::DEGRADED_CHANGED:
      return "degraded_changed";

    case SupervisorEventType::OPERATING_MODE_CHANGED:
      return "operating_mode_changed";

    case SupervisorEventType::SAFETY_CHANGED:
      return "safety_changed";

    case SupervisorEventType::REASON_CHANGED:
      return "reason_changed";
  }

  return "unknown";
}

std::string CompactTransitionJson(
  const SupervisorTransition & transition,
  const rclcpp::Time & stamp)
{
  Json event{
    {"schema_version", 1},
    {"node", "savo_supervisor"},
    {"event_type", ToString(transition.type)},
    {"stamp_s", stamp.seconds()},
    {"current", aggregate_state_json(transition.current)}
  };

  if (transition.has_previous) {
    event["previous"] =
      aggregate_state_json(transition.previous);
  } else {
    event["previous"] = nullptr;
  }

  if (transition.has_component) {
    event["component"] = {
      {"name", transition.component_name},
      {
        "previous_state",
        ToString(transition.previous_component_state)
      },
      {
        "current_state",
        ToString(transition.current_component_state)
      }
    };
  }

  return event.dump();
}

}  // namespace savo_supervisor
