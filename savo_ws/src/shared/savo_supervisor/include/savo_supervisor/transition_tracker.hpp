// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <optional>
#include <string>

#include "rclcpp/time.hpp"
#include "savo_supervisor/supervisor_state.hpp"

namespace savo_supervisor
{

enum class SupervisorEventType
{
  SUPERVISOR_STARTED,
  STARTUP_COMPLETED,
  COMPONENT_STALE,
  COMPONENT_INVALID,
  COMPONENT_ERROR,
  COMPONENT_RECOVERED,
  COMPONENT_STATE_CHANGED,
  READINESS_CHANGED,
  AGGREGATE_HEALTH_CHANGED,
  LIFECYCLE_CHANGED,
  DEGRADED_CHANGED,
  OPERATING_MODE_CHANGED,
  SAFETY_CHANGED,
  REASON_CHANGED
};

struct SupervisorTransition
{
  SupervisorEventType type =
    SupervisorEventType::SUPERVISOR_STARTED;

  bool has_previous = false;
  bool has_component = false;

  std::string component_name;

  ComponentState previous_component_state =
    ComponentState::UNKNOWN;

  ComponentState current_component_state =
    ComponentState::UNKNOWN;

  SupervisorState previous;
  SupervisorState current;
};

class TransitionTracker
{
public:
  std::optional<SupervisorTransition> Observe(
    const SupervisorState & current);

  void Reset();

private:
  std::optional<SupervisorState> previous_;
};

const char * ToString(SupervisorEventType value);

std::string CompactTransitionJson(
  const SupervisorTransition & transition,
  const rclcpp::Time & stamp);

}  // namespace savo_supervisor
