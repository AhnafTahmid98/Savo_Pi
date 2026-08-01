// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace savo_supervisor
{

enum class Lifecycle
{
  STARTING,
  RUNNING,
  STOPPING,
  FAULTED,
};

enum class OperatingMode
{
  BOOTING,
  IDLE,
  STOP,
  MANUAL,
  MAPPING,
  NAVIGATE,
  RECOVERY,
  ESTOP,
  ERROR,
  SHUTTING_DOWN,
  UNKNOWN,
};

enum class AggregateHealth
{
  UNKNOWN,
  OK,
  DEGRADED,
  ERROR,
};

enum class SafetyObservation
{
  UNKNOWN,
  CLEAR,
  SLOWDOWN,
  STOPPED,
};

enum class ComponentState
{
  DISABLED,
  UNKNOWN,
  INITIALIZING,
  OK,
  DEGRADED,
  STALE,
  ERROR,
  INVALID,
};

struct ComponentSummary
{
  std::string name;
  bool enabled{false};
  bool required{false};
  bool received{false};
  bool ready{false};
  bool degraded{false};
  bool health_valid{false};
  bool summary_valid{false};
  bool heartbeat_valid{false};
  ComponentState state{ComponentState::UNKNOWN};
  std::string reason_code;
  std::string detail;
  double last_message_age_s{-1.0};
  double timeout_s{0.0};
  std::size_t malformed_message_count{0};
  std::size_t recovery_count{0};
};

struct SafetySummary
{
  SafetyObservation observation{SafetyObservation::UNKNOWN};
  bool ready{false};
  bool stop_received{false};
  bool slowdown_received{false};
  bool stop_fresh{false};
  bool slowdown_fresh{false};
  bool stop_active{false};
  double slowdown_factor{1.0};
  std::string reason_code{"safety_state_unknown"};
  double last_message_age_s{-1.0};
};

struct CoreCapabilities
{
  bool core_health_ready{false};
  bool core_safety_ready{false};
  bool core_motion_ready{false};
  bool can_manual_drive{false};
  bool can_rotate{false};
  bool can_start_geometric_mapping{false};
};

struct SupervisorState
{
  Lifecycle lifecycle{Lifecycle::STARTING};
  OperatingMode operating_mode{OperatingMode::STOP};
  AggregateHealth health{AggregateHealth::UNKNOWN};
  SafetyObservation safety{SafetyObservation::UNKNOWN};
  SafetySummary safety_summary{};
  CoreCapabilities capabilities{};
  bool ready{false};
  bool degraded{false};
  std::string reason_code;
  std::vector<ComponentSummary> component_summaries;
};

inline const char * ToString(Lifecycle value)
{
  switch (value) {
    case Lifecycle::STARTING: return "STARTING";
    case Lifecycle::RUNNING: return "RUNNING";
    case Lifecycle::STOPPING: return "STOPPING";
    case Lifecycle::FAULTED: return "FAULTED";
  }
  return "UNKNOWN";
}

inline const char * ToString(OperatingMode value)
{
  switch (value) {
    case OperatingMode::BOOTING: return "BOOTING";
    case OperatingMode::IDLE: return "IDLE";
    case OperatingMode::STOP: return "STOP";
    case OperatingMode::MANUAL: return "MANUAL";
    case OperatingMode::MAPPING: return "MAPPING";
    case OperatingMode::NAVIGATE: return "NAVIGATE";
    case OperatingMode::RECOVERY: return "RECOVERY";
    case OperatingMode::ESTOP: return "ESTOP";
    case OperatingMode::ERROR: return "ERROR";
    case OperatingMode::SHUTTING_DOWN: return "SHUTTING_DOWN";
    case OperatingMode::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

inline const char * ToString(AggregateHealth value)
{
  switch (value) {
    case AggregateHealth::UNKNOWN: return "UNKNOWN";
    case AggregateHealth::OK: return "OK";
    case AggregateHealth::DEGRADED: return "DEGRADED";
    case AggregateHealth::ERROR: return "ERROR";
  }
  return "UNKNOWN";
}

inline const char * ToString(SafetyObservation value)
{
  switch (value) {
    case SafetyObservation::UNKNOWN: return "UNKNOWN";
    case SafetyObservation::CLEAR: return "CLEAR";
    case SafetyObservation::SLOWDOWN: return "SLOWDOWN";
    case SafetyObservation::STOPPED: return "STOPPED";
  }
  return "UNKNOWN";
}

inline const char * ToString(ComponentState value)
{
  switch (value) {
    case ComponentState::DISABLED: return "DISABLED";
    case ComponentState::UNKNOWN: return "UNKNOWN";
    case ComponentState::INITIALIZING: return "INITIALIZING";
    case ComponentState::OK: return "OK";
    case ComponentState::DEGRADED: return "DEGRADED";
    case ComponentState::STALE: return "STALE";
    case ComponentState::ERROR: return "ERROR";
    case ComponentState::INVALID: return "INVALID";
  }
  return "UNKNOWN";
}

}  // namespace savo_supervisor
