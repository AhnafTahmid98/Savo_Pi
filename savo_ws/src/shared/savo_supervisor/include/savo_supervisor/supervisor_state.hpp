#pragma once

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
  STOP,
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
  bool enabled = false;
  bool required = false;
  bool received = false;
  bool ready = false;
  bool degraded = false;
  bool health_valid = false;
  bool summary_valid = false;
  bool heartbeat_valid = false;
  ComponentState state = ComponentState::UNKNOWN;
  std::string reason_code;
  std::string detail;
  double last_message_age_s = 0.0;
  double timeout_s = 0.0;
  std::size_t malformed_message_count = 0;
  std::size_t recovery_count = 0;
};

struct SupervisorState
{
  Lifecycle lifecycle = Lifecycle::STARTING;
  OperatingMode operating_mode = OperatingMode::STOP;
  AggregateHealth health = AggregateHealth::UNKNOWN;
  SafetyObservation safety = SafetyObservation::UNKNOWN;
  bool ready = false;
  bool degraded = false;
  std::string reason_code;
  std::vector<ComponentSummary> component_summaries;
};

inline const char * ToString(Lifecycle value)
{
  switch (value) {
    case Lifecycle::STARTING:
      return "STARTING";
    case Lifecycle::RUNNING:
      return "RUNNING";
    case Lifecycle::STOPPING:
      return "STOPPING";
    case Lifecycle::FAULTED:
      return "FAULTED";
  }
  return "UNKNOWN";
}

inline const char * ToString(OperatingMode value)
{
  switch (value) {
    case OperatingMode::STOP:
      return "STOP";
    case OperatingMode::UNKNOWN:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

inline const char * ToString(AggregateHealth value)
{
  switch (value) {
    case AggregateHealth::UNKNOWN:
      return "UNKNOWN";
    case AggregateHealth::OK:
      return "OK";
    case AggregateHealth::DEGRADED:
      return "DEGRADED";
    case AggregateHealth::ERROR:
      return "ERROR";
  }
  return "UNKNOWN";
}

inline const char * ToString(SafetyObservation value)
{
  switch (value) {
    case SafetyObservation::UNKNOWN:
      return "UNKNOWN";
    case SafetyObservation::CLEAR:
      return "CLEAR";
    case SafetyObservation::SLOWDOWN:
      return "SLOWDOWN";
    case SafetyObservation::STOPPED:
      return "STOPPED";
  }
  return "UNKNOWN";
}

inline const char * ToString(ComponentState value)
{
  switch (value) {
    case ComponentState::DISABLED:
      return "DISABLED";
    case ComponentState::UNKNOWN:
      return "UNKNOWN";
    case ComponentState::INITIALIZING:
      return "INITIALIZING";
    case ComponentState::OK:
      return "OK";
    case ComponentState::DEGRADED:
      return "DEGRADED";
    case ComponentState::STALE:
      return "STALE";
    case ComponentState::ERROR:
      return "ERROR";
    case ComponentState::INVALID:
      return "INVALID";
  }
  return "UNKNOWN";
}

} // namespace savo_supervisor
