#pragma once

#include <string>

namespace savo_supervisor
{
namespace reason
{
static constexpr char kSupervisorStarting[] = "supervisor_starting";
static constexpr char kSupervisorOperational[] = "supervisor_operational";
static constexpr char kRequiredComponentUnavailable[] = "required_component_unavailable";
static constexpr char kLocalizationInitializing[] = "localization_initializing";
static constexpr char kLocalizationOperational[] = "localization_operational";
static constexpr char kLocalizationDegraded[] = "localization_degraded";
static constexpr char kLocalizationHealthMissing[] = "localization_health_missing";
static constexpr char kLocalizationHealthStale[] = "localization_health_stale";
static constexpr char kLocalizationHeartbeatMissing[] = "localization_heartbeat_missing";
static constexpr char kLocalizationHeartbeatStale[] = "localization_heartbeat_stale";
static constexpr char kLocalizationNotReady[] = "localization_not_ready";
static constexpr char kLocalizationError[] = "localization_error";
static constexpr char kLocalizationMessageInvalid[] = "localization_message_invalid";
static constexpr char kLocalizationSchemaUnsupported[] = "localization_schema_unsupported";
static constexpr char kLocalizationStateInconsistent[] = "localization_state_inconsistent";
static constexpr char kRosTimeRegressionDetected[] = "ros_time_regression_detected";
}

inline std::string ToString(const std::string & code)
{
  return code;
}

} // namespace savo_supervisor
