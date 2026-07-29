// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>

namespace savo_supervisor
{
namespace reason
{

inline constexpr char kComponentDisabled[] =
  "component_disabled";

inline constexpr char kSupervisorStarting[] =
  "supervisor_starting";

inline constexpr char kSupervisorOperational[] =
  "supervisor_operational";

inline constexpr char kRequiredComponentUnavailable[] =
  "required_component_unavailable";

inline constexpr char kLocalizationInitializing[] =
  "localization_initializing";

inline constexpr char kLocalizationOperational[] =
  "localization_operational";

inline constexpr char kLocalizationDegraded[] =
  "localization_degraded";

inline constexpr char kLocalizationHealthMissing[] =
  "localization_health_missing";

inline constexpr char kLocalizationHealthStale[] =
  "localization_health_stale";

inline constexpr char kLocalizationSummaryMissing[] =
  "localization_summary_missing";

inline constexpr char kLocalizationSummaryStale[] =
  "localization_summary_stale";

inline constexpr char kLocalizationHeartbeatMissing[] =
  "localization_heartbeat_missing";

inline constexpr char kLocalizationHeartbeatStale[] =
  "localization_heartbeat_stale";

inline constexpr char kLocalizationHeartbeatNotAlive[] =
  "localization_heartbeat_not_alive";

inline constexpr char kLocalizationNotReady[] =
  "localization_not_ready";

inline constexpr char kLocalizationError[] =
  "localization_error";

inline constexpr char kLocalizationMessageInvalid[] =
  "localization_message_invalid";

inline constexpr char kLocalizationSchemaUnsupported[] =
  "localization_schema_unsupported";

inline constexpr char kLocalizationStateInconsistent[] =
  "localization_state_inconsistent";

inline constexpr char kRosTimeRegressionDetected[] =
  "ros_time_regression_detected";

}  // namespace reason

inline std::string ToString(const std::string & code)
{
  return code;
}

}  // namespace savo_supervisor
