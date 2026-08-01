// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_OBSERVER__TOPIC_CONTRACT_HPP_
#define SAVO_OBSERVER__TOPIC_CONTRACT_HPP_

#include <array>
#include <string_view>

namespace savo_observer::topics
{

inline constexpr std::string_view kState = "/savo_observer/state";
inline constexpr std::string_view kConnected = "/savo_observer/connected";
inline constexpr std::string_view kHeartbeat = "/savo_observer/heartbeat";
inline constexpr std::string_view kDiagnostics = "/savo_observer/diagnostics";
inline constexpr std::string_view kTelemetry = "/savo_observer/telemetry";
inline constexpr std::string_view kAlerts = "/savo_observer/alerts";

inline constexpr std::array<std::string_view, 12> kProhibitedInterfaces{{
  "/cmd_vel", "/cmd_vel_manual", "/cmd_vel_nav", "/goal_pose", "/initialpose",
  "/savo_control/mode_cmd", "/savo_nav/navigation/navigate_to_pose",
  "/savo_mapping/autonomous/run", "/savo_mapping/map_release/promote",
  "/savo_supervisor/authorize_location_operation",
  "/savo_locations/candidates/approve", "/safety/reset"}};

}  // namespace savo_observer::topics

#endif  // SAVO_OBSERVER__TOPIC_CONTRACT_HPP_
