// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string_view>

namespace savo_nav::topics
{

// Map, TF, localization, and sensor inputs.
inline constexpr std::string_view kMap = "/map";
inline constexpr std::string_view kMapMetadata = "/map_metadata";
inline constexpr std::string_view kTf = "/tf";
inline constexpr std::string_view kTfStatic = "/tf_static";
inline constexpr std::string_view kFilteredOdometry =
  "/odometry/filtered";
inline constexpr std::string_view kLocalizationHealth =
  "/savo_localization/health";
inline constexpr std::string_view kLaserScan = "/scan";

// RealSense and three-dimensional obstacle inputs.
inline constexpr std::string_view kRealSenseStatus =
  "/savo_realsense/status";
inline constexpr std::string_view kRawRealSensePoints =
  "/camera/camera/depth/color/points";
inline constexpr std::string_view kFilteredObstaclePoints =
  "/savo_perception/obstacles/points";

inline constexpr std::string_view kGlobalCostmap =
  "/global_costmap/costmap";
inline constexpr std::string_view kLocalCostmap =
  "/local_costmap/costmap";

// Safety and control integration.
inline constexpr std::string_view kSafetyStop = "/safety/stop";
inline constexpr std::string_view kSafetySlowdownFactor =
  "/safety/slowdown_factor";
inline constexpr std::string_view kControlModeCommand =
  "/savo_control/mode_cmd";
inline constexpr std::string_view kControlModeState =
  "/savo_control/mode_state";

// The only velocity output owned by savo_nav.
inline constexpr std::string_view kNavigationVelocity =
  "/cmd_vel_nav";

// Navigation state and observer outputs.
inline constexpr std::string_view kState = "/savo_nav/state";
inline constexpr std::string_view kStatus = "/savo_nav/status";
inline constexpr std::string_view kReadiness =
  "/savo_nav/readiness";
inline constexpr std::string_view kReadinessReason =
  "/savo_nav/readiness_reason";
inline constexpr std::string_view kCurrentGoal =
  "/savo_nav/current_goal";
inline constexpr std::string_view kGoalSource =
  "/savo_nav/goal_source";
inline constexpr std::string_view kResult = "/savo_nav/result";
inline constexpr std::string_view kRecoveryState =
  "/savo_nav/recovery_state";
inline constexpr std::string_view kHeartbeat =
  "/savo_nav/heartbeat";
inline constexpr std::string_view kMarkers =
  "/savo_nav/markers";
inline constexpr std::string_view kMapContextStatus =
  "/savo_nav/map_context/status";
inline constexpr std::string_view kMapContextHeartbeat =
  "/savo_nav/map_context/heartbeat";

namespace forbidden
{

inline constexpr std::string_view kCommandVelocity =
  "/cmd_vel";
inline constexpr std::string_view kSafeCommandVelocity =
  "/cmd_vel_safe";
inline constexpr std::string_view kRecoveryCommandVelocity =
  "/cmd_vel_recovery";

}  // namespace forbidden


inline constexpr std::string_view
  kNavigationState =
  "/savo_nav/navigation/state";

inline constexpr std::string_view
  kNavigationStatus =
  "/savo_nav/navigation/status";

inline constexpr std::string_view
  kNavigationFeedback =
  "/savo_nav/navigation/feedback";

inline constexpr std::string_view
  kNavigationResult =
  "/savo_nav/navigation/result";


inline constexpr std::string_view
  kControlModeReason =
  "/savo_control/mode_reason";

inline constexpr std::string_view
  kControlStatus =
  "/savo_control/control_status";

inline constexpr std::string_view
  kRecoveryActive =
  "/savo_control/recovery_active";

inline constexpr std::string_view
  kControlRecoveryState =
  "/savo_control/recovery_state";

inline constexpr std::string_view
  kRecoveryStatus =
  "/savo_control/recovery_status";

inline constexpr std::string_view
  kControlRecoveryAllowed =
  "/savo_nav/control_recovery/allowed";

inline constexpr std::string_view
  kControlRecoveryReason =
  "/savo_nav/control_recovery/reason";

inline constexpr std::string_view
  kControlRecoveryStatus =
  "/savo_nav/control_recovery/status";


inline constexpr std::string_view
  kGoalAdmissionState =
  "/savo_nav/goal_admission/state";

inline constexpr std::string_view
  kGoalAdmissionReason =
  "/savo_nav/goal_admission/reason";

inline constexpr std::string_view
  kGoalAdmissionStatus =
  "/savo_nav/goal_admission/status";

}  // namespace savo_nav::topics
