#pragma once

// =============================================================================
// Robot SAVO — savo_control / topic_names.hpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Centralized topic and frame name constants for the `savo_control` package.
//
// Why this file exists
// --------------------
// - Avoid hard-coded strings spread across many nodes
// - Prevent typos in topic/frame names
// - Keep interfaces stable while implementations evolve
//   (e.g., 2-encoder -> 4-encoder wheel odometry upgrade)
// - Make bringup, testing, and future refactors easier
//
// Design policy
// -------------
// 1) Store stable interface names here (topics/frames used across packages)
// 2) Do NOT encode temporary hardware assumptions in topic names
// 3) Keep names aligned with the locked Robot SAVO contracts where possible
//
// Notes for Robot SAVO (current architecture)
// -------------------------------------------
// - `savo_perception` produces safety topics such as `/safety/stop` and
//   `/safety/slowdown_factor` and the C++ safety gate outputs `/cmd_vel_safe`.
// - `savo_localization` provides `/wheel/odom` and EKF `/odometry/filtered`.
// - `savo_control` should consume/publish using these stable interfaces.
// =============================================================================

namespace savo_control
{
namespace topic_names
{

// -----------------------------------------------------------------------------
// Core velocity command pipeline
// -----------------------------------------------------------------------------
// Upstream command inputs (from manual teleop / auto tests / future nav)
inline constexpr const char * kCmdVelManual = "/cmd_vel_manual";
inline constexpr const char * kCmdVelAuto   = "/cmd_vel_auto";
inline constexpr const char * kCmdVelNav    = "/cmd_vel_nav";      // future Nav2 / nav source
inline constexpr const char * kCmdVelRecovery = "/cmd_vel_recovery";

// Mux / shaping pipeline
inline constexpr const char * kCmdVelMuxSelected = "/cmd_vel_mux";   // output of twist_mux_node
inline constexpr const char * kCmdVelShaped      = "/cmd_vel";       // output of cmd_vel_shaper_node
inline constexpr const char * kCmdVelSafe        = "/cmd_vel_safe";  // output of safety gate (savo_perception)

// Optional raw/reference/test command topics
inline constexpr const char * kCmdVelRaw = "/cmd_vel_raw";
inline constexpr const char * kCmdVelTestPattern = "/cmd_vel_test_pattern";

// -----------------------------------------------------------------------------
// Localization / state feedback (savo_localization)
// -----------------------------------------------------------------------------
// Keep these stable regardless of encoder count (2 now -> 4 later).
inline constexpr const char * kWheelOdom     = "/wheel/odom";
inline constexpr const char * kOdomFiltered  = "/odometry/filtered";

// Optional pose/IMU related topics (only use if/when needed by nodes)
inline constexpr const char * kImuData       = "/imu/data";
inline constexpr const char * kImuDataRaw    = "/imu/data_raw";

// -----------------------------------------------------------------------------
// Perception safety interfaces (savo_perception)
// -----------------------------------------------------------------------------
inline constexpr const char * kSafetyStop           = "/safety/stop";
inline constexpr const char * kSafetySlowdownFactor = "/safety/slowdown_factor";

// Range/depth topics (optional subscriptions for test/recovery logic)
inline constexpr const char * kDepthMinFront        = "/depth/min_front_m";
inline constexpr const char * kRangeFrontUltrasonic = "/savo_perception/range/front_ultrasonic_m";
inline constexpr const char * kRangeLeft            = "/savo_perception/range/left_m";
inline constexpr const char * kRangeRight           = "/savo_perception/range/right_m";

// -----------------------------------------------------------------------------
// Control package internal topics (status / mode / debug / tests)
// -----------------------------------------------------------------------------
// Mode management
inline constexpr const char * kControlModeCmd       = "/savo_control/mode_cmd";
inline constexpr const char * kControlModeState     = "/savo_control/mode_state";

// Controller command / state topics
inline constexpr const char * kHeadingTarget        = "/savo_control/heading_target";
inline constexpr const char * kHeadingHoldEnable    = "/savo_control/heading_hold_enable";
inline constexpr const char * kRotateTarget         = "/savo_control/rotate_target";
inline constexpr const char * kRotateState          = "/savo_control/rotate_state";

// Test manager / PID test topics
inline constexpr const char * kAutoTestEnable       = "/savo_control/auto_test_enable";
inline constexpr const char * kAutoTestState        = "/savo_control/auto_test_state";
inline constexpr const char * kDistanceTestTarget   = "/savo_control/distance_test_target";
inline constexpr const char * kDistanceTestState    = "/savo_control/distance_test_state";
inline constexpr const char * kStraightTestEnable   = "/savo_control/straight_test_enable";
inline constexpr const char * kStraightTestState    = "/savo_control/straight_test_state";

// Recovery topics
inline constexpr const char * kStuckDetected        = "/savo_control/stuck_detected";
inline constexpr const char * kRecoveryTrigger      = "/savo_control/recovery_trigger";
inline constexpr const char * kRecoveryState        = "/savo_control/recovery_state";
inline constexpr const char * kRecoveryStatus       = "/savo_control/recovery_status";

// General control status / debug
inline constexpr const char * kControlStatus        = "/savo_control/control_status";
inline constexpr const char * kControlDebug         = "/savo_control/control_debug";

// -----------------------------------------------------------------------------
// Frame IDs (keep stable; independent of encoder count)
// -----------------------------------------------------------------------------
// Core robot frames
inline constexpr const char * kFrameMap      = "map";
inline constexpr const char * kFrameOdom     = "odom";
inline constexpr const char * kFrameBaseLink = "base_link";

// Optional common frames (use only if present in your stack)
inline constexpr const char * kFrameBaseFootprint = "base_footprint";
inline constexpr const char * kFrameImuLink       = "imu_link";
inline constexpr const char * kFrameLaser         = "laser";
inline constexpr const char * kFrameCameraLink    = "camera_link";

// -----------------------------------------------------------------------------
// Utility helpers (small categorization checks if needed later)
// -----------------------------------------------------------------------------
inline constexpr const char * kNamespaceControl = "/savo_control";
inline constexpr const char * kNamespaceSafety  = "/safety";

}  // namespace topic_names
}  // namespace savo_control