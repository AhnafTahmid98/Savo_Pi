#pragma once

// Centralized topic and frame name constants for savo_control.

namespace savo_control
{
namespace topic_names
{

// -----------------------------------------------------------------------------
// command sources
// -----------------------------------------------------------------------------
inline constexpr const char * kCmdVelManual = "/cmd_vel_manual";
inline constexpr const char * kCmdVelAuto   = "/cmd_vel_auto";
inline constexpr const char * kCmdVelNav    = "/cmd_vel_nav";      // Nav2 controller output (remapped)
inline constexpr const char * kCmdVelRecovery = "/cmd_vel_recovery";

inline constexpr const char * kCmdVelMuxSelected = "/cmd_vel_mux";   // output of twist_mux_node
inline constexpr const char * kCmdVelShaped      = "/cmd_vel";       // output of cmd_vel_shaper_node
inline constexpr const char * kCmdVelSafe        = "/cmd_vel_safe";  // output of safety gate (savo_perception)

inline constexpr const char * kCmdVelRaw = "/cmd_vel_raw";
inline constexpr const char * kCmdVelTestPattern = "/cmd_vel_test_pattern";

// -----------------------------------------------------------------------------
// localization / state feedback
// -----------------------------------------------------------------------------
// stable name regardless of encoder count
inline constexpr const char * kWheelOdom     = "/wheel/odom";
inline constexpr const char * kOdomFiltered  = "/odometry/filtered";

inline constexpr const char * kImuData       = "/imu/data";
inline constexpr const char * kImuDataRaw    = "/imu/data_raw";

// -----------------------------------------------------------------------------
// perception safety interfaces
// -----------------------------------------------------------------------------
inline constexpr const char * kSafetyStop           = "/safety/stop";
inline constexpr const char * kSafetySlowdownFactor = "/safety/slowdown_factor";

inline constexpr const char * kDepthMinFront        = "/depth/min_front_m";
inline constexpr const char * kRangeFrontUltrasonic = "/savo_perception/range/front_ultrasonic_m";
inline constexpr const char * kRangeLeft            = "/savo_perception/range/left_m";
inline constexpr const char * kRangeRight           = "/savo_perception/range/right_m";

// -----------------------------------------------------------------------------
// control package internal topics
// -----------------------------------------------------------------------------
inline constexpr const char * kControlModeCmd       = "/savo_control/mode_cmd";
inline constexpr const char * kControlModeState     = "/savo_control/mode_state";

inline constexpr const char * kHeadingTarget        = "/savo_control/heading_target";
inline constexpr const char * kHeadingHoldEnable    = "/savo_control/heading_hold_enable";
inline constexpr const char * kRotateTarget         = "/savo_control/rotate_target";
inline constexpr const char * kRotateState          = "/savo_control/rotate_state";

inline constexpr const char * kAutoTestEnable       = "/savo_control/auto_test_enable";
inline constexpr const char * kAutoTestState        = "/savo_control/auto_test_state";
inline constexpr const char * kDistanceTestTarget   = "/savo_control/distance_test_target";
inline constexpr const char * kDistanceTestState    = "/savo_control/distance_test_state";
inline constexpr const char * kStraightTestEnable   = "/savo_control/straight_test_enable";
inline constexpr const char * kStraightTestState    = "/savo_control/straight_test_state";

// recovery topics
inline constexpr const char * kStuckDetected        = "/savo_control/stuck_detected";
inline constexpr const char * kRecoveryRequest      = "/savo_control/recovery_request";
inline constexpr const char * kRecoveryActive       = "/savo_control/recovery_active";
// deprecated — use kRecoveryRequest
inline constexpr const char * kRecoveryTrigger      = "/savo_control/recovery_trigger";
inline constexpr const char * kRecoveryState        = "/savo_control/recovery_state";
inline constexpr const char * kRecoveryStatus       = "/savo_control/recovery_status";

inline constexpr const char * kControlStatus        = "/savo_control/control_status";
inline constexpr const char * kControlDebug         = "/savo_control/control_debug";

// -----------------------------------------------------------------------------
// frame IDs
// -----------------------------------------------------------------------------
inline constexpr const char * kFrameMap      = "map";
inline constexpr const char * kFrameOdom     = "odom";
inline constexpr const char * kFrameBaseLink = "base_link";

inline constexpr const char * kFrameBaseFootprint = "base_footprint";
inline constexpr const char * kFrameImuLink       = "imu_link";
inline constexpr const char * kFrameLaser         = "laser";
inline constexpr const char * kFrameCameraLink    = "camera_link";

inline constexpr const char * kNamespaceControl = "/savo_control";
inline constexpr const char * kNamespaceSafety  = "/safety";

}  // namespace topic_names
}  // namespace savo_control