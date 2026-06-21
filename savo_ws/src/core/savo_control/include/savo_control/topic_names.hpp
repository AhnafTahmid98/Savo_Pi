#pragma once

#include <string>

namespace savo_control::topics
{

// Command chain
inline constexpr const char * CMD_VEL_MANUAL = "/cmd_vel_manual";
inline constexpr const char * CMD_VEL_AUTO = "/cmd_vel_auto";
inline constexpr const char * CMD_VEL_NAV = "/cmd_vel_nav";
inline constexpr const char * CMD_VEL_RECOVERY = "/cmd_vel_recovery";
inline constexpr const char * CMD_VEL_MUX = "/cmd_vel_mux";
inline constexpr const char * CMD_VEL = "/cmd_vel";
inline constexpr const char * CMD_VEL_SAFE = "/cmd_vel_safe";

// Control mode
inline constexpr const char * CONTROL_MODE_CMD = "/savo_control/mode_cmd";
inline constexpr const char * CONTROL_MODE_STATE = "/savo_control/mode_state";
inline constexpr const char * CONTROL_MODE_REASON = "/savo_control/mode_reason";
inline constexpr const char * CONTROL_STATUS = "/savo_control/control_status";

// Safety
inline constexpr const char * SAFETY_STOP = "/safety/stop";
inline constexpr const char * SAFETY_SLOWDOWN_FACTOR = "/safety/slowdown_factor";

// Recovery
inline constexpr const char * RECOVERY_REQUEST = "/savo_control/recovery_request";
inline constexpr const char * RECOVERY_ACTIVE = "/savo_control/recovery_active";
inline constexpr const char * RECOVERY_STATUS = "/savo_control/recovery_status";
inline constexpr const char * STUCK_DETECTED = "/savo_control/stuck_detected";
inline constexpr const char * STUCK_STATE = "/savo_control/stuck_state";

// Auto / test
inline constexpr const char * AUTO_TEST_ENABLE = "/savo_control/auto_test_enable";
inline constexpr const char * AUTO_TEST_STATE = "/savo_control/auto_test_state";
inline constexpr const char * AUTO_TEST_CMD = "/savo_control/auto_test_cmd";

inline constexpr const char * RECOVERY_TEST_ENABLE = "/savo_control/recovery_test_enable";
inline constexpr const char * RECOVERY_TEST_STATUS = "/savo_control/recovery_test_status";
inline constexpr const char * RECOVERY_TEST_CMD = "/savo_control/recovery_test_cmd";

inline constexpr const char * STRAIGHT_LINE_ENABLE = "/savo_control/straight_line_enable";
inline constexpr const char * STRAIGHT_LINE_TARGET = "/savo_control/straight_line_target_m";
inline constexpr const char * STRAIGHT_LINE_STATE = "/savo_control/straight_line_state";

inline constexpr const char * DISTANCE_TEST_ENABLE = "/savo_control/distance_approach_enable";
inline constexpr const char * DISTANCE_TEST_TARGET = "/savo_control/distance_test_target_m";
inline constexpr const char * DISTANCE_TEST_STATE = "/savo_control/distance_test_state";

// Perception / odom inputs used by control
inline constexpr const char * ODOM_FILTERED = "/odometry/filtered";
inline constexpr const char * WHEEL_ODOM = "/wheel/odom";
inline constexpr const char * DEPTH_MIN_FRONT = "/depth/min_front_m";
inline constexpr const char * FRONT_ULTRASONIC = "/savo_perception/range/front_ultrasonic_m";
inline constexpr const char * RANGE_LEFT = "/savo_perception/range/left_m";
inline constexpr const char * RANGE_RIGHT = "/savo_perception/range/right_m";

// Dashboard / diagnostics
inline constexpr const char * CONTROL_DASHBOARD = "/savo_control/dashboard";
inline constexpr const char * CONTROL_DASHBOARD_TEXT = "/savo_control/dashboard_text";
inline constexpr const char * RECOVERY_MONITOR_STATUS = "/savo_control/recovery_monitor_status";
inline constexpr const char * BACKUP_ESCAPE_STATUS = "/savo_control/backup_escape_status";

inline std::string join_topic(const std::string & prefix, const std::string & suffix)
{
  if (prefix.empty()) {
    return suffix;
  }

  if (suffix.empty()) {
    return prefix;
  }

  if (prefix.back() == '/' && suffix.front() == '/') {
    return prefix + suffix.substr(1);
  }

  if (prefix.back() != '/' && suffix.front() != '/') {
    return prefix + "/" + suffix;
  }

  return prefix + suffix;
}

inline bool is_command_topic(const std::string & topic)
{
  return topic == CMD_VEL_MANUAL ||
         topic == CMD_VEL_AUTO ||
         topic == CMD_VEL_NAV ||
         topic == CMD_VEL_RECOVERY ||
         topic == CMD_VEL_MUX ||
         topic == CMD_VEL ||
         topic == CMD_VEL_SAFE;
}

inline bool is_status_topic(const std::string & topic)
{
  return topic == CONTROL_STATUS ||
         topic == CONTROL_MODE_STATE ||
         topic == CONTROL_MODE_REASON ||
         topic == RECOVERY_STATUS ||
         topic == RECOVERY_MONITOR_STATUS ||
         topic == CONTROL_DASHBOARD ||
         topic == CONTROL_DASHBOARD_TEXT ||
         topic == AUTO_TEST_STATE ||
         topic == RECOVERY_TEST_STATUS ||
         topic == STRAIGHT_LINE_STATE ||
         topic == DISTANCE_TEST_STATE;
}

inline bool is_sensor_topic(const std::string & topic)
{
  return topic == ODOM_FILTERED ||
         topic == WHEEL_ODOM ||
         topic == DEPTH_MIN_FRONT ||
         topic == FRONT_ULTRASONIC ||
         topic == RANGE_LEFT ||
         topic == RANGE_RIGHT;
}

}  // namespace savo_control::topics
