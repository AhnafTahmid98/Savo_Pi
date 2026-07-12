#pragma once

#include <string>
#include <string_view>

namespace savo_mapping::topics
{

// Core SLAM / mapping inputs
inline constexpr std::string_view SCAN = "/scan";
inline constexpr std::string_view MAP = "/map";
inline constexpr std::string_view MAP_METADATA = "/map_metadata";
inline constexpr std::string_view TF = "/tf";
inline constexpr std::string_view TF_STATIC = "/tf_static";
inline constexpr std::string_view ODOM = "/odom";
inline constexpr std::string_view ODOM_FILTERED = "/odometry/filtered";
inline constexpr std::string_view WHEEL_ODOM = "/wheel/odom";

// Edge RealSense mapping data
inline constexpr std::string_view REALSENSE_STATUS = "/savo_realsense/status";
inline constexpr std::string_view DEPTH_MIN_FRONT = "/depth/min_front_m";
inline constexpr std::string_view DEPTH_POINTS = "/depth/color/points";

// Semantic input from savo_head
inline constexpr std::string_view HEAD_SEMANTIC_CONFIRMATIONS =
  "/savo_head/semantic_confirmations";

// Mapping status outputs
inline constexpr std::string_view STATUS = "/savo_mapping/status";
inline constexpr std::string_view READINESS = "/savo_mapping/readiness";
inline constexpr std::string_view MODE = "/savo_mapping/mode";
inline constexpr std::string_view WORKFLOW_PHASE = "/savo_mapping/workflow_phase";
inline constexpr std::string_view SESSION_STATE = "/savo_mapping/session_state";
inline constexpr std::string_view MAP_QUALITY = "/savo_mapping/map_quality";
inline constexpr std::string_view MAP_SAVED = "/savo_mapping/map_saved";
inline constexpr std::string_view EXPLORATION_STATUS = "/savo_mapping/exploration_status";
inline constexpr std::string_view SEMANTIC_EVENTS = "/savo_mapping/semantic_events";
inline constexpr std::string_view DASHBOARD = "/savo_mapping/dashboard";

// Mapping command topics
inline constexpr std::string_view MODE_CMD = "/savo_mapping/mode_cmd";
inline constexpr std::string_view START_SESSION_CMD = "/savo_mapping/start_session_cmd";
inline constexpr std::string_view STOP_SESSION_CMD = "/savo_mapping/stop_session_cmd";
inline constexpr std::string_view SAVE_MAP_CMD = "/savo_mapping/save_map_cmd";
inline constexpr std::string_view CANCEL_SESSION_CMD = "/savo_mapping/cancel_session_cmd";
inline constexpr std::string_view SCAN360_CMD = "/savo_mapping/scan360_cmd";

// Exploration handoff topics. The action and cancel service are declared in
// exploration_goal_handoff.hpp because they are not ROS topics.
inline constexpr std::string_view EXPLORATION_SELECTED_GOAL =
  "/savo_mapping/exploration/selected_goal";
inline constexpr std::string_view EXPLORATION_GOAL_STATE =
  "/savo_mapping/exploration_goal/state";
inline constexpr std::string_view EXPLORATION_GOAL_STATUS =
  "/savo_mapping/exploration_goal/status";
inline constexpr std::string_view EXPLORATION_GOAL_FEEDBACK =
  "/savo_mapping/exploration_goal/feedback";

// Read-only awareness of savo_nav state
inline constexpr std::string_view NAV_STATUS = "/savo_nav/status";

// Safety/control awareness
inline constexpr std::string_view SAFETY_STOP = "/safety/stop";
inline constexpr std::string_view SAFETY_SLOWDOWN_FACTOR = "/safety/slowdown_factor";
inline constexpr std::string_view CONTROL_MODE_STATE = "/savo_control/mode_state";

std::string join_topic(std::string_view prefix, std::string_view suffix);

bool is_mapping_status_topic(std::string_view topic);
bool is_mapping_command_topic(std::string_view topic);
bool is_external_input_topic(std::string_view topic);
bool is_navigation_handoff_topic(std::string_view topic);
bool is_safety_awareness_topic(std::string_view topic);

}  // namespace savo_mapping::topics
