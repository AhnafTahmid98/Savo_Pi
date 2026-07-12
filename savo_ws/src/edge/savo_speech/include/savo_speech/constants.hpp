#pragma once

#include <array>
#include <string_view>

namespace savo_speech::constants
{

inline constexpr std::string_view PACKAGE_NAME = "savo_speech";
inline constexpr std::string_view ROBOT_NAME = "Robot Savo";
inline constexpr std::string_view RUNTIME_HOST = "savo-edge";
inline constexpr std::string_view PACKAGE_NAMESPACE = "/savo_speech";

inline constexpr std::string_view NODE_RESPEAKER_AUDIO = "respeaker_audio_node";
inline constexpr std::string_view NODE_SPEECH_MANAGER = "speech_manager_node";
inline constexpr std::string_view NODE_SAVOMIND_BRIDGE = "savomind_bridge_node";
inline constexpr std::string_view NODE_SPEECH_DASHBOARD = "speech_dashboard_node";

inline constexpr std::array<std::string_view, 6> FORBIDDEN_MOTION_TOPICS = {
  "/cmd_vel",
  "/cmd_vel_manual",
  "/cmd_vel_auto",
  "/cmd_vel_nav",
  "/cmd_vel_recovery",
  "/cmd_vel_safe",
};

inline constexpr std::string_view STATUS_OK = "OK";
inline constexpr std::string_view STATUS_WARN = "WARN";
inline constexpr std::string_view STATUS_ERROR = "ERROR";
inline constexpr std::string_view STATUS_STALE = "STALE";
inline constexpr std::string_view STATUS_DISABLED = "DISABLED";
inline constexpr std::string_view STATUS_UNKNOWN = "UNKNOWN";

}  // namespace savo_speech::constants
