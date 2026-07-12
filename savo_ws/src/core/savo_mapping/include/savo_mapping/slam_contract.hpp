#pragma once

#include <string>
#include <string_view>

namespace savo_mapping::slam
{

inline constexpr std::string_view NODE_NAME{
  "slam_toolbox"};

inline constexpr std::string_view ASYNC_EXECUTABLE{
  "async_slam_toolbox_node"};

inline constexpr std::string_view MODE_MAPPING{
  "mapping"};

inline constexpr std::string_view MAP_FRAME{
  "map"};

inline constexpr std::string_view ODOM_FRAME{
  "odom"};

inline constexpr std::string_view BASE_FRAME{
  "base_link"};

inline constexpr std::string_view LIDAR_FRAME{
  "laser_frame"};

inline constexpr std::string_view SCAN_TOPIC{
  "/scan"};

inline constexpr std::string_view MAP_TOPIC{
  "/map"};

inline constexpr std::string_view DYNAMIC_MAP_SERVICE{
  "/slam_toolbox/dynamic_map"};

inline constexpr std::string_view PAUSE_MEASUREMENTS_SERVICE{
  "/slam_toolbox/pause_new_measurements"};

inline constexpr std::string_view SAVE_MAP_SERVICE{
  "/slam_toolbox/save_map"};

inline constexpr std::string_view SERIALIZE_MAP_SERVICE{
  "/slam_toolbox/serialize_map"};

inline constexpr std::string_view RESET_SERVICE{
  "/slam_toolbox/reset"};

struct SlamOwnershipContract
{
  bool slam_toolbox_owns_map_to_odom{true};
  bool slam_toolbox_owns_map_publication{true};

  bool localization_owns_odom_to_base{true};
  bool robot_description_owns_base_to_lidar{true};

  bool supervisor_is_monitor_only{true};
  bool session_manager_owns_save_requests{true};

  bool mapping_publishes_velocity_commands{false};
  bool mapping_publishes_odom_to_base{false};
  bool navigation_handoff_enabled{false};
};

std::string validate_slam_ownership(
  const SlamOwnershipContract & contract);

}  // namespace savo_mapping::slam
