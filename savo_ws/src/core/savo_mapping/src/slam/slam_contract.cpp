#include "savo_mapping/slam_contract.hpp"

#include <string>

namespace savo_mapping::slam
{

std::string validate_slam_ownership(
  const SlamOwnershipContract & contract)
{
  if (!contract.slam_toolbox_owns_map_to_odom) {
    return "slam_toolbox_must_own_map_to_odom";
  }

  if (!contract.slam_toolbox_owns_map_publication) {
    return "slam_toolbox_must_own_map_publication";
  }

  if (!contract.localization_owns_odom_to_base) {
    return "localization_must_own_odom_to_base";
  }

  if (!contract.robot_description_owns_base_to_lidar) {
    return "robot_description_must_own_base_to_lidar";
  }

  if (!contract.supervisor_is_monitor_only) {
    return "mapping_supervisor_must_remain_monitor_only";
  }

  if (!contract.session_manager_owns_save_requests) {
    return "map_session_manager_must_own_save_requests";
  }

  if (contract.mapping_publishes_velocity_commands) {
    return "savo_mapping_must_not_publish_velocity_commands";
  }

  if (contract.mapping_publishes_odom_to_base) {
    return "savo_mapping_must_not_publish_odom_to_base";
  }

  if (contract.navigation_handoff_enabled) {
    return "navigation_handoff_must_remain_disabled";
  }

  return {};
}

}  // namespace savo_mapping::slam
