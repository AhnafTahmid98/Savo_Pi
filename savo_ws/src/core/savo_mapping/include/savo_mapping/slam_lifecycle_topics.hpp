#pragma once

#include <string_view>

namespace savo_mapping::slam_lifecycle_topics
{

inline constexpr std::string_view LIFECYCLE_STATE{
  "/savo_mapping/slam_lifecycle_state"};

inline constexpr std::string_view HEALTH{
  "/savo_mapping/slam_health"};

inline constexpr std::string_view MANUAL_WORKFLOW_STATE{
  "/savo_mapping/manual_workflow_state"};

inline constexpr std::string_view READINESS{
  "/savo_mapping/readiness"};

inline constexpr std::string_view SESSION_STATE{
  "/savo_mapping/map_session/state"};

inline constexpr std::string_view GET_STATE_SERVICE{
  "/slam_toolbox/get_state"};

}  // namespace savo_mapping::slam_lifecycle_topics
