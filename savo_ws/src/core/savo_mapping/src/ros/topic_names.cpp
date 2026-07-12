#include "savo_mapping/topic_names.hpp"

namespace savo_mapping::topics
{

std::string join_topic(std::string_view prefix, std::string_view suffix)
{
  if (prefix.empty()) {
    return std::string{suffix};
  }

  if (suffix.empty()) {
    return std::string{prefix};
  }

  const bool prefix_has_slash = prefix.back() == '/';
  const bool suffix_has_slash = suffix.front() == '/';

  if (prefix_has_slash && suffix_has_slash) {
    return std::string{prefix} + std::string{suffix.substr(1)};
  }

  if (!prefix_has_slash && !suffix_has_slash) {
    return std::string{prefix} + "/" + std::string{suffix};
  }

  return std::string{prefix} + std::string{suffix};
}

bool is_mapping_status_topic(std::string_view topic)
{
  return topic == STATUS ||
         topic == READINESS ||
         topic == MODE ||
         topic == WORKFLOW_PHASE ||
         topic == SESSION_STATE ||
         topic == MAP_QUALITY ||
         topic == MAP_SAVED ||
         topic == EXPLORATION_STATUS ||
         topic == SEMANTIC_EVENTS ||
         topic == DASHBOARD;
}

bool is_mapping_command_topic(std::string_view topic)
{
  return topic == MODE_CMD ||
         topic == START_SESSION_CMD ||
         topic == STOP_SESSION_CMD ||
         topic == SAVE_MAP_CMD ||
         topic == CANCEL_SESSION_CMD ||
         topic == SCAN360_CMD;
}

bool is_external_input_topic(std::string_view topic)
{
  return topic == SCAN ||
         topic == MAP ||
         topic == MAP_METADATA ||
         topic == TF ||
         topic == TF_STATIC ||
         topic == ODOM ||
         topic == ODOM_FILTERED ||
         topic == WHEEL_ODOM ||
         topic == REALSENSE_STATUS ||
         topic == DEPTH_MIN_FRONT ||
         topic == DEPTH_POINTS ||
         topic == HEAD_SEMANTIC_CONFIRMATIONS;
}

bool is_navigation_handoff_topic(std::string_view topic)
{
  return topic == EXPLORATION_SELECTED_GOAL ||
         topic == EXPLORATION_GOAL_STATE ||
         topic == EXPLORATION_GOAL_STATUS ||
         topic == EXPLORATION_GOAL_FEEDBACK ||
         topic == NAV_STATUS;
}

bool is_safety_awareness_topic(std::string_view topic)
{
  return topic == SAFETY_STOP ||
         topic == SAFETY_SLOWDOWN_FACTOR ||
         topic == CONTROL_MODE_STATE;
}

}  // namespace savo_mapping::topics
