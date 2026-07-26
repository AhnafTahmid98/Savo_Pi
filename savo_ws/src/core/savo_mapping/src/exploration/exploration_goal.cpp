#include "savo_mapping/exploration_goal.hpp"

#include <cmath>
#include <string>

namespace savo_mapping::exploration
{

std::string to_string(ExplorationPlanningStatus status)
{
  switch (status) {
    case ExplorationPlanningStatus::GoalSelected:
      return "goal_selected";

    case ExplorationPlanningStatus::NoFrontiers:
      return "no_frontiers";

    case ExplorationPlanningStatus::NoReachableFrontiers:
      return "no_reachable_frontiers";

    case ExplorationPlanningStatus::NoSelectableFrontier:
      return "no_selectable_frontier";
  }

  return "unknown";
}

std::string validate_exploration_goal(
  const ExplorationGoal & goal)
{
  if (goal.goal_id.empty()) {
    return "goal_id_must_not_be_empty";
  }

  if (!std::isfinite(goal.x_m) ||
    !std::isfinite(goal.y_m) ||
    !std::isfinite(goal.yaw_rad))
  {
    return "goal_pose_must_be_finite";
  }

  if (!std::isfinite(goal.score)) {
    return "goal_score_must_be_finite";
  }

  if (!std::isfinite(goal.distance_m) ||
    goal.distance_m < 0.0)
  {
    return "goal_distance_must_be_nonnegative";
  }

  if (!std::isfinite(goal.information_gain_m2) ||
    goal.information_gain_m2 < 0.0)
  {
    return "goal_information_gain_must_be_nonnegative";
  }

  return {};
}

}  // namespace savo_mapping::exploration
