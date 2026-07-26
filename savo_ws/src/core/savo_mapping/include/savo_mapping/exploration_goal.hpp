#pragma once

#include "savo_mapping/frontier_detector.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace savo_mapping::exploration
{

enum class ExplorationPlanningStatus
{
  GoalSelected,
  NoFrontiers,
  NoReachableFrontiers,
  NoSelectableFrontier,
};

std::string to_string(ExplorationPlanningStatus status);

struct ExplorationGoal
{
  std::string goal_id;
  std::size_t frontier_index{0};
  frontier::GridCell frontier_cell;
  frontier::GridCell approach_cell;
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double score{0.0};
  double distance_m{0.0};
  double information_gain_m2{0.0};
};

std::string validate_exploration_goal(
  const ExplorationGoal & goal);

struct ExplorationPlan
{
  ExplorationPlanningStatus status{
    ExplorationPlanningStatus::NoFrontiers};
  std::string reason;
  std::size_t detected_frontier_count{0};
  std::size_t reachable_frontier_count{0};
  std::optional<ExplorationGoal> goal;
};

}  // namespace savo_mapping::exploration
