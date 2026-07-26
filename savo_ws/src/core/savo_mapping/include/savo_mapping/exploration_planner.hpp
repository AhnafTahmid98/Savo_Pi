#pragma once

#include "savo_mapping/exploration_goal.hpp"
#include "savo_mapping/frontier_detector.hpp"
#include "savo_mapping/frontier_selector.hpp"

#include <string>

namespace savo_mapping::exploration
{

struct ExplorationPlannerConfig
{
  frontier::FrontierDetectorConfig detector;
  frontier::FrontierSelectorConfig selector;
  std::string goal_id_prefix{"frontier"};
};

std::string validate_exploration_planner_config(
  const ExplorationPlannerConfig & config);

class ExplorationPlanner
{
public:
  explicit ExplorationPlanner(
    ExplorationPlannerConfig config = {});

  const ExplorationPlannerConfig & config() const noexcept;

  ExplorationPlan plan(
    const frontier::OccupancyGrid & grid,
    double robot_x_m,
    double robot_y_m) const;

private:
  ExplorationPlannerConfig config_;
  frontier::FrontierDetector detector_;
  frontier::FrontierSelector selector_;
};

}  // namespace savo_mapping::exploration
