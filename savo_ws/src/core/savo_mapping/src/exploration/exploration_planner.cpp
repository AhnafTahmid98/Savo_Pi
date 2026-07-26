#include "savo_mapping/exploration_planner.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace savo_mapping::exploration
{
namespace
{

using frontier::FrontierCluster;
using frontier::GridCell;
using frontier::OccupancyGrid;

std::size_t linear_index(
  const OccupancyGrid & grid,
  const GridCell & cell)
{
  return cell.y * grid.width + cell.x;
}

bool is_inside(
  const OccupancyGrid & grid,
  std::int64_t x,
  std::int64_t y)
{
  return x >= 0 &&
         y >= 0 &&
         static_cast<std::size_t>(x) < grid.width &&
         static_cast<std::size_t>(y) < grid.height;
}

bool is_free(
  const OccupancyGrid & grid,
  const GridCell & cell,
  std::int8_t free_threshold)
{
  const std::int8_t value =
    grid.cells.at(linear_index(grid, cell));

  return value >= 0 && value <= free_threshold;
}

GridCell world_to_cell(
  const OccupancyGrid & grid,
  double x_m,
  double y_m)
{
  if (!std::isfinite(x_m) ||
    !std::isfinite(y_m))
  {
    throw std::invalid_argument(
            "robot pose must be finite");
  }

  const double cell_x =
    (x_m - grid.origin_x_m) / grid.resolution_m;

  const double cell_y =
    (y_m - grid.origin_y_m) / grid.resolution_m;

  if (!std::isfinite(cell_x) ||
    !std::isfinite(cell_y) ||
    cell_x < 0.0 ||
    cell_y < 0.0 ||
    cell_x >= static_cast<double>(grid.width) ||
    cell_y >= static_cast<double>(grid.height))
  {
    throw std::invalid_argument(
            "robot pose is outside occupancy grid");
  }

  return GridCell{
    static_cast<std::size_t>(std::floor(cell_x)),
    static_cast<std::size_t>(std::floor(cell_y)),
  };
}

std::pair<double, double> cell_center(
  const OccupancyGrid & grid,
  const GridCell & cell)
{
  return {
    grid.origin_x_m +
    (static_cast<double>(cell.x) + 0.5) *
    grid.resolution_m,

    grid.origin_y_m +
    (static_cast<double>(cell.y) + 0.5) *
    grid.resolution_m,
  };
}

std::vector<bool> reachable_free_cells(
  const OccupancyGrid & grid,
  const GridCell & robot_cell,
  std::int8_t free_threshold)
{
  if (!is_free(
      grid,
      robot_cell,
      free_threshold))
  {
    throw std::invalid_argument(
            "robot pose must occupy a free grid cell");
  }

  constexpr std::array<std::pair<int, int>, 4>
  offsets{{
    {1, 0},
    {-1, 0},
    {0, 1},
    {0, -1},
  }};

  std::vector<bool> reachable(
    grid.cells.size(),
    false);

  std::queue<GridCell> pending;

  reachable.at(
    linear_index(grid, robot_cell)) = true;

  pending.push(robot_cell);

  while (!pending.empty()) {
    const GridCell current = pending.front();
    pending.pop();

    for (const auto & [offset_x, offset_y] :
      offsets)
    {
      const std::int64_t neighbor_x =
        static_cast<std::int64_t>(current.x) +
        offset_x;

      const std::int64_t neighbor_y =
        static_cast<std::int64_t>(current.y) +
        offset_y;

      if (!is_inside(
          grid,
          neighbor_x,
          neighbor_y))
      {
        continue;
      }

      const GridCell neighbor{
        static_cast<std::size_t>(neighbor_x),
        static_cast<std::size_t>(neighbor_y),
      };

      const std::size_t index =
        linear_index(grid, neighbor);

      if (reachable.at(index) ||
        !is_free(
            grid,
            neighbor,
            free_threshold))
      {
        continue;
      }

      reachable.at(index) = true;
      pending.push(neighbor);
    }
  }

  return reachable;
}

bool row_major_less(
  const GridCell & left,
  const GridCell & right)
{
  if (left.y != right.y) {
    return left.y < right.y;
  }

  return left.x < right.x;
}

std::vector<GridCell> reachable_approach_cells(
  const OccupancyGrid & grid,
  const FrontierCluster & frontier,
  const std::vector<bool> & reachable,
  std::int8_t free_threshold)
{
  constexpr std::array<std::pair<int, int>, 4>
  offsets{{
    {1, 0},
    {-1, 0},
    {0, 1},
    {0, -1},
  }};

  std::vector<GridCell> approaches;

  for (const GridCell & frontier_cell :
    frontier.cells)
  {
    for (const auto & [offset_x, offset_y] :
      offsets)
    {
      const std::int64_t neighbor_x =
        static_cast<std::int64_t>(
        frontier_cell.x) + offset_x;

      const std::int64_t neighbor_y =
        static_cast<std::int64_t>(
        frontier_cell.y) + offset_y;

      if (!is_inside(
          grid,
          neighbor_x,
          neighbor_y))
      {
        continue;
      }

      const GridCell neighbor{
        static_cast<std::size_t>(neighbor_x),
        static_cast<std::size_t>(neighbor_y),
      };

      const std::size_t index =
        linear_index(grid, neighbor);

      if (!reachable.at(index) ||
        !is_free(
            grid,
            neighbor,
            free_threshold))
      {
        continue;
      }

      approaches.push_back(neighbor);
    }
  }

  std::sort(
    approaches.begin(),
    approaches.end(),
    row_major_less);

  approaches.erase(
    std::unique(
      approaches.begin(),
      approaches.end()),
    approaches.end());

  return approaches;
}

GridCell choose_approach_cell(
  const OccupancyGrid & grid,
  const std::vector<GridCell> & approaches,
  double robot_x_m,
  double robot_y_m)
{
  GridCell best = approaches.front();

  double best_squared_distance =
    std::numeric_limits<double>::infinity();

  constexpr double epsilon = 1.0e-12;

  for (const GridCell & candidate :
    approaches)
  {
    const auto [candidate_x_m, candidate_y_m] =
      cell_center(grid, candidate);

    const double delta_x =
      candidate_x_m - robot_x_m;

    const double delta_y =
      candidate_y_m - robot_y_m;

    const double squared_distance =
      delta_x * delta_x +
      delta_y * delta_y;

    if (squared_distance + epsilon <
      best_squared_distance)
    {
      best = candidate;
      best_squared_distance =
        squared_distance;
      continue;
    }

    if (std::abs(
        squared_distance -
        best_squared_distance) <= epsilon &&
      row_major_less(candidate, best))
    {
      best = candidate;
    }
  }

  return best;
}

std::string build_goal_id(
  const std::string & prefix,
  const FrontierCluster & frontier,
  const GridCell & approach)
{
  std::ostringstream stream;

  stream
    << prefix
    << "-f"
    << frontier.representative.x
    << "-"
    << frontier.representative.y
    << "-a"
    << approach.x
    << "-"
    << approach.y;

  return stream.str();
}

bool valid_goal_id_character(char character)
{
  const unsigned char value =
    static_cast<unsigned char>(character);

  return std::isalnum(value) != 0 ||
         character == '-' ||
         character == '_';
}

}  // namespace

std::string validate_exploration_planner_config(
  const ExplorationPlannerConfig & config)
{
  const std::string detector_error =
    frontier::validate_frontier_detector_config(
    config.detector);

  if (!detector_error.empty()) {
    return "detector_" + detector_error;
  }

  const std::string selector_error =
    frontier::validate_frontier_selector_config(
    config.selector);

  if (!selector_error.empty()) {
    return "selector_" + selector_error;
  }

  if (config.goal_id_prefix.empty()) {
    return "goal_id_prefix_must_not_be_empty";
  }

  if (!std::all_of(
      config.goal_id_prefix.begin(),
      config.goal_id_prefix.end(),
      valid_goal_id_character))
  {
    return
      "goal_id_prefix_contains_invalid_character";
  }

  return {};
}

ExplorationPlanner::ExplorationPlanner(
  ExplorationPlannerConfig config)
: config_(std::move(config)),
  detector_(config_.detector),
  selector_(config_.selector)
{
  const std::string error =
    validate_exploration_planner_config(
    config_);

  if (!error.empty()) {
    throw std::invalid_argument(
            "invalid exploration planner "
            "configuration: " + error);
  }
}

const ExplorationPlannerConfig &
ExplorationPlanner::config() const noexcept
{
  return config_;
}

ExplorationPlan ExplorationPlanner::plan(
  const OccupancyGrid & grid,
  double robot_x_m,
  double robot_y_m) const
{
  const std::string grid_error =
    frontier::validate_occupancy_grid(grid);

  if (!grid_error.empty()) {
    throw std::invalid_argument(
            "invalid occupancy grid: " +
            grid_error);
  }

  const GridCell robot_cell =
    world_to_cell(
    grid,
    robot_x_m,
    robot_y_m);

  const std::vector<bool> reachable =
    reachable_free_cells(
    grid,
    robot_cell,
    config_.detector.free_threshold);

  const std::vector<FrontierCluster> detected =
    detector_.detect(grid);

  ExplorationPlan result;

  result.detected_frontier_count =
    detected.size();

  if (detected.empty()) {
    result.status =
      ExplorationPlanningStatus::NoFrontiers;

    result.reason =
      to_string(result.status);

    return result;
  }

  std::vector<FrontierCluster>
  selectable_frontiers;

  std::vector<std::size_t>
  original_indices;

  std::vector<std::vector<GridCell>>
  approaches_by_frontier;

  selectable_frontiers.reserve(
    detected.size());

  original_indices.reserve(
    detected.size());

  approaches_by_frontier.reserve(
    detected.size());

  for (std::size_t index = 0;
    index < detected.size();
    ++index)
  {
    std::vector<GridCell> approaches =
      reachable_approach_cells(
      grid,
      detected.at(index),
      reachable,
      config_.detector.free_threshold);

    if (approaches.empty()) {
      continue;
    }

    selectable_frontiers.push_back(
      detected.at(index));

    original_indices.push_back(index);

    approaches_by_frontier.push_back(
      std::move(approaches));
  }

  result.reachable_frontier_count =
    selectable_frontiers.size();

  if (selectable_frontiers.empty()) {
    result.status =
      ExplorationPlanningStatus::
      NoReachableFrontiers;

    result.reason =
      to_string(result.status);

    return result;
  }

  const std::optional<
    frontier::FrontierSelection> selection =
    selector_.select(
    selectable_frontiers,
    robot_x_m,
    robot_y_m);

  if (!selection.has_value()) {
    result.status =
      ExplorationPlanningStatus::
      NoSelectableFrontier;

    result.reason =
      to_string(result.status);

    return result;
  }

  const std::size_t filtered_index =
    selection->frontier_index;

  const FrontierCluster & selected_frontier =
    selectable_frontiers.at(
    filtered_index);

  const GridCell approach =
    choose_approach_cell(
    grid,
    approaches_by_frontier.at(
      filtered_index),
    robot_x_m,
    robot_y_m);

  const auto [approach_x_m, approach_y_m] =
    cell_center(grid, approach);

  ExplorationGoal goal;

  goal.goal_id =
    build_goal_id(
    config_.goal_id_prefix,
    selected_frontier,
    approach);

  goal.frontier_index =
    original_indices.at(filtered_index);

  goal.frontier_cell =
    selected_frontier.representative;

  goal.approach_cell = approach;

  goal.x_m = approach_x_m;
  goal.y_m = approach_y_m;

  goal.yaw_rad =
    std::atan2(
    selected_frontier.centroid_y_m -
    approach_y_m,
    selected_frontier.centroid_x_m -
    approach_x_m);

  goal.score = selection->score;
  goal.distance_m = selection->distance_m;

  goal.information_gain_m2 =
    selected_frontier.information_gain_m2;

  const std::string goal_error =
    validate_exploration_goal(goal);

  if (!goal_error.empty()) {
    throw std::logic_error(
            "generated invalid exploration "
            "goal: " + goal_error);
  }

  result.status =
    ExplorationPlanningStatus::GoalSelected;

  result.reason =
    to_string(result.status);

  result.goal = std::move(goal);

  return result;
}

}  // namespace savo_mapping::exploration
