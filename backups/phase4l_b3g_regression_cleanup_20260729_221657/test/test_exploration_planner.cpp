#include "savo_mapping/exploration_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{

using savo_mapping::exploration::ExplorationGoal;
using savo_mapping::exploration::ExplorationPlanner;
using savo_mapping::exploration::ExplorationPlannerConfig;
using savo_mapping::exploration::
ExplorationPlanningStatus;
using savo_mapping::exploration::to_string;
using savo_mapping::exploration::
validate_exploration_goal;
using savo_mapping::frontier::GridCell;
using savo_mapping::frontier::OccupancyGrid;
using savo_mapping::frontier::UNKNOWN_CELL;

OccupancyGrid make_grid(
  std::size_t width,
  std::size_t height,
  std::int8_t value)
{
  OccupancyGrid grid;

  grid.width = width;
  grid.height = height;
  grid.resolution_m = 1.0;

  grid.cells.assign(
    width * height,
    value);

  return grid;
}

void set_cell(
  OccupancyGrid & grid,
  std::size_t x,
  std::size_t y,
  std::int8_t value)
{
  grid.cells.at(
    y * grid.width + x) = value;
}

}  // namespace

TEST(
  ExplorationGoal,
  ConvertsPlanningStatusesToStableStrings)
{
  EXPECT_EQ(
    to_string(
      ExplorationPlanningStatus::
      GoalSelected),
    "goal_selected");

  EXPECT_EQ(
    to_string(
      ExplorationPlanningStatus::
      NoFrontiers),
    "no_frontiers");

  EXPECT_EQ(
    to_string(
      ExplorationPlanningStatus::
      NoReachableFrontiers),
    "no_reachable_frontiers");

  EXPECT_EQ(
    to_string(
      ExplorationPlanningStatus::
      NoSelectableFrontier),
    "no_selectable_frontier");
}

TEST(
  ExplorationGoal,
  ValidatesGoalFields)
{
  ExplorationGoal goal;

  EXPECT_EQ(
    validate_exploration_goal(goal),
    "goal_id_must_not_be_empty");

  goal.goal_id =
    "frontier-f1-0-a1-1";

  goal.distance_m = 1.0;
  goal.information_gain_m2 = 2.0;

  EXPECT_TRUE(
    validate_exploration_goal(
      goal).empty());

  goal.yaw_rad =
    std::numeric_limits<double>::
    quiet_NaN();

  EXPECT_EQ(
    validate_exploration_goal(goal),
    "goal_pose_must_be_finite");
}

TEST(
  ExplorationPlanner,
  RejectsInvalidConfiguration)
{
  ExplorationPlannerConfig config;

  config.goal_id_prefix =
    "invalid prefix";

  EXPECT_THROW(
    static_cast<void>(
      ExplorationPlanner{config}),
    std::invalid_argument);
}

TEST(
  ExplorationPlanner,
  RejectsMalformedGrid)
{
  OccupancyGrid grid;

  grid.width = 2;
  grid.height = 2;
  grid.resolution_m = 1.0;
  grid.cells = {0, 0, 0};

  const ExplorationPlanner planner;

  EXPECT_THROW(
    planner.plan(
      grid,
      0.5,
      0.5),
    std::invalid_argument);
}

TEST(
  ExplorationPlanner,
  RejectsRobotPoseOutsideGrid)
{
  const OccupancyGrid grid =
    make_grid(3, 3, 0);

  const ExplorationPlanner planner;

  EXPECT_THROW(
    planner.plan(
      grid,
      -0.1,
      0.5),
    std::invalid_argument);
}

TEST(
  ExplorationPlanner,
  RejectsRobotPoseInNonfreeCell)
{
  OccupancyGrid grid =
    make_grid(3, 3, 100);

  set_cell(
    grid,
    1,
    0,
    UNKNOWN_CELL);

  const ExplorationPlanner planner;

  EXPECT_THROW(
    planner.plan(
      grid,
      1.5,
      1.5),
    std::invalid_argument);
}

TEST(
  ExplorationPlanner,
  FullyKnownMapReturnsNoFrontiers)
{
  const OccupancyGrid grid =
    make_grid(3, 3, 0);

  const ExplorationPlanner planner;

  const auto plan =
    planner.plan(
    grid,
    1.5,
    1.5);

  EXPECT_EQ(
    plan.status,
    ExplorationPlanningStatus::
    NoFrontiers);

  EXPECT_EQ(
    plan.reason,
    "no_frontiers");

  EXPECT_EQ(
    plan.detected_frontier_count,
    0u);

  EXPECT_EQ(
    plan.reachable_frontier_count,
    0u);

  EXPECT_FALSE(plan.goal.has_value());
}

TEST(
  ExplorationPlanner,
  ProducesFreeApproachGoalFacingFrontier)
{
  OccupancyGrid grid =
    make_grid(5, 5, 100);

  set_cell(grid, 2, 2, 0);

  set_cell(
    grid,
    2,
    1,
    UNKNOWN_CELL);

  const ExplorationPlanner planner;

  const auto plan =
    planner.plan(
    grid,
    2.5,
    2.5);

  ASSERT_EQ(
    plan.status,
    ExplorationPlanningStatus::
    GoalSelected);

  ASSERT_TRUE(plan.goal.has_value());

  const ExplorationGoal & goal =
    plan.goal.value();

  EXPECT_EQ(
    goal.frontier_cell,
    (GridCell{2, 1}));

  EXPECT_EQ(
    goal.approach_cell,
    (GridCell{2, 2}));

  EXPECT_DOUBLE_EQ(goal.x_m, 2.5);
  EXPECT_DOUBLE_EQ(goal.y_m, 2.5);

  EXPECT_NEAR(
    goal.yaw_rad,
    -std::acos(-1.0) / 2.0,
    1.0e-12);

  EXPECT_EQ(
    goal.goal_id,
    "frontier-f2-1-a2-2");

  EXPECT_DOUBLE_EQ(
    goal.information_gain_m2,
    1.0);
}

TEST(
  ExplorationPlanner,
  UsesOriginAndResolutionForApproachPose)
{
  OccupancyGrid grid =
    make_grid(3, 3, 100);

  grid.resolution_m = 0.5;
  grid.origin_x_m = -2.0;
  grid.origin_y_m = 3.0;

  set_cell(grid, 1, 1, 0);

  set_cell(
    grid,
    1,
    0,
    UNKNOWN_CELL);

  const ExplorationPlanner planner;

  const auto plan =
    planner.plan(
    grid,
    -1.25,
    3.75);

  ASSERT_TRUE(plan.goal.has_value());

  EXPECT_DOUBLE_EQ(
    plan.goal->x_m,
    -1.25);

  EXPECT_DOUBLE_EQ(
    plan.goal->y_m,
    3.75);
}

TEST(
  ExplorationPlanner,
  ReportsOnlyDisconnectedFrontiersAsUnreachable)
{
  OccupancyGrid grid =
    make_grid(7, 3, 100);

  set_cell(grid, 1, 1, 0);
  set_cell(grid, 5, 1, 0);

  set_cell(
    grid,
    5,
    0,
    UNKNOWN_CELL);

  const ExplorationPlanner planner;

  const auto plan =
    planner.plan(
    grid,
    1.5,
    1.5);

  EXPECT_EQ(
    plan.status,
    ExplorationPlanningStatus::
    NoReachableFrontiers);

  EXPECT_EQ(
    plan.detected_frontier_count,
    1u);

  EXPECT_EQ(
    plan.reachable_frontier_count,
    0u);

  EXPECT_FALSE(plan.goal.has_value());
}

TEST(
  ExplorationPlanner,
  PreservesOriginalIndexAfterReachabilityFiltering)
{
  OccupancyGrid grid =
    make_grid(9, 3, 100);

  set_cell(grid, 1, 1, 0);

  set_cell(
    grid,
    1,
    0,
    UNKNOWN_CELL);

  set_cell(grid, 6, 1, 0);

  set_cell(
    grid,
    6,
    0,
    UNKNOWN_CELL);

  const ExplorationPlanner planner;

  const auto plan =
    planner.plan(
    grid,
    6.5,
    1.5);

  ASSERT_TRUE(plan.goal.has_value());

  EXPECT_EQ(
    plan.detected_frontier_count,
    2u);

  EXPECT_EQ(
    plan.reachable_frontier_count,
    1u);

  EXPECT_EQ(
    plan.goal->frontier_index,
    1u);

  EXPECT_EQ(
    plan.goal->frontier_cell,
    (GridCell{6, 0}));
}

TEST(
  ExplorationPlanner,
  SelectorConstraintsCanRejectReachableFrontier)
{
  OccupancyGrid grid =
    make_grid(5, 5, 100);

  set_cell(grid, 2, 2, 0);

  set_cell(
    grid,
    2,
    1,
    UNKNOWN_CELL);

  ExplorationPlannerConfig config;

  config.selector.
  minimum_information_gain_m2 = 2.0;

  const ExplorationPlanner planner(config);

  const auto plan =
    planner.plan(
    grid,
    2.5,
    2.5);

  EXPECT_EQ(
    plan.status,
    ExplorationPlanningStatus::
    NoSelectableFrontier);

  EXPECT_EQ(
    plan.detected_frontier_count,
    1u);

  EXPECT_EQ(
    plan.reachable_frontier_count,
    1u);

  EXPECT_FALSE(plan.goal.has_value());
}

TEST(
  ExplorationPlanner,
  ChoosesNearestReachableApproachDeterministically)
{
  OccupancyGrid grid =
    make_grid(5, 5, 100);

  set_cell(grid, 1, 2, 0);
  set_cell(grid, 2, 2, 0);
  set_cell(grid, 3, 2, 0);

  set_cell(
    grid,
    2,
    1,
    UNKNOWN_CELL);

  set_cell(
    grid,
    3,
    1,
    UNKNOWN_CELL);

  const ExplorationPlanner planner;

  const auto plan =
    planner.plan(
    grid,
    1.5,
    2.5);

  ASSERT_TRUE(plan.goal.has_value());

  EXPECT_EQ(
    plan.goal->approach_cell,
    (GridCell{2, 2}));
}
