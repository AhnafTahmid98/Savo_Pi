#include "savo_mapping/coverage_planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using savo_mapping::coverage::Connectivity;
using savo_mapping::coverage::CoverageGrid;
using savo_mapping::coverage::CoverageGridMetadata;
using savo_mapping::coverage::CoverageGridOptions;
using savo_mapping::coverage::CoveragePlanner;
using savo_mapping::coverage::CoveragePlannerOptions;
using savo_mapping::coverage::GridIndex;
using savo_mapping::coverage::SweepAxis;
using savo_mapping::coverage::WorldPoint;

CoverageGrid make_grid(
  std::size_t width,
  std::size_t height,
  std::vector<std::int8_t> data,
  double resolution = 1.0,
  CoverageGridOptions grid_options = {})
{
  const auto data_size = data.size();
  return CoverageGrid(
    CoverageGridMetadata{
      width, height, resolution, 0.0, 0.0, 0.0, data_size},
    std::move(data), grid_options);
}

CoverageGrid open_grid(std::size_t width, std::size_t height)
{
  return make_grid(
    width, height, std::vector<std::int8_t>(width * height, 0));
}

std::vector<GridIndex> waypoint_cells(
  const savo_mapping::coverage::CoveragePlanResult & result)
{
  std::vector<GridIndex> cells;
  cells.reserve(result.plan.waypoints.size());
  std::transform(
    result.plan.waypoints.begin(), result.plan.waypoints.end(),
    std::back_inserter(cells),
    [](const auto & waypoint) {return waypoint.cell;});
  return cells;
}

std::string constructor_error(const CoveragePlannerOptions & options)
{
  try {
    CoveragePlanner planner(options);
  } catch (const std::invalid_argument & error) {
    return error.what();
  }
  return "";
}

}  // namespace

TEST(CoveragePlanner, RejectsZeroTrackSpacing)
{
  CoveragePlannerOptions options;
  options.track_spacing_m = 0.0;
  EXPECT_EQ(
    constructor_error(options),
    "coverage_planner_track_spacing_invalid");
}

TEST(CoveragePlanner, RejectsNonFiniteTrackSpacing)
{
  CoveragePlannerOptions options;
  options.track_spacing_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    constructor_error(options),
    "coverage_planner_track_spacing_invalid");
}

TEST(CoveragePlanner, RejectsNegativeMinimumSegmentLength)
{
  CoveragePlannerOptions options;
  options.minimum_segment_length_m = -0.1;
  EXPECT_EQ(
    constructor_error(options),
    "coverage_planner_minimum_segment_length_invalid");
}

TEST(CoveragePlanner, RejectsZeroWaypointLimit)
{
  CoveragePlannerOptions options;
  options.maximum_waypoint_count = 0;
  EXPECT_EQ(
    constructor_error(options),
    "coverage_planner_waypoint_limit_invalid");
}

TEST(CoveragePlanner, PlansSingleCellMap)
{
  const auto result = CoveragePlanner().plan(open_grid(1, 1), {0, 0});
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.plan.waypoints.size(), 1U);
  EXPECT_EQ(result.plan.waypoints.front().cell, (GridIndex{0, 0}));
  EXPECT_DOUBLE_EQ(result.plan.estimated_path_length_m, 0.0);
  EXPECT_DOUBLE_EQ(result.plan.estimated_coverage_ratio, 1.0);
}

TEST(CoveragePlanner, ProducesDeterministicBoustrophedonRows)
{
  const auto result = CoveragePlanner().plan(open_grid(3, 2), {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(
    waypoint_cells(result),
    (std::vector<GridIndex>{
    {0, 0}, {1, 0}, {2, 0}, {2, 1}, {1, 1}, {0, 1}}));
}

TEST(CoveragePlanner, SupportsColumnSweep)
{
  CoveragePlannerOptions options;
  options.sweep_axis = SweepAxis::Columns;
  const auto result = CoveragePlanner(options).plan(
    open_grid(2, 3), {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.plan.sweep_axis, SweepAxis::Columns);
  EXPECT_EQ(
    waypoint_cells(result),
    (std::vector<GridIndex>{
    {0, 0}, {0, 1}, {0, 2}, {1, 2}, {1, 1}, {1, 0}}));
}

TEST(CoveragePlanner, AutomaticSweepUsesLongDimension)
{
  const auto rows = CoveragePlanner().plan(open_grid(4, 2), {0, 0});
  const auto columns = CoveragePlanner().plan(open_grid(2, 4), {0, 0});
  ASSERT_TRUE(rows.valid);
  ASSERT_TRUE(columns.valid);
  EXPECT_EQ(rows.plan.sweep_axis, SweepAxis::Rows);
  EXPECT_EQ(columns.plan.sweep_axis, SweepAxis::Columns);
}

TEST(CoveragePlanner, HonorsMetricTrackSpacing)
{
  auto grid = make_grid(3, 5, std::vector<std::int8_t>(15, 0), 0.5);
  CoveragePlannerOptions options;
  options.track_spacing_m = 1.0;
  options.sweep_axis = SweepAxis::Rows;
  const auto result = CoveragePlanner(options).plan(grid, {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.plan.covered_cell_count, 9U);
  EXPECT_DOUBLE_EQ(result.plan.estimated_coverage_ratio, 0.6);
}

TEST(CoveragePlanner, FiltersShortSegments)
{
  auto grid = make_grid(4, 1, {0, 100, 0, 0});
  CoveragePlannerOptions options;
  options.minimum_segment_length_m = 2.0;
  const auto result = CoveragePlanner(options).plan(grid, {2, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(
    result.plan.coverage_cells,
    (std::vector<GridIndex>{{2, 0}, {3, 0}}));
}

TEST(CoveragePlanner, ReportsWhenAllSegmentsAreFiltered)
{
  CoveragePlannerOptions options;
  options.minimum_segment_length_m = 2.0;
  const auto result = CoveragePlanner(options).plan(
    open_grid(1, 1), {0, 0});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "coverage_planner_no_coverage_cells");
}

TEST(CoveragePlanner, RejectsOutOfBoundsGridStart)
{
  const auto result = CoveragePlanner().plan(open_grid(2, 2), {2, 0});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "coverage_planner_start_out_of_bounds");
}

TEST(CoveragePlanner, RejectsOutOfBoundsWorldStart)
{
  const auto result = CoveragePlanner().plan_from_world(
    open_grid(2, 2), WorldPoint{-0.1, 0.5});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "coverage_planner_start_out_of_bounds");
}

TEST(CoveragePlanner, RejectsBlockedStart)
{
  const auto result = CoveragePlanner().plan(
    make_grid(2, 1, {100, 0}), {0, 0});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "coverage_planner_start_blocked");
}

TEST(CoveragePlanner, PlansFromWorldStart)
{
  const auto result = CoveragePlanner().plan_from_world(
    open_grid(2, 1), WorldPoint{1.5, 0.5});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.plan.waypoints.front().cell, (GridIndex{1, 0}));
}

TEST(CoveragePlanner, PreservesNonzeroMapOrigin)
{
  auto grid = CoverageGrid(
    CoverageGridMetadata{2, 1, 0.5, -2.0, 3.0, 0.0, 2},
    {0, 0});
  const auto result = CoveragePlanner().plan_from_world(
    grid, WorldPoint{-1.75, 3.25});
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.plan.waypoints.front().position.x_m, -1.75);
  EXPECT_DOUBLE_EQ(result.plan.waypoints.front().position.y_m, 3.25);
}

TEST(CoveragePlanner, ExcludesDisconnectedFreeIsland)
{
  const auto result = CoveragePlanner().plan(
    make_grid(3, 1, {0, 100, 0}), {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.plan.reachable_cell_count, 1U);
  EXPECT_EQ(result.plan.coverage_cells, (std::vector<GridIndex>{{0, 0}}));
}

TEST(CoveragePlanner, RoutesAroundObstacleWithinReachableSpace)
{
  const auto result = CoveragePlanner().plan(
    make_grid(3, 3, {0, 0, 0, 0, 100, 0, 0, 0, 0}), {0, 0});
  ASSERT_TRUE(result.valid);
  for (const auto & waypoint : result.plan.waypoints) {
    EXPECT_NE(waypoint.cell, (GridIndex{1, 1}));
  }
  EXPECT_GT(result.plan.estimated_path_length_m, 0.0);
}

TEST(CoveragePlanner, HandlesNarrowPassage)
{
  const auto result = CoveragePlanner().plan(
    make_grid(3, 3, {100, 0, 100, 100, 0, 100, 100, 0, 100}),
    {1, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.plan.reachable_cell_count, 3U);
  EXPECT_EQ(result.plan.covered_cell_count, 3U);
}

TEST(CoveragePlanner, UnknownPolicyChangesReachableCoverage)
{
  auto blocked = make_grid(3, 1, {0, -1, 0});
  CoverageGridOptions options;
  options.allow_unknown = true;
  auto allowed = make_grid(3, 1, {0, -1, 0}, 1.0, options);
  const auto blocked_result = CoveragePlanner().plan(blocked, {0, 0});
  const auto allowed_result = CoveragePlanner().plan(allowed, {0, 0});
  ASSERT_TRUE(blocked_result.valid);
  ASSERT_TRUE(allowed_result.valid);
  EXPECT_EQ(blocked_result.plan.reachable_cell_count, 1U);
  EXPECT_EQ(allowed_result.plan.reachable_cell_count, 3U);
}

TEST(CoveragePlanner, InflationExcludesUnsafeCells)
{
  CoverageGridOptions options;
  options.inflation_radius_m = 1.0;
  auto grid = make_grid(4, 1, {0, 0, 0, 100}, 1.0, options);
  const auto result = CoveragePlanner().plan(grid, {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.plan.reachable_cell_count, 2U);
  EXPECT_EQ(result.plan.covered_cell_count, 2U);
}

TEST(CoveragePlanner, DistinguishesCoverageAndTransitCells)
{
  CoveragePlannerOptions options;
  options.track_spacing_m = 2.0;
  options.sweep_axis = SweepAxis::Rows;
  const auto result = CoveragePlanner(options).plan(
    open_grid(3, 3), {0, 1});
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.plan.transit_cells.empty());
  EXPECT_TRUE(result.plan.waypoints.front().transit);
}

TEST(CoveragePlanner, EnforcesWaypointLimit)
{
  CoveragePlannerOptions options;
  options.maximum_waypoint_count = 3;
  const auto result = CoveragePlanner(options).plan(
    open_grid(4, 2), {0, 0});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "coverage_planner_waypoint_limit_exceeded");
  EXPECT_TRUE(result.plan.waypoints.empty());
}

TEST(CoveragePlanner, RemovesConsecutiveDuplicateWaypoints)
{
  const auto result = CoveragePlanner().plan(open_grid(3, 2), {1, 0});
  ASSERT_TRUE(result.valid);
  for (std::size_t index = 1;
    index < result.plan.waypoints.size(); ++index)
  {
    EXPECT_NE(
      result.plan.waypoints[index - 1].cell,
      result.plan.waypoints[index].cell);
  }
}

TEST(CoveragePlanner, EveryPlanSegmentRemainsTraversable)
{
  auto grid = make_grid(
    4, 3, {0, 0, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0});
  const auto result = CoveragePlanner().plan(grid, {0, 0});
  ASSERT_TRUE(result.valid);
  for (const auto & waypoint : result.plan.waypoints) {
    EXPECT_TRUE(grid.traversable(waypoint.cell));
  }
  for (std::size_t index = 1;
    index < result.plan.waypoints.size(); ++index)
  {
    const auto previous = result.plan.waypoints[index - 1].cell;
    const auto current = result.plan.waypoints[index].cell;
    const auto column_delta = static_cast<std::int64_t>(current.column) -
      static_cast<std::int64_t>(previous.column);
    const auto row_delta = static_cast<std::int64_t>(current.row) -
      static_cast<std::int64_t>(previous.row);
    EXPECT_LE(std::abs(column_delta), 1);
    EXPECT_LE(std::abs(row_delta), 1);
  }
}

TEST(CoveragePlanner, ComputesFiniteBoundedHeadings)
{
  const auto result = CoveragePlanner().plan(open_grid(3, 2), {0, 0});
  ASSERT_TRUE(result.valid);
  constexpr double pi = 3.14159265358979323846;
  for (const auto & waypoint : result.plan.waypoints) {
    EXPECT_TRUE(std::isfinite(waypoint.yaw_rad));
    EXPECT_GE(waypoint.yaw_rad, -pi);
    EXPECT_LT(waypoint.yaw_rad, pi);
  }
}

TEST(CoveragePlanner, ComputesMetricPathLength)
{
  const auto result = CoveragePlanner().plan(open_grid(3, 1), {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.plan.estimated_path_length_m, 2.0);
}

TEST(CoveragePlanner, CoverageRatioIsBounded)
{
  CoveragePlannerOptions options;
  options.track_spacing_m = 2.0;
  const auto result = CoveragePlanner(options).plan(
    open_grid(3, 3), {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.plan.estimated_coverage_ratio, 0.0);
  EXPECT_LE(result.plan.estimated_coverage_ratio, 1.0);
}

TEST(CoveragePlanner, IsDeterministicAcrossRepeatedCalls)
{
  auto grid = open_grid(4, 3);
  CoveragePlanner planner;
  const auto first = planner.plan(grid, {1, 1});
  const auto second = planner.plan(grid, {1, 1});
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(waypoint_cells(first), waypoint_cells(second));
  EXPECT_EQ(first.plan.coverage_cells, second.plan.coverage_cells);
}

TEST(CoveragePlanner, EightConnectivityStillPreventsCornerCutting)
{
  CoveragePlannerOptions options;
  options.connectivity = Connectivity::Eight;
  const auto result = CoveragePlanner(options).plan(
    make_grid(2, 2, {0, 100, 100, 0}), {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.plan.reachable_cell_count, 1U);
}

TEST(CoveragePlanner, PlanningDoesNotMutateOccupancy)
{
  const std::vector<std::int8_t> data{0, 0, 100, 0};
  auto grid = make_grid(2, 2, data);
  const auto result = CoveragePlanner().plan(grid, {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(grid.occupancy(), data);
}

TEST(CoveragePlanner, CorePlanContainsOnlyDataAndNoExecutionHooks)
{
  const auto result = CoveragePlanner().plan(open_grid(2, 2), {0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.plan.waypoints.empty());
  EXPECT_EQ(result.reason, "coverage_plan_ready");
}
