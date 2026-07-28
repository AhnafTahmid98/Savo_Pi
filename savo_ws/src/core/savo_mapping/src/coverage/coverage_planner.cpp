#include "savo_mapping/coverage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>

namespace savo_mapping::coverage
{
namespace
{

using CellSet = std::set<GridIndex>;

double normalize_angle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double two_pi = 2.0 * pi;
  while (angle >= pi) {
    angle -= two_pi;
  }
  while (angle < -pi) {
    angle += two_pi;
  }
  return angle;
}

std::vector<GridIndex> neighbors(
  GridIndex current,
  Connectivity connectivity,
  const CellSet & allowed)
{
  constexpr int offsets[8][2] = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  const std::size_t count =
    connectivity == Connectivity::Four ? 4U : 8U;
  std::vector<GridIndex> result;
  for (std::size_t index = 0; index < count; ++index) {
    const auto column =
      static_cast<std::int64_t>(current.column) + offsets[index][0];
    const auto row =
      static_cast<std::int64_t>(current.row) + offsets[index][1];
    if (column < 0 || row < 0) {
      continue;
    }
    const GridIndex candidate{
      static_cast<std::size_t>(column),
      static_cast<std::size_t>(row)};
    if (allowed.find(candidate) == allowed.end()) {
      continue;
    }
    if (index >= 4 &&
      (allowed.find({current.column, candidate.row}) == allowed.end() ||
      allowed.find({candidate.column, current.row}) == allowed.end()))
    {
      continue;
    }
    result.push_back(candidate);
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::vector<GridIndex> shortest_path(
  GridIndex start,
  GridIndex goal,
  Connectivity connectivity,
  const CellSet & allowed)
{
  if (start == goal) {
    return {start};
  }

  std::queue<GridIndex> pending;
  std::set<GridIndex> visited;
  std::map<GridIndex, GridIndex> parent;
  pending.push(start);
  visited.insert(start);

  while (!pending.empty()) {
    const auto current = pending.front();
    pending.pop();
    for (const auto next :
      neighbors(current, connectivity, allowed))
    {
      if (!visited.insert(next).second) {
        continue;
      }
      parent[next] = current;
      if (next == goal) {
        std::vector<GridIndex> path{goal};
        auto cursor = goal;
        while (cursor != start) {
          cursor = parent.at(cursor);
          path.push_back(cursor);
        }
        std::reverse(path.begin(), path.end());
        return path;
      }
      pending.push(next);
    }
  }
  return {};
}

SweepAxis resolved_axis(
  SweepAxis requested,
  const CoverageGridMetadata & metadata)
{
  if (requested != SweepAxis::Automatic) {
    return requested;
  }
  return metadata.width >= metadata.height ?
         SweepAxis::Rows : SweepAxis::Columns;
}

std::vector<std::vector<GridIndex>> make_segments(
  const CoverageGrid & grid,
  const CellSet & reachable,
  SweepAxis axis,
  std::size_t spacing_cells,
  double minimum_segment_length_m)
{
  const auto & metadata = grid.metadata();
  const std::size_t lane_count =
    axis == SweepAxis::Rows ? metadata.height : metadata.width;
  const std::size_t lane_length =
    axis == SweepAxis::Rows ? metadata.width : metadata.height;
  std::vector<std::vector<GridIndex>> segments;
  std::size_t accepted_lane = 0;
  const auto first_reachable = std::min_element(
    reachable.begin(), reachable.end(),
    [axis](GridIndex lhs, GridIndex rhs) {
      const auto lhs_lane =
      axis == SweepAxis::Rows ? lhs.row : lhs.column;
      const auto rhs_lane =
      axis == SweepAxis::Rows ? rhs.row : rhs.column;
      return lhs_lane < rhs_lane;
    });
  const std::size_t first_lane =
    axis == SweepAxis::Rows ?
    first_reachable->row : first_reachable->column;

  for (std::size_t lane = first_lane; lane < lane_count;
    lane += spacing_cells)
  {
    std::vector<std::vector<GridIndex>> lane_segments;
    std::vector<GridIndex> current;
    for (std::size_t position = 0; position < lane_length; ++position) {
      const GridIndex cell = axis == SweepAxis::Rows ?
        GridIndex{position, lane} : GridIndex{lane, position};
      if (reachable.find(cell) != reachable.end()) {
        current.push_back(cell);
      } else if (!current.empty()) {
        lane_segments.push_back(std::move(current));
        current.clear();
      }
    }
    if (!current.empty()) {
      lane_segments.push_back(std::move(current));
    }

    lane_segments.erase(
      std::remove_if(
        lane_segments.begin(), lane_segments.end(),
        [&metadata, minimum_segment_length_m](const auto & segment) {
          return static_cast<double>(segment.size()) *
                 metadata.resolution_m < minimum_segment_length_m;
        }),
      lane_segments.end());
    if (lane_segments.empty()) {
      continue;
    }

    if (accepted_lane % 2U == 1U) {
      std::reverse(lane_segments.begin(), lane_segments.end());
      for (auto & segment : lane_segments) {
        std::reverse(segment.begin(), segment.end());
      }
    }
    for (auto & segment : lane_segments) {
      segments.push_back(std::move(segment));
    }
    ++accepted_lane;
  }
  return segments;
}

CoveragePlanResult failure(std::string reason)
{
  return {false, std::move(reason), {}};
}

}  // namespace

std::string validate_coverage_planner_options(
  const CoveragePlannerOptions & options)
{
  if (!std::isfinite(options.track_spacing_m) ||
    options.track_spacing_m <= 0.0)
  {
    return "coverage_planner_track_spacing_invalid";
  }
  if (!std::isfinite(options.minimum_segment_length_m) ||
    options.minimum_segment_length_m < 0.0)
  {
    return "coverage_planner_minimum_segment_length_invalid";
  }
  if (options.maximum_waypoint_count == 0) {
    return "coverage_planner_waypoint_limit_invalid";
  }
  if (options.connectivity != Connectivity::Four &&
    options.connectivity != Connectivity::Eight)
  {
    return "coverage_planner_connectivity_invalid";
  }
  if (options.sweep_axis != SweepAxis::Automatic &&
    options.sweep_axis != SweepAxis::Rows &&
    options.sweep_axis != SweepAxis::Columns)
  {
    return "coverage_planner_sweep_axis_invalid";
  }
  return "";
}

CoveragePlanner::CoveragePlanner(CoveragePlannerOptions options)
: options_(std::move(options))
{
  const auto error = validate_coverage_planner_options(options_);
  if (!error.empty()) {
    throw std::invalid_argument(error);
  }
}

const CoveragePlannerOptions & CoveragePlanner::options() const
{
  return options_;
}

CoveragePlanResult CoveragePlanner::plan(
  const CoverageGrid & grid,
  GridIndex start) const
{
  const auto reachable_result =
    grid.reachable_from(start, options_.connectivity);
  if (!reachable_result.valid) {
    if (reachable_result.reason == "coverage_grid_start_out_of_bounds") {
      return failure("coverage_planner_start_out_of_bounds");
    }
    if (reachable_result.reason == "coverage_grid_start_blocked") {
      return failure("coverage_planner_start_blocked");
    }
    return failure("coverage_planner_no_reachable_cells");
  }

  const CellSet reachable(
    reachable_result.cells.begin(), reachable_result.cells.end());
  if (reachable.empty()) {
    return failure("coverage_planner_no_reachable_cells");
  }

  const auto axis = resolved_axis(options_.sweep_axis, grid.metadata());
  const auto spacing_cells = std::max<std::size_t>(
    1U, static_cast<std::size_t>(
      std::ceil(options_.track_spacing_m /
      grid.metadata().resolution_m)));
  auto segments = make_segments(
    grid, reachable, axis, spacing_cells,
    options_.minimum_segment_length_m);
  if (segments.empty()) {
    return failure("coverage_planner_no_coverage_cells");
  }

  CellSet coverage_targets;
  for (const auto & segment : segments) {
    coverage_targets.insert(segment.begin(), segment.end());
  }

  std::vector<GridIndex> route;
  auto append_cell = [&route, this](GridIndex cell) {
      if (!route.empty() && route.back() == cell) {
        return true;
      }
      if (route.size() >= options_.maximum_waypoint_count) {
        return false;
      }
      route.push_back(cell);
      return true;
    };

  if (!append_cell(start)) {
    return failure("coverage_planner_waypoint_limit_exceeded");
  }
  auto cursor = start;
  for (const auto & segment : segments) {
    const auto connection = shortest_path(
      cursor, segment.front(), options_.connectivity, reachable);
    if (connection.empty()) {
      return failure("coverage_planner_connection_failed");
    }
    for (const auto connection_cell : connection) {
      if (!append_cell(connection_cell)) {
        return failure("coverage_planner_waypoint_limit_exceeded");
      }
    }
    for (const auto coverage_cell : segment) {
      if (!append_cell(coverage_cell)) {
        return failure("coverage_planner_waypoint_limit_exceeded");
      }
    }
    cursor = segment.back();
  }

  CoveragePlan plan;
  plan.reachable_cell_count = reachable.size();
  plan.covered_cell_count = coverage_targets.size();
  plan.estimated_coverage_ratio =
    static_cast<double>(plan.covered_cell_count) /
    static_cast<double>(plan.reachable_cell_count);
  plan.sweep_axis = axis;
  plan.coverage_cells.assign(
    coverage_targets.begin(), coverage_targets.end());

  CellSet transit_cells;
  plan.waypoints.reserve(route.size());
  for (const auto route_cell : route) {
    const auto world = grid.grid_to_world(route_cell);
    if (!world) {
      return failure("coverage_planner_internal_geometry_error");
    }
    const bool transit =
      coverage_targets.find(route_cell) == coverage_targets.end();
    if (transit) {
      transit_cells.insert(route_cell);
    }
    plan.waypoints.push_back({route_cell, *world, 0.0, transit});
  }
  plan.transit_cells.assign(transit_cells.begin(), transit_cells.end());

  for (std::size_t index = 1; index < plan.waypoints.size(); ++index) {
    const auto & previous = plan.waypoints[index - 1].position;
    const auto & current = plan.waypoints[index].position;
    plan.estimated_path_length_m +=
      std::hypot(current.x_m - previous.x_m, current.y_m - previous.y_m);
  }
  for (std::size_t index = 0; index + 1 < plan.waypoints.size(); ++index) {
    const auto & current = plan.waypoints[index].position;
    const auto & next = plan.waypoints[index + 1].position;
    plan.waypoints[index].yaw_rad = normalize_angle(
      std::atan2(next.y_m - current.y_m, next.x_m - current.x_m));
  }
  if (plan.waypoints.size() > 1) {
    plan.waypoints.back().yaw_rad =
      plan.waypoints[plan.waypoints.size() - 2].yaw_rad;
  }

  return {true, "coverage_plan_ready", std::move(plan)};
}

CoveragePlanResult CoveragePlanner::plan_from_world(
  const CoverageGrid & grid,
  WorldPoint start) const
{
  const auto grid_start = grid.world_to_grid(start);
  if (!grid_start) {
    return failure("coverage_planner_start_out_of_bounds");
  }
  return plan(grid, *grid_start);
}

}  // namespace savo_mapping::coverage
