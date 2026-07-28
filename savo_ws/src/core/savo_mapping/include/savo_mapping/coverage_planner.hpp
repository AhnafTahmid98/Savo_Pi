#pragma once

#include "savo_mapping/coverage_grid.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace savo_mapping::coverage
{

enum class SweepAxis
{
  Automatic,
  Rows,
  Columns,
};

struct CoveragePlannerOptions
{
  double track_spacing_m{0.5};
  double minimum_segment_length_m{0.0};
  Connectivity connectivity{Connectivity::Four};
  SweepAxis sweep_axis{SweepAxis::Automatic};
  std::size_t maximum_waypoint_count{10000};
};

std::string validate_coverage_planner_options(
  const CoveragePlannerOptions & options);

struct CoverageWaypoint
{
  GridIndex cell;
  WorldPoint position;
  double yaw_rad{0.0};
  bool transit{false};
};

struct CoveragePlan
{
  std::vector<CoverageWaypoint> waypoints;
  std::vector<GridIndex> coverage_cells;
  std::vector<GridIndex> transit_cells;
  std::size_t reachable_cell_count{0};
  std::size_t covered_cell_count{0};
  double estimated_coverage_ratio{0.0};
  double estimated_path_length_m{0.0};
  SweepAxis sweep_axis{SweepAxis::Rows};
};

struct CoveragePlanResult
{
  bool valid{false};
  std::string reason;
  CoveragePlan plan;
};

class CoveragePlanner
{
public:
  explicit CoveragePlanner(CoveragePlannerOptions options = {});

  const CoveragePlannerOptions & options() const;

  CoveragePlanResult plan(
    const CoverageGrid & grid,
    GridIndex start) const;

  CoveragePlanResult plan_from_world(
    const CoverageGrid & grid,
    WorldPoint start) const;

private:
  CoveragePlannerOptions options_;
};

}  // namespace savo_mapping::coverage
