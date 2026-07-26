#include "savo_mapping/frontier_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace savo_mapping::frontier
{
namespace
{

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

bool is_free_cell(
  std::int8_t value,
  std::int8_t free_threshold)
{
  return value >= 0 && value <= free_threshold;
}

bool is_frontier_cell(
  const OccupancyGrid & grid,
  const FrontierDetectorConfig & config,
  const GridCell & cell)
{
  if (grid.cells.at(linear_index(grid, cell)) != UNKNOWN_CELL) {
    return false;
  }

  constexpr std::array<std::pair<int, int>, 4> offsets{{
    {1, 0},
    {-1, 0},
    {0, 1},
    {0, -1},
  }};

  for (const auto & [offset_x, offset_y] : offsets) {
    const std::int64_t neighbor_x =
      static_cast<std::int64_t>(cell.x) + offset_x;

    const std::int64_t neighbor_y =
      static_cast<std::int64_t>(cell.y) + offset_y;

    if (!is_inside(grid, neighbor_x, neighbor_y)) {
      continue;
    }

    const GridCell neighbor{
      static_cast<std::size_t>(neighbor_x),
      static_cast<std::size_t>(neighbor_y),
    };

    if (is_free_cell(
        grid.cells.at(linear_index(grid, neighbor)),
        config.free_threshold))
    {
      return true;
    }
  }

  return false;
}

std::vector<GridCell> cluster_neighbors(
  const OccupancyGrid & grid,
  const GridCell & cell,
  bool include_diagonals)
{
  constexpr std::array<std::pair<int, int>, 8> offsets{{
    {1, 0},
    {-1, 0},
    {0, 1},
    {0, -1},
    {1, 1},
    {1, -1},
    {-1, 1},
    {-1, -1},
  }};

  const std::size_t count = include_diagonals ? 8 : 4;
  std::vector<GridCell> neighbors;
  neighbors.reserve(count);

  for (std::size_t index = 0; index < count; ++index) {
    const auto & [offset_x, offset_y] = offsets.at(index);

    const std::int64_t neighbor_x =
      static_cast<std::int64_t>(cell.x) + offset_x;

    const std::int64_t neighbor_y =
      static_cast<std::int64_t>(cell.y) + offset_y;

    if (!is_inside(grid, neighbor_x, neighbor_y)) {
      continue;
    }

    neighbors.push_back(
      GridCell{
          static_cast<std::size_t>(neighbor_x),
          static_cast<std::size_t>(neighbor_y),
      });
  }

  return neighbors;
}

FrontierCluster build_cluster(
  const OccupancyGrid & grid,
  std::vector<GridCell> cells)
{
  std::sort(
    cells.begin(),
    cells.end(),
    [&grid](const GridCell & left, const GridCell & right) {
      return linear_index(grid, left) < linear_index(grid, right);
    });

  double centroid_cell_x = 0.0;
  double centroid_cell_y = 0.0;

  for (const GridCell & cell : cells) {
    centroid_cell_x += static_cast<double>(cell.x) + 0.5;
    centroid_cell_y += static_cast<double>(cell.y) + 0.5;
  }

  centroid_cell_x /= static_cast<double>(cells.size());
  centroid_cell_y /= static_cast<double>(cells.size());

  GridCell representative = cells.front();
  double best_squared_distance =
    std::numeric_limits<double>::infinity();

  for (const GridCell & cell : cells) {
    const double delta_x =
      static_cast<double>(cell.x) + 0.5 - centroid_cell_x;

    const double delta_y =
      static_cast<double>(cell.y) + 0.5 - centroid_cell_y;

    const double squared_distance =
      delta_x * delta_x + delta_y * delta_y;

    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      representative = cell;
    }
  }

  FrontierCluster cluster;
  cluster.cells = std::move(cells);
  cluster.representative = representative;
  cluster.centroid_x_m =
    grid.origin_x_m + centroid_cell_x * grid.resolution_m;
  cluster.centroid_y_m =
    grid.origin_y_m + centroid_cell_y * grid.resolution_m;
  cluster.information_gain_m2 =
    static_cast<double>(cluster.cells.size()) *
    grid.resolution_m * grid.resolution_m;

  return cluster;
}

}  // namespace

bool operator==(const GridCell & left, const GridCell & right)
{
  return left.x == right.x && left.y == right.y;
}

bool operator!=(const GridCell & left, const GridCell & right)
{
  return !(left == right);
}

std::string validate_occupancy_grid(const OccupancyGrid & grid)
{
  if (grid.width == 0 || grid.height == 0) {
    return "grid_dimensions_must_be_nonzero";
  }

  if (grid.width >
    std::numeric_limits<std::size_t>::max() / grid.height)
  {
    return "grid_dimensions_overflow";
  }

  if (!std::isfinite(grid.resolution_m) ||
    grid.resolution_m <= 0.0)
  {
    return "grid_resolution_must_be_positive";
  }

  if (!std::isfinite(grid.origin_x_m) ||
    !std::isfinite(grid.origin_y_m))
  {
    return "grid_origin_must_be_finite";
  }

  if (grid.cells.size() != grid.width * grid.height) {
    return "grid_cell_count_mismatch";
  }

  for (const std::int8_t value : grid.cells) {
    if (value < UNKNOWN_CELL || value > 100) {
      return "grid_cell_value_outside_ros_range";
    }
  }

  return {};
}

std::string validate_frontier_detector_config(
  const FrontierDetectorConfig & config)
{
  if (config.free_threshold < 0 || config.free_threshold > 100) {
    return "free_threshold_outside_ros_range";
  }

  if (config.minimum_cluster_size == 0) {
    return "minimum_cluster_size_must_be_nonzero";
  }

  return {};
}

FrontierDetector::FrontierDetector(
  FrontierDetectorConfig config)
: config_(config)
{
  const std::string error =
    validate_frontier_detector_config(config_);

  if (!error.empty()) {
    throw std::invalid_argument(
            "invalid frontier detector configuration: " + error);
  }
}

const FrontierDetectorConfig & FrontierDetector::config() const noexcept
{
  return config_;
}

std::vector<FrontierCluster> FrontierDetector::detect(
  const OccupancyGrid & grid) const
{
  const std::string error = validate_occupancy_grid(grid);

  if (!error.empty()) {
    throw std::invalid_argument(
            "invalid occupancy grid: " + error);
  }

  const std::size_t cell_count = grid.width * grid.height;
  std::vector<bool> frontier_mask(cell_count, false);

  for (std::size_t y = 0; y < grid.height; ++y) {
    for (std::size_t x = 0; x < grid.width; ++x) {
      const GridCell cell{x, y};
      frontier_mask.at(linear_index(grid, cell)) =
        is_frontier_cell(grid, config_, cell);
    }
  }

  std::vector<bool> visited(cell_count, false);
  std::vector<FrontierCluster> clusters;

  for (std::size_t index = 0; index < cell_count; ++index) {
    if (!frontier_mask.at(index) || visited.at(index)) {
      continue;
    }

    std::queue<GridCell> pending;
    std::vector<GridCell> cluster_cells;

    const GridCell seed{
      index % grid.width,
      index / grid.width,
    };

    visited.at(index) = true;
    pending.push(seed);

    while (!pending.empty()) {
      const GridCell cell = pending.front();
      pending.pop();
      cluster_cells.push_back(cell);

      for (const GridCell & neighbor : cluster_neighbors(
          grid,
          cell,
          config_.cluster_diagonally))
      {
        const std::size_t neighbor_index =
          linear_index(grid, neighbor);

        if (!frontier_mask.at(neighbor_index) ||
          visited.at(neighbor_index))
        {
          continue;
        }

        visited.at(neighbor_index) = true;
        pending.push(neighbor);
      }
    }

    if (cluster_cells.size() < config_.minimum_cluster_size) {
      continue;
    }

    clusters.push_back(
      build_cluster(grid, std::move(cluster_cells)));
  }

  std::sort(
    clusters.begin(),
    clusters.end(),
    [](const FrontierCluster & left, const FrontierCluster & right) {
      if (left.representative.y != right.representative.y) {
        return left.representative.y < right.representative.y;
      }

      return left.representative.x < right.representative.x;
    });

  return clusters;
}

}  // namespace savo_mapping::frontier
