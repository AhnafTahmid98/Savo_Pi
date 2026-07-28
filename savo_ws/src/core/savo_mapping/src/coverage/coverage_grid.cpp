#include "savo_mapping/coverage_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace savo_mapping::coverage
{
namespace
{

constexpr double kGeometryTolerance = 1.0e-12;

bool dimensions_overflow(const CoverageGridMetadata & metadata)
{
  return metadata.height != 0 &&
         metadata.width > std::numeric_limits<std::size_t>::max() /
         metadata.height;
}

std::size_t expected_size(const CoverageGridMetadata & metadata)
{
  return metadata.width * metadata.height;
}

}  // namespace

bool operator==(const GridIndex & lhs, const GridIndex & rhs)
{
  return lhs.column == rhs.column && lhs.row == rhs.row;
}

bool operator!=(const GridIndex & lhs, const GridIndex & rhs)
{
  return !(lhs == rhs);
}

bool operator<(const GridIndex & lhs, const GridIndex & rhs)
{
  return lhs.row < rhs.row ||
         (lhs.row == rhs.row && lhs.column < rhs.column);
}

std::string validate_coverage_grid_metadata(
  const CoverageGridMetadata & metadata)
{
  if (metadata.width == 0 || metadata.height == 0 ||
    dimensions_overflow(metadata))
  {
    return "coverage_grid_dimensions_invalid";
  }

  if (!std::isfinite(metadata.resolution_m) ||
    metadata.resolution_m <= 0.0)
  {
    return "coverage_grid_resolution_invalid";
  }

  if (!std::isfinite(metadata.origin_x_m) ||
    !std::isfinite(metadata.origin_y_m) ||
    !std::isfinite(metadata.origin_yaw_rad))
  {
    return "coverage_grid_origin_invalid";
  }

  if (std::abs(metadata.origin_yaw_rad) > kGeometryTolerance) {
    return "coverage_grid_origin_yaw_unsupported";
  }

  if (metadata.occupancy_data_size != expected_size(metadata)) {
    return "coverage_grid_data_size_mismatch";
  }

  return "";
}

std::string validate_coverage_grid_options(
  const CoverageGridOptions & options)
{
  if (options.free_threshold < 0 ||
    options.free_threshold > 100 ||
    options.occupied_threshold < 0 ||
    options.occupied_threshold > 100 ||
    options.free_threshold >= options.occupied_threshold)
  {
    return "coverage_grid_thresholds_invalid";
  }

  if (!std::isfinite(options.inflation_radius_m) ||
    options.inflation_radius_m < 0.0)
  {
    return "coverage_grid_inflation_invalid";
  }

  return "";
}

CoverageGrid::CoverageGrid(
  CoverageGridMetadata metadata,
  std::vector<std::int8_t> occupancy,
  CoverageGridOptions options)
: metadata_(std::move(metadata)),
  occupancy_(std::move(occupancy)),
  options_(std::move(options))
{
  const auto metadata_error =
    validate_coverage_grid_metadata(metadata_);
  if (!metadata_error.empty()) {
    throw std::invalid_argument(metadata_error);
  }

  const auto options_error = validate_coverage_grid_options(options_);
  if (!options_error.empty()) {
    throw std::invalid_argument(options_error);
  }

  if (occupancy_.size() != metadata_.occupancy_data_size) {
    throw std::invalid_argument("coverage_grid_data_size_mismatch");
  }

  if (std::any_of(
      occupancy_.begin(), occupancy_.end(),
      [](std::int8_t value) {return value < -1 || value > 100;}))
  {
    throw std::invalid_argument("coverage_grid_occupancy_value_invalid");
  }

  build_cell_classification();
  build_inflation_mask();
}

const CoverageGridMetadata & CoverageGrid::metadata() const
{
  return metadata_;
}

const CoverageGridOptions & CoverageGrid::options() const
{
  return options_;
}

const std::vector<std::int8_t> & CoverageGrid::occupancy() const
{
  return occupancy_;
}

std::size_t CoverageGrid::size() const
{
  return occupancy_.size();
}

std::optional<std::size_t> CoverageGrid::linear_index(
  GridIndex index) const
{
  if (index.column >= metadata_.width || index.row >= metadata_.height) {
    return std::nullopt;
  }
  return index.row * metadata_.width + index.column;
}

std::optional<GridIndex> CoverageGrid::grid_index(
  std::size_t linear_index_value) const
{
  if (linear_index_value >= size()) {
    return std::nullopt;
  }
  return GridIndex{
    linear_index_value % metadata_.width,
    linear_index_value / metadata_.width};
}

std::optional<WorldPoint> CoverageGrid::grid_to_world(
  GridIndex index) const
{
  if (!linear_index(index)) {
    return std::nullopt;
  }
  return WorldPoint{
    metadata_.origin_x_m +
    (static_cast<double>(index.column) + 0.5) *
    metadata_.resolution_m,
    metadata_.origin_y_m +
    (static_cast<double>(index.row) + 0.5) *
    metadata_.resolution_m};
}

std::optional<GridIndex> CoverageGrid::world_to_grid(
  WorldPoint point) const
{
  if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m)) {
    return std::nullopt;
  }

  const double column =
    (point.x_m - metadata_.origin_x_m) / metadata_.resolution_m;
  const double row =
    (point.y_m - metadata_.origin_y_m) / metadata_.resolution_m;
  if (column < 0.0 || row < 0.0 ||
    column >= static_cast<double>(metadata_.width) ||
    row >= static_cast<double>(metadata_.height))
  {
    return std::nullopt;
  }

  return GridIndex{
    static_cast<std::size_t>(std::floor(column)),
    static_cast<std::size_t>(std::floor(row))};
}

CoverageCell CoverageGrid::cell(GridIndex index) const
{
  const auto linear = linear_index(index);
  if (!linear) {
    throw std::out_of_range("coverage_grid_index_out_of_bounds");
  }
  if (cells_[*linear] == CoverageCell::Free && inflated_[*linear]) {
    return CoverageCell::Inflated;
  }
  return cells_[*linear];
}

bool CoverageGrid::is_inflated(GridIndex index) const
{
  const auto linear = linear_index(index);
  if (!linear) {
    return false;
  }
  return inflated_[*linear];
}

bool CoverageGrid::traversable(GridIndex index) const
{
  const auto linear = linear_index(index);
  if (!linear || inflated_[*linear]) {
    return false;
  }
  return cells_[*linear] == CoverageCell::Free ||
         (cells_[*linear] == CoverageCell::Unknown &&
         options_.allow_unknown);
}

ReachabilityResult CoverageGrid::reachable_from(
  GridIndex start,
  Connectivity connectivity) const
{
  if (!linear_index(start)) {
    return {false, "coverage_grid_start_out_of_bounds", {}};
  }
  if (!traversable(start)) {
    return {false, "coverage_grid_start_blocked", {}};
  }

  std::vector<bool> visited(size(), false);
  std::queue<GridIndex> pending;
  std::vector<GridIndex> reachable;
  pending.push(start);
  visited[*linear_index(start)] = true;

  constexpr int offsets[8][2] = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  const std::size_t offset_count =
    connectivity == Connectivity::Four ? 4U : 8U;

  while (!pending.empty()) {
    const auto current = pending.front();
    pending.pop();
    reachable.push_back(current);

    for (std::size_t offset = 0; offset < offset_count; ++offset) {
      const auto next_column =
        static_cast<std::int64_t>(current.column) + offsets[offset][0];
      const auto next_row =
        static_cast<std::int64_t>(current.row) + offsets[offset][1];
      if (next_column < 0 || next_row < 0) {
        continue;
      }
      const GridIndex next{
        static_cast<std::size_t>(next_column),
        static_cast<std::size_t>(next_row)};
      const auto linear = linear_index(next);
      if (!linear || visited[*linear] || !traversable(next)) {
        continue;
      }
      if (offset >= 4 && !diagonal_step_is_clear(current, next)) {
        continue;
      }
      visited[*linear] = true;
      pending.push(next);
    }
  }

  std::sort(reachable.begin(), reachable.end());
  if (reachable.empty()) {
    return {false, "coverage_grid_no_reachable_space", {}};
  }
  return {true, "", std::move(reachable)};
}

bool CoverageGrid::diagonal_step_is_clear(
  GridIndex from,
  GridIndex to) const
{
  if (from.column == to.column || from.row == to.row) {
    return true;
  }
  return traversable({from.column, to.row}) &&
         traversable({to.column, from.row});
}

void CoverageGrid::build_cell_classification()
{
  cells_.reserve(occupancy_.size());
  for (const auto value : occupancy_) {
    if (value == -1) {
      cells_.push_back(CoverageCell::Unknown);
    } else if (value <= options_.free_threshold) {
      cells_.push_back(CoverageCell::Free);
    } else {
      cells_.push_back(CoverageCell::Occupied);
    }
  }
}

void CoverageGrid::build_inflation_mask()
{
  inflated_.assign(size(), false);
  if (options_.inflation_radius_m <= 0.0) {
    return;
  }

  const auto radius_cells = static_cast<std::int64_t>(
    std::ceil(options_.inflation_radius_m / metadata_.resolution_m));
  for (std::size_t occupied_linear = 0;
    occupied_linear < size(); ++occupied_linear)
  {
    if (cells_[occupied_linear] != CoverageCell::Occupied) {
      continue;
    }
    const auto occupied = *grid_index(occupied_linear);
    for (std::int64_t row_offset = -radius_cells;
      row_offset <= radius_cells; ++row_offset)
    {
      for (std::int64_t column_offset = -radius_cells;
        column_offset <= radius_cells; ++column_offset)
      {
        const double distance = std::hypot(
          static_cast<double>(column_offset),
          static_cast<double>(row_offset)) * metadata_.resolution_m;
        if (distance > options_.inflation_radius_m +
          kGeometryTolerance)
        {
          continue;
        }
        const auto column =
          static_cast<std::int64_t>(occupied.column) + column_offset;
        const auto row =
          static_cast<std::int64_t>(occupied.row) + row_offset;
        if (column < 0 || row < 0) {
          continue;
        }
        const auto candidate = linear_index({
            static_cast<std::size_t>(column),
            static_cast<std::size_t>(row)});
        if (candidate && cells_[*candidate] != CoverageCell::Occupied) {
          inflated_[*candidate] = true;
        }
      }
    }
  }
}

}  // namespace savo_mapping::coverage
