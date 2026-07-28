#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace savo_mapping::coverage
{

struct GridIndex
{
  std::size_t column{0};
  std::size_t row{0};
};

bool operator==(const GridIndex & lhs, const GridIndex & rhs);
bool operator!=(const GridIndex & lhs, const GridIndex & rhs);
bool operator<(const GridIndex & lhs, const GridIndex & rhs);

struct WorldPoint
{
  double x_m{0.0};
  double y_m{0.0};
};

enum class CoverageCell
{
  Free,
  Occupied,
  Unknown,
  Inflated,
  Unreachable,
};

enum class Connectivity
{
  Four,
  Eight,
};

struct CoverageGridMetadata
{
  std::size_t width{0};
  std::size_t height{0};
  double resolution_m{0.0};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  double origin_yaw_rad{0.0};
  std::size_t occupancy_data_size{0};
};

struct CoverageGridOptions
{
  std::int8_t free_threshold{0};
  std::int8_t occupied_threshold{65};
  bool allow_unknown{false};
  double inflation_radius_m{0.0};
};

std::string validate_coverage_grid_metadata(
  const CoverageGridMetadata & metadata);

std::string validate_coverage_grid_options(
  const CoverageGridOptions & options);

struct ReachabilityResult
{
  bool valid{false};
  std::string reason;
  std::vector<GridIndex> cells;
};

class CoverageGrid
{
public:
  CoverageGrid(
    CoverageGridMetadata metadata,
    std::vector<std::int8_t> occupancy,
    CoverageGridOptions options = {});

  const CoverageGridMetadata & metadata() const;
  const CoverageGridOptions & options() const;
  const std::vector<std::int8_t> & occupancy() const;
  std::size_t size() const;

  std::optional<std::size_t> linear_index(GridIndex index) const;
  std::optional<GridIndex> grid_index(std::size_t linear_index) const;
  std::optional<WorldPoint> grid_to_world(GridIndex index) const;
  std::optional<GridIndex> world_to_grid(WorldPoint point) const;

  CoverageCell cell(GridIndex index) const;
  bool is_inflated(GridIndex index) const;
  bool traversable(GridIndex index) const;

  ReachabilityResult reachable_from(
    GridIndex start,
    Connectivity connectivity = Connectivity::Four) const;

private:
  bool diagonal_step_is_clear(
    GridIndex from,
    GridIndex to) const;
  void build_cell_classification();
  void build_inflation_mask();

  CoverageGridMetadata metadata_;
  std::vector<std::int8_t> occupancy_;
  CoverageGridOptions options_;
  std::vector<CoverageCell> cells_;
  std::vector<bool> inflated_;
};

}  // namespace savo_mapping::coverage
