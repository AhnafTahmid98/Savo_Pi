#include "savo_mapping/coverage_grid.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using savo_mapping::coverage::Connectivity;
using savo_mapping::coverage::CoverageCell;
using savo_mapping::coverage::CoverageGrid;
using savo_mapping::coverage::CoverageGridMetadata;
using savo_mapping::coverage::CoverageGridOptions;
using savo_mapping::coverage::GridIndex;
using savo_mapping::coverage::WorldPoint;

CoverageGridMetadata metadata(
  std::size_t width = 3,
  std::size_t height = 3,
  double resolution = 1.0)
{
  return {width, height, resolution, 0.0, 0.0, 0.0, width * height};
}

std::string exception_message(
  const std::function<void()> & operation)
{
  try {
    operation();
  } catch (const std::invalid_argument & error) {
    return error.what();
  }
  return "";
}

}  // namespace

TEST(CoverageGrid, RejectsZeroWidth)
{
  auto value = metadata();
  value.width = 0;
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_metadata(value),
    "coverage_grid_dimensions_invalid");
}

TEST(CoverageGrid, RejectsZeroHeight)
{
  auto value = metadata();
  value.height = 0;
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_metadata(value),
    "coverage_grid_dimensions_invalid");
}

TEST(CoverageGrid, RejectsDimensionOverflow)
{
  auto value = metadata();
  value.width = std::numeric_limits<std::size_t>::max();
  value.height = 2;
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_metadata(value),
    "coverage_grid_dimensions_invalid");
}

TEST(CoverageGrid, RejectsNonPositiveResolution)
{
  auto value = metadata();
  value.resolution_m = 0.0;
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_metadata(value),
    "coverage_grid_resolution_invalid");
}

TEST(CoverageGrid, RejectsNonFiniteResolution)
{
  auto value = metadata();
  value.resolution_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_metadata(value),
    "coverage_grid_resolution_invalid");
}

TEST(CoverageGrid, RejectsNonFiniteOrigin)
{
  auto value = metadata();
  value.origin_x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_metadata(value),
    "coverage_grid_origin_invalid");
}

TEST(CoverageGrid, RejectsRotatedOrigin)
{
  auto value = metadata();
  value.origin_yaw_rad = 0.1;
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_metadata(value),
    "coverage_grid_origin_yaw_unsupported");
}

TEST(CoverageGrid, RejectsMetadataDataSizeMismatch)
{
  auto value = metadata();
  --value.occupancy_data_size;
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_metadata(value),
    "coverage_grid_data_size_mismatch");
}

TEST(CoverageGrid, RejectsPayloadDataSizeMismatch)
{
  EXPECT_EQ(
    exception_message(
      []() {CoverageGrid grid(metadata(), std::vector<std::int8_t>(8));}),
    "coverage_grid_data_size_mismatch");
}

TEST(CoverageGrid, RejectsInvalidOccupancyValue)
{
  auto data = std::vector<std::int8_t>(9, 0);
  data[2] = -2;
  EXPECT_EQ(
    exception_message([&data]() {CoverageGrid grid(metadata(), data);}),
    "coverage_grid_occupancy_value_invalid");
}

TEST(CoverageGrid, RejectsInvalidThresholdOrdering)
{
  CoverageGridOptions options;
  options.free_threshold = 65;
  options.occupied_threshold = 65;
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_options(options),
    "coverage_grid_thresholds_invalid");
}

TEST(CoverageGrid, RejectsNegativeInflation)
{
  CoverageGridOptions options;
  options.inflation_radius_m = -0.1;
  EXPECT_EQ(
    savo_mapping::coverage::validate_coverage_grid_options(options),
    "coverage_grid_inflation_invalid");
}

TEST(CoverageGrid, ClassifiesFreeOccupiedAndUnknown)
{
  CoverageGrid grid(
    metadata(3, 1), {0, 50, -1},
    CoverageGridOptions{0, 65, false, 0.0});
  EXPECT_EQ(grid.cell({0, 0}), CoverageCell::Free);
  EXPECT_EQ(grid.cell({1, 0}), CoverageCell::Unknown);
  EXPECT_EQ(grid.cell({2, 0}), CoverageCell::Unknown);
}

TEST(CoverageGrid, UnknownIsBlockedByDefault)
{
  CoverageGrid grid(metadata(1, 1), {-1});
  EXPECT_FALSE(grid.traversable({0, 0}));
}

TEST(CoverageGrid, UnknownCanBeExplicitlyAllowed)
{
  auto options = CoverageGridOptions{};
  options.allow_unknown = true;
  CoverageGrid grid(metadata(1, 1), {-1}, options);
  EXPECT_TRUE(grid.traversable({0, 0}));
  EXPECT_EQ(grid.cell({0, 0}), CoverageCell::Unknown);
}

TEST(CoverageGrid, ConvertsGridCentersToWorldCoordinates)
{
  auto value = metadata(3, 2, 0.5);
  value.origin_x_m = -1.0;
  value.origin_y_m = 2.0;
  CoverageGrid grid(value, std::vector<std::int8_t>(6, 0));
  const auto point = grid.grid_to_world({2, 1});
  ASSERT_TRUE(point);
  EXPECT_DOUBLE_EQ(point->x_m, 0.25);
  EXPECT_DOUBLE_EQ(point->y_m, 2.75);
}

TEST(CoverageGrid, ConvertsWorldCoordinatesToGrid)
{
  auto value = metadata(3, 2, 0.5);
  value.origin_x_m = -1.0;
  value.origin_y_m = 2.0;
  CoverageGrid grid(value, std::vector<std::int8_t>(6, 0));
  EXPECT_EQ(grid.world_to_grid({0.24, 2.74}), (GridIndex{2, 1}));
}

TEST(CoverageGrid, RejectsWorldCoordinatesAtUpperBoundary)
{
  CoverageGrid grid(metadata(2, 2), std::vector<std::int8_t>(4, 0));
  EXPECT_FALSE(grid.world_to_grid({2.0, 1.0}));
  EXPECT_FALSE(grid.world_to_grid({1.0, 2.0}));
}

TEST(CoverageGrid, LinearIndexIsRowMajorAndBounded)
{
  CoverageGrid grid(metadata(3, 2), std::vector<std::int8_t>(6, 0));
  EXPECT_EQ(grid.linear_index({2, 1}), 5U);
  EXPECT_EQ(grid.grid_index(4), (GridIndex{1, 1}));
  EXPECT_FALSE(grid.linear_index({3, 0}));
  EXPECT_FALSE(grid.grid_index(6));
}

TEST(CoverageGrid, InflationUsesMetricEuclideanRadius)
{
  CoverageGridOptions options;
  options.inflation_radius_m = 1.0;
  CoverageGrid grid(
    metadata(), {0, 0, 0, 0, 100, 0, 0, 0, 0}, options);
  EXPECT_EQ(grid.cell({1, 0}), CoverageCell::Inflated);
  EXPECT_EQ(grid.cell({0, 0}), CoverageCell::Free);
  EXPECT_FALSE(grid.traversable({1, 0}));
}

TEST(CoverageGrid, ZeroInflationChangesNothing)
{
  CoverageGrid grid(metadata(3, 1), {0, 100, 0});
  EXPECT_EQ(grid.cell({0, 0}), CoverageCell::Free);
  EXPECT_EQ(grid.cell({1, 0}), CoverageCell::Occupied);
  EXPECT_EQ(grid.cell({2, 0}), CoverageCell::Free);
}

TEST(CoverageGrid, InflationIsBoundedAtMapEdges)
{
  CoverageGridOptions options;
  options.inflation_radius_m = 1.0;
  CoverageGrid grid(metadata(2, 2), {100, 0, 0, 0}, options);
  EXPECT_EQ(grid.cell({1, 0}), CoverageCell::Inflated);
  EXPECT_EQ(grid.cell({0, 1}), CoverageCell::Inflated);
  EXPECT_EQ(grid.cell({1, 1}), CoverageCell::Free);
}

TEST(CoverageGrid, UnknownRemainsSemanticallyDistinctWhenInflated)
{
  CoverageGridOptions options;
  options.allow_unknown = true;
  options.inflation_radius_m = 1.0;
  CoverageGrid grid(metadata(3, 1), {100, -1, 0}, options);
  EXPECT_EQ(grid.cell({1, 0}), CoverageCell::Unknown);
  EXPECT_TRUE(grid.is_inflated({1, 0}));
  EXPECT_FALSE(grid.traversable({1, 0}));
}

TEST(CoverageGrid, FourConnectedReachabilityExcludesDiagonalIsland)
{
  CoverageGrid grid(metadata(2, 2), {0, 100, 100, 0});
  const auto result = grid.reachable_from({0, 0}, Connectivity::Four);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.cells, (std::vector<GridIndex>{{0, 0}}));
}

TEST(CoverageGrid, EightConnectivityPreventsCornerCutting)
{
  CoverageGrid grid(metadata(2, 2), {0, 100, 100, 0});
  const auto result = grid.reachable_from({0, 0}, Connectivity::Eight);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.cells, (std::vector<GridIndex>{{0, 0}}));
}

TEST(CoverageGrid, EightConnectivityIncludesClearDiagonal)
{
  CoverageGrid grid(metadata(2, 2), {0, 0, 0, 0});
  const auto result = grid.reachable_from({0, 0}, Connectivity::Eight);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.cells.size(), 4U);
}

TEST(CoverageGrid, ReachabilityExcludesDisconnectedRegion)
{
  CoverageGrid grid(metadata(3, 1), {0, 100, 0});
  const auto result = grid.reachable_from({0, 0});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.cells, (std::vector<GridIndex>{{0, 0}}));
}

TEST(CoverageGrid, ReachabilityOrderingIsDeterministic)
{
  CoverageGrid grid(metadata(), std::vector<std::int8_t>(9, 0));
  const auto first = grid.reachable_from({1, 1});
  const auto second = grid.reachable_from({1, 1});
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(first.cells, second.cells);
}

TEST(CoverageGrid, ReachabilityRejectsInvalidStart)
{
  CoverageGrid grid(metadata(2, 1), {100, 0});
  EXPECT_EQ(
    grid.reachable_from({2, 0}).reason,
    "coverage_grid_start_out_of_bounds");
  EXPECT_EQ(
    grid.reachable_from({0, 0}).reason,
    "coverage_grid_start_blocked");
}

TEST(CoverageGrid, DoesNotMutateSourceOccupancy)
{
  const std::vector<std::int8_t> data{0, 100, -1};
  CoverageGridOptions options;
  options.inflation_radius_m = 1.0;
  CoverageGrid grid(metadata(3, 1), data, options);
  EXPECT_EQ(grid.occupancy(), data);
}

TEST(CoverageGrid, OutOfBoundsCellAccessHasStableFailure)
{
  CoverageGrid grid(metadata(1, 1), {0});
  try {
    static_cast<void>(grid.cell({1, 0}));
    FAIL() << "Expected out_of_range";
  } catch (const std::out_of_range & error) {
    EXPECT_STREQ(error.what(), "coverage_grid_index_out_of_bounds");
  }
}

TEST(CoverageGrid, RejectsNonFiniteWorldCoordinates)
{
  CoverageGrid grid(metadata(1, 1), {0});
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(grid.world_to_grid({nan, 0.5}));
  EXPECT_FALSE(grid.world_to_grid({0.5, nan}));
}
