#include "savo_mapping/frontier_detector.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace
{

using savo_mapping::frontier::FrontierDetector;
using savo_mapping::frontier::FrontierDetectorConfig;
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
  grid.cells.assign(width * height, value);
  return grid;
}

void set_cell(
  OccupancyGrid & grid,
  std::size_t x,
  std::size_t y,
  std::int8_t value)
{
  grid.cells.at(y * grid.width + x) = value;
}

}  // namespace

TEST(FrontierDetector, RejectsMalformedGrid)
{
  OccupancyGrid grid;
  grid.width = 2;
  grid.height = 2;
  grid.resolution_m = 1.0;
  grid.cells = {0, 0, 0};

  const FrontierDetector detector;

  EXPECT_THROW(detector.detect(grid), std::invalid_argument);
}

TEST(FrontierDetector, RejectsInvalidConfiguration)
{
  FrontierDetectorConfig config;
  config.minimum_cluster_size = 0;

  EXPECT_THROW(
    static_cast<void>(FrontierDetector{config}),
    std::invalid_argument);
}

TEST(FrontierDetector, FullyKnownGridHasNoFrontier)
{
  const OccupancyGrid grid = make_grid(5, 5, 0);
  const FrontierDetector detector;

  EXPECT_TRUE(detector.detect(grid).empty());
}

TEST(FrontierDetector, FullyUnknownGridHasNoFrontier)
{
  const OccupancyGrid grid = make_grid(5, 5, UNKNOWN_CELL);
  const FrontierDetector detector;

  EXPECT_TRUE(detector.detect(grid).empty());
}

TEST(FrontierDetector, DetectsUnknownCellsAdjacentToFreeSpace)
{
  OccupancyGrid grid = make_grid(5, 5, UNKNOWN_CELL);
  set_cell(grid, 2, 2, 0);

  const FrontierDetector detector;
  const auto frontiers = detector.detect(grid);

  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers.front().cells.size(), 4u);
  EXPECT_EQ(frontiers.front().representative, (GridCell{2, 1}));
  EXPECT_DOUBLE_EQ(frontiers.front().centroid_x_m, 2.5);
  EXPECT_DOUBLE_EQ(frontiers.front().centroid_y_m, 2.5);
  EXPECT_DOUBLE_EQ(frontiers.front().information_gain_m2, 4.0);
}

TEST(FrontierDetector, UsesOriginAndResolutionForMetricOutput)
{
  OccupancyGrid grid = make_grid(3, 3, UNKNOWN_CELL);
  grid.resolution_m = 0.5;
  grid.origin_x_m = -2.0;
  grid.origin_y_m = 3.0;
  set_cell(grid, 1, 1, 0);

  const FrontierDetector detector;
  const auto frontiers = detector.detect(grid);

  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_DOUBLE_EQ(frontiers.front().centroid_x_m, -1.25);
  EXPECT_DOUBLE_EQ(frontiers.front().centroid_y_m, 3.75);
  EXPECT_DOUBLE_EQ(frontiers.front().information_gain_m2, 1.0);
}

TEST(FrontierDetector, MinimumClusterSizeFiltersNoise)
{
  OccupancyGrid grid = make_grid(3, 3, UNKNOWN_CELL);
  set_cell(grid, 1, 1, 0);

  FrontierDetectorConfig config;
  config.minimum_cluster_size = 5;

  const FrontierDetector detector(config);

  EXPECT_TRUE(detector.detect(grid).empty());
}

TEST(FrontierDetector, CardinalClusteringSeparatesDiagonalCells)
{
  OccupancyGrid grid = make_grid(3, 3, 100);
  set_cell(grid, 0, 0, 0);
  set_cell(grid, 1, 0, UNKNOWN_CELL);
  set_cell(grid, 0, 1, UNKNOWN_CELL);

  FrontierDetectorConfig config;
  config.cluster_diagonally = false;

  const FrontierDetector detector(config);
  const auto frontiers = detector.detect(grid);

  ASSERT_EQ(frontiers.size(), 2u);
  EXPECT_EQ(frontiers.at(0).representative, (GridCell{1, 0}));
  EXPECT_EQ(frontiers.at(1).representative, (GridCell{0, 1}));
}

TEST(FrontierDetector, DiagonalClusteringCombinesDiagonalCells)
{
  OccupancyGrid grid = make_grid(3, 3, 100);
  set_cell(grid, 0, 0, 0);
  set_cell(grid, 1, 0, UNKNOWN_CELL);
  set_cell(grid, 0, 1, UNKNOWN_CELL);

  const FrontierDetector detector;
  const auto frontiers = detector.detect(grid);

  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers.front().cells.size(), 2u);
}

TEST(FrontierDetector, FreeThresholdIsApplied)
{
  OccupancyGrid grid = make_grid(3, 3, 100);
  set_cell(grid, 1, 1, 20);
  set_cell(grid, 1, 0, UNKNOWN_CELL);

  const FrontierDetector default_detector;
  EXPECT_TRUE(default_detector.detect(grid).empty());

  FrontierDetectorConfig config;
  config.free_threshold = 20;

  const FrontierDetector threshold_detector(config);
  const auto frontiers = threshold_detector.detect(grid);

  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers.front().representative, (GridCell{1, 0}));
}

TEST(FrontierDetector, ReturnsClustersInDeterministicRowMajorOrder)
{
  OccupancyGrid grid = make_grid(7, 3, 100);

  set_cell(grid, 1, 1, 0);
  set_cell(grid, 1, 0, UNKNOWN_CELL);
  set_cell(grid, 0, 1, UNKNOWN_CELL);
  set_cell(grid, 2, 1, UNKNOWN_CELL);
  set_cell(grid, 1, 2, UNKNOWN_CELL);

  set_cell(grid, 5, 1, 0);
  set_cell(grid, 5, 0, UNKNOWN_CELL);
  set_cell(grid, 4, 1, UNKNOWN_CELL);
  set_cell(grid, 6, 1, UNKNOWN_CELL);
  set_cell(grid, 5, 2, UNKNOWN_CELL);

  const FrontierDetector detector;
  const auto frontiers = detector.detect(grid);

  ASSERT_EQ(frontiers.size(), 2u);
  EXPECT_EQ(frontiers.at(0).representative, (GridCell{1, 0}));
  EXPECT_EQ(frontiers.at(1).representative, (GridCell{5, 0}));
}
