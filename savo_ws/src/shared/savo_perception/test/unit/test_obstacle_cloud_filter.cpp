// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_perception/obstacle_cloud_filter.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using savo_perception::ObstacleCloudFilterConfig;
using savo_perception::PointXYZ;

savo_perception::ObstacleCloudFilterResult filter(
  const std::vector<PointXYZ> & points,
  ObstacleCloudFilterConfig config = {})
{
  config.self_filter_enabled = false;

  return savo_perception::filter_obstacle_cloud(
    points,
    config);
}

}  // namespace

TEST(ObstacleCloudFilterConfigTest, ValidConfigPasses)
{
  EXPECT_TRUE(
    savo_perception::
    validate_obstacle_cloud_filter_config(
      ObstacleCloudFilterConfig{}).empty());
}

TEST(ObstacleCloudFilterConfigTest, NonFiniteConfigIsRejected)
{
  ObstacleCloudFilterConfig config;
  config.max_range_m =
    std::numeric_limits<double>::infinity();

  EXPECT_EQ(
    savo_perception::
    validate_obstacle_cloud_filter_config(config),
    "non_finite_configuration");
}

TEST(ObstacleCloudFilterConfigTest, NegativeMinimumRangeIsRejected)
{
  ObstacleCloudFilterConfig config;
  config.min_range_m = -0.1;
  EXPECT_EQ(
    savo_perception::
    validate_obstacle_cloud_filter_config(config),
    "negative_minimum_range");
}

TEST(ObstacleCloudFilterConfigTest, ReversedRangeLimitsAreRejected)
{
  ObstacleCloudFilterConfig config;
  config.max_range_m = config.min_range_m;
  EXPECT_EQ(
    savo_perception::
    validate_obstacle_cloud_filter_config(config),
    "invalid_range_bounds");
}

TEST(ObstacleCloudFilterConfigTest, ReversedHeightLimitsAreRejected)
{
  ObstacleCloudFilterConfig config;
  config.max_height_m = config.min_height_m;
  EXPECT_EQ(
    savo_perception::
    validate_obstacle_cloud_filter_config(config),
    "invalid_height_bounds");
}

TEST(ObstacleCloudFilterConfigTest, InvalidVoxelSizeIsRejected)
{
  ObstacleCloudFilterConfig config;
  config.voxel_size_m = 0.0;
  EXPECT_EQ(
    savo_perception::
    validate_obstacle_cloud_filter_config(config),
    "invalid_voxel_size");
}

TEST(ObstacleCloudFilterConfigTest, InvalidSelfFilterBoundsAreRejected)
{
  ObstacleCloudFilterConfig config;
  config.self_max_x_m = config.self_min_x_m;
  EXPECT_EQ(
    savo_perception::
    validate_obstacle_cloud_filter_config(config),
    "invalid_self_filter_bounds");
}

TEST(ObstacleCloudFilterConfigTest, ZeroMaximumOutputIsRejected)
{
  ObstacleCloudFilterConfig config;
  config.max_output_points = 0U;
  EXPECT_EQ(
    savo_perception::
    validate_obstacle_cloud_filter_config(config),
    "zero_maximum_output_points");
}

TEST(ObstacleCloudFilterPointTest, NanPointIsRejected)
{
  const auto result = filter(
    {{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.2}});
  EXPECT_TRUE(result.points.empty());
  EXPECT_EQ(result.stats.finite_points, 0U);
}

TEST(ObstacleCloudFilterPointTest, PositiveInfinityPointIsRejected)
{
  const auto result = filter(
    {{std::numeric_limits<double>::infinity(), 0.0, 0.2}});
  EXPECT_TRUE(result.points.empty());
}

TEST(ObstacleCloudFilterPointTest, NegativeInfinityPointIsRejected)
{
  const auto result = filter(
    {{-std::numeric_limits<double>::infinity(), 0.0, 0.2}});
  EXPECT_TRUE(result.points.empty());
}

TEST(ObstacleCloudFilterPointTest, PointBelowMinimumRangeIsRejected)
{
  const auto result = filter({{0.1, 0.0, 0.2}});
  EXPECT_TRUE(result.points.empty());
  EXPECT_EQ(result.stats.range_rejected, 1U);
}

TEST(ObstacleCloudFilterPointTest, PointAboveMaximumRangeIsRejected)
{
  const auto result = filter({{3.1, 0.0, 0.2}});
  EXPECT_TRUE(result.points.empty());
  EXPECT_EQ(result.stats.range_rejected, 1U);
}

TEST(ObstacleCloudFilterPointTest, FloorPointIsRejected)
{
  const auto result = filter({{1.0, 0.0, 0.01}});
  EXPECT_TRUE(result.points.empty());
  EXPECT_EQ(result.stats.height_rejected, 1U);
}

TEST(ObstacleCloudFilterPointTest, OverHeightPointIsRejected)
{
  const auto result = filter({{1.0, 0.0, 1.7}});
  EXPECT_TRUE(result.points.empty());
  EXPECT_EQ(result.stats.height_rejected, 1U);
}

TEST(ObstacleCloudFilterPointTest, RobotSelfVolumePointIsRejected)
{
  ObstacleCloudFilterConfig config;
  const auto result =
    savo_perception::filter_obstacle_cloud(
    {{0.25, 0.0, 0.2}},
    config);
  EXPECT_TRUE(result.points.empty());
  EXPECT_EQ(result.stats.self_rejected, 1U);
}

TEST(ObstacleCloudFilterPointTest, ValidObstacleIsRetained)
{
  const auto result = filter({{1.0, 0.2, 0.3}});
  ASSERT_EQ(result.points.size(), 1U);
  EXPECT_DOUBLE_EQ(result.points.front().x, 1.0);
}

TEST(ObstacleCloudFilterPointTest, SameVoxelDuplicateIsRejected)
{
  const auto result =
    filter({{1.01, 0.21, 0.31}, {1.02, 0.22, 0.32}});
  EXPECT_EQ(result.points.size(), 1U);
  EXPECT_EQ(result.stats.voxel_rejected, 1U);
}

TEST(ObstacleCloudFilterPointTest, SeparateVoxelPointsAreRetained)
{
  const auto result =
    filter({{1.00, 0.20, 0.30}, {1.10, 0.20, 0.30}});
  EXPECT_EQ(result.points.size(), 2U);
}

TEST(ObstacleCloudFilterPointTest, FirstPointInVoxelIsRetained)
{
  const auto result =
    filter({{1.01, 0.21, 0.31}, {1.02, 0.22, 0.32}});
  ASSERT_EQ(result.points.size(), 1U);
  EXPECT_DOUBLE_EQ(result.points.front().x, 1.01);
  EXPECT_DOUBLE_EQ(result.points.front().y, 0.21);
}

TEST(ObstacleCloudFilterPointTest, MaximumOutputLimitIsEnforced)
{
  ObstacleCloudFilterConfig config;
  config.self_filter_enabled = false;
  config.max_output_points = 1U;

  const auto result =
    savo_perception::filter_obstacle_cloud(
    {{1.0, 0.0, 0.2}, {1.1, 0.0, 0.2}},
    config);

  EXPECT_EQ(result.points.size(), 1U);
  EXPECT_TRUE(result.stats.output_limited);
}

TEST(ObstacleCloudFilterPointTest, StatisticsCountsAreCorrect)
{
  ObstacleCloudFilterConfig config;
  config.self_filter_enabled = true;

  const auto result =
    savo_perception::filter_obstacle_cloud(
  {
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.2},
    {0.1, 0.0, 0.2},
    {1.0, 0.0, 0.01},
    {0.25, 0.0, 0.2},
    {1.0, 0.0, 0.2},
    {1.01, 0.0, 0.2}},
    config);

  EXPECT_EQ(result.stats.input_points, 6U);
  EXPECT_EQ(result.stats.finite_points, 5U);
  EXPECT_EQ(result.stats.range_rejected, 1U);
  EXPECT_EQ(result.stats.height_rejected, 1U);
  EXPECT_EQ(result.stats.self_rejected, 1U);
  EXPECT_EQ(result.stats.voxel_rejected, 1U);
  EXPECT_EQ(result.stats.output_points, 1U);
}

TEST(ObstacleCloudFilterPointTest, EmptyInputProducesEmptyOutput)
{
  const auto result = filter({});
  EXPECT_TRUE(result.points.empty());
  EXPECT_EQ(result.stats.input_points, 0U);
  EXPECT_EQ(result.stats.output_points, 0U);
}

TEST(ObstacleCloudFilterPointTest, DisabledSelfFilterRetainsPoint)
{
  ObstacleCloudFilterConfig config;
  config.self_filter_enabled = false;

  const auto result =
    savo_perception::filter_obstacle_cloud(
    {{0.25, 0.0, 0.2}},
    config);

  EXPECT_EQ(result.points.size(), 1U);
  EXPECT_EQ(result.stats.self_rejected, 0U);
}
