#include "savo_mapping/simulated_scan_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

TEST(SimulatedScanModel, DefaultConfigurationIsValid)
{
  const savo_mapping::simulation::SimulatedRoomScanConfig config;

  EXPECT_TRUE(
    savo_mapping::simulation::
    validate_simulated_room_scan_config(config).empty());
}

TEST(SimulatedScanModel, GeneratesFullFiniteScan)
{
  const savo_mapping::simulation::SimulatedRoomScanConfig config;

  const auto ranges =
    savo_mapping::simulation::
    generate_rectangular_room_scan(config);

  ASSERT_EQ(ranges.size(), config.sample_count);

  for (const float range : ranges) {
    EXPECT_TRUE(std::isfinite(range));
    EXPECT_GE(range, config.range_min_m);
    EXPECT_LE(range, config.range_max_m);
  }
}

TEST(SimulatedScanModel, CardinalRangesMatchRoomWalls)
{
  const savo_mapping::simulation::SimulatedRoomScanConfig config;

  const auto ranges =
    savo_mapping::simulation::
    generate_rectangular_room_scan(config);

  ASSERT_EQ(ranges.size(), 360u);

  EXPECT_NEAR(ranges.at(0), 2.50, 1.0e-4);
  EXPECT_NEAR(ranges.at(90), 1.75, 1.0e-4);
  EXPECT_NEAR(ranges.at(180), 2.50, 1.0e-4);
  EXPECT_NEAR(ranges.at(270), 1.75, 1.0e-4);
}

TEST(SimulatedScanModel, DiagonalRangeHitsNearestWall)
{
  const savo_mapping::simulation::SimulatedRoomScanConfig config;

  const auto ranges =
    savo_mapping::simulation::
    generate_rectangular_room_scan(config);

  const double expected =
    config.half_height_m /
    std::sin(savo_mapping::simulation::PI / 4.0);

  EXPECT_NEAR(ranges.at(225), expected, 1.0e-4);
}

TEST(SimulatedScanModel, AngleContractCoversOneRotation)
{
  EXPECT_NEAR(
    savo_mapping::simulation::scan_angle_min_rad(),
    -savo_mapping::simulation::PI,
    1.0e-12);

  EXPECT_NEAR(
    savo_mapping::simulation::
    scan_angle_increment_rad(360),
    2.0 * savo_mapping::simulation::PI / 360.0,
    1.0e-12);

  EXPECT_LT(
    savo_mapping::simulation::
    scan_angle_max_rad(360),
    savo_mapping::simulation::PI);
}

TEST(SimulatedScanModel, RejectsUnsafeConfiguration)
{
  savo_mapping::simulation::SimulatedRoomScanConfig config;

  config.sample_count = 0;

  EXPECT_EQ(
    savo_mapping::simulation::
    validate_simulated_room_scan_config(config),
    "sample_count_outside_supported_range");

  config = {};
  config.range_max_m = config.range_min_m;

  EXPECT_EQ(
    savo_mapping::simulation::
    validate_simulated_room_scan_config(config),
    "range_max_m_must_exceed_range_min_m");

  config = {};
  config.half_width_m = 0.0;

  EXPECT_EQ(
    savo_mapping::simulation::
    validate_simulated_room_scan_config(config),
    "half_width_m_must_be_positive");
}

TEST(SimulatedScanModel, InvalidGenerationThrows)
{
  savo_mapping::simulation::SimulatedRoomScanConfig config;
  config.half_height_m = -1.0;

  EXPECT_THROW(
    savo_mapping::simulation::
    generate_rectangular_room_scan(config),
    std::invalid_argument);
}
