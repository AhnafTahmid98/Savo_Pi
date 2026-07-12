#include "savo_mapping/saved_map_quality.hpp"

#include <gtest/gtest.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace
{

nav_msgs::msg::OccupancyGrid
make_connected_test_map()
{
  nav_msgs::msg::OccupancyGrid map;

  map.header.frame_id = "map";
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 0.05F;

  map.data.assign(100, 0);

  for (std::size_t x = 0; x < 10; ++x) {
    map.data[x] = 100;
    map.data[90 + x] = 100;
  }

  for (std::size_t y = 0; y < 10; ++y) {
    map.data[y * 10] = 100;
    map.data[(y * 10) + 9] = 100;
  }

  return map;
}

savo_mapping::quality::QualityPolicy
permissive_policy()
{
  savo_mapping::quality::QualityPolicy policy;

  policy.min_width_cells = 5;
  policy.min_height_cells = 5;

  policy.min_known_cells = 20;
  policy.min_free_cells = 10;
  policy.min_occupied_cells = 1;

  policy.min_resolution_m = 0.01;
  policy.max_resolution_m = 0.20;

  policy.min_known_ratio = 0.10;
  policy.min_largest_free_component_ratio =
    0.80;

  return policy;
}

}  // namespace

TEST(SavedMapQuality, ConnectedMapPasses)
{
  const auto evaluation =
    savo_mapping::quality::
    evaluate_occupancy_grid(
      "test_map",
      "map",
      make_connected_test_map(),
      permissive_policy());

  EXPECT_TRUE(evaluation.evaluated);
  EXPECT_TRUE(evaluation.passed);
  EXPECT_EQ(
    evaluation.reason,
    "quality_passed");

  EXPECT_EQ(
    evaluation.metrics.total_cells,
    100U);

  EXPECT_EQ(
    evaluation.metrics.free_cells,
    64U);

  EXPECT_EQ(
    evaluation.metrics.occupied_cells,
    36U);

  EXPECT_DOUBLE_EQ(
    evaluation.metrics
    .largest_free_component_ratio,
    1.0);
}

TEST(SavedMapQuality, DisconnectedFreeSpaceFails)
{
  nav_msgs::msg::OccupancyGrid map;

  map.header.frame_id = "map";
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 0.05F;
  map.data.assign(100, -1);

  map.data[11] = 0;
  map.data[12] = 0;
  map.data[21] = 0;
  map.data[22] = 0;

  map.data[77] = 0;
  map.data[78] = 0;
  map.data[87] = 0;
  map.data[88] = 0;

  map.data[50] = 100;

  auto policy =
    permissive_policy();

  policy.min_known_cells = 1;
  policy.min_free_cells = 1;
  policy.min_known_ratio = 0.01;
  policy.min_largest_free_component_ratio =
    0.75;

  const auto evaluation =
    savo_mapping::quality::
    evaluate_occupancy_grid(
      "test_map",
      "map",
      map,
      policy);

  EXPECT_FALSE(evaluation.passed);
  EXPECT_DOUBLE_EQ(
    evaluation.metrics
    .largest_free_component_ratio,
    0.5);
}

TEST(SavedMapQuality, InvalidDataSizeFails)
{
  nav_msgs::msg::OccupancyGrid map;

  map.header.frame_id = "map";
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 0.05F;
  map.data.assign(50, 0);

  const auto evaluation =
    savo_mapping::quality::
    evaluate_occupancy_grid(
      "test_map",
      "map",
      map,
      permissive_policy());

  EXPECT_FALSE(evaluation.passed);

  ASSERT_EQ(
    evaluation.failed_checks.size(),
    1U);

  EXPECT_EQ(
    evaluation.failed_checks.front(),
    "occupancy_data_size_mismatch");
}
