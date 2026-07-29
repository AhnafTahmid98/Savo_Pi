#include "savo_mapping/mapping_types.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

TEST(MappingTypesContract, Pose2DDefaultsToZero)
{
  const savo_mapping::Pose2D pose;

  EXPECT_DOUBLE_EQ(pose.x_m, 0.0);
  EXPECT_DOUBLE_EQ(pose.y_m, 0.0);
  EXPECT_DOUBLE_EQ(pose.yaw_rad, 0.0);
  EXPECT_TRUE(savo_mapping::is_valid_pose(pose));
}

TEST(MappingTypesContract, Pose2DRejectsNonFiniteValues)
{
  savo_mapping::Pose2D pose;

  pose.x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(savo_mapping::is_valid_pose(pose));

  pose.x_m = 0.0;
  pose.y_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(savo_mapping::is_valid_pose(pose));

  pose.y_m = 0.0;
  pose.yaw_rad = -std::numeric_limits<double>::infinity();
  EXPECT_FALSE(savo_mapping::is_valid_pose(pose));
}

TEST(MappingTypesContract, MapQualityMetricsDefaultsAreSafe)
{
  const savo_mapping::MapQualityMetrics metrics;

  EXPECT_FALSE(metrics.map_received);
  EXPECT_FALSE(metrics.scan_received);
  EXPECT_FALSE(metrics.tf_ok);
  EXPECT_FALSE(metrics.odom_ok);

  EXPECT_DOUBLE_EQ(metrics.quality_score, 0.0);
  EXPECT_DOUBLE_EQ(metrics.free_ratio, 0.0);
  EXPECT_DOUBLE_EQ(metrics.occupied_ratio, 0.0);
  EXPECT_DOUBLE_EQ(metrics.unknown_ratio, 1.0);
  EXPECT_DOUBLE_EQ(metrics.map_area_m2, 0.0);

  EXPECT_EQ(metrics.map_updates, 0u);
  EXPECT_EQ(metrics.scan_updates, 0u);
}

TEST(MappingTypesContract, QualityScoreValidationWorks)
{
  EXPECT_TRUE(savo_mapping::is_quality_score_valid(0.0));
  EXPECT_TRUE(savo_mapping::is_quality_score_valid(0.5));
  EXPECT_TRUE(savo_mapping::is_quality_score_valid(1.0));

  EXPECT_FALSE(savo_mapping::is_quality_score_valid(-0.01));
  EXPECT_FALSE(savo_mapping::is_quality_score_valid(1.01));
  EXPECT_FALSE(savo_mapping::is_quality_score_valid(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(savo_mapping::is_quality_score_valid(std::numeric_limits<double>::infinity()));
}

TEST(MappingTypesContract, NavigationReadyScoreUsesThreshold)
{
  EXPECT_TRUE(savo_mapping::is_quality_score_navigation_ready(0.85, 0.80));
  EXPECT_TRUE(savo_mapping::is_quality_score_navigation_ready(0.80, 0.80));

  EXPECT_FALSE(savo_mapping::is_quality_score_navigation_ready(0.79, 0.80));
  EXPECT_FALSE(savo_mapping::is_quality_score_navigation_ready(-1.0, 0.80));
  EXPECT_FALSE(savo_mapping::is_quality_score_navigation_ready(0.90, 1.5));
}

TEST(MappingTypesContract, SavedMapInfoRequiresCorePaths)
{
  savo_mapping::SavedMapInfo info;

  EXPECT_FALSE(savo_mapping::has_saved_map_paths(info));

  info.map_name = "savonia_floor_1";
  info.map_yaml_path = "/home/savo/Savo_Pi/maps/saved/savonia_floor_1/map.yaml";
  info.map_image_path = "/home/savo/Savo_Pi/maps/saved/savonia_floor_1/map.pgm";

  EXPECT_TRUE(savo_mapping::has_saved_map_paths(info));
}

TEST(MappingTypesContract, SemanticLandmarkObservationValidationWorks)
{
  savo_mapping::SemanticLandmarkObservation observation;

  EXPECT_FALSE(savo_mapping::is_semantic_observation_valid(observation));

  observation.label = "A201";
  observation.source = "savo_head";
  observation.map_pose.x_m = 4.2;
  observation.map_pose.y_m = 1.8;
  observation.map_pose.yaw_rad = 1.5708;
  observation.confidence = 0.95;

  EXPECT_TRUE(savo_mapping::is_semantic_observation_valid(observation));
}

TEST(MappingTypesContract, SemanticLandmarkRejectsInvalidConfidence)
{
  savo_mapping::SemanticLandmarkObservation observation;
  observation.label = "A201";
  observation.source = "savo_head";
  observation.map_pose.x_m = 4.2;
  observation.map_pose.y_m = 1.8;
  observation.map_pose.yaw_rad = 1.5708;
  observation.confidence = 1.5;

  EXPECT_FALSE(savo_mapping::is_semantic_observation_valid(observation));
}
