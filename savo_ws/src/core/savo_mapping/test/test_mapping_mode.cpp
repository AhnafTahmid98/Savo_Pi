#include "savo_mapping/mapping_mode.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

TEST(MappingModeContract, ConvertsModeToStableString)
{
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::MappingMode::MonitorOnly)}, "monitor_only");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::MappingMode::Manual)}, "manual");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::MappingMode::Autonomous)}, "autonomous");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::MappingMode::Frontier)}, "frontier");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::MappingMode::Scan360)}, "scan360");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::MappingMode::Coverage)}, "coverage");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::MappingMode::SemanticReview)}, "semantic_review");
}

TEST(MappingModeContract, ParsesStableStringsToModes)
{
  EXPECT_EQ(savo_mapping::mapping_mode_from_string("monitor_only"), savo_mapping::MappingMode::MonitorOnly);
  EXPECT_EQ(savo_mapping::mapping_mode_from_string("manual"), savo_mapping::MappingMode::Manual);
  EXPECT_EQ(savo_mapping::mapping_mode_from_string("autonomous"), savo_mapping::MappingMode::Autonomous);
  EXPECT_EQ(savo_mapping::mapping_mode_from_string("frontier"), savo_mapping::MappingMode::Frontier);
  EXPECT_EQ(savo_mapping::mapping_mode_from_string("scan360"), savo_mapping::MappingMode::Scan360);
  EXPECT_EQ(savo_mapping::mapping_mode_from_string("coverage"), savo_mapping::MappingMode::Coverage);
  EXPECT_EQ(savo_mapping::mapping_mode_from_string("semantic_review"), savo_mapping::MappingMode::SemanticReview);
}

TEST(MappingModeContract, RejectsUnknownModeText)
{
  EXPECT_FALSE(savo_mapping::mapping_mode_from_string("").has_value());
  EXPECT_FALSE(savo_mapping::mapping_mode_from_string("navigation").has_value());
  EXPECT_FALSE(savo_mapping::mapping_mode_from_string("slam").has_value());
  EXPECT_FALSE(savo_mapping::mapping_mode_from_string("manual_mapping").has_value());
}

TEST(MappingModeContract, ClassifiesAutonomousModes)
{
  EXPECT_FALSE(savo_mapping::is_autonomous_mode(savo_mapping::MappingMode::MonitorOnly));
  EXPECT_FALSE(savo_mapping::is_autonomous_mode(savo_mapping::MappingMode::Manual));
  EXPECT_TRUE(savo_mapping::is_autonomous_mode(savo_mapping::MappingMode::Autonomous));
  EXPECT_TRUE(savo_mapping::is_autonomous_mode(savo_mapping::MappingMode::Frontier));
  EXPECT_TRUE(savo_mapping::is_autonomous_mode(savo_mapping::MappingMode::Scan360));
  EXPECT_TRUE(savo_mapping::is_autonomous_mode(savo_mapping::MappingMode::Coverage));
  EXPECT_FALSE(savo_mapping::is_autonomous_mode(savo_mapping::MappingMode::SemanticReview));
}

TEST(MappingModeContract, ClassifiesMovementRequestingModes)
{
  EXPECT_FALSE(savo_mapping::is_movement_requesting_mode(savo_mapping::MappingMode::MonitorOnly));
  EXPECT_FALSE(savo_mapping::is_movement_requesting_mode(savo_mapping::MappingMode::Manual));
  EXPECT_TRUE(savo_mapping::is_movement_requesting_mode(savo_mapping::MappingMode::Autonomous));
  EXPECT_TRUE(savo_mapping::is_movement_requesting_mode(savo_mapping::MappingMode::Frontier));
  EXPECT_TRUE(savo_mapping::is_movement_requesting_mode(savo_mapping::MappingMode::Scan360));
  EXPECT_TRUE(savo_mapping::is_movement_requesting_mode(savo_mapping::MappingMode::Coverage));
  EXPECT_FALSE(savo_mapping::is_movement_requesting_mode(savo_mapping::MappingMode::SemanticReview));
}

TEST(MappingModeContract, ClassifiesSemanticMode)
{
  EXPECT_FALSE(savo_mapping::is_semantic_mode(savo_mapping::MappingMode::MonitorOnly));
  EXPECT_FALSE(savo_mapping::is_semantic_mode(savo_mapping::MappingMode::Manual));
  EXPECT_FALSE(savo_mapping::is_semantic_mode(savo_mapping::MappingMode::Autonomous));
  EXPECT_FALSE(savo_mapping::is_semantic_mode(savo_mapping::MappingMode::Frontier));
  EXPECT_FALSE(savo_mapping::is_semantic_mode(savo_mapping::MappingMode::Scan360));
  EXPECT_FALSE(savo_mapping::is_semantic_mode(savo_mapping::MappingMode::Coverage));
  EXPECT_TRUE(savo_mapping::is_semantic_mode(savo_mapping::MappingMode::SemanticReview));
}
