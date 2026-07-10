#include "savo_mapping/exploration_mode.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(ExplorationModeContract, ConvertsModeToStableString)
{
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::ExplorationMode::Idle)}, "idle");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::ExplorationMode::Frontier)}, "frontier");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::ExplorationMode::Scan360)}, "scan360");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::ExplorationMode::Coverage)}, "coverage");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::ExplorationMode::SemanticReview)}, "semantic_review");
}

TEST(ExplorationModeContract, ParsesStableStringsToModes)
{
  EXPECT_EQ(savo_mapping::exploration_mode_from_string("idle"), savo_mapping::ExplorationMode::Idle);
  EXPECT_EQ(savo_mapping::exploration_mode_from_string("frontier"), savo_mapping::ExplorationMode::Frontier);
  EXPECT_EQ(savo_mapping::exploration_mode_from_string("scan360"), savo_mapping::ExplorationMode::Scan360);
  EXPECT_EQ(savo_mapping::exploration_mode_from_string("coverage"), savo_mapping::ExplorationMode::Coverage);
  EXPECT_EQ(
    savo_mapping::exploration_mode_from_string("semantic_review"),
    savo_mapping::ExplorationMode::SemanticReview);
}

TEST(ExplorationModeContract, RejectsUnknownModeText)
{
  EXPECT_FALSE(savo_mapping::exploration_mode_from_string("").has_value());
  EXPECT_FALSE(savo_mapping::exploration_mode_from_string("manual").has_value());
  EXPECT_FALSE(savo_mapping::exploration_mode_from_string("autonomous").has_value());
  EXPECT_FALSE(savo_mapping::exploration_mode_from_string("navigation").has_value());
  EXPECT_FALSE(savo_mapping::exploration_mode_from_string("slam").has_value());
}

TEST(ExplorationModeContract, ClassifiesNavigationGoalStrategies)
{
  EXPECT_FALSE(savo_mapping::requires_navigation_goal(savo_mapping::ExplorationMode::Idle));
  EXPECT_TRUE(savo_mapping::requires_navigation_goal(savo_mapping::ExplorationMode::Frontier));
  EXPECT_FALSE(savo_mapping::requires_navigation_goal(savo_mapping::ExplorationMode::Scan360));
  EXPECT_TRUE(savo_mapping::requires_navigation_goal(savo_mapping::ExplorationMode::Coverage));
  EXPECT_FALSE(savo_mapping::requires_navigation_goal(savo_mapping::ExplorationMode::SemanticReview));
}

TEST(ExplorationModeContract, ClassifiesRotationCommandStrategies)
{
  EXPECT_FALSE(savo_mapping::requires_rotation_command(savo_mapping::ExplorationMode::Idle));
  EXPECT_FALSE(savo_mapping::requires_rotation_command(savo_mapping::ExplorationMode::Frontier));
  EXPECT_TRUE(savo_mapping::requires_rotation_command(savo_mapping::ExplorationMode::Scan360));
  EXPECT_FALSE(savo_mapping::requires_rotation_command(savo_mapping::ExplorationMode::Coverage));
  EXPECT_FALSE(savo_mapping::requires_rotation_command(savo_mapping::ExplorationMode::SemanticReview));
}

TEST(ExplorationModeContract, ClassifiesSemanticConfirmationStrategies)
{
  EXPECT_FALSE(savo_mapping::requires_semantic_confirmation(savo_mapping::ExplorationMode::Idle));
  EXPECT_FALSE(savo_mapping::requires_semantic_confirmation(savo_mapping::ExplorationMode::Frontier));
  EXPECT_FALSE(savo_mapping::requires_semantic_confirmation(savo_mapping::ExplorationMode::Scan360));
  EXPECT_FALSE(savo_mapping::requires_semantic_confirmation(savo_mapping::ExplorationMode::Coverage));
  EXPECT_TRUE(savo_mapping::requires_semantic_confirmation(savo_mapping::ExplorationMode::SemanticReview));
}

TEST(ExplorationModeContract, ClassifiesActiveExplorationModes)
{
  EXPECT_FALSE(savo_mapping::is_active_exploration_mode(savo_mapping::ExplorationMode::Idle));
  EXPECT_TRUE(savo_mapping::is_active_exploration_mode(savo_mapping::ExplorationMode::Frontier));
  EXPECT_TRUE(savo_mapping::is_active_exploration_mode(savo_mapping::ExplorationMode::Scan360));
  EXPECT_TRUE(savo_mapping::is_active_exploration_mode(savo_mapping::ExplorationMode::Coverage));
  EXPECT_TRUE(savo_mapping::is_active_exploration_mode(savo_mapping::ExplorationMode::SemanticReview));
}
