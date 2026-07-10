#include "savo_mapping/workflow_phase.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(WorkflowPhaseContract, ConvertsPhaseToStableString)
{
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Idle)}, "idle");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Preflight)}, "preflight");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::StartingSlam)}, "starting_slam");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Mapping)}, "mapping");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Exploring)}, "exploring");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Scan360)}, "scan360");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Coverage)}, "coverage");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::SemanticReview)}, "semantic_review");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Saving)}, "saving");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Validating)}, "validating");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Complete)}, "complete");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::WorkflowPhase::Error)}, "error");
}

TEST(WorkflowPhaseContract, ParsesStableStringsToPhases)
{
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("idle"), savo_mapping::WorkflowPhase::Idle);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("preflight"), savo_mapping::WorkflowPhase::Preflight);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("starting_slam"), savo_mapping::WorkflowPhase::StartingSlam);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("mapping"), savo_mapping::WorkflowPhase::Mapping);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("exploring"), savo_mapping::WorkflowPhase::Exploring);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("scan360"), savo_mapping::WorkflowPhase::Scan360);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("coverage"), savo_mapping::WorkflowPhase::Coverage);
  EXPECT_EQ(
    savo_mapping::workflow_phase_from_string("semantic_review"),
    savo_mapping::WorkflowPhase::SemanticReview);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("saving"), savo_mapping::WorkflowPhase::Saving);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("validating"), savo_mapping::WorkflowPhase::Validating);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("complete"), savo_mapping::WorkflowPhase::Complete);
  EXPECT_EQ(savo_mapping::workflow_phase_from_string("error"), savo_mapping::WorkflowPhase::Error);
}

TEST(WorkflowPhaseContract, RejectsUnknownPhaseText)
{
  EXPECT_FALSE(savo_mapping::workflow_phase_from_string("").has_value());
  EXPECT_FALSE(savo_mapping::workflow_phase_from_string("manual").has_value());
  EXPECT_FALSE(savo_mapping::workflow_phase_from_string("frontier").has_value());
  EXPECT_FALSE(savo_mapping::workflow_phase_from_string("navigation").has_value());
  EXPECT_FALSE(savo_mapping::workflow_phase_from_string("slam").has_value());
}

TEST(WorkflowPhaseContract, ClassifiesTerminalPhases)
{
  EXPECT_FALSE(savo_mapping::is_terminal_phase(savo_mapping::WorkflowPhase::Idle));
  EXPECT_FALSE(savo_mapping::is_terminal_phase(savo_mapping::WorkflowPhase::Mapping));
  EXPECT_FALSE(savo_mapping::is_terminal_phase(savo_mapping::WorkflowPhase::Saving));
  EXPECT_TRUE(savo_mapping::is_terminal_phase(savo_mapping::WorkflowPhase::Complete));
  EXPECT_TRUE(savo_mapping::is_terminal_phase(savo_mapping::WorkflowPhase::Error));
}

TEST(WorkflowPhaseContract, ClassifiesActivePhases)
{
  EXPECT_FALSE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Idle));
  EXPECT_FALSE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Preflight));
  EXPECT_FALSE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::StartingSlam));

  EXPECT_TRUE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Mapping));
  EXPECT_TRUE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Exploring));
  EXPECT_TRUE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Scan360));
  EXPECT_TRUE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Coverage));
  EXPECT_TRUE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::SemanticReview));

  EXPECT_FALSE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Saving));
  EXPECT_FALSE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Validating));
  EXPECT_FALSE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Complete));
  EXPECT_FALSE(savo_mapping::is_active_phase(savo_mapping::WorkflowPhase::Error));
}

TEST(WorkflowPhaseContract, ClassifiesMotionRelatedPhases)
{
  EXPECT_FALSE(savo_mapping::is_motion_related_phase(savo_mapping::WorkflowPhase::Idle));
  EXPECT_FALSE(savo_mapping::is_motion_related_phase(savo_mapping::WorkflowPhase::Mapping));
  EXPECT_TRUE(savo_mapping::is_motion_related_phase(savo_mapping::WorkflowPhase::Exploring));
  EXPECT_TRUE(savo_mapping::is_motion_related_phase(savo_mapping::WorkflowPhase::Scan360));
  EXPECT_TRUE(savo_mapping::is_motion_related_phase(savo_mapping::WorkflowPhase::Coverage));
  EXPECT_FALSE(savo_mapping::is_motion_related_phase(savo_mapping::WorkflowPhase::SemanticReview));
  EXPECT_FALSE(savo_mapping::is_motion_related_phase(savo_mapping::WorkflowPhase::Saving));
}

TEST(WorkflowPhaseContract, ClassifiesMapOutputPhases)
{
  EXPECT_FALSE(savo_mapping::is_map_output_phase(savo_mapping::WorkflowPhase::Idle));
  EXPECT_FALSE(savo_mapping::is_map_output_phase(savo_mapping::WorkflowPhase::Mapping));
  EXPECT_FALSE(savo_mapping::is_map_output_phase(savo_mapping::WorkflowPhase::Exploring));

  EXPECT_TRUE(savo_mapping::is_map_output_phase(savo_mapping::WorkflowPhase::Saving));
  EXPECT_TRUE(savo_mapping::is_map_output_phase(savo_mapping::WorkflowPhase::Validating));
  EXPECT_TRUE(savo_mapping::is_map_output_phase(savo_mapping::WorkflowPhase::Complete));

  EXPECT_FALSE(savo_mapping::is_map_output_phase(savo_mapping::WorkflowPhase::Error));
}
