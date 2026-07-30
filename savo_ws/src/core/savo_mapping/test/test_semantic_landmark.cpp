// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_mapping/semantic_landmark_recorder.hpp"

namespace
{

savo_mapping::SemanticLandmarkDraft valid_draft()
{
  savo_mapping::SemanticLandmarkDraft draft;
  draft.candidate_id = "candidate-a201-tag27";
  draft.map_id = "campus_main";
  draft.map_revision = 7U;
  draft.tag_family = "tag36h11";
  draft.tag_id = 27;
  draft.tag_pose_map = {"map", 4.0, 2.0, 0.0};
  draft.detection_quality = 0.95;
  draft.accepted_observations = 8U;
  draft.position_stddev_m = 0.01;
  draft.yaw_stddev_rad = 0.02;
  draft.approach_pose_valid = true;
  draft.approach_pose = {"map", 3.2, 2.0, 0.0};
  draft.confirmation_pose_valid = true;
  draft.confirmation_pose = {"map", 3.5, 2.0, 0.0};
  return draft;
}

TEST(SemanticLandmarkRecorder, AcceptsSafeMappedCandidateDraft)
{
  savo_mapping::SemanticLandmarkRecorder recorder;
  const auto result = recorder.Validate(valid_draft());
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(
    result.code,
    savo_mapping::SemanticLandmarkValidationCode::kValid);
}

TEST(SemanticLandmarkRecorder, RejectsTagPoseAsApproachPose)
{
  savo_mapping::SemanticLandmarkRecorder recorder;
  auto draft = valid_draft();
  draft.approach_pose = draft.tag_pose_map;

  const auto result = recorder.Validate(draft);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.code,
    savo_mapping::SemanticLandmarkValidationCode::kDirectTagPoseTarget);
}

TEST(SemanticLandmarkRecorder, AllowsPendingCandidateWithoutApproachPose)
{
  savo_mapping::SemanticLandmarkRecorder recorder;
  auto draft = valid_draft();
  draft.approach_pose_valid = false;

  const auto result = recorder.Validate(draft);
  EXPECT_TRUE(result.valid);
}

}  // namespace
