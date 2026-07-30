// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_mapping/semantic_landmark_recorder.hpp"

#include <cmath>
#include <utility>

namespace savo_mapping
{
namespace
{

bool finite_pose(const SemanticPlanarPose & pose)
{
  return !pose.frame_id.empty() &&
         std::isfinite(pose.x) &&
         std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

SemanticLandmarkValidationResult reject(
  const SemanticLandmarkValidationCode code,
  std::string reason)
{
  return {false, code, std::move(reason)};
}

}  // namespace

SemanticLandmarkRecorder::SemanticLandmarkRecorder(
  const double minimum_tag_to_approach_distance_m)
: minimum_tag_to_approach_distance_m_(minimum_tag_to_approach_distance_m)
{
  if (!std::isfinite(minimum_tag_to_approach_distance_m_) ||
    minimum_tag_to_approach_distance_m_ < 0.0)
  {
    minimum_tag_to_approach_distance_m_ = 0.20;
  }
}

SemanticLandmarkValidationResult SemanticLandmarkRecorder::Validate(
  const SemanticLandmarkDraft & draft) const
{
  if (draft.candidate_id.empty()) {
    return reject(
      SemanticLandmarkValidationCode::kInvalidIdentity,
      "candidate_id_required");
  }

  if (draft.map_id.empty() || draft.map_revision == 0U) {
    return reject(
      SemanticLandmarkValidationCode::kInvalidMapContext,
      "map_id_and_revision_required");
  }

  if (draft.tag_family.empty() || draft.tag_id < 0) {
    return reject(
      SemanticLandmarkValidationCode::kInvalidTag,
      "valid_tag_identity_required");
  }

  if (!std::isfinite(draft.detection_quality) ||
    draft.detection_quality < 0.0 ||
    draft.detection_quality > 1.0 ||
    draft.accepted_observations == 0U ||
    !std::isfinite(draft.position_stddev_m) ||
    draft.position_stddev_m < 0.0 ||
    !std::isfinite(draft.yaw_stddev_rad) ||
    draft.yaw_stddev_rad < 0.0)
  {
    return reject(
      SemanticLandmarkValidationCode::kInvalidEvidence,
      "stable_detection_evidence_required");
  }

  if (!finite_pose(draft.tag_pose_map) ||
    draft.tag_pose_map.frame_id != "map")
  {
    return reject(
      SemanticLandmarkValidationCode::kInvalidTagPose,
      "tag_pose_map_must_be_finite_in_map_frame");
  }

  if (draft.approach_pose_valid) {
    if (!finite_pose(draft.approach_pose) ||
      draft.approach_pose.frame_id != draft.tag_pose_map.frame_id)
    {
      return reject(
        SemanticLandmarkValidationCode::kInvalidApproachPose,
        "approach_pose_must_be_finite_in_map_frame");
    }

    const double distance = std::hypot(
      draft.approach_pose.x - draft.tag_pose_map.x,
      draft.approach_pose.y - draft.tag_pose_map.y);

    if (distance < minimum_tag_to_approach_distance_m_) {
      return reject(
        SemanticLandmarkValidationCode::kDirectTagPoseTarget,
        "approach_pose_must_not_be_tag_pose_map");
    }
  }

  if (draft.confirmation_pose_valid &&
    (!finite_pose(draft.confirmation_pose) ||
    draft.confirmation_pose.frame_id != draft.tag_pose_map.frame_id))
  {
    return reject(
      SemanticLandmarkValidationCode::kInvalidConfirmationPose,
      "confirmation_pose_must_be_finite_in_map_frame");
  }

  return {
    true,
    SemanticLandmarkValidationCode::kValid,
    "semantic_landmark_draft_valid"};
}

double SemanticLandmarkRecorder::minimum_tag_to_approach_distance_m() const
{
  return minimum_tag_to_approach_distance_m_;
}

}  // namespace savo_mapping
