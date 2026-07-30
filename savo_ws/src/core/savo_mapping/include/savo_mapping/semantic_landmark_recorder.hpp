// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>

#include "savo_mapping/semantic_landmark.hpp"

namespace savo_mapping
{

enum class SemanticLandmarkValidationCode : std::uint8_t
{
  kValid = 0U,
  kInvalidIdentity,
  kInvalidMapContext,
  kInvalidTag,
  kInvalidEvidence,
  kInvalidTagPose,
  kInvalidApproachPose,
  kDirectTagPoseTarget,
  kInvalidConfirmationPose,
};

struct SemanticLandmarkValidationResult
{
  bool valid{false};
  SemanticLandmarkValidationCode code{
    SemanticLandmarkValidationCode::kInvalidIdentity};
  std::string reason{"not_evaluated"};
};

class SemanticLandmarkRecorder
{
public:
  explicit SemanticLandmarkRecorder(
    double minimum_tag_to_approach_distance_m = 0.20);

  [[nodiscard]] SemanticLandmarkValidationResult Validate(
    const SemanticLandmarkDraft & draft) const;

  [[nodiscard]] double minimum_tag_to_approach_distance_m() const;

private:
  double minimum_tag_to_approach_distance_m_{0.20};
};

}  // namespace savo_mapping
