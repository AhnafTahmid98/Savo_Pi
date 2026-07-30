// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace savo_mapping
{

struct SemanticPlanarPose
{
  std::string frame_id{"map"};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SemanticLandmarkDraft
{
  std::string candidate_id{};
  std::string map_id{};
  std::uint32_t map_revision{0U};
  std::string map_release_id{};

  std::string tag_family{"tag36h11"};
  std::int32_t tag_id{-1};
  SemanticPlanarPose tag_pose_map{};

  double detection_quality{0.0};
  std::uint32_t accepted_observations{0U};
  double position_stddev_m{0.0};
  double yaw_stddev_rad{0.0};

  bool approach_pose_valid{false};
  SemanticPlanarPose approach_pose{};

  bool confirmation_pose_valid{false};
  SemanticPlanarPose confirmation_pose{};

  std::string suggested_location_id{};
  std::string suggested_display_name{};
  std::vector<std::string> suggested_aliases{};
  std::string suggested_semantic_type{};

  std::string building{};
  std::string floor{};
  std::string area{};
  std::string notes{};
  std::string source_session_id{};
};

}  // namespace savo_mapping
