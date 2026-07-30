// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>

namespace savo_nav
{

struct LocationPlanarPose
{
  std::string frame_id{"map"};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool valid{false};
};

struct LocationRecordView
{
  bool approved{false};
  bool enabled{false};
  bool retired{false};
  std::string location_id{};
  std::string map_id{};
  std::uint32_t map_revision{0U};
  LocationPlanarPose approach_pose{};
  LocationPlanarPose tag_pose_map{};
  bool tag_pose_map_valid{false};
  bool arrival_confirmation_required{false};
};

struct LocationNavigationRequestView
{
  std::string query{};
  bool enforce_map_context{false};
  std::string map_id{};
  std::uint32_t map_revision{0U};
  bool require_arrival_confirmation{false};
};

enum class LocationTargetCode : std::uint8_t
{
  kAccepted = 0U,
  kInvalidRequest,
  kNotApproved,
  kDisabled,
  kRetired,
  kMapMismatch,
  kInvalidApproachPose,
  kDirectTagPoseTarget,
};

enum class LocationTargetSource : std::uint8_t
{
  kNone = 0U,
  kApproachPose = 1U,
};

struct LocationTargetDecision
{
  bool accepted{false};
  LocationTargetCode code{LocationTargetCode::kInvalidRequest};
  std::string reason{"not_evaluated"};
  LocationTargetSource target_source{LocationTargetSource::kNone};
  LocationPlanarPose target{};
  bool arrival_confirmation_required{false};
};

class LocationNavigationContract
{
public:
  explicit LocationNavigationContract(
    double direct_tag_pose_epsilon_m = 0.05);

  [[nodiscard]] LocationTargetDecision SelectTarget(
    const LocationNavigationRequestView & request,
    const LocationRecordView & record) const;

private:
  double direct_tag_pose_epsilon_m_{0.05};
};

}  // namespace savo_nav
