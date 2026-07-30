// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/location_navigation_contract.hpp"

#include <cmath>
#include <utility>

namespace savo_nav
{
namespace
{

bool finite_pose(const LocationPlanarPose & pose)
{
  return pose.valid &&
         pose.frame_id == "map" &&
         std::isfinite(pose.x) &&
         std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

LocationTargetDecision reject(
  const LocationTargetCode code,
  std::string reason)
{
  LocationTargetDecision decision;
  decision.code = code;
  decision.reason = std::move(reason);
  return decision;
}

}  // namespace

LocationNavigationContract::LocationNavigationContract(
  const double direct_tag_pose_epsilon_m)
: direct_tag_pose_epsilon_m_(direct_tag_pose_epsilon_m)
{
  if (!std::isfinite(direct_tag_pose_epsilon_m_) ||
    direct_tag_pose_epsilon_m_ < 0.0)
  {
    direct_tag_pose_epsilon_m_ = 0.05;
  }
}

LocationTargetDecision LocationNavigationContract::SelectTarget(
  const LocationNavigationRequestView & request,
  const LocationRecordView & record) const
{
  if (request.query.empty()) {
    return reject(
      LocationTargetCode::kInvalidRequest,
      "location_query_required");
  }

  if (!record.approved) {
    return reject(
      LocationTargetCode::kNotApproved,
      "location_not_approved");
  }

  if (record.retired) {
    return reject(
      LocationTargetCode::kRetired,
      "location_retired");
  }

  if (!record.enabled) {
    return reject(
      LocationTargetCode::kDisabled,
      "location_disabled");
  }

  if (request.enforce_map_context &&
    (request.map_id.empty() || request.map_revision == 0U ||
    record.map_id != request.map_id ||
    record.map_revision != request.map_revision))
  {
    return reject(
      LocationTargetCode::kMapMismatch,
      "location_map_context_mismatch");
  }

  if (!finite_pose(record.approach_pose)) {
    return reject(
      LocationTargetCode::kInvalidApproachPose,
      "approved_approach_pose_required");
  }

  if (record.tag_pose_map_valid && finite_pose(record.tag_pose_map)) {
    const double distance = std::hypot(
      record.approach_pose.x - record.tag_pose_map.x,
      record.approach_pose.y - record.tag_pose_map.y);

    if (distance <= direct_tag_pose_epsilon_m_) {
      return reject(
        LocationTargetCode::kDirectTagPoseTarget,
        "tag_pose_map_must_never_be_navigation_target");
    }
  }

  LocationTargetDecision decision;
  decision.accepted = true;
  decision.code = LocationTargetCode::kAccepted;
  decision.reason = "approved_approach_pose_selected";
  decision.target_source = LocationTargetSource::kApproachPose;
  decision.target = record.approach_pose;
  decision.arrival_confirmation_required =
    record.arrival_confirmation_required ||
    request.require_arrival_confirmation;
  return decision;
}

}  // namespace savo_nav
