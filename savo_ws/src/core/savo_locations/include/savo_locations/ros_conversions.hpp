// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_LOCATIONS__ROS_CONVERSIONS_HPP_
#define SAVO_LOCATIONS__ROS_CONVERSIONS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "savo_msgs/msg/location_candidate.hpp"
#include "savo_msgs/msg/location_record.hpp"
#include "savo_msgs/srv/approve_location.hpp"

#include "savo_locations/model.hpp"


namespace savo_locations
{

[[nodiscard]]
PoseData from_ros_pose(
  const geometry_msgs::msg::PoseStamped & pose);

[[nodiscard]]
geometry_msgs::msg::PoseStamped
 to_ros_pose(
  const PoseData & pose);

[[nodiscard]]
CandidateDraft from_ros_candidate(
  const savo_msgs::msg::LocationCandidate & candidate);

[[nodiscard]]
ApprovalRequest from_ros_approval_request(
  const savo_msgs::srv::ApproveLocation::Request & request);

[[nodiscard]]
savo_msgs::msg::LocationCandidate
 to_ros_candidate_record(
  const CandidateRecordData & record);

[[nodiscard]]
savo_msgs::msg::LocationRecord
 to_ros_location_record(
  const LocationRecordData & record);

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__ROS_CONVERSIONS_HPP_
