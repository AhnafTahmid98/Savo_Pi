// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_locations/ros_conversions.hpp"

#include <cstdint>


namespace savo_locations
{

PoseData from_ros_pose(
  const geometry_msgs::msg::PoseStamped & pose)
{
  PoseData output;

  output.frame_id = pose.header.frame_id;

  output.x = pose.pose.position.x;
  output.y = pose.pose.position.y;
  output.z = pose.pose.position.z;

  output.qx = pose.pose.orientation.x;
  output.qy = pose.pose.orientation.y;
  output.qz = pose.pose.orientation.z;
  output.qw = pose.pose.orientation.w;

  return output;
}


geometry_msgs::msg::PoseStamped
 to_ros_pose(
  const PoseData & pose)
{
  geometry_msgs::msg::PoseStamped message;

  message.header.frame_id = pose.frame_id;

  message.pose.position.x = pose.x;
  message.pose.position.y = pose.y;
  message.pose.position.z = pose.z;

  message.pose.orientation.x = pose.qx;
  message.pose.orientation.y = pose.qy;
  message.pose.orientation.z = pose.qz;
  message.pose.orientation.w = pose.qw;

  return message;
}


CandidateDraft from_ros_candidate(
  const savo_msgs::msg::LocationCandidate & candidate)
{
  CandidateDraft output;

  output.candidate_id = candidate.candidate_id;

  output.map.map_id = candidate.map_id;
  output.map.map_revision = candidate.map_revision;
  output.map.map_release_id = candidate.map_release_id;

  output.tag.family = candidate.tag_family;
  output.tag.id = candidate.tag_id;
  output.tag_pose_map = from_ros_pose(candidate.tag_pose_map);

  output.detection_quality =
    static_cast<double>(candidate.detection_quality);

  output.accepted_observations =
    candidate.accepted_observations;

  output.position_stddev_m =
    static_cast<double>(candidate.position_stddev_m);

  output.yaw_stddev_rad =
    static_cast<double>(candidate.yaw_stddev_rad);

  if (candidate.approach_pose_valid) {
    output.approach_pose =
      from_ros_pose(candidate.approach_pose);
  }

  if (candidate.confirmation_pose_valid) {
    output.confirmation_pose =
      from_ros_pose(candidate.confirmation_pose);
  }

  output.suggested_location_id =
    candidate.suggested_location_id;

  output.suggested_display_name =
    candidate.suggested_display_name;

  output.suggested_aliases =
    candidate.suggested_aliases;

  output.suggested_semantic_type =
    candidate.suggested_semantic_type;

  output.building = candidate.building;
  output.floor = candidate.floor;
  output.area = candidate.area;
  output.notes = candidate.notes;

  output.source_session_id =
    candidate.source_session_id;

  output.source_component =
    candidate.source_component;

  return output;
}


ApprovalRequest from_ros_approval_request(
  const savo_msgs::srv::ApproveLocation::Request & request)
{
  ApprovalRequest output;

  output.candidate_id = request.candidate_id;

  output.expected_candidate_revision =
    request.expected_candidate_revision;

  output.location_id = request.location_id;
  output.display_name = request.display_name;
  output.aliases = request.aliases;
  output.semantic_type = request.semantic_type;

  if (!request.approach_pose.header.frame_id.empty()) {
    output.approach_pose =
      from_ros_pose(request.approach_pose);
  }

  if (request.confirmation_pose_valid) {
    output.confirmation_pose =
      from_ros_pose(request.confirmation_pose);
  }

  output.arrival_confirmation_required =
    request.arrival_confirmation_required;

  output.building = request.building;
  output.floor = request.floor;
  output.area = request.area;
  output.notes = request.notes;

  return output;
}


savo_msgs::msg::LocationCandidate
 to_ros_candidate_record(
  const CandidateRecordData & record)
{
  savo_msgs::msg::LocationCandidate message;

  message.state =
    static_cast<std::uint8_t>(record.state);

  message.candidate_revision =
    record.candidate_revision;

  message.candidate_id =
    record.candidate.candidate_id;

  message.map_id = record.candidate.map.map_id;
  message.map_revision =
    record.candidate.map.map_revision;
  message.map_release_id =
    record.candidate.map.map_release_id;

  message.tag_family = record.candidate.tag.family;
  message.tag_id = record.candidate.tag.id;
  message.tag_pose_map =
    to_ros_pose(record.candidate.tag_pose_map);

  message.detection_quality =
    static_cast<float>(record.candidate.detection_quality);

  message.accepted_observations =
    record.candidate.accepted_observations;

  message.position_stddev_m =
    static_cast<float>(record.candidate.position_stddev_m);

  message.yaw_stddev_rad =
    static_cast<float>(record.candidate.yaw_stddev_rad);

  message.approach_pose_valid =
    record.candidate.approach_pose.has_value();

  if (record.candidate.approach_pose.has_value()) {
    message.approach_pose =
      to_ros_pose(record.candidate.approach_pose.value());
  }

  message.confirmation_pose_valid =
    record.candidate.confirmation_pose.has_value();

  if (record.candidate.confirmation_pose.has_value()) {
    message.confirmation_pose =
      to_ros_pose(record.candidate.confirmation_pose.value());
  }

  message.suggested_location_id =
    record.candidate.suggested_location_id;

  message.suggested_display_name =
    record.candidate.suggested_display_name;

  message.suggested_aliases =
    record.candidate.suggested_aliases;

  message.suggested_semantic_type =
    record.candidate.suggested_semantic_type;

  message.building = record.candidate.building;
  message.floor = record.candidate.floor;
  message.area = record.candidate.area;
  message.notes = record.candidate.notes;

  message.source_session_id =
    record.candidate.source_session_id;

  message.source_component =
    record.candidate.source_component;

  message.review_reason = record.review_reason;

  message.created_at.sec = 0;
  message.created_at.nanosec = 0U;
  message.updated_at.sec = 0;
  message.updated_at.nanosec = 0U;

  return message;
}


savo_msgs::msg::LocationRecord
 to_ros_location_record(
  const LocationRecordData & record)
{
  savo_msgs::msg::LocationRecord message;

  message.state =
    static_cast<std::uint8_t>(record.state);

  message.enabled = record.enabled;
  message.record_revision = record.record_revision;

  message.location_id = record.location.location_id;
  message.display_name = record.location.display_name;
  message.aliases = record.location.aliases;
  message.semantic_type = record.location.semantic_type;

  message.map_id = record.location.map.map_id;
  message.map_revision = record.location.map.map_revision;
  message.map_release_id =
    record.location.map.map_release_id;

  message.approach_pose =
    to_ros_pose(record.location.approach_pose);

  message.confirmation_pose_valid =
    record.location.confirmation_pose.has_value();

  if (record.location.confirmation_pose.has_value()) {
    message.confirmation_pose =
      to_ros_pose(record.location.confirmation_pose.value());
  }

  message.tag_family = record.location.tag.family;
  message.tag_id = record.location.tag.id;

  message.tag_pose_map_valid =
    record.location.tag_pose_map.has_value();

  if (record.location.tag_pose_map.has_value()) {
    message.tag_pose_map =
      to_ros_pose(record.location.tag_pose_map.value());
  }

  message.arrival_confirmation_required =
    record.location.arrival_confirmation_required;

  message.building = record.location.building;
  message.floor = record.location.floor;
  message.area = record.location.area;
  message.notes = record.location.notes;

  message.source_candidate_id =
    record.source_candidate_id;

  message.created_at.sec = 0;
  message.created_at.nanosec = 0U;
  message.updated_at.sec = 0;
  message.updated_at.nanosec = 0U;

  return message;
}

}  // namespace savo_locations
