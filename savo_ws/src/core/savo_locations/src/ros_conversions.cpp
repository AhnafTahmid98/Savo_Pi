// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_locations/ros_conversions.hpp"

#include <cstdint>


namespace savo_locations
{

geometry_msgs::msg::PoseStamped
to_ros_pose(
  const PoseData & pose)
{
  geometry_msgs::msg::PoseStamped message;

  message.header.frame_id =
    pose.frame_id;

  message.pose.position.x = pose.x;
  message.pose.position.y = pose.y;
  message.pose.position.z = pose.z;

  message.pose.orientation.x = pose.qx;
  message.pose.orientation.y = pose.qy;
  message.pose.orientation.z = pose.qz;
  message.pose.orientation.w = pose.qw;

  return message;
}


savo_msgs::msg::LocationRecord
to_ros_location_record(
  const LocationRecordData & record)
{
  savo_msgs::msg::LocationRecord message;

  message.state =
    static_cast<std::uint8_t>(
      record.state);

  message.enabled = record.enabled;

  message.record_revision =
    record.record_revision;

  message.location_id =
    record.location.location_id;

  message.display_name =
    record.location.display_name;

  message.aliases =
    record.location.aliases;

  message.semantic_type =
    record.location.semantic_type;

  message.map_id =
    record.location.map.map_id;

  message.map_revision =
    record.location.map.map_revision;

  message.map_release_id =
    record.location.map.map_release_id;

  message.approach_pose =
    to_ros_pose(
      record.location.approach_pose);

  message.confirmation_pose_valid =
    record.location
      .confirmation_pose
      .has_value();

  if (
    record.location
      .confirmation_pose
      .has_value())
  {
    message.confirmation_pose =
      to_ros_pose(
        record.location
          .confirmation_pose
          .value());
  }

  message.tag_family =
    record.location.tag.family;

  message.tag_id =
    record.location.tag.id;

  message.tag_pose_map_valid =
    record.location
      .tag_pose_map
      .has_value();

  if (
    record.location
      .tag_pose_map
      .has_value())
  {
    message.tag_pose_map =
      to_ros_pose(
        record.location
          .tag_pose_map
          .value());
  }

  message.arrival_confirmation_required =
    record.location
      .arrival_confirmation_required;

  message.building =
    record.location.building;

  message.floor =
    record.location.floor;

  message.area =
    record.location.area;

  message.notes =
    record.location.notes;

  message.source_candidate_id =
    record.source_candidate_id;

  // LOC-2 persistence currently retains database timestamps
  // internally rather than in LocationRecordData. Do not invent
  // timestamps in the ROS response; zero means unavailable.
  message.created_at.sec = 0;
  message.created_at.nanosec = 0U;
  message.updated_at.sec = 0;
  message.updated_at.nanosec = 0U;

  return message;
}

}  // namespace savo_locations
