// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_LOCATIONS__ROS_CONVERSIONS_HPP_
#define SAVO_LOCATIONS__ROS_CONVERSIONS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "savo_msgs/msg/location_record.hpp"

#include "savo_locations/model.hpp"


namespace savo_locations
{

[[nodiscard]]
geometry_msgs::msg::PoseStamped
to_ros_pose(
  const PoseData & pose);

[[nodiscard]]
savo_msgs::msg::LocationRecord
to_ros_location_record(
  const LocationRecordData & record);

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__ROS_CONVERSIONS_HPP_
