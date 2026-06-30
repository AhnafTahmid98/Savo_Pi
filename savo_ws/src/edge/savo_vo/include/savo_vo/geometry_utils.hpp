#pragma once

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace savo_vo
{

double normalize_angle(double angle_rad);

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw_rad);

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & quaternion);

geometry_msgs::msg::TransformStamped odometry_to_transform(
  const nav_msgs::msg::Odometry & odometry);

}  // namespace savo_vo
