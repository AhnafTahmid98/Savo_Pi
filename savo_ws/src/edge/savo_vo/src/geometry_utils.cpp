#include "savo_vo/geometry_utils.hpp"

#include <cmath>

namespace savo_vo
{

double normalize_angle(const double angle_rad)
{
  double normalized = std::fmod(angle_rad + M_PI, 2.0 * M_PI);

  if (normalized < 0.0) {
    normalized += 2.0 * M_PI;
  }

  return normalized - M_PI;
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw_rad)
{
  geometry_msgs::msg::Quaternion quaternion;

  const double half_yaw = 0.5 * yaw_rad;

  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(half_yaw);
  quaternion.w = std::cos(half_yaw);

  return quaternion;
}

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp =
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);

  const double cosy_cosp =
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);

  return normalize_angle(std::atan2(siny_cosp, cosy_cosp));
}

geometry_msgs::msg::TransformStamped odometry_to_transform(
  const nav_msgs::msg::Odometry & odometry)
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header = odometry.header;
  transform.child_frame_id = odometry.child_frame_id;

  transform.transform.translation.x = odometry.pose.pose.position.x;
  transform.transform.translation.y = odometry.pose.pose.position.y;
  transform.transform.translation.z = odometry.pose.pose.position.z;
  transform.transform.rotation = odometry.pose.pose.orientation;

  return transform;
}

}  // namespace savo_vo
