"""Convert internal VO samples into ROS odometry messages."""

from math import cos, sin

from nav_msgs.msg import Odometry

from savo_vo.core.covariance_builder import build_planar_covariance
from savo_vo.models.vo_sample import VOSample
from savo_vo.utils.ros_time import seconds_to_stamp_parts


def yaw_to_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half_yaw = yaw_rad * 0.5
    return 0.0, 0.0, sin(half_yaw), cos(half_yaw)


def sample_to_odometry_msg(
    sample: VOSample,
    odom_frame: str,
    base_frame: str,
) -> Odometry:
    msg = Odometry()

    sec, nanosec = seconds_to_stamp_parts(sample.timestamp_s)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = odom_frame
    msg.child_frame_id = base_frame

    msg.pose.pose.position.x = sample.x_m
    msg.pose.pose.position.y = sample.y_m
    msg.pose.pose.position.z = sample.z_m

    qx, qy, qz, qw = yaw_to_quaternion(sample.yaw_rad)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    msg.twist.twist.linear.x = sample.vx_mps
    msg.twist.twist.linear.y = sample.vy_mps
    msg.twist.twist.linear.z = sample.vz_mps
    msg.twist.twist.angular.z = sample.yaw_rate_radps

    covariance = build_planar_covariance(sample.quality)
    msg.pose.covariance = covariance.as_pose_covariance()
    msg.twist.covariance = covariance.as_twist_covariance()

    return msg