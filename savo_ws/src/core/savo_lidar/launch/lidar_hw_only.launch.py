#!/usr/bin/env python3
"""Launch only the real RPLIDAR hardware driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration("profile")

    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="bench_test.yaml",
        description=(
            "Hardware profile file under config/profiles. "
            "Use bench_test.yaml, real_rplidar_a1.yaml, "
            "mapping_rplidar_a1.yaml, or nav_rplidar_a1.yaml."
        ),
    )

    profile_path = PathJoinSubstitution(
        [
            FindPackageShare("savo_lidar"),
            "config",
            "profiles",
            profile,
        ]
    )

    driver_node = Node(
        package="savo_lidar",
        executable="lidar_driver_node",
        name="lidar_driver_node",
        output="screen",
        parameters=[profile_path],
    )

    return LaunchDescription(
        [
            profile_arg,
            LogInfo(
                msg=[
                    "Starting Robot Savo LiDAR hardware-only bringup | profile=",
                    profile,
                    " | driver=lidar_driver_node | scan=/scan",
                ]
            ),
            driver_node,
        ]
    )