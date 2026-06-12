#!/usr/bin/env python3
"""Launch only the LiDAR driver node for direct hardware bringup."""

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
        description="Profile file under config/profiles.",
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
        executable="lidar_driver_node.py",
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
                    " | output=/scan",
                ]
            ),
            driver_node,
        ]
    )