#!/usr/bin/env python3
"""Launch the LiDAR driver and filter nodes for scan-filter validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration("profile")

    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="dryrun_sim.yaml",
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
        executable="lidar_py_driver_node.py",
        name="lidar_driver_node",
        output="screen",
        parameters=[profile_path],
    )

    filter_node = Node(
        package="savo_lidar",
        executable="lidar_filter_node.py",
        name="lidar_filter_node",
        output="screen",
        parameters=[profile_path],
    )

    return LaunchDescription(
        [
            profile_arg,
            LogInfo(
                msg=[
                    "Starting Robot Savo LiDAR filter test | profile=",
                    profile,
                    " | /scan -> /scan/filtered",
                ]
            ),
            driver_node,
            filter_node,
        ]
    )
