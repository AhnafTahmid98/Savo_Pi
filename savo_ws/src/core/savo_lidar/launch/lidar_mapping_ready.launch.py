#!/usr/bin/env python3
"""Launch the LiDAR stack with mapping-oriented scan checks."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration("profile")

    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="mapping_rplidar_a1.yaml",
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

    filter_node = Node(
        package="savo_lidar",
        executable="lidar_filter_node.py",
        name="lidar_filter_node",
        output="screen",
        parameters=[profile_path],
    )

    watchdog_node = Node(
        package="savo_lidar",
        executable="lidar_watchdog_node.py",
        name="lidar_watchdog_node",
        output="screen",
        parameters=[profile_path],
    )

    health_node = Node(
        package="savo_lidar",
        executable="lidar_health_node.py",
        name="lidar_health_node",
        output="screen",
        parameters=[profile_path],
    )

    state_summary_node = Node(
        package="savo_lidar",
        executable="lidar_state_publisher_node.py",
        name="lidar_state_publisher_node",
        output="screen",
        parameters=[profile_path],
    )

    return LaunchDescription(
        [
            profile_arg,
            LogInfo(
                msg=[
                    "Starting Robot Savo LiDAR mapping-ready stack | profile=",
                    profile,
                    " | scan=/scan | filtered=/scan/filtered",
                ]
            ),
            driver_node,
            filter_node,
            watchdog_node,
            health_node,
            state_summary_node,
        ]
    )