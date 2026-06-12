#!/usr/bin/env python3
"""Bring up the complete Robot Savo LiDAR stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration("profile")
    use_filter = LaunchConfiguration("use_filter")
    use_watchdog = LaunchConfiguration("use_watchdog")
    use_health = LaunchConfiguration("use_health")
    use_state_summary = LaunchConfiguration("use_state_summary")

    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="real_rplidar_a1.yaml",
        description="Profile file under config/profiles.",
    )
    use_filter_arg = DeclareLaunchArgument(
        "use_filter",
        default_value="true",
        description="Start lidar_filter_node.",
    )
    use_watchdog_arg = DeclareLaunchArgument(
        "use_watchdog",
        default_value="true",
        description="Start lidar_watchdog_node.",
    )
    use_health_arg = DeclareLaunchArgument(
        "use_health",
        default_value="true",
        description="Start lidar_health_node.",
    )
    use_state_summary_arg = DeclareLaunchArgument(
        "use_state_summary",
        default_value="true",
        description="Start lidar_state_publisher_node.",
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
        condition=IfCondition(use_filter),
    )

    watchdog_node = Node(
        package="savo_lidar",
        executable="lidar_watchdog_node.py",
        name="lidar_watchdog_node",
        output="screen",
        parameters=[profile_path],
        condition=IfCondition(use_watchdog),
    )

    health_node = Node(
        package="savo_lidar",
        executable="lidar_health_node.py",
        name="lidar_health_node",
        output="screen",
        parameters=[profile_path],
        condition=IfCondition(use_health),
    )

    state_summary_node = Node(
        package="savo_lidar",
        executable="lidar_state_publisher_node.py",
        name="lidar_state_publisher_node",
        output="screen",
        parameters=[profile_path],
        condition=IfCondition(use_state_summary),
    )

    return LaunchDescription(
        [
            profile_arg,
            use_filter_arg,
            use_watchdog_arg,
            use_health_arg,
            use_state_summary_arg,
            LogInfo(
                msg=[
                    "Starting Robot Savo LiDAR bringup | profile=",
                    profile,
                    " | output=/scan",
                ]
            ),
            driver_node,
            filter_node,
            watchdog_node,
            health_node,
            state_summary_node,
        ]
    )