#!/usr/bin/env python3
"""Launch the LiDAR stack with a synthetic scan source for safe PC testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration("profile")
    use_filter = LaunchConfiguration("use_filter")
    use_health = LaunchConfiguration("use_health")
    use_state_summary = LaunchConfiguration("use_state_summary")

    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="dryrun_sim.yaml",
        description="Profile file under config/profiles.",
    )
    use_filter_arg = DeclareLaunchArgument(
        "use_filter",
        default_value="true",
        description="Start lidar_filter_node.",
    )
    use_health_arg = DeclareLaunchArgument(
        "use_health",
        default_value="true",
        description="Start watchdog and health nodes.",
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
        condition=_if_arg(use_filter),
    )

    watchdog_node = Node(
        package="savo_lidar",
        executable="lidar_watchdog_node.py",
        name="lidar_watchdog_node",
        output="screen",
        parameters=[profile_path],
        condition=_if_arg(use_health),
    )

    health_node = Node(
        package="savo_lidar",
        executable="lidar_health_node.py",
        name="lidar_health_node",
        output="screen",
        parameters=[profile_path],
        condition=_if_arg(use_health),
    )

    state_summary_node = Node(
        package="savo_lidar",
        executable="lidar_state_publisher_node.py",
        name="lidar_state_publisher_node",
        output="screen",
        parameters=[profile_path],
        condition=_if_arg(use_state_summary),
    )

    return LaunchDescription(
        [
            profile_arg,
            use_filter_arg,
            use_health_arg,
            use_state_summary_arg,
            LogInfo(
                msg=[
                    "Starting Robot Savo LiDAR dryrun stack | profile=",
                    profile,
                    " | scan=/scan",
                ]
            ),
            driver_node,
            filter_node,
            watchdog_node,
            health_node,
            state_summary_node,
        ]
    )


def _if_arg(arg: LaunchConfiguration):
    from launch.conditions import IfCondition

    return IfCondition(arg)