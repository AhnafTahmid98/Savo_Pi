#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Brings up the base stack with no motion commands — good for first power-on and pre-drive checks.

  ros2 launch savo_base base_safe_idle.launch.py profile:=dryrun_sim_motoroff.yaml
  ros2 launch savo_base base_safe_idle.launch.py profile:=bench_test.yaml
  ros2 launch savo_base base_safe_idle.launch.py profile:=real_robot_v1.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("savo_base")

    profile = LaunchConfiguration("profile")
    output = LaunchConfiguration("output")
    log_level = LaunchConfiguration("log_level")

    use_watchdog = LaunchConfiguration("use_watchdog")
    use_state_publisher = LaunchConfiguration("use_state_publisher")
    use_heartbeat = LaunchConfiguration("use_heartbeat")
    use_diag_runner = LaunchConfiguration("use_diag_runner")

    profile_path = LaunchConfiguration("profile_path")

    base_bringup_launch = PathJoinSubstitution([pkg_share, "launch", "base_bringup.launch.py"])

    include_base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_bringup_launch),
        launch_arguments={
            "profile": profile,
            "profile_path": profile_path,
            "use_watchdog": use_watchdog,
            "use_state_publisher": use_state_publisher,
            "use_heartbeat": use_heartbeat,
            "use_diag_runner": use_diag_runner,
            "output": output,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription([
        # profile
        DeclareLaunchArgument(
            "profile",
            default_value="bench_test.yaml",
            description=(
                "Profile YAML filename under config/profiles/ "
                "(dryrun_sim_motoroff.yaml | bench_test.yaml | real_robot_v1.yaml). "
                "Default is bench_test.yaml for a conservative idle bringup."
            ),
        ),
        DeclareLaunchArgument(
            "profile_path",
            default_value="",
            description=(
                "Optional absolute profile YAML path. If non-empty, overrides 'profile'."
            ),
        ),

        # helper nodes
        DeclareLaunchArgument(
            "use_watchdog",
            default_value="true",
            description="Start base_watchdog_node (recommended for safe idle).",
        ),
        DeclareLaunchArgument(
            "use_state_publisher",
            default_value="true",
            description="Start base_state_publisher_node (recommended for monitoring).",
        ),
        DeclareLaunchArgument(
            "use_heartbeat",
            default_value="true",
            description="Start base_heartbeat_node (recommended for liveness checks).",
        ),
        DeclareLaunchArgument(
            "use_diag_runner",
            default_value="false",
            description="Start base_diag_runner_node (optional).",
        ),

        # output
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="Node output mode (screen|log).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level (debug|info|warn|error|fatal).",
        ),

        LogInfo(msg=["[savo_base] base_safe_idle profile: ", profile]),
        include_base_bringup,
    ])
