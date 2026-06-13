#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
base_driver_node only — no helper nodes. Useful for isolated hardware testing.

  ros2 launch savo_base base_hw_only.launch.py profile:=dryrun_sim_motoroff.yaml
  ros2 launch savo_base base_hw_only.launch.py profile:=bench_test.yaml
  ros2 launch savo_base base_hw_only.launch.py profile:=real_robot_v1.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("savo_base")

    profile = LaunchConfiguration("profile")
    output = LaunchConfiguration("output")
    log_level = LaunchConfiguration("log_level")

    topics_yaml = PathJoinSubstitution([pkg_share, "config", "topics.yaml"])
    safety_yaml = PathJoinSubstitution([pkg_share, "config", "safety_timeouts.yaml"])
    board_yaml = PathJoinSubstitution([pkg_share, "config", "motor_board_freenove.yaml"])
    mecanum_yaml = PathJoinSubstitution([pkg_share, "config", "mecanum_kinematics.yaml"])
    profile_yaml = PathJoinSubstitution([pkg_share, "config", "profiles", profile])

    # profile must include the filename, e.g. "bench_test.yaml"

    base_driver = Node(
        package="savo_base",
        executable="base_driver_node.py",
        name="base_driver_node",
        output=output,
        parameters=[
            topics_yaml,
            safety_yaml,
            board_yaml,
            mecanum_yaml,
            profile_yaml,   # profile overrides last
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="dryrun_sim_motoroff.yaml",
                description=(
                    "Profile YAML filename under config/profiles/ "
                    "(e.g. dryrun_sim_motoroff.yaml, bench_test.yaml, real_robot_v1.yaml)"
                ),
            ),
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
            LogInfo(msg="[savo_base] Starting base_hw_only.launch.py"),
            LogInfo(msg=["[savo_base] Profile: ", profile]),
            base_driver,
        ]
    )
