#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — Base Hardware-Only Launch (savo_base)
--------------------------------------------------
Minimal bringup for the low-level base hardware execution node only.

Starts:
- base_driver_node.py

Purpose
-------
Use this launch when you want to test only the base driver + motor board path
without starting helper/supervisor nodes (watchdog monitor, heartbeat, summary,
diag runner).

Typical use-cases
-----------------
- Real hardware board bringup
- Focused cmd_vel_safe -> wheel output validation
- Isolated testing of Freenove/PCA9685 backend
- Debugging base_driver_node parameters

Profiles
--------
Loads layered config files and applies a profile last:
- dryrun_sim_motoroff.yaml
- bench_test.yaml
- real_robot_v1.yaml

Examples
--------
# Safe dryrun (motor-off)
ros2 launch savo_base base_hw_only.launch.py profile:=dryrun_sim_motoroff.yaml

# Bench test (robot lifted)
ros2 launch savo_base base_hw_only.launch.py profile:=bench_test.yaml

# Real robot hardware
ros2 launch savo_base base_hw_only.launch.py profile:=real_robot_v1.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("savo_base")

    # -------------------------------------------------------------------------
    # Launch args
    # -------------------------------------------------------------------------
    profile = LaunchConfiguration("profile")
    output = LaunchConfiguration("output")
    log_level = LaunchConfiguration("log_level")

    # -------------------------------------------------------------------------
    # Layered config files (shared + profile override last)
    # -------------------------------------------------------------------------
    topics_yaml = PathJoinSubstitution([pkg_share, "config", "topics.yaml"])
    safety_yaml = PathJoinSubstitution([pkg_share, "config", "safety_timeouts.yaml"])
    board_yaml = PathJoinSubstitution([pkg_share, "config", "motor_board_freenove.yaml"])
    mecanum_yaml = PathJoinSubstitution([pkg_share, "config", "mecanum_kinematics.yaml"])
    profile_yaml = PathJoinSubstitution([pkg_share, "config", "profiles", profile])

    # NOTE:
    # `profile` should include the filename, e.g. "bench_test.yaml".
    # This avoids PythonExpression path tricks and keeps launch robust/simple.

    base_driver = Node(
        package="savo_base",
        executable="base_driver_node.py",  # hybrid package executable installed via CMake
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
            LogInfo(
                msg=(
                    "[savo_base] Hardware-only mode: starting only base_driver_node "
                    "(no watchdog/state/heartbeat helper nodes)."
                )
            ),
            base_driver,
        ]
    )