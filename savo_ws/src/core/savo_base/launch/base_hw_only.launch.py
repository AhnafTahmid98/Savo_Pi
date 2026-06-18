#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
base_driver_node only — no helper nodes. Useful for isolated hardware testing.

  ros2 launch savo_base base_hw_only.launch.py profile:=dryrun_sim_motoroff.yaml
  ros2 launch savo_base base_hw_only.launch.py profile:=bench_test.yaml
  ros2 launch savo_base base_hw_only.launch.py profile:=real_robot_v1.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_base_driver_node(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    output = LaunchConfiguration("output").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    driver_impl = LaunchConfiguration("driver_impl").perform(context)

    if driver_impl == "cpp":
        driver_executable = "base_driver_node"
    elif driver_impl == "py":
        driver_executable = "base_driver_node_py"
    else:
        raise RuntimeError("driver_impl must be 'cpp' or 'py'")

    pkg_share = get_package_share_directory("savo_base")
    profile_path = os.path.join(pkg_share, "config", "profiles", profile)

    params = [
        os.path.join(pkg_share, "config", "topics.yaml"),
        os.path.join(pkg_share, "config", "safety_timeouts.yaml"),
        os.path.join(pkg_share, "config", "motor_board_freenove.yaml"),
        os.path.join(pkg_share, "config", "mecanum_kinematics.yaml"),
        profile_path,
    ]

    return [
        Node(
            package="savo_base",
            executable=driver_executable,
            name="base_driver_node",
            output=output,
            parameters=params,
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration("profile")
    driver_impl = LaunchConfiguration("driver_impl")

    # profile must include the filename, e.g. "bench_test.yaml"

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
                "driver_impl",
                default_value="cpp",
                description="Base driver implementation: cpp or py",
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
            LogInfo(msg=["[savo_base] Driver implementation: ", driver_impl]),
            OpaqueFunction(function=_build_base_driver_node),
        ]
    )
