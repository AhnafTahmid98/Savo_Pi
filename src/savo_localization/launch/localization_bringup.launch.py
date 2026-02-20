#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_localization/launch/localization_bringup.launch.py
--------------------------------------------------------------------
Professional one-command bringup for the localization stack.

Starts (toggleable):
- wheel_odom_node (C++): publishes /wheel/odom (rear encoders only)
- imu_node (Python):     publishes /imu/data
- ekf_node (robot_localization): fuses /wheel/odom + /imu/data -> /odometry/filtered and TF (odom->base_link)

Notes:
- If robot_localization is not installed on the Pi, set use_ekf:=false.
- This launch file is designed to be clean and explicit (no magic).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pkg_share(pkg: str) -> str:
    return get_package_share_directory(pkg)


def generate_launch_description() -> LaunchDescription:
    pkg = "savo_localization"
    share = _pkg_share(pkg)

    # -------------------------
    # Launch arguments
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_wheel_odom = LaunchConfiguration("use_wheel_odom")
    use_imu = LaunchConfiguration("use_imu")
    use_ekf = LaunchConfiguration("use_ekf")

    # Optional: allow overriding YAML paths from CLI
    frames_yaml = LaunchConfiguration("frames_yaml")
    common_yaml = LaunchConfiguration("common_yaml")
    ekf_yaml = LaunchConfiguration("ekf_yaml")

    # Defaults
    default_frames_yaml = os.path.join(share, "config", "frames.yaml")
    default_common_yaml = os.path.join(share, "config", "localization_common.yaml")
    default_ekf_yaml = os.path.join(share, "config", "ekf_odom.yaml")

    # -------------------------
    # Nodes
    # -------------------------

    # Wheel odometry (C++): publishes /wheel/odom
    wheel_odom_node = Node(
        package=pkg,
        executable="wheel_odom_node",
        name="wheel_odom_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            common_yaml,
            frames_yaml,
        ],
        condition=None,  # controlled below via action list
    )

    # IMU (Python): publishes /imu/data
    imu_node = Node(
        package=pkg,
        executable="imu_node",
        name="imu_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            common_yaml,
            frames_yaml,
        ],
        condition=None,
    )

    # EKF (robot_localization): publishes /odometry/filtered and TF (odom->base_link)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ekf_yaml,
        ],
        condition=None,
    )

    # -------------------------
    # Build LaunchDescription
    # -------------------------
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock (simulation time). Keep false on the real robot.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_wheel_odom",
        default_value="true",
        description="Start wheel_odom_node (rear encoders).",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_imu",
        default_value="true",
        description="Start imu_node (BNO055).",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_ekf",
        default_value="true",
        description="Start robot_localization ekf_node (requires ros-jazzy-robot-localization).",
    ))

    ld.add_action(DeclareLaunchArgument(
        "frames_yaml",
        default_value=default_frames_yaml,
        description="Path to frames.yaml (frame-id contract).",
    ))
    ld.add_action(DeclareLaunchArgument(
        "common_yaml",
        default_value=default_common_yaml,
        description="Path to localization_common.yaml (shared params).",
    ))
    ld.add_action(DeclareLaunchArgument(
        "ekf_yaml",
        default_value=default_ekf_yaml,
        description="Path to ekf_odom.yaml (robot_localization EKF config).",
    ))

    # Helpful logs
    ld.add_action(LogInfo(msg=[
        "[savo_localization] bringup: wheel_odom=", use_wheel_odom,
        " imu=", use_imu,
        " ekf=", use_ekf,
    ]))
    ld.add_action(LogInfo(msg=["[savo_localization] frames_yaml: ", frames_yaml]))
    ld.add_action(LogInfo(msg=["[savo_localization] common_yaml: ", common_yaml]))
    ld.add_action(LogInfo(msg=["[savo_localization] ekf_yaml: ", ekf_yaml]))

    # Conditional start (simple pattern: add nodes only when enabled)
    # Launch conditions are typically handled with IfCondition, but we keep it explicit and readable.

    from launch.conditions import IfCondition  # local import (clean)
    wheel_odom_node.condition = IfCondition(use_wheel_odom)
    imu_node.condition = IfCondition(use_imu)
    ekf_node.condition = IfCondition(use_ekf)

    ld.add_action(wheel_odom_node)
    ld.add_action(imu_node)
    ld.add_action(ekf_node)

    return ld