#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_control / heading_pid_test.launch.py
======================================================

Heading PID test launch file for Robot Savo.

Purpose
-------
Starts the control chain plus heading/straight-line test support for validating
heading correction using fused odometry:

    /odometry/filtered

Future fused odometry inputs behind /odometry/filtered:
    - 4 wheel encoder odometry on savo-core
    - IMU on savo-core
    - VO / visual odometry from savo-edge

Command chain:

    straight_line_pid_test_node.py / heading PID behavior
        -> /cmd_vel_auto
        -> twist_mux_node
        -> /cmd_vel_mux
        -> cmd_vel_shaper_node
        -> /cmd_vel
        -> savo_perception/cmd_vel_safety_gate
        -> /cmd_vel_safe
        -> savo_base/base_driver_node
        -> motors

Important package boundaries:
    - this launch does not start hardware drivers
    - this launch does not start the safety gate
    - this launch does not publish directly to /cmd_vel_safe
    - this launch expects /odometry/filtered from savo_localization

Safety
------
First test:
    - use a very short distance
    - keep speed low
    - safety gate running before floor tests
    - wheels lifted or open floor
    - hand near E-stop / power switch
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("savo_control")

    control_common_yaml = os.path.join(pkg_share, "config", "control_common.yaml")
    twist_mux_yaml = os.path.join(pkg_share, "config", "twist_mux.yaml")
    cmd_vel_shaper_yaml = os.path.join(pkg_share, "config", "cmd_vel_shaper.yaml")
    heading_pid_yaml = os.path.join(pkg_share, "config", "heading_pid.yaml")

    use_mode_manager = LaunchConfiguration("use_mode_manager")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_cmd_vel_shaper = LaunchConfiguration("use_cmd_vel_shaper")
    use_straight_line_test = LaunchConfiguration("use_straight_line_test")
    use_control_status = LaunchConfiguration("use_control_status")
    use_dashboard = LaunchConfiguration("use_dashboard")

    startup_mode = LaunchConfiguration("startup_mode")
    auto_start = LaunchConfiguration("auto_start")
    target_distance_m = LaunchConfiguration("target_distance_m")
    direction = LaunchConfiguration("direction")
    hold_heading = LaunchConfiguration("hold_heading")
    max_forward_vx = LaunchConfiguration("max_forward_vx")
    max_backward_vx = LaunchConfiguration("max_backward_vx")

    return LaunchDescription(
        [
            # -----------------------------------------------------------------
            # Launch arguments
            # -----------------------------------------------------------------
            DeclareLaunchArgument(
                "use_mode_manager",
                default_value="true",
                description="Start control_mode_manager_node.",
            ),
            DeclareLaunchArgument(
                "use_twist_mux",
                default_value="true",
                description="Start twist_mux_node.",
            ),
            DeclareLaunchArgument(
                "use_cmd_vel_shaper",
                default_value="true",
                description="Start cmd_vel_shaper_node.",
            ),
            DeclareLaunchArgument(
                "use_straight_line_test",
                default_value="true",
                description="Start straight_line_pid_test_node.py.",
            ),
            DeclareLaunchArgument(
                "use_control_status",
                default_value="true",
                description="Start control_status_node.py.",
            ),
            DeclareLaunchArgument(
                "use_dashboard",
                default_value="false",
                description=(
                    "Start curses dashboard. Usually false; run dashboard "
                    "separately in its own terminal."
                ),
            ),
            DeclareLaunchArgument(
                "startup_mode",
                default_value="AUTO",
                description="Startup control mode. For heading tests this should be AUTO.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description=(
                    "If true, straight_line_pid_test_node starts immediately. "
                    "Keep false unless wheels are lifted or test area is safe."
                ),
            ),
            DeclareLaunchArgument(
                "target_distance_m",
                default_value="0.30",
                description="Straight-line target distance in meters.",
            ),
            DeclareLaunchArgument(
                "direction",
                default_value="forward",
                description="Straight-line direction: forward or backward.",
            ),
            DeclareLaunchArgument(
                "hold_heading",
                default_value="true",
                description="Use heading correction while driving straight.",
            ),
            DeclareLaunchArgument(
                "max_forward_vx",
                default_value="0.12",
                description="Maximum forward command for heading test.",
            ),
            DeclareLaunchArgument(
                "max_backward_vx",
                default_value="0.08",
                description="Maximum backward command for heading test.",
            ),
            LogInfo(
                msg=[
                    "Starting Robot Savo heading PID test | startup_mode=",
                    startup_mode,
                    " | auto_start=",
                    auto_start,
                    " | target_distance_m=",
                    target_distance_m,
                    " | direction=",
                    direction,
                    " | command path: /cmd_vel_auto -> /cmd_vel_mux -> /cmd_vel -> /cmd_vel_safe",
                ]
            ),

            # -----------------------------------------------------------------
            # Core control chain
            # -----------------------------------------------------------------
            Node(
                condition=IfCondition(use_mode_manager),
                package="savo_control",
                executable="control_mode_manager_node",
                name="control_mode_manager_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    {
                        "startup_mode": startup_mode,
                    },
                ],
            ),
            Node(
                condition=IfCondition(use_twist_mux),
                package="savo_control",
                executable="twist_mux_node",
                name="twist_mux_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    twist_mux_yaml,
                    {
                        "default_mode": startup_mode,
                    },
                ],
            ),
            Node(
                condition=IfCondition(use_cmd_vel_shaper),
                package="savo_control",
                executable="cmd_vel_shaper_node",
                name="cmd_vel_shaper_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    cmd_vel_shaper_yaml,
                    {
                        "default_profile": "safe_default",
                    },
                ],
            ),

            # -----------------------------------------------------------------
            # Straight-line heading hold test
            # -----------------------------------------------------------------
            Node(
                condition=IfCondition(use_straight_line_test),
                package="savo_control",
                executable="straight_line_pid_test_node.py",
                name="straight_line_pid_test_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    heading_pid_yaml,
                    {
                        "auto_start": auto_start,
                        "target_distance_m": target_distance_m,
                        "direction": direction,
                        "hold_heading": hold_heading,
                        "request_auto_mode_on_start": True,
                        "required_mode": "AUTO",
                        "max_forward_vx": max_forward_vx,
                        "max_backward_vx": max_backward_vx,
                    },
                ],
            ),

            # -----------------------------------------------------------------
            # Status/dashboard
            # -----------------------------------------------------------------
            Node(
                condition=IfCondition(use_control_status),
                package="savo_control",
                executable="control_status_node.py",
                name="control_status_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    {
                        "watch_odom": True,
                        "publish_console_log": True,
                    },
                ],
            ),
            Node(
                condition=IfCondition(use_dashboard),
                package="savo_control",
                executable="control_dashboard_node.py",
                name="control_dashboard_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    control_common_yaml,
                    {
                        "watch_odom": True,
                        "watch_depth": False,
                    },
                ],
            ),
        ]
    )