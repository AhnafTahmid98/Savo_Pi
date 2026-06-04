#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Rotate-to-heading test launch. First test: small angle (~0.30 rad), wheels lifted, safety gate running."""

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
    rotate_to_heading_yaml = os.path.join(pkg_share, "config", "rotate_to_heading.yaml")
    heading_pid_yaml = os.path.join(pkg_share, "config", "heading_pid.yaml")

    use_mode_manager = LaunchConfiguration("use_mode_manager")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_cmd_vel_shaper = LaunchConfiguration("use_cmd_vel_shaper")
    use_rotate_to_heading = LaunchConfiguration("use_rotate_to_heading")
    use_control_status = LaunchConfiguration("use_control_status")
    use_dashboard = LaunchConfiguration("use_dashboard")

    startup_mode = LaunchConfiguration("startup_mode")
    target_heading_rad = LaunchConfiguration("target_heading_rad")
    auto_publish_target = LaunchConfiguration("auto_publish_target")
    max_wz = LaunchConfiguration("max_wz")

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
                "use_rotate_to_heading",
                default_value="true",
                description="Start rotate_to_heading_node.",
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
                description="Startup control mode. For rotate tests this should be AUTO.",
            ),
            DeclareLaunchArgument(
                "target_heading_rad",
                default_value="0.30",
                description=(
                    "Target heading in radians. First test should be small, "
                    "for example 0.30 rad."
                ),
            ),
            DeclareLaunchArgument(
                "auto_publish_target",
                default_value="false",
                description=(
                    "If true, rotate_to_heading_node may auto-use target_heading_rad "
                    "depending on node support. Keep false for safest first test."
                ),
            ),
            DeclareLaunchArgument(
                "max_wz",
                default_value="0.35",
                description="Maximum angular.z command for rotate test.",
            ),
            LogInfo(
                msg=[
                    "Starting Robot Savo rotate test | startup_mode=",
                    startup_mode,
                    " | target_heading_rad=",
                    target_heading_rad,
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
            # Rotate-to-heading controller
            # -----------------------------------------------------------------
            Node(
                condition=IfCondition(use_rotate_to_heading),
                package="savo_control",
                executable="rotate_to_heading_node",
                name="rotate_to_heading_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    heading_pid_yaml,
                    rotate_to_heading_yaml,
                    {
                        "target_heading_rad": target_heading_rad,
                        "auto_publish_target": auto_publish_target,
                        "request_mode_on_start": True,
                        "requested_mode": "AUTO",
                        "max_wz": max_wz,
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