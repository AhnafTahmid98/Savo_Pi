#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Keyboard teleop plus control chain for manual driving and mapping.

The keyboard node requires an interactive terminal. If keyboard input does not
work when launched, run keyboard_teleop_node.py with ros2 run in its own terminal.
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
    recovery_yaml = os.path.join(pkg_share, "config", "recovery.yaml")

    use_keyboard = LaunchConfiguration("use_keyboard")
    use_mode_manager = LaunchConfiguration("use_mode_manager")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_cmd_vel_shaper = LaunchConfiguration("use_cmd_vel_shaper")
    use_control_status = LaunchConfiguration("use_control_status")
    use_recovery_status = LaunchConfiguration("use_recovery_status")
    use_dashboard = LaunchConfiguration("use_dashboard")
    use_recovery_manager = LaunchConfiguration("use_recovery_manager")
    use_backup_escape = LaunchConfiguration("use_backup_escape")

    startup_mode = LaunchConfiguration("startup_mode")

    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    default_linear_speed = LaunchConfiguration("default_linear_speed")
    default_angular_speed = LaunchConfiguration("default_angular_speed")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    max_angular_speed = LaunchConfiguration("max_angular_speed")
    key_timeout_s = LaunchConfiguration("key_timeout_s")

    return LaunchDescription(
        [
            # -----------------------------------------------------------------
            # Launch arguments
            # -----------------------------------------------------------------
            DeclareLaunchArgument(
                "use_keyboard",
                default_value="true",
                description="Start keyboard_teleop_node.py.",
            ),
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
                "use_recovery_manager",
                default_value="false",
                description=(
                    "Start recovery_manager_node. Default false for simple "
                    "manual mapping teleop."
                ),
            ),
            DeclareLaunchArgument(
                "use_backup_escape",
                default_value="false",
                description=(
                    "Start backup_escape_node. Default false for simple "
                    "manual mapping teleop."
                ),
            ),
            DeclareLaunchArgument(
                "use_control_status",
                default_value="true",
                description="Start control_status_node.py.",
            ),
            DeclareLaunchArgument(
                "use_recovery_status",
                default_value="false",
                description="Start recovery_status_node.py.",
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
                default_value="MANUAL",
                description="Startup control mode. For teleop this should be MANUAL.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel_manual",
                description="Keyboard teleop output topic.",
            ),
            DeclareLaunchArgument(
                "default_linear_speed",
                default_value="0.12",
                description="Default linear teleop speed/command strength.",
            ),
            DeclareLaunchArgument(
                "default_angular_speed",
                default_value="0.35",
                description="Default angular teleop speed/command strength.",
            ),
            DeclareLaunchArgument(
                "max_linear_speed",
                default_value="0.25",
                description="Maximum linear teleop speed/command strength.",
            ),
            DeclareLaunchArgument(
                "max_angular_speed",
                default_value="0.60",
                description="Maximum angular teleop speed/command strength.",
            ),
            DeclareLaunchArgument(
                "key_timeout_s",
                default_value="0.50",
                description="Stop if no key is received within this time.",
            ),
            LogInfo(
                msg=[
                    "Starting Robot Savo teleop control | startup_mode=",
                    startup_mode,
                    " | keyboard output=",
                    cmd_vel_topic,
                    " | expected safety chain: /cmd_vel -> /cmd_vel_safe",
                ]
            ),

            # -----------------------------------------------------------------
            # Keyboard teleop source
            # -----------------------------------------------------------------
            Node(
                condition=IfCondition(use_keyboard),
                package="savo_control",
                executable="keyboard_teleop_node.py",
                name="keyboard_teleop_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    control_common_yaml,
                    {
                        "cmd_vel_topic": cmd_vel_topic,
                        "default_linear_speed": default_linear_speed,
                        "default_angular_speed": default_angular_speed,
                        "max_linear_speed": max_linear_speed,
                        "max_angular_speed": max_angular_speed,
                        "key_timeout_s": key_timeout_s,
                    },
                ],
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
                        # Keep teleop launch explicitly manual.
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
                        "default_profile": "manual_mapping",
                    },
                ],
            ),

            # -----------------------------------------------------------------
            # Optional recovery/status tools
            # -----------------------------------------------------------------
            Node(
                condition=IfCondition(use_recovery_manager),
                package="savo_control",
                executable="recovery_manager_node",
                name="recovery_manager_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    recovery_yaml,
                ],
            ),
            Node(
                condition=IfCondition(use_backup_escape),
                package="savo_control",
                executable="backup_escape_node",
                name="backup_escape_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    recovery_yaml,
                ],
            ),

            # -----------------------------------------------------------------
            # Status/diagnostics
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
                        "watch_odom": False,
                        "publish_console_log": True,
                    },
                ],
            ),
            Node(
                condition=IfCondition(use_recovery_status),
                package="savo_control",
                executable="recovery_status_node.py",
                name="recovery_status_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    recovery_yaml,
                    {
                        "watch_odom": False,
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
                        "watch_odom": False,
                        "watch_depth": False,
                    },
                ],
            ),
        ]
    )