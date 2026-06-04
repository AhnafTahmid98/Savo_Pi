#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Auto-test launch: control chain plus auto_test_manager_node. First tests: wheels lifted, low speed, safety gate running."""

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
    auto_test_modes_yaml = os.path.join(pkg_share, "config", "auto_test_modes.yaml")
    recovery_yaml = os.path.join(pkg_share, "config", "recovery.yaml")

    use_mode_manager = LaunchConfiguration("use_mode_manager")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_cmd_vel_shaper = LaunchConfiguration("use_cmd_vel_shaper")
    use_auto_test_manager = LaunchConfiguration("use_auto_test_manager")
    use_control_status = LaunchConfiguration("use_control_status")
    use_recovery_manager = LaunchConfiguration("use_recovery_manager")
    use_backup_escape = LaunchConfiguration("use_backup_escape")
    use_dashboard = LaunchConfiguration("use_dashboard")

    startup_mode = LaunchConfiguration("startup_mode")
    auto_start = LaunchConfiguration("auto_start")
    default_test_name = LaunchConfiguration("default_test_name")

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
                "use_auto_test_manager",
                default_value="true",
                description="Start auto_test_manager_node.py.",
            ),
            DeclareLaunchArgument(
                "use_control_status",
                default_value="true",
                description="Start control_status_node.py.",
            ),
            DeclareLaunchArgument(
                "use_recovery_manager",
                default_value="false",
                description=(
                    "Start recovery_manager_node. Default false for simple auto tests."
                ),
            ),
            DeclareLaunchArgument(
                "use_backup_escape",
                default_value="false",
                description=(
                    "Start backup_escape_node. Default false for simple auto tests."
                ),
            ),
            DeclareLaunchArgument(
                "use_dashboard",
                default_value="false",
                description=(
                    "Start curses dashboard. Usually false; run separately in its own terminal."
                ),
            ),
            DeclareLaunchArgument(
                "startup_mode",
                default_value="AUTO",
                description="Startup control mode. For auto tests this should be AUTO.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description=(
                    "If true, auto_test_manager_node starts default_test_name immediately. "
                    "Keep false for safety unless wheels are lifted."
                ),
            ),
            DeclareLaunchArgument(
                "default_test_name",
                default_value="forward_slow",
                description=(
                    "Default auto test name. Examples: forward_slow, backward_slow, "
                    "strafe_left_slow, rotate_ccw_slow, forward_pulse, mecanum_sanity."
                ),
            ),
            LogInfo(
                msg=[
                    "Starting Robot Savo auto test control | startup_mode=",
                    startup_mode,
                    " | auto_start=",
                    auto_start,
                    " | default_test=",
                    default_test_name,
                    " | output path: /cmd_vel_auto -> /cmd_vel_mux -> /cmd_vel -> /cmd_vel_safe",
                ]
            ),

            # -----------------------------------------------------------------
            # Core mode/mux/shaper chain
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
            # Auto test manager
            # -----------------------------------------------------------------
            Node(
                condition=IfCondition(use_auto_test_manager),
                package="savo_control",
                executable="auto_test_manager_node.py",
                name="auto_test_manager_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    auto_test_modes_yaml,
                    {
                        "auto_start": auto_start,
                        "default_test_name": default_test_name,
                        "request_auto_mode_on_start": True,
                        "required_mode": "AUTO",
                    },
                ],
            ),

            # -----------------------------------------------------------------
            # Optional recovery helpers
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
                        "watch_odom": False,
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
                        "watch_odom": False,
                        "watch_depth": False,
                    },
                ],
            ),
        ]
    )