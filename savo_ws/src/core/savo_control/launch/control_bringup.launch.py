#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Main bringup for the savo_control layer: mode manager, twist mux, shaper, recovery, and status nodes."""

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
    stuck_detector_yaml = os.path.join(pkg_share, "config", "stuck_detector.yaml")

    use_mode_manager = LaunchConfiguration("use_mode_manager")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_cmd_vel_shaper = LaunchConfiguration("use_cmd_vel_shaper")
    use_recovery_manager = LaunchConfiguration("use_recovery_manager")
    use_backup_escape = LaunchConfiguration("use_backup_escape")
    use_stuck_detector = LaunchConfiguration("use_stuck_detector")
    use_control_status = LaunchConfiguration("use_control_status")
    use_recovery_status = LaunchConfiguration("use_recovery_status")
    use_dashboard = LaunchConfiguration("use_dashboard")

    # Default startup mode should remain STOP.
    startup_mode = LaunchConfiguration("startup_mode")

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
                "use_recovery_manager",
                default_value="true",
                description="Start recovery_manager_node.",
            ),
            DeclareLaunchArgument(
                "use_backup_escape",
                default_value="true",
                description="Start backup_escape_node.",
            ),
            DeclareLaunchArgument(
                "use_stuck_detector",
                default_value="false",
                description=(
                    "Start stuck_detector_node. Default false until "
                    "/odometry/filtered is stable."
                ),
            ),
            DeclareLaunchArgument(
                "use_control_status",
                default_value="true",
                description="Start Python control_status_node.",
            ),
            DeclareLaunchArgument(
                "use_recovery_status",
                default_value="true",
                description="Start Python recovery_status_node.",
            ),
            DeclareLaunchArgument(
                "use_dashboard",
                default_value="false",
                description=(
                    "Start curses dashboard. Usually false because dashboard "
                    "should run in its own terminal."
                ),
            ),
            DeclareLaunchArgument(
                "startup_mode",
                default_value="STOP",
                description="Initial control mode: STOP, MANUAL, AUTO, NAV, RECOVERY.",
            ),
            LogInfo(
                msg=[
                    "Starting Robot Savo control bringup | startup_mode=",
                    startup_mode,
                    " | output=/cmd_vel, safety gate should produce /cmd_vel_safe",
                ]
            ),

            # -----------------------------------------------------------------
            # Core C++ control nodes
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
                ],
            ),

            # -----------------------------------------------------------------
            # Recovery / stuck logic
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
            Node(
                condition=IfCondition(use_stuck_detector),
                package="savo_control",
                executable="stuck_detector_node",
                name="stuck_detector_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    stuck_detector_yaml,
                ],
            ),

            # -----------------------------------------------------------------
            # Python status nodes
            # -----------------------------------------------------------------
            Node(
                condition=IfCondition(use_control_status),
                package="savo_control",
                executable="control_status_node.py",
                name="control_status_node",
                output="screen",
                parameters=[
                    control_common_yaml,
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
                ],
            ),

            # -----------------------------------------------------------------
            # Optional dashboard
            # -----------------------------------------------------------------
            Node(
                condition=IfCondition(use_dashboard),
                package="savo_control",
                executable="control_dashboard_node.py",
                name="control_dashboard_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                ],
            ),
        ]
    )