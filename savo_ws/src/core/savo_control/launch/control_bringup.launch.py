#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Main bringup for the Robot Savo control layer."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("savo_control")

    control_common_yaml = os.path.join(pkg_share, "config", "control_common.yaml")
    control_mode_manager_yaml = os.path.join(
        pkg_share, "config", "control_mode_manager.yaml"
    )
    twist_mux_yaml = os.path.join(pkg_share, "config", "twist_mux.yaml")
    cmd_vel_shaper_yaml = os.path.join(pkg_share, "config", "cmd_vel_shaper.yaml")
    recovery_yaml = os.path.join(pkg_share, "config", "recovery.yaml")
    stuck_detector_yaml = os.path.join(pkg_share, "config", "stuck_detector.yaml")
    distance_approach_yaml = os.path.join(pkg_share, "config", "distance_approach.yaml")

    startup_mode = LaunchConfiguration("startup_mode")

    use_mode_manager = LaunchConfiguration("use_mode_manager")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_cmd_vel_shaper = LaunchConfiguration("use_cmd_vel_shaper")
    use_recovery_manager = LaunchConfiguration("use_recovery_manager")
    use_backup_escape = LaunchConfiguration("use_backup_escape")
    use_stuck_detector = LaunchConfiguration("use_stuck_detector")
    use_distance_approach = LaunchConfiguration("use_distance_approach")
    use_control_status = LaunchConfiguration("use_control_status")
    use_recovery_status = LaunchConfiguration("use_recovery_status")
    use_dashboard = LaunchConfiguration("use_dashboard")

    approach_impl = LaunchConfiguration("approach_impl")
    distance_auto_start = LaunchConfiguration("distance_auto_start")
    target_distance_m = LaunchConfiguration("target_distance_m")

    distance_cpp_condition = IfCondition(
        PythonExpression(
            ["'", use_distance_approach, "' == 'true' and '", approach_impl, "' == 'cpp'"]
        )
    )
    distance_py_condition = IfCondition(
        PythonExpression(
            ["'", use_distance_approach, "' == 'true' and '", approach_impl, "' == 'py'"]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "startup_mode",
                default_value="STOP",
                description="Initial control mode: STOP, MANUAL, AUTO, NAV, RECOVERY.",
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
                default_value="true",
                description="Start recovery_manager_node.",
            ),
            DeclareLaunchArgument(
                "use_backup_escape",
                default_value="false",
                description="Start backup_escape_node.",
            ),
            DeclareLaunchArgument(
                "use_stuck_detector",
                default_value="false",
                description="Start stuck_detector_node.",
            ),
            DeclareLaunchArgument(
                "use_distance_approach",
                default_value="false",
                description="Start distance approach controller.",
            ),
            DeclareLaunchArgument(
                "approach_impl",
                default_value="cpp",
                description="Distance approach implementation: cpp or py.",
            ),
            DeclareLaunchArgument(
                "distance_auto_start",
                default_value="false",
                description="Start distance approach immediately.",
            ),
            DeclareLaunchArgument(
                "target_distance_m",
                default_value="0.60",
                description="Distance approach target in meters.",
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
                description="Start dashboard. Usually run it in a separate terminal.",
            ),
            LogInfo(
                msg=[
                    "Starting savo_control | startup_mode=",
                    startup_mode,
                    " | distance_approach=",
                    use_distance_approach,
                    " | approach_impl=",
                    approach_impl,
                ]
            ),
            Node(
                condition=IfCondition(use_mode_manager),
                package="savo_control",
                executable="control_mode_manager_node",
                name="control_mode_manager_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    control_mode_manager_yaml,
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
                ],
            ),
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
            Node(
                condition=distance_cpp_condition,
                package="savo_control",
                executable="distance_approach_node",
                name="distance_approach_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    distance_approach_yaml,
                    {
                        "auto_start": ParameterValue(
                            distance_auto_start,
                            value_type=bool,
                        ),
                        "target_distance_m": ParameterValue(
                            target_distance_m,
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                condition=distance_py_condition,
                package="savo_control",
                executable="distance_pid_test_node.py",
                name="distance_pid_test_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    distance_approach_yaml,
                    {
                        "auto_start": ParameterValue(
                            distance_auto_start,
                            value_type=bool,
                        ),
                        "target_distance_m": ParameterValue(
                            target_distance_m,
                            value_type=float,
                        ),
                    },
                ],
            ),
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
            Node(
                condition=IfCondition(use_dashboard),
                package="savo_control",
                executable="control_dashboard_node.py",
                name="control_dashboard_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    control_common_yaml,
                ],
            ),
        ]
    )
