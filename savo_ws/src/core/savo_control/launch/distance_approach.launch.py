#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Distance approach launch: control chain plus C++ production or Python fallback controller."""

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
    distance_approach_yaml = os.path.join(pkg_share, "config", "distance_approach.yaml")

    approach_impl = LaunchConfiguration("approach_impl")
    startup_mode = LaunchConfiguration("startup_mode")
    auto_start = LaunchConfiguration("auto_start")
    target_distance_m = LaunchConfiguration("target_distance_m")

    use_mode_manager = LaunchConfiguration("use_mode_manager")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_cmd_vel_shaper = LaunchConfiguration("use_cmd_vel_shaper")
    use_distance_approach = LaunchConfiguration("use_distance_approach")
    use_control_status = LaunchConfiguration("use_control_status")
    use_dashboard = LaunchConfiguration("use_dashboard")

    cpp_condition = IfCondition(PythonExpression(["'", approach_impl, "' == 'cpp'"]))
    py_condition = IfCondition(PythonExpression(["'", approach_impl, "' == 'py'"]))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "approach_impl",
                default_value="cpp",
                description="Distance approach implementation: cpp or py.",
            ),
            DeclareLaunchArgument(
                "startup_mode",
                default_value="AUTO",
                description="Initial control mode for distance approach.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="Start approach immediately. Keep false for real hardware.",
            ),
            DeclareLaunchArgument(
                "target_distance_m",
                default_value="0.60",
                description="Target front distance in meters.",
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
                "use_distance_approach",
                default_value="true",
                description="Start distance approach controller.",
            ),
            DeclareLaunchArgument(
                "use_control_status",
                default_value="true",
                description="Start Python control_status_node.",
            ),
            DeclareLaunchArgument(
                "use_dashboard",
                default_value="false",
                description="Start dashboard. Usually run it separately.",
            ),
            LogInfo(
                msg=[
                    "Starting distance approach | impl=",
                    approach_impl,
                    " | startup_mode=",
                    startup_mode,
                    " | auto_start=",
                    auto_start,
                    " | target_distance_m=",
                    target_distance_m,
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
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            approach_impl,
                            "' == 'cpp' and '",
                            use_distance_approach,
                            "' == 'true'",
                        ]
                    )
                ),
                package="savo_control",
                executable="distance_approach_node",
                name="distance_approach_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    distance_approach_yaml,
                    {
                        "auto_start": ParameterValue(auto_start, value_type=bool),
                        "target_distance_m": ParameterValue(
                            target_distance_m, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            approach_impl,
                            "' == 'py' and '",
                            use_distance_approach,
                            "' == 'true'",
                        ]
                    )
                ),
                package="savo_control",
                executable="distance_pid_test_node.py",
                name="distance_pid_test_node",
                output="screen",
                parameters=[
                    control_common_yaml,
                    distance_approach_yaml,
                    {
                        "auto_start": ParameterValue(auto_start, value_type=bool),
                        "target_distance_m": ParameterValue(
                            target_distance_m, value_type=float
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
                    {
                        "watch_depth": True,
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
                        "watch_depth": True,
                        "watch_odom": False,
                    },
                ],
            ),
        ]
    )
