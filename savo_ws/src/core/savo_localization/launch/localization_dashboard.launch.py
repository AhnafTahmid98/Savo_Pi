#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the Robot Savo localization dashboard only."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("savo_localization")

    topics_config = LaunchConfiguration("topics_config")
    frames_config = LaunchConfiguration("frames_config")
    diagnostics_config = LaunchConfiguration("diagnostics_config")
    dashboard_config = LaunchConfiguration("dashboard_config")
    profile_config = LaunchConfiguration("profile_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "topics_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "topics.yaml"]
                ),
                description="Shared localization topic contract.",
            ),
            DeclareLaunchArgument(
                "frames_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "frames.yaml"]
                ),
                description="Shared localization frame contract.",
            ),
            DeclareLaunchArgument(
                "diagnostics_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "diagnostics.yaml"]
                ),
                description="Localization diagnostic behavior config.",
            ),
            DeclareLaunchArgument(
                "dashboard_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "localization_dashboard.yaml"]
                ),
                description="Localization dashboard display config.",
            ),
            DeclareLaunchArgument(
                "profile_config",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "profiles",
                        "robot_savo_4enc_imu_ekf.yaml",
                    ]
                ),
                description="Dashboard profile overlay.",
            ),
            Node(
                package="savo_localization",
                executable="localization_dashboard.py",
                name="localization_dashboard",
                output="screen",
                emulate_tty=True,
                parameters=[
                    topics_config,
                    frames_config,
                    diagnostics_config,
                    dashboard_config,
                    profile_config,
                ],
            ),
        ]
    )
