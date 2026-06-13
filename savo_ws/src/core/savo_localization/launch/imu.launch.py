#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the Robot Savo IMU pipeline."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("savo_localization")

    imu_config = LaunchConfiguration("imu_config")
    topics_config = LaunchConfiguration("topics_config")
    frames_config = LaunchConfiguration("frames_config")
    diagnostics_config = LaunchConfiguration("diagnostics_config")
    profile_config = LaunchConfiguration("profile_config")

    use_health = LaunchConfiguration("use_health")
    use_dashboard = LaunchConfiguration("use_dashboard")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imu_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "imu.yaml"]
                ),
                description="IMU node configuration.",
            ),
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
                description="Diagnostic behavior profile.",
            ),
            DeclareLaunchArgument(
                "profile_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "profiles", "bench_imu.yaml"]
                ),
                description="IMU launch profile overlay.",
            ),
            DeclareLaunchArgument(
                "use_health",
                default_value="false",
                description="Start localization_health_node with IMU-only checks.",
            ),
            DeclareLaunchArgument(
                "use_dashboard",
                default_value="false",
                description="Start localization_dashboard for terminal monitoring.",
            ),
            Node(
                package="savo_localization",
                executable="imu_node",
                name="imu_node",
                output="screen",
                parameters=[
                    imu_config,
                    topics_config,
                    frames_config,
                    profile_config,
                ],
            ),
            Node(
                package="savo_localization",
                executable="localization_health_node.py",
                name="localization_health_node",
                output="screen",
                condition=IfCondition(use_health),
                parameters=[
                    topics_config,
                    frames_config,
                    diagnostics_config,
                    profile_config,
                ],
            ),
            Node(
                package="savo_localization",
                executable="localization_dashboard.py",
                name="localization_dashboard",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(use_dashboard),
                parameters=[
                    topics_config,
                    frames_config,
                    diagnostics_config,
                    profile_config,
                ],
            ),
        ]
    )