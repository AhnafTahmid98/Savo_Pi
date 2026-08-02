#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dry-run launch for Robot Savo localization without hardware nodes."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("savo_localization")

    ekf_config = LaunchConfiguration("ekf_config")
    topics_config = LaunchConfiguration("topics_config")
    frames_config = LaunchConfiguration("frames_config")
    diagnostics_config = LaunchConfiguration("diagnostics_config")
    profile_config = LaunchConfiguration("profile_config")

    use_ekf = LaunchConfiguration("use_ekf")
    use_health = LaunchConfiguration("use_health")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ekf_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "ekf_odom.yaml"]
                ),
                description="Base robot_localization EKF configuration.",
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
                description="Localization diagnostic behavior config.",
            ),
            DeclareLaunchArgument(
                "profile_config",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "profiles",
                        "dryrun_localization.yaml",
                    ]
                ),
                description="Dry-run localization profile overlay.",
            ),
            DeclareLaunchArgument(
                "use_ekf",
                default_value="true",
                description="Start robot_localization EKF for dry-run input topics.",
            ),
            DeclareLaunchArgument(
                "use_health",
                default_value="false",
                description="Start localization_health_node.",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=IfCondition(use_ekf),
                parameters=[
                    topics_config,
                    frames_config,
                    ekf_config,
                    profile_config,
                ],
                remappings=[
                    ("odometry/filtered", "/odometry/filtered"),
                ],
            ),
            Node(
                package="savo_localization",
                executable="localization_health_node",
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
        ]
    )
