#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Bench launch for testing the Robot Savo BNO055 IMU."""

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

    use_imu = LaunchConfiguration("use_imu")
    use_health = LaunchConfiguration("use_health")
    calibration_restore_enabled = LaunchConfiguration(
        "calibration_restore_enabled"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imu_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "imu.yaml"]
                ),
                description="BNO055 IMU configuration.",
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
                    [package_share, "config", "profiles", "bench_imu.yaml"]
                ),
                description="IMU bench profile overlay.",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="true",
                description="Start the C++ IMU node.",
            ),
            DeclareLaunchArgument(
                "use_health",
                default_value="false",
                description="Start localization_health_node for IMU checks.",
            ),
            DeclareLaunchArgument(
                "calibration_restore_enabled",
                default_value="true",
                description="Restore the verified BNO055 calibration profile at startup.",
            ),
            Node(
                package="savo_localization",
                executable="imu_node",
                name="imu_node",
                output="screen",
                condition=IfCondition(use_imu),
                parameters=[
                    topics_config,
                    frames_config,
                    imu_config,
                    profile_config,
                    {
                        "calibration_restore_enabled": calibration_restore_enabled,
                    },
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
