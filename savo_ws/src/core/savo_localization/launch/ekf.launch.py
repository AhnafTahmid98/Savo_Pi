#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch Robot Savo EKF localization."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("savo_localization")

    ekf_config = LaunchConfiguration("ekf_config")
    vo_config = LaunchConfiguration("vo_config")
    topics_config = LaunchConfiguration("topics_config")
    frames_config = LaunchConfiguration("frames_config")
    diagnostics_config = LaunchConfiguration("diagnostics_config")
    profile_config = LaunchConfiguration("profile_config")
    vo_profile_config = LaunchConfiguration("vo_profile_config")

    use_vo = LaunchConfiguration("use_vo")
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
                "vo_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "ekf_vo_input_optional.yaml"]
                ),
                description="Optional EKF overlay for /vo/odom from savo-edge.",
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
                    [
                        package_share,
                        "config",
                        "profiles",
                        "robot_savo_4enc_imu_ekf.yaml",
                    ]
                ),
                description="Baseline EKF and health profile overlay.",
            ),
            DeclareLaunchArgument(
                "vo_profile_config",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "profiles",
                        "robot_savo_4enc_imu_vo_ekf.yaml",
                    ]
                ),
                description="VO-enabled EKF and health profile overlay.",
            ),
            DeclareLaunchArgument(
                "use_vo",
                default_value="false",
                description="Fuse /vo/odom from savo-edge.",
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
                condition=UnlessCondition(use_vo),
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
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=IfCondition(use_vo),
                parameters=[
                    topics_config,
                    frames_config,
                    ekf_config,
                    vo_config,
                    vo_profile_config,
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
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_health, "' == 'true' and '", use_vo, "' == 'false'"]
                    )
                ),
                parameters=[
                    topics_config,
                    frames_config,
                    diagnostics_config,
                    profile_config,
                ],
            ),
            Node(
                package="savo_localization",
                executable="localization_health_node",
                name="localization_health_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_health, "' == 'true' and '", use_vo, "' == 'true'"]
                    )
                ),
                parameters=[
                    topics_config,
                    frames_config,
                    diagnostics_config,
                    vo_profile_config,
                ],
            ),
        ]
    )
