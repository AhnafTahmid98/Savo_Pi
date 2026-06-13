#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shortcut launch for Robot Savo four-wheel encoder odometry."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("savo_localization")

    encoders_config = LaunchConfiguration("encoders_config")
    wheel_odom_config = LaunchConfiguration("wheel_odom_config")
    topics_config = LaunchConfiguration("topics_config")
    frames_config = LaunchConfiguration("frames_config")
    profile_config = LaunchConfiguration("profile_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "encoders_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "encoders.yaml"]
                ),
                description="Four-wheel encoder GPIO configuration.",
            ),
            DeclareLaunchArgument(
                "wheel_odom_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "wheel_odom.yaml"]
                ),
                description="Wheel odometry configuration.",
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
                "profile_config",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "profiles",
                        "wheel_odom_4enc.yaml",
                    ]
                ),
                description="Four-encoder wheel odometry profile.",
            ),
            Node(
                package="savo_localization",
                executable="wheel_odom_node",
                name="wheel_odom_node",
                output="screen",
                parameters=[
                    topics_config,
                    frames_config,
                    encoders_config,
                    wheel_odom_config,
                    profile_config,
                ],
            ),
        ]
    )
