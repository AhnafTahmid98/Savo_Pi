#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the PC-safe savo_speech dry-run runtime."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("savo_speech"),
                    "launch",
                    "speech_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "profile": "dryrun_no_hardware",
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for dry-run speech nodes.",
            ),
            bringup,
        ]
    )
