#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch only the Piper TTS node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("savo_speech")

    voice_profile_arg = DeclareLaunchArgument(
        "voice_profile",
        default_value="male",
        description="TTS voice profile to use: 'male' or 'female'",
    )

    output_device_arg = DeclareLaunchArgument(
        "output_device_index",
        default_value="0",
        description=(
            "sounddevice output index "
            "(-1 = default device, 0 = first device, 1 = second, ...)"
        ),
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="ROS 2 log level for the TTS node (DEBUG, INFO, WARN, ERROR)",
    )

    tts_params = PathJoinSubstitution(
        [pkg_share, "config", "tts_piper.yaml"]
    )

    tts_node = Node(
        package="savo_speech",
        executable="tts_node",
        name="tts_node",
        output="screen",
        parameters=[
            tts_params,
            {
                "voice_profile": LaunchConfiguration("voice_profile"),
                "output_device_index": LaunchConfiguration("output_device_index"),
            },
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    return LaunchDescription(
        [
            voice_profile_arg,
            output_device_arg,
            log_level_arg,
            tts_node,
        ]
    )
