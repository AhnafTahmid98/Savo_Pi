#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — TTS-only launch (Piper)

Brings up just the Piper-based TTS node with its YAML config.
Use this when you want to test /savo_speech/tts_text → audio only.

Example:

  cd ~/Savo_Pi
  source tools/scripts/env.sh

  ros2 launch savo_speech tts_only.launch.py

Then in another terminal:

  ros2 topic pub /savo_speech/tts_text std_msgs/String \
    "{data: 'Hello, I am Robot Savo, testing TTS-only launch.'}" --once
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Package share so we can find the YAML config at runtime
    pkg_share = FindPackageShare("savo_speech")

    # ------------------------------------------------------------------
    # Launch arguments (overrides for key TTS parameters)
    # ------------------------------------------------------------------
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

    # Path to the Piper TTS parameter file (tts_piper.yaml)
    tts_params = PathJoinSubstitution(
        [pkg_share, "config", "tts_piper.yaml"]
    )

    # ------------------------------------------------------------------
    # TTS node
    # ------------------------------------------------------------------
    tts_node = Node(
        package="savo_speech",
        executable="tts_node",
        name="tts_node",
        output="screen",
        parameters=[
            tts_params,
            {
                # These two can be overridden from the command line:
                #   ros2 launch savo_speech tts_only.launch.py voice_profile:=female
                #   ros2 launch savo_speech tts_only.launch.py output_device_index:=-1
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

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            voice_profile_arg,
            output_device_arg,
            log_level_arg,
            tts_node,
        ]
    )
