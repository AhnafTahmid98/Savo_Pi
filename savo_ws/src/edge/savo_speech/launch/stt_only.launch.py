#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch only the Faster-Whisper STT node."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("savo_speech")
    stt_config_path = os.path.join(pkg_share, "config", "stt_whisper.yaml")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 log level for stt_node (debug, info, warn, error, fatal).",
    )

    stt_node = Node(
        package="savo_speech",
        executable="stt_node",
        name="stt_node",
        output="screen",
        emulate_tty=True,
        parameters=[stt_config_path],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    info = LogInfo(msg=(
        "[savo_speech/stt_only.launch] Starting Faster-Whisper STT node "
        f"with config: {stt_config_path}"
    ))

    return LaunchDescription(
        [
            log_level_arg,
            info,
            stt_node,
        ]
    )
