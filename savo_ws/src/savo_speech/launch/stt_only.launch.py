#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — STT-only launch (Faster-Whisper)

This launch file starts only the speech-to-text pipeline, using the
savo_speech/stt_node with the Faster-Whisper backend.

It loads parameters from stt_whisper.yaml, which controls:
  - model_dir / model_name (e.g. small.en)
  - device (cpu / cuda / auto)
  - VAD / chunking behaviour
  - input/output topics

Typical usage on the Pi:

  cd ~/Savo_Pi
  source tools/scripts/env.sh

  ros2 launch savo_speech stt_only.launch.py

You can still override log level from the command line:

  ros2 launch savo_speech stt_only.launch.py log_level:=debug
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Locate package share + STT config file
    # ------------------------------------------------------------------
    pkg_share = get_package_share_directory("savo_speech")
    stt_config_path = os.path.join(pkg_share, "config", "stt_whisper.yaml")

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 log level for stt_node (debug, info, warn, error, fatal).",
    )

    # In the future we could add overrides like:
    #   model_name:=medium.en
    #   device:=cuda
    # but right now the defaults are taken from stt_whisper.yaml.

    # ------------------------------------------------------------------
    # STT node (Faster-Whisper)
    # ------------------------------------------------------------------
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
