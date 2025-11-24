#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Speech bringup launch (STT + TTS + mouth animation)

This launch file starts the full speech stack for Robot Savo:

  - Faster-Whisper STT node        (savo_speech/stt_node)
  - Piper TTS node                 (savo_speech/tts_node)
  - Optional mouth animation node  (savo_speech/mouth_anim_node)

It loads parameters from the installed YAML configs:

  - config/stt_whisper.yaml
  - config/tts_piper.yaml
  - config/mouth_anim.yaml   (optional, if you have this file)

Typical usage on the Pi:

  cd ~/Savo_Pi
  source tools/scripts/env.sh

  # Start full speech stack with default INFO logs
  ros2 launch savo_speech speech_bringup.launch.py

  # More verbose for debugging:
  ros2 launch savo_speech speech_bringup.launch.py log_level:=debug

You can also enable/disable individual components:

  ros2 launch savo_speech speech_bringup.launch.py \
    enable_stt:=true enable_tts:=true enable_mouth_anim:=false
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Locate package share + config files
    # ------------------------------------------------------------------
    pkg_share = get_package_share_directory("savo_speech")

    stt_config_path = os.path.join(pkg_share, "config", "stt_whisper.yaml")
    tts_config_path = os.path.join(pkg_share, "config", "tts_piper.yaml")
    mouth_config_path = os.path.join(pkg_share, "config", "mouth_anim.yaml")

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Global ROS 2 log level for speech nodes (debug, info, warn, error, fatal).",
    )

    enable_stt_arg = DeclareLaunchArgument(
        "enable_stt",
        default_value="true",
        description="Whether to start the Faster-Whisper STT node.",
    )

    enable_tts_arg = DeclareLaunchArgument(
        "enable_tts",
        default_value="true",
        description="Whether to start the Piper TTS node.",
    )

    enable_mouth_anim_arg = DeclareLaunchArgument(
        "enable_mouth_anim",
        default_value="true",
        description="Whether to start the mouth animation node.",
    )

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
        condition=IfCondition(LaunchConfiguration("enable_stt")),
    )

    stt_info = LogInfo(
        condition=IfCondition(LaunchConfiguration("enable_stt")),
        msg=(
            "[speech_bringup] Starting STT (Faster-Whisper) node "
            f"with config: {stt_config_path}"
        ),
    )

    # ------------------------------------------------------------------
    # TTS node (Piper)
    # ------------------------------------------------------------------
    tts_node = Node(
        package="savo_speech",
        executable="tts_node",
        name="tts_node",
        output="screen",
        emulate_tty=True,
        parameters=[tts_config_path],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        condition=IfCondition(LaunchConfiguration("enable_tts")),
    )

    tts_info = LogInfo(
        condition=IfCondition(LaunchConfiguration("enable_tts")),
        msg=(
            "[speech_bringup] Starting TTS (Piper) node "
            f"with config: {tts_config_path}"
        ),
    )

    # ------------------------------------------------------------------
    # Mouth animation node (optional)
    # ------------------------------------------------------------------
    mouth_anim_node = Node(
        package="savo_speech",
        executable="mouth_anim_node",  # make sure this matches setup.cfg entry point
        name="mouth_anim_node",
        output="screen",
        emulate_tty=True,
        parameters=[mouth_config_path],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        condition=IfCondition(LaunchConfiguration("enable_mouth_anim")),
    )

    mouth_info = LogInfo(
        condition=IfCondition(LaunchConfiguration("enable_mouth_anim")),
        msg=(
            "[speech_bringup] Starting mouth animation node "
            f"with config: {mouth_config_path}"
        ),
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            log_level_arg,
            enable_stt_arg,
            enable_tts_arg,
            enable_mouth_anim_arg,
            # Info messages
            stt_info,
            tts_info,
            mouth_info,
            # Nodes
            stt_node,
            tts_node,
            mouth_anim_node,
        ]
    )
