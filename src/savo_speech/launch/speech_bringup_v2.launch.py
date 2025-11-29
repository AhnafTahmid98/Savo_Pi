#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Speech bringup v2 (Robot_Savo_Server /speech pipeline)

This launch file starts on the Pi:
  - remote_speech_client_node
      * records from ReSpeaker mic
      * sends audio to Robot_Savo_Server /speech endpoint
      * publishes:
          /savo_speech/stt_text        (String)
          /savo_speech/tts_text        (String)
          /savo_intent/intent_result   (savo_msgs/IntentResult)
          /savo_ui/face_state          (String: idle/listening/thinking/speaking)

  - tts_node (Piper TTS)
      * subscribes to /savo_speech/tts_text
      * plays audio via sounddevice
      * publishes:
          /savo_speech/tts_speaking    (Bool)   — STT gate + face state
          /savo_speech/tts_pcm         (Int16MultiArray) — full utterance PCM
          /savo_speech/tts_done        (Empty) (optional)
          /savo_speech/mouth_open      (Float32) (simple mouth flag, optional)

  - optional mouth_anim_node
      * subscribes to /savo_speech/tts_pcm (PCM mode) or /savo_speech/mouth_open
      * publishes /savo_ui/mouth_level (Float32 0.0–1.0) for the face UI

Usage on the Pi:

  cd ~/Savo_Pi
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  source tools/scripts/env_robot_server.sh

  ros2 launch savo_speech speech_bringup_v2.launch.py \
    robot_id:=robot_savo_pi \
    enable_mouth_anim:=true \
    timeout_s:=20.0

Notes:
  - speech_remote.yaml controls remote_speech_client_node params
    (SPEECH_SERVER_URL, device index, VAD, wake words, etc.).
  - tts_piper.yaml controls Piper TTS (voices, device, topics).
  - mouth_anim uses PCM mode by default (real audio-driven mouth).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="robot_savo_pi",
        description="Robot ID to pass into remote_speech_client_node (IntentResult.robot_id).",
    )

    enable_mouth_anim_arg = DeclareLaunchArgument(
        "enable_mouth_anim",
        default_value="true",
        choices=["true", "false"],
        description="If true, also start the mouth_anim_node for screen animation.",
    )

    # Kept for compatibility, wired to request_timeout_s in the client.
    timeout_s_arg = DeclareLaunchArgument(
        "timeout_s",
        default_value="20.0",
        description=(
            "Timeout (seconds) for a single /speech HTTP request. "
            "Passed through as request_timeout_s to remote_speech_client_node."
        ),
    )

    # Mouth animation mode: "pcm" (recommended) or "activity"
    mouth_mode_arg = DeclareLaunchArgument(
        "mouth_mode",
        default_value="pcm",
        description=(
            "Mouth animation mode: 'pcm' uses full TTS audio from /savo_speech/tts_pcm, "
            "'activity' uses simple /savo_speech/mouth_open Float32."
        ),
    )

    # Mouth animation debug logging flag
    mouth_debug_arg = DeclareLaunchArgument(
        "mouth_debug_logging",
        default_value="false",
        choices=["true", "false"],
        description="Enable extra debug logging in mouth_anim_node.",
    )

    robot_id = LaunchConfiguration("robot_id")
    enable_mouth_anim = LaunchConfiguration("enable_mouth_anim")
    timeout_s = LaunchConfiguration("timeout_s")
    mouth_mode = LaunchConfiguration("mouth_mode")
    mouth_debug_logging = LaunchConfiguration("mouth_debug_logging")

    # -------------------------------------------------------------------------
    # Config file paths
    # -------------------------------------------------------------------------
    pkg_share = get_package_share_directory("savo_speech")

    speech_remote_yaml = os.path.join(
        pkg_share, "config", "speech_remote.yaml"
    )
    tts_piper_yaml = os.path.join(
        pkg_share, "config", "tts_piper.yaml"
    )

    # -------------------------------------------------------------------------
    # Nodes
    # -------------------------------------------------------------------------

    # 1) Remote speech client node
    #
    # - Records from ReSpeaker (input_device_index from speech_remote.yaml).
    # - Sends utterances to Robot_Savo_Server /speech endpoint.
    # - Publishes:
    #     /savo_speech/stt_text        (String)
    #     /savo_speech/tts_text        (String)
    #     /savo_intent/intent_result   (IntentResult)
    #     /savo_ui/face_state          (String state for display_manager)
    #
    remote_speech_node = Node(
        package="savo_speech",
        executable="remote_speech_client_node",
        name="remote_speech_client_node",
        output="screen",
        parameters=[
            speech_remote_yaml,
            {
                # Override from launch arguments:
                "robot_id": robot_id,
                # Wire timeout_s → request_timeout_s in the node:
                "request_timeout_s": timeout_s,
            },
        ],
    )

    # 2) TTS node (Piper)
    #
    # - Subscribes to /savo_speech/tts_text (String).
    # - Plays audio via sounddevice.
    # - Publishes:
    #     /savo_speech/tts_speaking  (Bool) as a gate for STT + face state
    #     /savo_speech/tts_pcm       (Int16MultiArray) for audio-driven mouth
    #     /savo_speech/tts_done      (Empty) optional
    #     /savo_speech/mouth_open    (Float32) optional
    #
    tts_node = Node(
        package="savo_speech",
        executable="tts_node",
        name="tts_node",
        output="screen",
        parameters=[tts_piper_yaml],
    )

    # 3) Optional mouth animation node
    #
    # - PCM mode (default):
    #     subscribes to /savo_speech/tts_pcm (Int16MultiArray),
    #     outputs /savo_ui/mouth_level (Float32 0–1).
    #
    # - Activity mode:
    #     subscribes to /savo_speech/mouth_open (Float32),
    #     outputs /savo_ui/mouth_level (Float32 0–1).
    #
    # - Only started if enable_mouth_anim:=true.
    #
    mouth_anim_node = Node(
        package="savo_speech",
        executable="mouth_anim_node",
        name="mouth_anim_node",
        output="screen",
        condition=IfCondition(enable_mouth_anim),
        parameters=[
            {
                "mode": mouth_mode,
                "debug_logging": mouth_debug_logging,
                # You can also override other parameters here if needed, e.g.:
                # "pcm_sample_rate": 16000.0,
                # "update_rate_hz": 30.0,
            }
        ],
    )

    # -------------------------------------------------------------------------
    # Info log (static message to avoid frontend substitution issues)
    # -------------------------------------------------------------------------
    info_msg = LogInfo(
        msg="[speech_bringup_v2] Starting remote speech pipeline "
            "(remote_speech_client_node + tts_node + optional mouth_anim_node)"
    )

    # -------------------------------------------------------------------------
    # Launch description
    # -------------------------------------------------------------------------
    ld = LaunchDescription()

    # Arguments
    ld.add_action(robot_id_arg)
    ld.add_action(enable_mouth_anim_arg)
    ld.add_action(timeout_s_arg)
    ld.add_action(mouth_mode_arg)
    ld.add_action(mouth_debug_arg)

    # Info
    ld.add_action(info_msg)

    # Nodes
    ld.add_action(remote_speech_node)
    ld.add_action(tts_node)
    ld.add_action(mouth_anim_node)

    return ld
