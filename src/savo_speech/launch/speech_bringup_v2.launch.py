#!/usr/bin/env python3
"""
Robot Savo — Speech bringup v2 (Robot_Savo_Server /speech pipeline)

This launch file starts:
  - remote_speech_client_node (Pi mic -> Robot_Savo_Server /speech)
  - tts_node (Piper TTS on the Pi)
  - optional mouth_anim node for screen animation

Usage on the Pi:

  cd ~/Savo_Pi
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  source tools/scripts/env_robot_server.sh

  ros2 launch savo_speech speech_bringup_v2.launch.py \
    robot_id:=robot_savo_pi \
    enable_mouth_anim:=false \
    timeout_s:=20.0

Notes:
  - speech_remote.yaml controls remote_speech_client_node params.
  - tts_piper.yaml controls Piper TTS (voices, device, topics).
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
        default_value="false",
        choices=["true", "false"],
        description="If true, also start the mouth_anim node for screen animation.",
    )

    # Kept for compatibility with earlier bringup files; not used directly here.
    timeout_s_arg = DeclareLaunchArgument(
        "timeout_s",
        default_value="20.0",
        description="Legacy argument (kept for compatibility). Not used in this launch file.",
    )

    robot_id = LaunchConfiguration("robot_id")
    enable_mouth_anim = LaunchConfiguration("enable_mouth_anim")

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
    # - Records from ReSpeaker (input_device_index from speech_remote.yaml)
    # - Buffers utterances, sends to /speech (Robot_Savo_Server)
    # - Publishes:
    #     /savo_speech/stt_text        (String)
    #     /savo_speech/tts_text        (String)
    #     /savo_intent/intent_result   (IntentResult)
    #
    remote_speech_node = Node(
        package="savo_speech",
        executable="remote_speech_client_node",
        name="remote_speech_client_node",
        output="screen",
        parameters=[
            speech_remote_yaml,
            {
                # Allow overriding robot_id from launch argument
                "robot_id": robot_id,
            },
        ],
    )

    # 2) TTS node (Piper)
    #
    # - Subscribes to /savo_speech/tts_text (String)
    # - Publishes audio via sounddevice
    # - Publishes /savo_speech/tts_speaking (Bool) as a gate for STT
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
    # - Subscribes to mouth/TTS topics and drives the on-robot screen.
    # - Only started if enable_mouth_anim:=true.
    #
    mouth_anim_node = Node(
        package="savo_speech",
        executable="mouth_anim_node",
        name="mouth_anim_node",
        output="screen",
        condition=IfCondition(enable_mouth_anim),
    )

    # -------------------------------------------------------------------------
    # Info log (static message to avoid substitution issues at import time)
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

    # Info
    ld.add_action(info_msg)

    # Nodes
    ld.add_action(remote_speech_node)
    ld.add_action(tts_node)
    ld.add_action(mouth_anim_node)

    return ld
