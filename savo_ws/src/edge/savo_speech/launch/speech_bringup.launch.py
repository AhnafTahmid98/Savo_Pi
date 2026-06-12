#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the local STT, intent bridge, TTS, and mouth animation stack."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("savo_speech")

    stt_config_path = os.path.join(pkg_share, "config", "stt_whisper.yaml")
    tts_config_path = os.path.join(pkg_share, "config", "tts_piper.yaml")
    mouth_config_path = os.path.join(pkg_share, "config", "mouth_anim.yaml")

    # env.sh loads LLM_SERVER_URL from env_llm.sh on the Pi.
    default_llm_url = os.environ.get("LLM_SERVER_URL", "http://127.0.0.1:8000")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Global ROS 2 log level for speech-related nodes (debug, info, warn, error, fatal).",
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

    enable_speech_bridge_arg = DeclareLaunchArgument(
        "enable_speech_bridge",
        default_value="true",
        description="Whether to start the STT ↔ LLM ↔ TTS speech bridge node.",
    )

    enable_intent_client_arg = DeclareLaunchArgument(
        "enable_intent_client",
        default_value="true",
        description="Whether to start the LLM intent client node (HTTP bridge to LLM server).",
    )

    llm_server_url_arg = DeclareLaunchArgument(
        "llm_server_url",
        default_value=default_llm_url,
        description=(
            "Base URL of the Robot Savo LLM server (FastAPI /chat endpoint). "
            "Default comes from LLM_SERVER_URL environment variable."
        ),
    )

    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="robot_savo_pi",
        description="Robot ID to send to the LLM server (used for logging / multi-robot setups).",
    )

    timeout_s_arg = DeclareLaunchArgument(
        "timeout_s",
        default_value="20.0",
        description="Timeout in seconds for LLM requests in intent_client_node.",
    )

    log_level = LaunchConfiguration("log_level")
    enable_stt = LaunchConfiguration("enable_stt")
    enable_tts = LaunchConfiguration("enable_tts")
    enable_mouth_anim = LaunchConfiguration("enable_mouth_anim")
    enable_speech_bridge = LaunchConfiguration("enable_speech_bridge")
    enable_intent_client = LaunchConfiguration("enable_intent_client")
    llm_server_url = LaunchConfiguration("llm_server_url")
    robot_id = LaunchConfiguration("robot_id")
    timeout_s = LaunchConfiguration("timeout_s")

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
            log_level,
        ],
        condition=IfCondition(enable_stt),
    )

    stt_info = LogInfo(
        condition=IfCondition(enable_stt),
        msg=(
            "[speech_bringup] Starting STT (Faster-Whisper) node "
            f"with config: {stt_config_path}"
        ),
    )

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
            log_level,
        ],
        condition=IfCondition(enable_tts),
    )

    tts_info = LogInfo(
        condition=IfCondition(enable_tts),
        msg=(
            "[speech_bringup] Starting TTS (Piper) node "
            f"with config: {tts_config_path}"
        ),
    )

    speech_bridge_node = Node(
        package="savo_speech",
        executable="speech_bridge_node",
        name="speech_bridge_node",
        output="screen",
        emulate_tty=True,
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        condition=IfCondition(enable_speech_bridge),
    )

    speech_bridge_info = LogInfo(
        condition=IfCondition(enable_speech_bridge),
        msg=(
            "[speech_bringup] Starting SpeechBridgeNode "
            "(STT ↔ LLM ↔ TTS topic router)."
        ),
    )

    intent_client_node = Node(
        package="savo_intent",
        executable="intent_client_node",
        name="intent_client_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "llm_server_url": llm_server_url,
                "robot_id": robot_id,
                "timeout_s": timeout_s,
            }
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        condition=IfCondition(enable_intent_client),
    )

    intent_client_info = LogInfo(
        condition=IfCondition(enable_intent_client),
        msg=(
            "[speech_bringup] Starting LLM intent_client_node "
            "(llm_server_url comes from LLM_SERVER_URL env or launch arg)."
        ),
    )

    mouth_anim_node = Node(
        package="savo_speech",
        executable="mouth_anim_node",
        name="mouth_anim_node",
        output="screen",
        emulate_tty=True,
        parameters=[mouth_config_path],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        condition=IfCondition(enable_mouth_anim),
    )

    mouth_info = LogInfo(
        condition=IfCondition(enable_mouth_anim),
        msg=(
            "[speech_bringup] Starting mouth animation node "
            f"with config: {mouth_config_path}"
        ),
    )

    return LaunchDescription(
        [
            log_level_arg,
            enable_stt_arg,
            enable_tts_arg,
            enable_mouth_anim_arg,
            enable_speech_bridge_arg,
            enable_intent_client_arg,
            llm_server_url_arg,
            robot_id_arg,
            timeout_s_arg,
            stt_info,
            tts_info,
            speech_bridge_info,
            intent_client_info,
            mouth_info,
            stt_node,
            tts_node,
            speech_bridge_node,
            intent_client_node,
            mouth_anim_node,
        ]
    )
