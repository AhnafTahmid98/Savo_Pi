#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Speech bringup launch (STT + LLM bridge + TTS + mouth animation)

This launch file starts the full speech & dialog stack for Robot Savo:

  - Faster-Whisper STT node        (savo_speech/stt_node)
  - Speech bridge node             (savo_speech/speech_bridge_node)
        /savo_speech/stt_text   -> /savo_intent/user_text
        /savo_intent/reply_text -> /savo_speech/tts_text
  - LLM intent client node         (savo_intent/intent_client_node)
        HTTP /chat -> Robot Savo LLM server
  - Piper TTS node                 (savo_speech/tts_node)
  - Optional mouth animation node  (savo_speech/mouth_anim_node)

Config files loaded from the installed package:

  - config/stt_whisper.yaml
  - config/tts_piper.yaml
  - config/mouth_anim.yaml   (optional, if present)

Typical usage on the Pi:

  cd ~/Savo_Pi
  source tools/scripts/env.sh   # this loads env_llm.sh -> LLM_SERVER_URL

  # Start full voice loop with LLM_SERVER_URL from env_llm.sh
  ros2 launch savo_speech speech_bringup.launch.py \
    robot_id:=robot_savo_pi

  # Override LLM server URL manually (if needed)
  ros2 launch savo_speech speech_bringup.launch.py \
    robot_id:=robot_savo_pi \
    llm_server_url:=http://server_IP:8000

  # More verbose for debugging:
  ros2 launch savo_speech speech_bringup.launch.py \
    robot_id:=robot_savo_pi \
    log_level:=debug

You can also enable/disable individual components, e.g.:

  ros2 launch savo_speech speech_bringup.launch.py \
    robot_id:=robot_savo_pi \
    enable_mouth_anim:=false
    
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
    # Default LLM URL from environment (env_llm.sh via env.sh)
    # ------------------------------------------------------------------
    # On the Pi you set this in:
    #   tools/scripts/env_llm.sh  ->  export LLM_SERVER_URL="http://<IP>:8000"
    default_llm_url = os.environ.get("LLM_SERVER_URL", "http://127.0.0.1:8000")

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
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

    # Short-hands for launch configurations
    log_level = LaunchConfiguration("log_level")
    enable_stt = LaunchConfiguration("enable_stt")
    enable_tts = LaunchConfiguration("enable_tts")
    enable_mouth_anim = LaunchConfiguration("enable_mouth_anim")
    enable_speech_bridge = LaunchConfiguration("enable_speech_bridge")
    enable_intent_client = LaunchConfiguration("enable_intent_client")
    llm_server_url = LaunchConfiguration("llm_server_url")
    robot_id = LaunchConfiguration("robot_id")
    timeout_s = LaunchConfiguration("timeout_s")

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

    # ------------------------------------------------------------------
    # Speech bridge node (STT ↔ LLM ↔ TTS)
    # ------------------------------------------------------------------
    speech_bridge_node = Node(
        package="savo_speech",
        executable="speech_bridge_node",
        name="speech_bridge_node",
        output="screen",
        emulate_tty=True,
        # The node has sensible defaults for topics; we don't *need* parameters here.
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

    # ------------------------------------------------------------------
    # LLM intent client node (HTTP bridge to LLM server)
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Mouth animation node (optional)
    # ------------------------------------------------------------------
    mouth_anim_node = Node(
        package="savo_speech",
        executable="mouth_anim_node",  # matches setup.cfg / setup.py entry point
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
            enable_speech_bridge_arg,
            enable_intent_client_arg,
            llm_server_url_arg,
            robot_id_arg,
            timeout_s_arg,
            # Info messages
            stt_info,
            tts_info,
            speech_bridge_info,
            intent_client_info,
            mouth_info,
            # Nodes
            stt_node,
            tts_node,
            speech_bridge_node,
            intent_client_node,
            mouth_anim_node,
        ]
    )
