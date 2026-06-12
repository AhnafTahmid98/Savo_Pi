#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the remote /speech pipeline, Piper TTS, and mouth animation."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
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

    # Kept for older launch commands; wired to request_timeout_s.
    timeout_s_arg = DeclareLaunchArgument(
        "timeout_s",
        default_value="20.0",
        description=(
            "Timeout (seconds) for a single /speech HTTP request. "
            "Passed through as request_timeout_s to remote_speech_client_node."
        ),
    )

    mouth_mode_arg = DeclareLaunchArgument(
        "mouth_mode",
        default_value="pcm",
        description=(
            "Mouth animation mode: 'pcm' uses full TTS audio from /savo_speech/tts_pcm, "
            "'activity' uses simple /savo_speech/mouth_open Float32."
        ),
    )

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

    pkg_share = get_package_share_directory("savo_speech")

    speech_remote_yaml = os.path.join(
        pkg_share, "config", "speech_remote.yaml"
    )
    tts_piper_yaml = os.path.join(
        pkg_share, "config", "tts_piper.yaml"
    )

    remote_speech_node = Node(
        package="savo_speech",
        executable="remote_speech_client_node",
        name="remote_speech_client_node",
        output="screen",
        parameters=[
            speech_remote_yaml,
            {
                "robot_id": robot_id,
                "request_timeout_s": timeout_s,
            },
        ],
    )

    tts_node = Node(
        package="savo_speech",
        executable="tts_node",
        name="tts_node",
        output="screen",
        parameters=[tts_piper_yaml],
    )

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
            }
        ],
    )

    # Static text avoids LaunchConfiguration formatting surprises in logs.
    info_msg = LogInfo(
        msg="[speech_bringup_v2] Starting remote speech pipeline "
            "(remote_speech_client_node + tts_node + optional mouth_anim_node)"
    )

    ld = LaunchDescription()

    ld.add_action(robot_id_arg)
    ld.add_action(enable_mouth_anim_arg)
    ld.add_action(timeout_s_arg)
    ld.add_action(mouth_mode_arg)
    ld.add_action(mouth_debug_arg)

    ld.add_action(info_msg)

    ld.add_action(remote_speech_node)
    ld.add_action(tts_node)
    ld.add_action(mouth_anim_node)

    return ld
