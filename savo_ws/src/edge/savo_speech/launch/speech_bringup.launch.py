#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Bring up the currently supported savo_speech production runtime."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from savo_speech.ros.launch_config import (
    LaunchConfigError,
    build_phase0d_runtime_config,
)


def _shutdown_handler(node_action, node_name: str):
    reason = f"{node_name} exited"

    return RegisterEventHandler(
        OnProcessExit(
            target_action=node_action,
            on_exit=[
                LogInfo(
                    msg=f"[savo_speech] {reason}; shutting down bringup"
                ),
                EmitEvent(
                    event=Shutdown(reason=reason)
                ),
            ],
        )
    )


def _launch_setup(context):
    profile_name = LaunchConfiguration("profile").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)

    package_share = Path(
        get_package_share_directory("savo_speech")
    )

    config_dir = package_share / "config"

    try:
        runtime = build_phase0d_runtime_config(
            config_dir,
            profile_name,
        )
    except LaunchConfigError as exc:
        raise RuntimeError(
            f"Invalid savo_speech launch configuration: {exc}"
        ) from exc

    audio_node = Node(
        package="savo_speech",
        executable="respeaker_audio_node",
        name="respeaker_audio_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            runtime["respeaker_audio_node"],
        ],
        ros_arguments=[
            "--log-level",
            log_level,
        ],
    )

    manager_node = Node(
        package="savo_speech",
        executable="speech_manager_node",
        name="speech_manager_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            runtime["speech_manager_node"],
        ],
        ros_arguments=[
            "--log-level",
            log_level,
        ],
    )

    return [
        LogInfo(
            msg=(
                "[savo_speech] Starting Phase 0D runtime "
                f"with profile={profile_name}"
            )
        ),
        audio_node,
        manager_node,
        _shutdown_handler(
            audio_node,
            "respeaker_audio_node",
        ),
        _shutdown_handler(
            manager_node,
            "speech_manager_node",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="dryrun_no_hardware",
                description=(
                    "Validated savo_speech deployment profile name."
                ),
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for speech production nodes.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
