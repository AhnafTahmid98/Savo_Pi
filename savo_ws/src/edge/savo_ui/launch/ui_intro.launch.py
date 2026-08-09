# Copyright 2026 Ahnaf Tahmid
"""
Start the always-on Savo UI through the boot Intro screen.

This is the preferred Edge Pi boot entrypoint.
One runtime process owns the framebuffer before, during and after Intro.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = Path(
        get_package_share_directory('savo_ui')
    )

    runtime_launch = (
        pkg_share
        / 'launch'
        / 'ui_runtime_bringup.launch.py'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'profile',
                default_value='pi',
            ),

            DeclareLaunchArgument(
                'loop_hz',
                default_value='30.0',
            ),

            DeclareLaunchArgument(
                'intro_seconds',
                default_value='4.0',
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(runtime_launch)
                ),

                launch_arguments={
                    'profile':
                        LaunchConfiguration('profile'),

                    'loop_hz':
                        LaunchConfiguration('loop_hz'),

                    'intro_seconds':
                        LaunchConfiguration(
                            'intro_seconds'
                        ),
                }.items(),
            ),
        ]
    )
