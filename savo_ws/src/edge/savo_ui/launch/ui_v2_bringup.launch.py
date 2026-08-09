# Copyright 2026 Ahnaf Tahmid
"""
Alternative Robot Savo UI V2 bringup.

This launch file starts the new non-destructive UI implementation.

The existing ui_bringup.launch.py and classic ui_node are not replaced.

Profiles:
    dryrun
        PC/software validation.
        Loads config/profiles/pc_dryrun.yaml.

    pi
        Edge Pi real-hardware runtime.
        Loads config/profiles/edge_real_robot_v1.yaml.

Page modes:
    auto
        Voice has priority while actively interacting.
        Navigation is displayed while navigation is active.

    voice
        Force the V2 Voice page for isolated testing.

    navigation
        Force the V2 Navigation page for isolated testing.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_PROFILE_FILES = {
    'dryrun': 'pc_dryrun.yaml',
    'pi': 'edge_real_robot_v1.yaml',
}

_VALID_PAGE_MODES = {
    'auto',
    'voice',
    'navigation',
}


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    pkg_share = Path(get_package_share_directory('savo_ui'))

    profile = (
        LaunchConfiguration('profile')
        .perform(context)
        .strip()
        .lower()
    )

    page_mode = (
        LaunchConfiguration('page_mode')
        .perform(context)
        .strip()
        .lower()
    )

    if profile not in _PROFILE_FILES:
        raise RuntimeError(
            f"Unsupported savo_ui V2 profile '{profile}'. "
            'Use profile:=pi or profile:=dryrun.'
        )

    if page_mode not in _VALID_PAGE_MODES:
        raise RuntimeError(
            f"Unsupported savo_ui V2 page_mode '{page_mode}'. "
            'Use auto, voice, or navigation.'
        )

    base_config = pkg_share / 'config' / 'ui_v2.yaml'

    profile_config = (
        pkg_share
        / 'config'
        / 'profiles'
        / _PROFILE_FILES[profile]
    )

    if not base_config.is_file():
        raise RuntimeError(
            f'savo_ui V2 base configuration not found: {base_config}'
        )

    if not profile_config.is_file():
        raise RuntimeError(
            f'savo_ui V2 profile configuration not found: {profile_config}'
        )

    return [
        Node(
            package='savo_ui',
            executable='ui_v2_node',
            name='savo_ui_v2_node',
            output='screen',
            parameters=[
                str(base_config),
                str(profile_config),
                {
                    'page_mode': page_mode,
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'profile',
                default_value='dryrun',
                description='savo_ui V2 runtime profile: pi or dryrun',
            ),
            DeclareLaunchArgument(
                'page_mode',
                default_value='auto',
                description=(
                    'savo_ui V2 page arbitration: '
                    'auto, voice, or navigation'
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
