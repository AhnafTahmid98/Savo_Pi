# Copyright 2026 Ahnaf Tahmid
"""
Launch the always-on hybrid Savo UI runtime.

The executable is a separate copy of the Classic UI runtime.
Only Voice and Navigation rendering use the V2 face.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    pkg_share = Path(
        get_package_share_directory('savo_ui')
    )

    config_file = pkg_share / 'config' / 'ui.yaml'
    asset_root = pkg_share / 'assets'

    profile = (
        LaunchConfiguration('profile')
        .perform(context)
        .strip()
        .lower()
    )

    loop_hz = float(
        LaunchConfiguration('loop_hz')
        .perform(context)
    )

    intro_seconds = float(
        LaunchConfiguration('intro_seconds')
        .perform(context)
    )

    if profile not in ('pi', 'dryrun'):
        raise RuntimeError(
            f'Unsupported savo_ui runtime profile {profile!r}. '
            'Use profile:=pi or profile:=dryrun.'
        )

    if loop_hz <= 0.0:
        raise RuntimeError(
            'loop_hz must be greater than zero'
        )

    if intro_seconds <= 0.0:
        raise RuntimeError(
            'intro_seconds must be greater than zero'
        )

    if profile == 'pi':
        enable_framebuffer = True
        enable_touch = True
        export_preview_frames = False
    else:
        enable_framebuffer = False
        enable_touch = False
        export_preview_frames = True

    return [
        Node(
            package='savo_ui',
            executable='ui_runtime_node',

            # Keep the original ROS node name so existing
            # config/ui.yaml remains authoritative.
            name='savo_ui_node',

            output='screen',

            parameters=[
                str(config_file),
                {
                    'asset_root': str(asset_root),

                    'enable_framebuffer':
                        enable_framebuffer,

                    'enable_touch':
                        enable_touch,

                    'export_preview_frames':
                        export_preview_frames,

                    'loop_hz':
                        loop_hz,

                    'intro_seconds':
                        intro_seconds,
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'profile',
                default_value='pi',
                description=(
                    'Hybrid Savo UI profile: '
                    'pi or dryrun'
                ),
            ),

            DeclareLaunchArgument(
                'loop_hz',
                default_value='30.0',
                description='UI render/update rate',
            ),

            DeclareLaunchArgument(
                'intro_seconds',
                default_value='4.0',
                description=(
                    'Boot Intro duration before Home'
                ),
            ),

            OpaqueFunction(
                function=launch_setup
            ),
        ]
    )
