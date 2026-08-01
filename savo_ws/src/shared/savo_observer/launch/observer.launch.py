"""Canonical role-selecting Robot SAVO observer launch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    launches = {
        'rviz': 'rviz_observer.launch.py',
        'dashboard': 'dashboard_observer.launch.py',
        'full': 'full_observer.launch.py',
    }
    if mode not in launches:
        raise RuntimeError(f'unknown observer mode: {mode}')
    launch_file = (
        Path(get_package_share_directory('savo_observer'))
        / 'launch'
        / launches[mode]
    )
    arguments = {
        name: LaunchConfiguration(name)
        for name in [
            'profile',
            'view',
            'use_sim_time',
            'rviz_config',
            'fixed_frame',
            'dashboard_bind_address',
            'dashboard_port',
            'enable_camera_preview',
            'enable_pointclouds',
            'log_level',
        ]
    }
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)),
            launch_arguments=arguments.items(),
        )
    ]


def generate_launch_description():
    """Select the RViz, dashboard, or full read-only observer surface."""
    return LaunchDescription(
        [
            DeclareLaunchArgument('mode', default_value='full'),
            DeclareLaunchArgument('profile', default_value='standard'),
            DeclareLaunchArgument('view', default_value='overview'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('rviz_config', default_value=''),
            DeclareLaunchArgument('fixed_frame', default_value=''),
            DeclareLaunchArgument('dashboard_bind_address', default_value='127.0.0.1'),
            DeclareLaunchArgument('dashboard_port', default_value='8765'),
            DeclareLaunchArgument('enable_camera_preview', default_value='false'),
            DeclareLaunchArgument('enable_pointclouds', default_value='false'),
            DeclareLaunchArgument('log_level', default_value='info'),
            OpaqueFunction(function=_setup),
        ]
    )
