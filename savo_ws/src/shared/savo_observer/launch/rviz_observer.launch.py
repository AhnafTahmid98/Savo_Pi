"""Launch one read-only Robot SAVO RViz view."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VIEWS = {
    'overview': 'overview.rviz',
    'robot_model': 'robot_model.rviz',
    'tf': 'tf.rviz',
    'sensors': 'sensors.rviz',
    'safety': 'safety.rviz',
    'localization': 'localization.rviz',
    'mapping': 'mapping_overview.rviz',
    'manual_mapping': 'manual_mapping.rviz',
    'autonomous_mapping': 'autonomous_mapping.rviz',
    'coverage': 'coverage_mapping.rviz',
    'scan360': 'scan360_mapping.rviz',
    'map_quality': 'map_quality.rviz',
    'navigation': 'navigation.rviz',
    'costmaps': 'costmaps.rviz',
    'locations': 'locations_apriltags.rviz',
    'full_debug': 'full_debug.rviz',
}
PROFILES = {'low_bandwidth', 'standard', 'full_debug', 'mobile'}


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _setup(context):
    view = _value(context, 'view')
    profile = _value(context, 'profile')
    if view not in VIEWS:
        raise RuntimeError(f'unknown observer view: {view}')
    if profile not in PROFILES:
        raise RuntimeError(f'unknown observer profile: {profile}')
    configured = _value(context, 'rviz_config')
    config = configured or str(
        Path(get_package_share_directory('savo_observer')) / 'rviz' / VIEWS[view]
    )
    if not Path(config).is_file():
        raise RuntimeError(f'RViz configuration does not exist: {config}')
    arguments = ['-d', config]
    fixed_frame = _value(context, 'fixed_frame')
    if fixed_frame:
        arguments.extend(['--fixed-frame', fixed_frame])
    arguments.extend(['--ros-args', '--log-level', _value(context, 'log_level')])
    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='savo_observer_rviz',
            output='screen',
            arguments=arguments,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    ]


def generate_launch_description():
    """Declare and validate the RViz observer contract."""
    return LaunchDescription(
        [
            DeclareLaunchArgument('view', default_value='overview'),
            DeclareLaunchArgument('profile', default_value='standard'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('rviz_config', default_value=''),
            DeclareLaunchArgument('fixed_frame', default_value=''),
            DeclareLaunchArgument('log_level', default_value='info'),
            DeclareLaunchArgument('enable_camera_preview', default_value='false'),
            DeclareLaunchArgument('enable_pointclouds', default_value='false'),
            OpaqueFunction(function=_setup),
        ]
    )
