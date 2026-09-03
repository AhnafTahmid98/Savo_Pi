"""Launch one read-only Robot SAVO RViz view."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from savo_observer.rviz_config import (
    create_runtime_config,
    parse_d435_image_transport,
    parse_launch_boolean,
    remove_runtime_config,
)


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


def _cleanup_runtime_config(_context, path):
    remove_runtime_config(path)
    return []


def _setup(context):
    view = _value(context, 'view')
    profile = _value(context, 'profile')
    if view not in VIEWS:
        raise RuntimeError(f'unknown observer view: {view}')
    if profile not in PROFILES:
        raise RuntimeError(f'unknown observer profile: {profile}')
    configured = _value(context, 'rviz_config')
    config = configured or str(
        Path(get_package_share_directory('savo_observer'))
        / 'rviz'
        / VIEWS[view]
    )
    if not Path(config).is_file():
        raise RuntimeError(f'RViz configuration does not exist: {config}')

    runtime_config = None
    enable_camera_preview = parse_launch_boolean(
        _value(context, 'enable_camera_preview'),
        'enable_camera_preview',
    )
    enable_pointclouds = parse_launch_boolean(
        _value(context, 'enable_pointclouds'),
        'enable_pointclouds',
    )
    enable_raw_d435_pointcloud = parse_launch_boolean(
        _value(context, 'enable_raw_d435_pointcloud'),
        'enable_raw_d435_pointcloud',
    )
    enable_localization_markers = parse_launch_boolean(
        _value(context, 'enable_localization_markers'),
        'enable_localization_markers',
    )
    enable_range_markers = parse_launch_boolean(
        _value(context, 'enable_range_markers'),
        'enable_range_markers',
    )
    d435_image_transport = parse_d435_image_transport(
        _value(context, 'd435_image_transport')
    )
    if (
        enable_camera_preview
        or enable_pointclouds
        or enable_raw_d435_pointcloud
        or d435_image_transport != 'compressed'
    ):
        runtime_config, _enabled_counts = create_runtime_config(
            config,
            enable_camera_preview=enable_camera_preview,
            enable_pointclouds=enable_pointclouds,
            enable_raw_d435_pointcloud=enable_raw_d435_pointcloud,
            d435_image_transport=d435_image_transport,
        )
        config = str(runtime_config)

    arguments = ['-d', config]
    fixed_frame = _value(context, 'fixed_frame')
    if fixed_frame:
        arguments.extend(['--fixed-frame', fixed_frame])
    arguments.extend(
        ['--ros-args', '--log-level', _value(context, 'log_level')]
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='savo_observer_rviz',
        output='screen',
        arguments=arguments,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    actions = []
    observer_parameters = [
        {'use_sim_time': LaunchConfiguration('use_sim_time')}
    ]
    if enable_localization_markers:
        actions.append(
            Node(
                package='savo_observer',
                executable='localization_visualizer_node',
                name='localization_visualizer_node',
                output='screen',
                parameters=observer_parameters,
                ros_arguments=[
                    '--log-level', _value(context, 'log_level')
                ],
            )
        )
    if enable_range_markers:
        actions.append(
            Node(
                package='savo_observer',
                executable='range_visualizer_node',
                name='range_visualizer_node',
                output='screen',
                parameters=observer_parameters,
                ros_arguments=[
                    '--log-level', _value(context, 'log_level')
                ],
            )
        )
    actions.append(rviz)
    if runtime_config is not None:
        cleanup = OpaqueFunction(
            function=_cleanup_runtime_config,
            args=[str(runtime_config)],
        )
        actions.insert(
            0,
            RegisterEventHandler(
                OnProcessExit(target_action=rviz, on_exit=[cleanup])
            ),
        )
    return actions


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
            DeclareLaunchArgument(
                'enable_camera_preview', default_value='false'
            ),
            DeclareLaunchArgument('enable_pointclouds', default_value='false'),
            DeclareLaunchArgument(
                'enable_raw_d435_pointcloud', default_value='false'
            ),
            DeclareLaunchArgument(
                'enable_localization_markers', default_value='true'
            ),
            DeclareLaunchArgument(
                'enable_range_markers', default_value='true'
            ),
            DeclareLaunchArgument(
                'd435_image_transport', default_value='compressed'
            ),
            OpaqueFunction(function=_setup),
        ]
    )
