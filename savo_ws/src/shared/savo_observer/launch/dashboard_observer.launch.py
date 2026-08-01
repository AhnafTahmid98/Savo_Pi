"""Launch C++ read-only telemetry and browser dashboard nodes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILES = {'low_bandwidth', 'standard', 'full_debug', 'mobile'}


def _setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in PROFILES:
        raise RuntimeError(f'unknown observer profile: {profile}')
    share = Path(get_package_share_directory('savo_observer'))
    parameters = [
        str(share / 'config' / 'observer.yaml'),
        str(share / 'config' / 'profiles' / f'{profile}.yaml'),
    ]
    return [
        Node(
            package='savo_observer',
            executable='observer_telemetry_node',
            name='observer_telemetry_node',
            output='screen',
            parameters=parameters,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
        Node(
            package='savo_observer',
            executable='observer_dashboard_node',
            name='observer_dashboard_node',
            output='screen',
            parameters=parameters
            + [
                {
                    'bind_address': LaunchConfiguration('dashboard_bind_address'),
                    'port': LaunchConfiguration('dashboard_port'),
                }
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ]


def generate_launch_description():
    """Declare dashboard arguments and launch only observer-owned nodes."""
    return LaunchDescription(
        [
            DeclareLaunchArgument('profile', default_value='standard'),
            DeclareLaunchArgument('dashboard_bind_address', default_value='127.0.0.1'),
            DeclareLaunchArgument('dashboard_port', default_value='8765'),
            DeclareLaunchArgument('enable_camera_preview', default_value='false'),
            DeclareLaunchArgument('log_level', default_value='info'),
            OpaqueFunction(function=_setup),
        ]
    )
