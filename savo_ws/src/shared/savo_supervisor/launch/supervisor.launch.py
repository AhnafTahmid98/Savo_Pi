# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_MODE_POLICY_FILES = {
    'safe_idle': 'safe_idle.yaml',
    'manual_mapping': 'manual_mapping.yaml',
    'autonomous_mapping': 'autonomous_mapping.yaml',
    'saved_map_navigation': 'saved_map_navigation.yaml',
}


def _launch_supervisor(context):
    robot_mode = LaunchConfiguration('robot_mode').perform(context).strip()
    try:
        policy_file = _MODE_POLICY_FILES[robot_mode]
    except KeyError as error:
        raise RuntimeError(
            f'unsupported Supervisor robot_mode policy: {robot_mode}'
        ) from error

    mode_policy = PathJoinSubstitution([
        FindPackageShare('savo_supervisor'), 'config', 'modes', policy_file,
    ])

    return [
        Node(
            package='savo_supervisor',
            executable='supervisor_node',
            name='savo_supervisor_node',
            output='screen',
            parameters=[
                LaunchConfiguration('supervisor_config'),
                mode_policy,
                LaunchConfiguration('location_authorization_config'),
                {
                    'system_authority.state_path': LaunchConfiguration(
                        'system_state_path'),
                    'system_authority.auto_arm': ParameterValue(
                        LaunchConfiguration('auto_arm'), value_type=bool),
                },
            ],
        ),
    ]


def generate_launch_description():
    package_share = FindPackageShare('savo_supervisor')
    default_supervisor_config = PathJoinSubstitution([
        package_share, 'config', 'supervisor.yaml',
    ])
    default_location_authorization_config = PathJoinSubstitution([
        package_share, 'config', 'location_authorization.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_mode',
            default_value='safe_idle',
            description='Robot mode selecting the Supervisor policy overlay.',
        ),
        DeclareLaunchArgument(
            'supervisor_config',
            default_value=default_supervisor_config,
            description='Supervisor parameter file.',
        ),
        DeclareLaunchArgument(
            'location_authorization_config',
            default_value=default_location_authorization_config,
            description='Location authorization parameter file.',
        ),
        DeclareLaunchArgument(
            'system_state_path',
            default_value='/var/lib/robot_savo/supervisor/system_state.json',
            description='Persistent fault-latch state file.',
        ),
        DeclareLaunchArgument(
            'auto_arm',
            default_value='false',
            description='Automatically arm when startup dependencies are ready.',
        ),
        OpaqueFunction(function=_launch_supervisor),
    ])
