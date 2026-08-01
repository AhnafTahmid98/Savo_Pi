# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('savo_supervisor')
    default_supervisor_config = PathJoinSubstitution([
        package_share, 'config', 'supervisor.yaml',
    ])
    default_location_authorization_config = PathJoinSubstitution([
        package_share, 'config', 'location_authorization.yaml',
    ])

    supervisor_config = LaunchConfiguration('supervisor_config')
    location_authorization_config = LaunchConfiguration(
        'location_authorization_config')
    system_state_path = LaunchConfiguration('system_state_path')
    auto_arm = LaunchConfiguration('auto_arm')

    return LaunchDescription([
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
        Node(
            package='savo_supervisor',
            executable='supervisor_node',
            name='savo_supervisor_node',
            output='screen',
            parameters=[
                supervisor_config,
                location_authorization_config,
                {
                    'system_authority.state_path': system_state_path,
                    'system_authority.auto_arm': ParameterValue(
                        auto_arm, value_type=bool),
                },
            ],
        ),
    ])
