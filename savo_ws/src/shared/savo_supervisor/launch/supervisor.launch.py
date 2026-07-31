# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('savo_supervisor')
    supervisor_config = PathJoinSubstitution([
        package_share, 'config', 'supervisor.yaml',
    ])
    location_authorization_config = PathJoinSubstitution([
        package_share, 'config', 'location_authorization.yaml',
    ])

    return LaunchDescription([
        Node(
            package='savo_supervisor',
            executable='supervisor_node',
            name='savo_supervisor_node',
            output='screen',
            parameters=[supervisor_config, location_authorization_config],
        ),
    ])
