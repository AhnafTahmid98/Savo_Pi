# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Launch the edge-hosted filtered RealSense obstacle-cloud producer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the obstacle-cloud filter launch description."""
    default_config = PathJoinSubstitution(
        [
            FindPackageShare('savo_perception'),
            'config',
            'edge',
            'obstacle_cloud_filter.yaml',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Obstacle-cloud filter parameter file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the ROS simulation clock.',
            ),
            Node(
                package='savo_perception',
                executable='obstacle_cloud_filter_node',
                name='obstacle_cloud_filter_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            ),
        ]
    )
