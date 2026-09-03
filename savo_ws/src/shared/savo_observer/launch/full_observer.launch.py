"""Launch the read-only RViz and browser observer surfaces."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Compose RViz and dashboard without launching robot authorities."""
    launch_dir = Path(get_package_share_directory('savo_observer')) / 'launch'
    common = {
        'profile': LaunchConfiguration('profile'),
        'log_level': LaunchConfiguration('log_level'),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument('view', default_value='overview'),
            DeclareLaunchArgument('profile', default_value='standard'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('rviz_config', default_value=''),
            DeclareLaunchArgument('fixed_frame', default_value=''),
            DeclareLaunchArgument('dashboard_bind_address', default_value='127.0.0.1'),
            DeclareLaunchArgument('dashboard_port', default_value='8765'),
            DeclareLaunchArgument('enable_camera_preview', default_value='false'),
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
            DeclareLaunchArgument('log_level', default_value='info'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / 'rviz_observer.launch.py')),
                launch_arguments={
                    **common,
                    'view': LaunchConfiguration('view'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'rviz_config': LaunchConfiguration('rviz_config'),
                    'fixed_frame': LaunchConfiguration('fixed_frame'),
                    'enable_camera_preview': LaunchConfiguration(
                        'enable_camera_preview'
                    ),
                    'enable_pointclouds': LaunchConfiguration('enable_pointclouds'),
                    'enable_raw_d435_pointcloud': LaunchConfiguration(
                        'enable_raw_d435_pointcloud'
                    ),
                    'enable_localization_markers': LaunchConfiguration(
                        'enable_localization_markers'
                    ),
                    'enable_range_markers': LaunchConfiguration(
                        'enable_range_markers'
                    ),
                    'd435_image_transport': LaunchConfiguration(
                        'd435_image_transport'
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_dir / 'dashboard_observer.launch.py')
                ),
                launch_arguments={
                    **common,
                    'dashboard_bind_address': LaunchConfiguration(
                        'dashboard_bind_address'
                    ),
                    'dashboard_port': LaunchConfiguration('dashboard_port'),
                    'enable_camera_preview': LaunchConfiguration(
                        'enable_camera_preview'
                    ),
                }.items(),
            ),
        ]
    )
