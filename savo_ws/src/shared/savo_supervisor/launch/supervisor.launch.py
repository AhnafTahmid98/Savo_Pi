from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_share = FindPackageShare('savo_supervisor')
    config_file = PathJoinSubstitution([package_share, 'config', 'supervisor.yaml'])

    return LaunchDescription([
        Node(
            package='savo_supervisor',
            executable='supervisor_node',
            name='savo_supervisor_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
