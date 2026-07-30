from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare('savo_bridge'), 'config', 'savo_bridge.edge.yaml']
    )

    config_file = LaunchConfiguration('config_file')
    active_map_id = LaunchConfiguration('active_map_id')
    active_map_revision = LaunchConfiguration('active_map_revision')

    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=default_config),
            DeclareLaunchArgument('active_map_id', default_value='saved_map'),
            DeclareLaunchArgument('active_map_revision', default_value='1'),
            Node(
                package='savo_bridge',
                executable='savo_bridge_node',
                namespace='savo_bridge',
                name='savo_bridge_node',
                output='screen',
                emulate_tty=True,
                parameters=[
                    config_file,
                    {
                        'command_dispatcher.active_map_id': ParameterValue(
                            active_map_id,
                            value_type=str,
                        ),
                        'command_dispatcher.active_map_revision': ParameterValue(
                            active_map_revision,
                            value_type=int,
                        ),
                    },
                ],
            ),
        ]
    )
