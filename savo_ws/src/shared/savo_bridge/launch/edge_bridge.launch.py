import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_MODE_POLICY_FILES = {
    'safe_idle': 'safe_idle.yaml',
    'manual_mapping': 'manual_mapping.yaml',
    'autonomous_mapping': 'autonomous_mapping.yaml',
    'saved_map_navigation': 'saved_map_navigation.yaml',
}


def _runtime_directory(context) -> str:
    explicit = (
        LaunchConfiguration('runtime_directory').perform(context).strip()
    )
    if explicit:
        return explicit

    service_runtime = os.environ.get('SAVO_BRIDGE_RUNTIME_DIR', '').strip()
    if service_runtime:
        return service_runtime

    return f'/tmp/savo_bridge-runtime-{os.geteuid()}'


def _launch_bridge(context):
    robot_mode = LaunchConfiguration('robot_mode').perform(context).strip()
    try:
        policy_file = _MODE_POLICY_FILES[robot_mode]
    except KeyError as error:
        raise RuntimeError(
            f'unsupported Bridge robot_mode policy: {robot_mode}'
        ) from error

    mode_policy = PathJoinSubstitution([
        FindPackageShare('savo_bridge'), 'config', 'modes', policy_file,
    ])
    runtime_directory = _runtime_directory(context)

    return [
        Node(
            package='savo_bridge',
            executable='savo_bridge_node',
            namespace='savo_bridge',
            name='savo_bridge_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('config_file'),
                mode_policy,
                {
                    'command_dispatcher.active_map_id': ParameterValue(
                        LaunchConfiguration('active_map_id'),
                        value_type=str,
                    ),
                    'command_dispatcher.active_map_revision': ParameterValue(
                        LaunchConfiguration('active_map_revision'),
                        value_type=int,
                    ),
                    'command_server.socket_path': os.path.join(
                        runtime_directory, 'command.sock'
                    ),
                    'snapshot_path': os.path.join(
                        runtime_directory, 'snapshot.json'
                    ),
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare('savo_bridge'), 'config', 'savo_bridge.edge.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_mode', default_value='safe_idle'),
            DeclareLaunchArgument('config_file', default_value=default_config),
            DeclareLaunchArgument('active_map_id', default_value=''),
            DeclareLaunchArgument('active_map_revision', default_value='0'),
            DeclareLaunchArgument('runtime_directory', default_value=''),
            OpaqueFunction(function=_launch_bridge),
        ]
    )
