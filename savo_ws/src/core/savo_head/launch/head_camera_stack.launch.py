from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _camera_actions(context):
    mode = (
        LaunchConfiguration('camera_mode')
        .perform(context)
        .strip()
        .lower()
    )

    valid_modes = {'disabled', 'ros', 'udp'}

    if mode not in valid_modes:
        raise RuntimeError(
            "camera_mode must be 'disabled', 'ros', or 'udp', "
            f'got: {mode!r}'
        )

    if mode == 'disabled':
        return []

    package_share = FindPackageShare('savo_head')

    if mode == 'ros':
        camera_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        package_share,
                        'launch',
                        'head_camera_ros.launch.py',
                    ]
                )
            ),
            launch_arguments={
                'config_file': LaunchConfiguration(
                    'ros_config_file'
                ),
                'source': LaunchConfiguration('source'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'source_format': LaunchConfiguration(
                    'ros_source_format'
                ),
                'camera_name': LaunchConfiguration(
                    'camera_name'
                ),
                'frame_id': LaunchConfiguration('frame_id'),
                'camera_info_url': LaunchConfiguration(
                    'camera_info_url'
                ),
            }.items(),
        )

        camera_status = Node(
            package='savo_head',
            executable='head_camera_status_node',
            namespace='savo_head',
            name='head_camera_status_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration(
                    'camera_health_config_file'
                )
            ],
        )

        return [camera_driver, camera_status]

    camera_stream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    'launch',
                    'head_camera_stream.launch.py',
                ]
            )
        ),
        launch_arguments={
            'enabled': 'true',
            'source': LaunchConfiguration('source'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'format': LaunchConfiguration('udp_format'),
            'bitrate_kbps': LaunchConfiguration(
                'udp_bitrate_kbps'
            ),
            'udp_host': LaunchConfiguration('udp_host'),
            'udp_port': LaunchConfiguration('udp_port'),
        }.items(),
    )

    return [camera_stream]


def generate_launch_description():
    package_share = FindPackageShare('savo_head')

    default_ros_config = PathJoinSubstitution(
        [
            package_share,
            'config',
            'camera_ros.yaml',
        ]
    )

    default_health_config = PathJoinSubstitution(
        [
            package_share,
            'config',
            'camera_health.yaml',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_mode',
                default_value='disabled',
                description=(
                    'Camera transport: disabled, ros, or udp. '
                    'Only one transport is started.'
                ),
            ),
            DeclareLaunchArgument(
                'source',
                default_value='libcamerasrc',
                description=(
                    'Use libcamerasrc on the robot or '
                    'videotestsrc for PC validation.'
                ),
            ),
            DeclareLaunchArgument(
                'width',
                default_value='640',
                description='Camera image width.',
            ),
            DeclareLaunchArgument(
                'height',
                default_value='480',
                description='Camera image height.',
            ),
            DeclareLaunchArgument(
                'fps',
                default_value='30',
                description='Camera frame rate.',
            ),
            DeclareLaunchArgument(
                'ros_source_format',
                default_value='I420',
                description=(
                    'Raw source format for the ROS gscam path.'
                ),
            ),
            DeclareLaunchArgument(
                'udp_format',
                default_value='I420',
                description=(
                    'Raw source format for the UDP encoder path.'
                ),
            ),
            DeclareLaunchArgument(
                'camera_name',
                default_value='savo_head_camera',
                description='Logical ROS camera name.',
            ),
            DeclareLaunchArgument(
                'frame_id',
                default_value='pi_camera_optical_frame',
                description=(
                    'Frame used by Image and CameraInfo.'
                ),
            ),
            DeclareLaunchArgument(
                'camera_info_url',
                default_value='',
                description='Camera calibration URL.',
            ),
            DeclareLaunchArgument(
                'ros_config_file',
                default_value=default_ros_config,
                description=(
                    'Parameter file for the ROS camera driver.'
                ),
            ),
            DeclareLaunchArgument(
                'camera_health_config_file',
                default_value=default_health_config,
                description=(
                    'Parameter file for '
                    'head_camera_status_node.'
                ),
            ),
            DeclareLaunchArgument(
                'udp_bitrate_kbps',
                default_value='2000',
                description='UDP H.264 bitrate in kbps.',
            ),
            DeclareLaunchArgument(
                'udp_host',
                default_value='127.0.0.1',
                description='UDP receiver address.',
            ),
            DeclareLaunchArgument(
                'udp_port',
                default_value='5000',
                description='UDP receiver port.',
            ),
            OpaqueFunction(function=_camera_actions),
        ]
    )
