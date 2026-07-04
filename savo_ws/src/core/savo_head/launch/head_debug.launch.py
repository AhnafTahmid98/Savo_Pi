from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("savo_head")

    head_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    "launch",
                    "head_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "backend": LaunchConfiguration("backend"),
            "use_python_fallback": LaunchConfiguration("use_python_fallback"),
            "enable_scan": LaunchConfiguration("enable_scan"),
            "enable_tf": LaunchConfiguration("enable_tf"),
            "enable_status": LaunchConfiguration("enable_status"),
            "enable_apriltag_confirm": LaunchConfiguration("enable_apriltag_confirm"),
            "center_on_start": LaunchConfiguration("center_on_start"),
            "center_on_shutdown": LaunchConfiguration("center_on_shutdown"),
        }.items(),
    )

    camera_test_stream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    "launch",
                    "head_camera_stream.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_camera_test_stream")),
        launch_arguments={
            "enabled": "true",
            "source": LaunchConfiguration("camera_source"),
            "width": LaunchConfiguration("camera_width"),
            "height": LaunchConfiguration("camera_height"),
            "fps": LaunchConfiguration("camera_fps"),
            "format": LaunchConfiguration("camera_format"),
            "bitrate_kbps": LaunchConfiguration("camera_bitrate_kbps"),
            "udp_host": LaunchConfiguration("udp_host"),
            "udp_port": LaunchConfiguration("udp_port"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="dryrun",
                description="Debug backend. Default dryrun is safe for PC testing.",
            ),
            DeclareLaunchArgument(
                "use_python_fallback",
                default_value="false",
                description="Use Python fallback nodes instead of C++ production nodes.",
            ),
            DeclareLaunchArgument(
                "enable_scan",
                default_value="true",
                description="Start scan node.",
            ),
            DeclareLaunchArgument(
                "enable_tf",
                default_value="true",
                description="Start TF node.",
            ),
            DeclareLaunchArgument(
                "enable_status",
                default_value="true",
                description="Start status node.",
            ),
            DeclareLaunchArgument(
                "enable_apriltag_confirm",
                default_value="true",
                description="Start AprilTag semantic confirmation node.",
            ),
            DeclareLaunchArgument(
                "center_on_start",
                default_value="false",
                description="Center pan-tilt on startup.",
            ),
            DeclareLaunchArgument(
                "center_on_shutdown",
                default_value="false",
                description="Do not move servos on PC debug shutdown.",
            ),
            DeclareLaunchArgument(
                "start_camera_test_stream",
                default_value="false",
                description="Start GStreamer camera test stream.",
            ),
            DeclareLaunchArgument(
                "camera_source",
                default_value="videotestsrc",
                description="Use videotestsrc for PC debug, libcamerasrc on robot.",
            ),
            DeclareLaunchArgument(
                "camera_width",
                default_value="640",
                description="Debug camera width.",
            ),
            DeclareLaunchArgument(
                "camera_height",
                default_value="480",
                description="Debug camera height.",
            ),
            DeclareLaunchArgument(
                "camera_fps",
                default_value="30",
                description="Debug camera FPS.",
            ),
            DeclareLaunchArgument(
                "camera_format",
                default_value="I420",
                description="Debug camera raw format.",
            ),
            DeclareLaunchArgument(
                "camera_bitrate_kbps",
                default_value="2000",
                description="Debug H264 bitrate in kbps.",
            ),
            DeclareLaunchArgument(
                "udp_host",
                default_value="127.0.0.1",
                description="UDP receiver host.",
            ),
            DeclareLaunchArgument(
                "udp_port",
                default_value="5000",
                description="UDP receiver port.",
            ),
            head_bringup,
            camera_test_stream,
        ]
    )
