from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _make_nodes(context):
    backend = LaunchConfiguration("backend").perform(context)
    use_python_fallback = _as_bool(LaunchConfiguration("use_python_fallback").perform(context))
    enable_scan = _as_bool(LaunchConfiguration("enable_scan").perform(context))
    enable_tf = _as_bool(LaunchConfiguration("enable_tf").perform(context))
    enable_status = _as_bool(LaunchConfiguration("enable_status").perform(context))
    enable_apriltag_confirm = _as_bool(
        LaunchConfiguration("enable_apriltag_confirm").perform(context)
    )

    center_on_start = _as_bool(LaunchConfiguration("center_on_start").perform(context))
    center_on_shutdown = _as_bool(LaunchConfiguration("center_on_shutdown").perform(context))

    executable_suffix = "_py" if use_python_fallback else ""

    nodes = [
        Node(
            package="savo_head",
            executable=f"head_controller_node{executable_suffix}",
            name="head_controller_node",
            output="screen",
            parameters=[
                {
                    "backend": backend,
                    "center_on_start": center_on_start,
                    "center_on_shutdown": center_on_shutdown,
                }
            ],
        )
    ]

    if enable_scan:
        nodes.append(
            Node(
                package="savo_head",
                executable=f"head_scan_node{executable_suffix}",
                name="head_scan_node",
                output="screen",
                parameters=[
                    {
                        "enabled": True,
                        "auto_start": False,
                    }
                ],
            )
        )

    if enable_tf:
        nodes.append(
            Node(
                package="savo_head",
                executable=f"head_tf_node{executable_suffix}",
                name="head_tf_node",
                output="screen",
            )
        )

    if enable_status:
        nodes.append(
            Node(
                package="savo_head",
                executable=f"head_status_node{executable_suffix}",
                name="head_status_node",
                output="screen",
            )
        )

    if enable_apriltag_confirm:
        nodes.append(
            Node(
                package="savo_head",
                executable=f"apriltag_confirm_node{executable_suffix}",
                name="apriltag_confirm_node",
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    package_share = FindPackageShare("savo_head")

    camera_stream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    "launch",
                    "head_camera_stream.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_camera_stream")),
        launch_arguments={
            "enabled": "true",
            "source": LaunchConfiguration("camera_source"),
            "width": LaunchConfiguration("camera_width"),
            "height": LaunchConfiguration("camera_height"),
            "fps": LaunchConfiguration("camera_fps"),
            "format": LaunchConfiguration("camera_format"),
            "bitrate_kbps": LaunchConfiguration("camera_bitrate_kbps"),
            "udp_host": LaunchConfiguration("camera_udp_host"),
            "udp_port": LaunchConfiguration("camera_udp_port"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="pca9685",
                description="Pan-tilt backend: pca9685 for robot, dryrun for PC smoke tests.",
            ),
            DeclareLaunchArgument(
                "use_python_fallback",
                default_value="false",
                description="Use Python fallback executables instead of C++ production executables.",
            ),
            DeclareLaunchArgument(
                "enable_scan",
                default_value="true",
                description="Start head scan node.",
            ),
            DeclareLaunchArgument(
                "enable_tf",
                default_value="true",
                description="Start head TF node.",
            ),
            DeclareLaunchArgument(
                "enable_status",
                default_value="true",
                description="Start head status aggregator.",
            ),
            DeclareLaunchArgument(
                "enable_apriltag_confirm",
                default_value="true",
                description="Start AprilTag semantic confirmation node.",
            ),
            DeclareLaunchArgument(
                "center_on_start",
                default_value="false",
                description="Center pan-tilt when controller starts.",
            ),
            DeclareLaunchArgument(
                "center_on_shutdown",
                default_value="true",
                description="Center pan-tilt when controller shuts down.",
            ),
            DeclareLaunchArgument(
                "enable_camera_stream",
                default_value="false",
                description="Start optional GStreamer Pi camera UDP stream.",
            ),
            DeclareLaunchArgument(
                "camera_source",
                default_value="libcamerasrc",
                description="Camera source. Use libcamerasrc on robot, videotestsrc for PC smoke.",
            ),
            DeclareLaunchArgument(
                "camera_width",
                default_value="640",
                description="Camera stream width.",
            ),
            DeclareLaunchArgument(
                "camera_height",
                default_value="480",
                description="Camera stream height.",
            ),
            DeclareLaunchArgument(
                "camera_fps",
                default_value="30",
                description="Camera stream FPS.",
            ),
            DeclareLaunchArgument(
                "camera_format",
                default_value="I420",
                description="Raw camera format before H264 encoding.",
            ),
            DeclareLaunchArgument(
                "camera_bitrate_kbps",
                default_value="2000",
                description="H264 bitrate in kbps.",
            ),
            DeclareLaunchArgument(
                "camera_udp_host",
                default_value="127.0.0.1",
                description="Receiver IP address for UDP stream.",
            ),
            DeclareLaunchArgument(
                "camera_udp_port",
                default_value="5000",
                description="Receiver UDP port.",
            ),
            OpaqueFunction(function=_make_nodes),
            camera_stream,
        ]
    )
