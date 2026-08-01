from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _make_nodes(context):
    enable_detector = _as_bool(
        LaunchConfiguration("enable_detector").perform(context)
    )
    enable_confirmation_action = _as_bool(
        LaunchConfiguration("enable_confirmation_action").perform(context)
    )
    enable_legacy_semantics = _as_bool(
        LaunchConfiguration("enable_legacy_semantics").perform(context)
    )
    enable_tf = _as_bool(
        LaunchConfiguration("enable_tf").perform(context)
    )
    enable_status = _as_bool(
        LaunchConfiguration("enable_status").perform(context)
    )
    use_python_fallback = _as_bool(
        LaunchConfiguration("use_python_fallback").perform(context)
    )

    nodes = []

    if enable_detector:
        nodes.append(
            Node(
                package="savo_head",
                executable="apriltag_detector_node",
                name="apriltag_detector_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("detector_config_file").perform(
                        context
                    )
                ],
            )
        )

    if enable_confirmation_action:
        nodes.append(
            Node(
                package="savo_head",
                executable="apriltag_confirmation_action_node",
                name="apriltag_confirmation_action_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration(
                        "confirmation_action_config_file"
                    ).perform(context)
                ],
            )
        )

    # The JSON semantic bridge is retained only for compatibility with old
    # publishers. It is not part of the default production detection path.
    if enable_legacy_semantics:
        executable_suffix = "_py" if use_python_fallback else ""
        nodes.append(
            Node(
                package="savo_head",
                executable=(
                    f"apriltag_confirm_node{executable_suffix}"
                ),
                name="apriltag_confirm_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration(
                        "legacy_semantics_config_file"
                    ).perform(context)
                ],
            )
        )

    if enable_status:
        executable_suffix = "_py" if use_python_fallback else ""
        nodes.append(
            Node(
                package="savo_head",
                executable=f"head_status_node{executable_suffix}",
                name="head_status_node",
                output="screen",
            )
        )

    # Disabled by default until the measured physical head transforms replace
    # the zero translation placeholders in head_frames.yaml.
    if enable_tf:
        executable_suffix = "_py" if use_python_fallback else ""
        nodes.append(
            Node(
                package="savo_head",
                executable=f"head_tf_node{executable_suffix}",
                name="head_tf_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("head_frames_config_file").perform(
                        context
                    )
                ],
            )
        )

    return nodes


def generate_launch_description():
    package_share = FindPackageShare("savo_head")

    camera_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    "launch",
                    "head_camera_stack.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        launch_arguments={
            "camera_mode": "ros",
            "source": LaunchConfiguration("camera_source"),
            "width": LaunchConfiguration("camera_width"),
            "height": LaunchConfiguration("camera_height"),
            "fps": LaunchConfiguration("camera_fps"),
            "ros_source_format": LaunchConfiguration(
                "camera_source_format"
            ),
            "camera_name": LaunchConfiguration("camera_name"),
            "frame_id": LaunchConfiguration("camera_frame_id"),
            "camera_info_url": LaunchConfiguration("camera_info_url"),
            "ros_config_file": LaunchConfiguration(
                "camera_ros_config_file"
            ),
            "camera_health_config_file": LaunchConfiguration(
                "camera_health_config_file"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description=(
                    "Start the ROS Pi Camera path required by the "
                    "AprilTag detector. Disable when another process "
                    "already owns the camera topics."
                ),
            ),
            DeclareLaunchArgument(
                "enable_detector",
                default_value="true",
                description="Start the production C++ AprilTag detector.",
            ),
            DeclareLaunchArgument(
                "enable_confirmation_action",
                default_value="true",
                description=(
                    "Start the typed ConfirmAprilTag action server."
                ),
            ),
            DeclareLaunchArgument(
                "enable_legacy_semantics",
                default_value="false",
                description=(
                    "Start the legacy JSON semantic bridge. It is "
                    "disabled in the production typed path."
                ),
            ),
            DeclareLaunchArgument(
                "enable_tf",
                default_value="false",
                description=(
                    "Start head TF publishing only after physical "
                    "pan-tilt-camera translations are measured."
                ),
            ),
            DeclareLaunchArgument(
                "enable_status",
                default_value="true",
                description="Start the head status aggregator.",
            ),
            DeclareLaunchArgument(
                "use_python_fallback",
                default_value="false",
                description=(
                    "Use Python fallback for optional legacy/status/TF "
                    "nodes. Detector and action server remain C++."
                ),
            ),
            DeclareLaunchArgument(
                "head_frames_config_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "head_frames.yaml"]
                ),
                description=(
                    "Head TF parameters. Publishing remains calibration-gated "
                    "until measured transforms are approved."
                ),
            ),
            DeclareLaunchArgument(
                "detector_config_file",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "apriltag_detector.yaml",
                    ]
                ),
                description="Production AprilTag detector parameters.",
            ),
            DeclareLaunchArgument(
                "confirmation_action_config_file",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "apriltag_confirmation_action.yaml",
                    ]
                ),
                description="Typed AprilTag confirmation parameters.",
            ),
            DeclareLaunchArgument(
                "legacy_semantics_config_file",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "apriltag_semantics.yaml",
                    ]
                ),
                description="Legacy JSON bridge parameters.",
            ),
            DeclareLaunchArgument(
                "camera_source",
                default_value="libcamerasrc",
                description=(
                    "Use libcamerasrc on Robot SAVO or videotestsrc "
                    "for a camera-pipeline smoke test."
                ),
            ),
            DeclareLaunchArgument(
                "camera_width",
                default_value="640",
                description="Camera image width.",
            ),
            DeclareLaunchArgument(
                "camera_height",
                default_value="480",
                description="Camera image height.",
            ),
            DeclareLaunchArgument(
                "camera_fps",
                default_value="30",
                description="Camera frame rate.",
            ),
            DeclareLaunchArgument(
                "camera_source_format",
                default_value="I420",
                description="Raw GStreamer source format for gscam.",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="savo_head_camera",
                description="Logical ROS camera name.",
            ),
            DeclareLaunchArgument(
                "camera_frame_id",
                default_value="pi_camera_optical_frame",
                description=(
                    "Optical frame written into Image and CameraInfo."
                ),
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value="",
                description=(
                    "Camera calibration URL. With an empty value the "
                    "detector publishes ID-only observations."
                ),
            ),
            DeclareLaunchArgument(
                "camera_ros_config_file",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "camera_ros.yaml",
                    ]
                ),
                description="ROS camera driver parameters.",
            ),
            DeclareLaunchArgument(
                "camera_health_config_file",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "camera_health.yaml",
                    ]
                ),
                description="Camera health monitor parameters.",
            ),
            camera_stack,
            OpaqueFunction(function=_make_nodes),
        ]
    )
