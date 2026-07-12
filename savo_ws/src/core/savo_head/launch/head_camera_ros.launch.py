from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _positive_int(name: str, value: str) -> int:
    """Parse and validate a positive integer launch argument."""
    try:
        parsed = int(value)
    except ValueError as exc:
        raise RuntimeError(
            f"{name} must be an integer, got: {value!r}"
        ) from exc

    if parsed <= 0:
        raise RuntimeError(
            f"{name} must be greater than zero, got: {parsed}"
        )

    return parsed


def _make_camera_node(context):
    """Build the GStreamer pipeline and create the gscam ROS node."""
    source = LaunchConfiguration("source").perform(context).strip()

    width = _positive_int(
        "width",
        LaunchConfiguration("width").perform(context),
    )
    height = _positive_int(
        "height",
        LaunchConfiguration("height").perform(context),
    )
    fps = _positive_int(
        "fps",
        LaunchConfiguration("fps").perform(context),
    )

    source_format = (
        LaunchConfiguration("source_format")
        .perform(context)
        .strip()
        .upper()
    )

    camera_name = (
        LaunchConfiguration("camera_name")
        .perform(context)
        .strip()
    )
    frame_id = (
        LaunchConfiguration("frame_id")
        .perform(context)
        .strip()
    )
    camera_info_url = (
        LaunchConfiguration("camera_info_url")
        .perform(context)
        .strip()
    )
    config_file = (
        LaunchConfiguration("config_file")
        .perform(context)
        .strip()
    )

    # Support the real Pi camera and a hardware-independent test source.
    if source == "libcamerasrc":
        source_element = "libcamerasrc"
    elif source == "videotestsrc":
        source_element = "videotestsrc is-live=true pattern=ball"
    else:
        raise RuntimeError(
            "source must be 'libcamerasrc' or 'videotestsrc', "
            f"got: {source!r}"
        )

    if source_format not in {"I420", "NV12", "YUY2"}:
        raise RuntimeError(
            "source_format must be I420, NV12, or YUY2, "
            f"got: {source_format!r}"
        )

    # gscam automatically attaches its own appsink.
    # The pipeline must therefore finish with one unlinked RGB output.
    gscam_config = (
        f"{source_element} ! "
        f"video/x-raw,"
        f"format={source_format},"
        f"width={width},"
        f"height={height},"
        f"framerate={fps}/1 ! "
        "queue max-size-buffers=2 leaky=downstream ! "
        "videoconvert ! "
        "video/x-raw,format=RGB"
    )

    return [
        Node(
            package="gscam",
            executable="gscam_node",
            namespace="savo_head",
            name="camera_driver",
            output="screen",
            emulate_tty=True,
            parameters=[
                config_file,
                {
                    "gscam_config": gscam_config,
                    "camera_name": camera_name,
                    "frame_id": frame_id,
                    "camera_info_url": camera_info_url,
                },
            ],
        )
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_head"),
            "config",
            "camera_ros.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="ROS parameter file for the gscam node.",
            ),
            DeclareLaunchArgument(
                "source",
                default_value="libcamerasrc",
                description=(
                    "GStreamer source: libcamerasrc for the Pi camera "
                    "or videotestsrc for a ROS-only test."
                ),
            ),
            DeclareLaunchArgument(
                "width",
                default_value="640",
                description="Published image width in pixels.",
            ),
            DeclareLaunchArgument(
                "height",
                default_value="480",
                description="Published image height in pixels.",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="30",
                description="Requested camera frame rate.",
            ),
            DeclareLaunchArgument(
                "source_format",
                default_value="I420",
                description=(
                    "Raw GStreamer source format: "
                    "I420, NV12, or YUY2."
                ),
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="savo_head_camera",
                description="Logical name used in CameraInfo.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="pi_camera_optical_frame",
                description=(
                    "Frame ID written into Image and CameraInfo headers."
                ),
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value="",
                description=(
                    "Camera calibration URL. Leave empty during "
                    "Phase 1; Phase 2 will add calibration."
                ),
            ),
            OpaqueFunction(function=_make_camera_node),
        ]
    )
