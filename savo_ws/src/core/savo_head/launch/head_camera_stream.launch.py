from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _make_camera_process(context):
    enabled = _as_bool(LaunchConfiguration("enabled").perform(context))
    if not enabled:
        return []

    source = LaunchConfiguration("source").perform(context).strip()
    width = LaunchConfiguration("width").perform(context).strip()
    height = LaunchConfiguration("height").perform(context).strip()
    fps = LaunchConfiguration("fps").perform(context).strip()
    fmt = LaunchConfiguration("format").perform(context).strip()
    bitrate_kbps = LaunchConfiguration("bitrate_kbps").perform(context).strip()
    udp_host = LaunchConfiguration("udp_host").perform(context).strip()
    udp_port = LaunchConfiguration("udp_port").perform(context).strip()

    source_args = [source]
    if source == "videotestsrc":
        source_args.append("is-live=true")

    command = [
        "gst-launch-1.0",
        "-e",
        *source_args,
        "!",
        f"video/x-raw,format={fmt},width={width},height={height},framerate={fps}/1",
        "!",
        "queue",
        "!",
        "x264enc",
        "tune=zerolatency",
        "speed-preset=ultrafast",
        f"bitrate={bitrate_kbps}",
        "key-int-max=30",
        "!",
        "rtph264pay",
        "config-interval=1",
        "pt=96",
        "!",
        "udpsink",
        f"host={udp_host}",
        f"port={udp_port}",
        "sync=false",
        "async=false",
    ]

    return [
        ExecuteProcess(
            cmd=command,
            output="screen",
            name="savo_head_camera_stream",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enabled",
                default_value="true",
                description="Start the Pi camera UDP stream.",
            ),
            DeclareLaunchArgument(
                "source",
                default_value="libcamerasrc",
                description="GStreamer source. Use libcamerasrc on robot, videotestsrc for PC pipeline smoke.",
            ),
            DeclareLaunchArgument(
                "width",
                default_value="640",
                description="Camera stream width.",
            ),
            DeclareLaunchArgument(
                "height",
                default_value="480",
                description="Camera stream height.",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="30",
                description="Camera stream framerate.",
            ),
            DeclareLaunchArgument(
                "format",
                default_value="I420",
                description="Raw camera format before H264 encoding.",
            ),
            DeclareLaunchArgument(
                "bitrate_kbps",
                default_value="2000",
                description="H264 encoder bitrate in kbps.",
            ),
            DeclareLaunchArgument(
                "udp_host",
                default_value="127.0.0.1",
                description="Receiver IP address. On robot, set this to laptop IP.",
            ),
            DeclareLaunchArgument(
                "udp_port",
                default_value="5000",
                description="Receiver UDP port.",
            ),
            OpaqueFunction(function=_make_camera_process),
        ]
    )
