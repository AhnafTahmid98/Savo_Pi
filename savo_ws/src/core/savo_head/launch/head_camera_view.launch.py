from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _make_viewer_process(context):
    enabled = _as_bool(LaunchConfiguration("enabled").perform(context))
    if not enabled:
        return []

    udp_port = LaunchConfiguration("udp_port").perform(context).strip()
    sink = LaunchConfiguration("sink").perform(context).strip()
    sync = LaunchConfiguration("sync").perform(context).strip().lower()
    show_fps = _as_bool(LaunchConfiguration("show_fps").perform(context))

    caps = "application/x-rtp,media=video,encoding-name=H264,payload=96"

    base_pipeline = [
        "gst-launch-1.0",
        "-v",
        "udpsrc",
        f"port={udp_port}",
        f"caps={caps}",
        "!",
        "rtph264depay",
        "!",
        "avdec_h264",
        "!",
        "videoconvert",
        "!",
    ]

    if show_fps:
        command = [
            *base_pipeline,
            "fpsdisplaysink",
            f"video-sink={sink}",
            "text-overlay=false",
            f"sync={sync}",
        ]
    else:
        command = [
            *base_pipeline,
            sink,
            f"sync={sync}",
        ]

    return [
        ExecuteProcess(
            cmd=command,
            output="screen",
            name="savo_head_camera_view",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enabled",
                default_value="true",
                description="Start the UDP H264 camera viewer.",
            ),
            DeclareLaunchArgument(
                "udp_port",
                default_value="5000",
                description="UDP port receiving the RTP/H264 stream.",
            ),
            DeclareLaunchArgument(
                "sink",
                default_value="autovideosink",
                description="GStreamer video sink. Use autovideosink on desktop, fbdevsink on TTY if available.",
            ),
            DeclareLaunchArgument(
                "sync",
                default_value="false",
                description="GStreamer sink sync mode.",
            ),
            DeclareLaunchArgument(
                "show_fps",
                default_value="false",
                description="Wrap sink with fpsdisplaysink for debugging.",
            ),
            OpaqueFunction(function=_make_viewer_process),
        ]
    )
