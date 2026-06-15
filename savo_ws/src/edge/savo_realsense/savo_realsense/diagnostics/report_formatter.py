# Copyright 2026 Ahnaf Tahmid
from savo_realsense.models.camera_status import CameraStatus
from savo_realsense.models.stream_status import StreamStatus


def format_stream_line(name: str, status: StreamStatus) -> str:
    state = "OK" if status.ok else "BAD"

    return (
        f"{name}: {state} | "
        f"seen={status.seen} "
        f"stale={status.stale} "
        f"rate={status.rate_hz:.2f}Hz "
        f"expected={status.expected_hz:.2f}Hz "
        f"age={status.last_age_s:.3f}s"
    )


def format_camera_report(status: CameraStatus) -> str:
    lines = [
        f"RealSense: {'OK' if status.ok else 'BAD'}",
        f"Message: {status.message}",
        format_stream_line("Color image", status.color),
        format_stream_line("Color info", status.color_info),
        format_stream_line("Depth image", status.depth),
        format_stream_line("Depth info", status.depth_info),
    ]

    if status.pointcloud is not None:
        lines.append(format_stream_line("Pointcloud", status.pointcloud))

    return "\n".join(lines)


def format_topic_list(topics: tuple[str, ...]) -> str:
    return "\n".join(f"- {topic}" for topic in topics)
