from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from savo_realsense.models.camera_status import CameraStatus
from savo_realsense.models.stream_status import StreamStatus


def stream_status_to_key_values(status: StreamStatus) -> list[KeyValue]:
    return [
        KeyValue(key="topic", value=status.topic),
        KeyValue(key="seen", value=str(status.seen)),
        KeyValue(key="stale", value=str(status.stale)),
        KeyValue(key="rate_hz", value=f"{status.rate_hz:.2f}"),
        KeyValue(key="expected_hz", value=f"{status.expected_hz:.2f}"),
        KeyValue(key="last_age_s", value=f"{status.last_age_s:.3f}"),
    ]


def camera_status_to_diagnostic_level(status: CameraStatus) -> int:
    if status.ok:
        return DiagnosticStatus.OK
    if status.any_stream_seen:
        return DiagnosticStatus.WARN
    return DiagnosticStatus.ERROR


def camera_status_to_key_values(status: CameraStatus) -> list[KeyValue]:
    return [
        KeyValue(key="ok", value=str(status.ok)),
        KeyValue(key="color_ok", value=str(status.color_ok)),
        KeyValue(key="depth_ok", value=str(status.depth_ok)),
        KeyValue(key="color_info_ok", value=str(status.color_info_ok)),
        KeyValue(key="depth_info_ok", value=str(status.depth_info_ok)),
        KeyValue(key="pointcloud_ok", value=str(status.pointcloud_ok)),
        KeyValue(key="any_stream_seen", value=str(status.any_stream_seen)),
        KeyValue(key="message", value=status.message),
    ]