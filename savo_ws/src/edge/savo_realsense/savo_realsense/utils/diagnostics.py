from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import Clock

from savo_realsense.models.camera_status import CameraStatus
from savo_realsense.models.stream_status import StreamStatus
from savo_realsense.ros.adapters import (
    camera_status_to_diagnostic_level,
    camera_status_to_key_values,
    stream_status_to_key_values,
)


def make_key_value(key: str, value: object) -> KeyValue:
    return KeyValue(key=key, value=str(value))


def make_stream_diagnostic(
    name: str,
    status: StreamStatus,
    hardware_id: str = "intel_realsense",
) -> DiagnosticStatus:
    level = DiagnosticStatus.OK
    message = "stream OK"

    if not status.seen:
        level = DiagnosticStatus.ERROR
        message = "stream not seen"
    elif status.stale:
        level = DiagnosticStatus.ERROR
        message = "stream stale"
    elif status.below_expected_rate:
        level = DiagnosticStatus.WARN
        message = "stream below expected rate"

    return DiagnosticStatus(
        level=level,
        name=name,
        message=message,
        hardware_id=hardware_id,
        values=stream_status_to_key_values(status),
    )


def make_camera_diagnostic(
    status: CameraStatus,
    name: str = "RealSense camera",
    hardware_id: str = "intel_realsense",
) -> DiagnosticStatus:
    return DiagnosticStatus(
        level=camera_status_to_diagnostic_level(status),
        name=name,
        message=status.message,
        hardware_id=hardware_id,
        values=camera_status_to_key_values(status),
    )


def make_diagnostic_array(
    statuses: list[DiagnosticStatus],
    clock: Clock,
) -> DiagnosticArray:
    msg = DiagnosticArray()
    msg.header.stamp = clock.now().to_msg()
    msg.status = statuses
    return msg