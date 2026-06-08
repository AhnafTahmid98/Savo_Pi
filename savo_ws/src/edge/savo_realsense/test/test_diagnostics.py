from diagnostic_msgs.msg import DiagnosticStatus

from savo_realsense.diagnostics.realsense_frame_check import (
    format_missing_frames_report,
    frames_available,
    missing_frames,
    optical_frames_present,
)
from savo_realsense.diagnostics.realsense_topic_check import (
    format_missing_topics_report,
    missing_topics,
    optional_topics_present,
    topics_available,
)
from savo_realsense.diagnostics.realsense_usb_check import (
    format_usb_report,
    realsense_usb_detected,
    realsense_usb_lines,
)
from savo_realsense.models.camera_status import CameraStatus
from savo_realsense.models.stream_status import StreamStatus
from savo_realsense.utils.diagnostics import (
    make_camera_diagnostic,
    make_stream_diagnostic,
)


def test_missing_topics_returns_only_required_missing_topics() -> None:
    available = [
        "/camera/camera/color/image_raw",
        "/camera/camera/color/camera_info",
    ]

    missing = missing_topics(
        available,
        required_topics=[
            "/camera/camera/color/image_raw",
            "/camera/camera/color/camera_info",
            "/camera/camera/depth/image_rect_raw",
        ],
    )

    assert missing == ["/camera/camera/depth/image_rect_raw"]


def test_topics_available_accepts_complete_topic_set() -> None:
    available = [
        "/camera/camera/color/image_raw",
        "/camera/camera/depth/image_rect_raw",
    ]

    assert topics_available(
        available,
        required_topics=[
            "/camera/camera/color/image_raw",
            "/camera/camera/depth/image_rect_raw",
        ],
    )


def test_optional_topics_present_returns_detected_optional_topics() -> None:
    available = [
        "/camera/camera/color/image_raw",
        "/camera/camera/depth/color/points",
    ]

    assert optional_topics_present(available) == [
        "/camera/camera/depth/color/points",
    ]


def test_format_missing_topics_report_for_complete_set() -> None:
    assert format_missing_topics_report([]) == "All required RealSense topics are available"


def test_format_missing_topics_report_for_missing_topics() -> None:
    report = format_missing_topics_report(["/camera/depth"])

    assert "Missing RealSense topics:" in report
    assert "- /camera/depth" in report


def test_missing_frames_ignores_leading_slash() -> None:
    available = [
        "/base_link",
        "/camera_link",
    ]

    missing = missing_frames(
        available,
        required_frames=[
            "base_link",
            "camera_link",
            "camera_color_optical_frame",
        ],
    )

    assert missing == ["camera_color_optical_frame"]


def test_frames_available_accepts_complete_frame_set() -> None:
    available = [
        "base_link",
        "camera_link",
    ]

    assert frames_available(
        available,
        required_frames=[
            "base_link",
            "camera_link",
        ],
    )


def test_optical_frames_present_returns_optical_frames() -> None:
    frames = [
        "base_link",
        "camera_link",
        "camera_color_optical_frame",
        "camera_depth_optical_frame",
    ]

    assert optical_frames_present(frames) == [
        "camera_color_optical_frame",
        "camera_depth_optical_frame",
    ]


def test_format_missing_frames_report_for_complete_set() -> None:
    assert format_missing_frames_report([]) == "All required RealSense frames are available"


def test_format_missing_frames_report_for_missing_frames() -> None:
    report = format_missing_frames_report(["camera_link"])

    assert "Missing RealSense frames:" in report
    assert "- camera_link" in report


def test_realsense_usb_lines_detects_intel_vendor_id() -> None:
    usb_lines = [
        "Bus 002 Device 003: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera",
        "Bus 001 Device 002: ID 2109:3431 VIA Labs, Inc. Hub",
    ]

    matches = realsense_usb_lines(usb_lines)

    assert len(matches) == 1
    assert "8086" in matches[0]


def test_realsense_usb_detected_returns_true_for_match() -> None:
    usb_lines = [
        "Bus 002 Device 003: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera",
    ]

    assert realsense_usb_detected(usb_lines)


def test_format_usb_report_for_detected_device() -> None:
    report = format_usb_report(
        [
            "Bus 002 Device 003: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera",
        ]
    )

    assert "RealSense USB device detected:" in report


def test_format_usb_report_for_missing_device() -> None:
    assert format_usb_report([]) == "No RealSense USB device detected"


def test_make_stream_diagnostic_ok() -> None:
    status = StreamStatus(
        topic="/camera/camera/color/image_raw",
        seen=True,
        stale=False,
        rate_hz=30.0,
        expected_hz=30.0,
        last_age_s=0.01,
    )

    diagnostic = make_stream_diagnostic("RealSense color image", status)

    assert diagnostic.level == DiagnosticStatus.OK
    assert diagnostic.message == "stream OK"


def test_make_stream_diagnostic_stale() -> None:
    status = StreamStatus(
        topic="/camera/camera/depth/image_rect_raw",
        seen=True,
        stale=True,
        rate_hz=30.0,
        expected_hz=30.0,
        last_age_s=1.0,
    )

    diagnostic = make_stream_diagnostic("RealSense depth image", status)

    assert diagnostic.level == DiagnosticStatus.ERROR
    assert diagnostic.message == "stream stale"


def test_make_stream_diagnostic_low_rate() -> None:
    status = StreamStatus(
        topic="/camera/camera/color/image_raw",
        seen=True,
        stale=False,
        rate_hz=8.0,
        expected_hz=30.0,
        last_age_s=0.01,
    )

    diagnostic = make_stream_diagnostic("RealSense color image", status)

    assert diagnostic.level == DiagnosticStatus.WARN
    assert diagnostic.message == "stream below expected rate"


def test_make_camera_diagnostic_ok() -> None:
    stream = StreamStatus(
        topic="/camera",
        seen=True,
        stale=False,
        rate_hz=30.0,
        expected_hz=30.0,
        last_age_s=0.01,
    )
    status = CameraStatus(
        color=stream,
        color_info=stream,
        depth=stream,
        depth_info=stream,
        pointcloud=None,
        require_pointcloud=False,
    )

    diagnostic = make_camera_diagnostic(status)

    assert diagnostic.level == DiagnosticStatus.OK
    assert diagnostic.message == "RealSense streams OK"


def test_make_camera_diagnostic_error_when_no_stream_seen() -> None:
    stream = StreamStatus(
        topic="/camera",
        seen=False,
        stale=False,
        rate_hz=0.0,
        expected_hz=30.0,
        last_age_s=float("inf"),
    )
    status = CameraStatus(
        color=stream,
        color_info=stream,
        depth=stream,
        depth_info=stream,
        pointcloud=None,
        require_pointcloud=False,
    )

    diagnostic = make_camera_diagnostic(status)

    assert diagnostic.level == DiagnosticStatus.ERROR
    assert diagnostic.message == "No RealSense streams detected"