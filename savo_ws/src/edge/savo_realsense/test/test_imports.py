def test_import_package() -> None:
    import savo_realsense

    assert savo_realsense is not None


def test_import_models() -> None:
    from savo_realsense.models import CameraStatus, DepthSample, StreamStatus

    assert CameraStatus is not None
    assert DepthSample is not None
    assert StreamStatus is not None


def test_import_ros_contracts() -> None:
    from savo_realsense.ros import DEFAULT_FRAMES, DEFAULT_TOPICS

    assert DEFAULT_FRAMES is not None
    assert DEFAULT_TOPICS is not None


def test_import_utils() -> None:
    from savo_realsense.utils import RateTracker, normalize_frame_name, normalize_topic_name

    assert RateTracker is not None
    assert normalize_frame_name is not None
    assert normalize_topic_name is not None


def test_import_diagnostics() -> None:
    from savo_realsense.diagnostics import (
        format_usb_report,
        missing_frames,
        missing_topics,
    )

    assert format_usb_report is not None
    assert missing_frames is not None
    assert missing_topics is not None