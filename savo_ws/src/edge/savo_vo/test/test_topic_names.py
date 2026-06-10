"""Tests for savo_vo topic name contracts."""

from savo_vo.contracts.topic_names import (
    COLOR_CAMERA_INFO_TOPIC,
    COLOR_IMAGE_TOPIC,
    DEPTH_CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
    DIAGNOSTICS_TOPIC,
    VO_HEALTH_TOPIC,
    VO_ODOM_RAW_TOPIC,
    VO_ODOM_TOPIC,
    VO_RESET_TOPIC,
    VO_STATUS_TOPIC,
    VO_TRACKING_QUALITY_TOPIC,
)


def test_realsense_input_topics_match_expected_defaults() -> None:
    assert COLOR_IMAGE_TOPIC == "/camera/camera/color/image_raw"
    assert COLOR_CAMERA_INFO_TOPIC == "/camera/camera/color/camera_info"
    assert DEPTH_IMAGE_TOPIC == "/camera/camera/depth/image_rect_raw"
    assert DEPTH_CAMERA_INFO_TOPIC == "/camera/camera/depth/camera_info"


def test_vo_output_topics_match_expected_defaults() -> None:
    assert VO_ODOM_TOPIC == "/vo/odom"
    assert VO_ODOM_RAW_TOPIC == "/vo/odom/raw"
    assert VO_STATUS_TOPIC == "/vo/status"
    assert VO_HEALTH_TOPIC == "/vo/health"
    assert VO_TRACKING_QUALITY_TOPIC == "/vo/tracking_quality"
    assert VO_RESET_TOPIC == "/vo/reset"


def test_diagnostics_topic_matches_expected_default() -> None:
    assert DIAGNOSTICS_TOPIC == "/diagnostics"


def test_topic_names_are_absolute() -> None:
    topics = [
        COLOR_IMAGE_TOPIC,
        COLOR_CAMERA_INFO_TOPIC,
        DEPTH_IMAGE_TOPIC,
        DEPTH_CAMERA_INFO_TOPIC,
        VO_ODOM_TOPIC,
        VO_ODOM_RAW_TOPIC,
        VO_STATUS_TOPIC,
        VO_HEALTH_TOPIC,
        VO_TRACKING_QUALITY_TOPIC,
        VO_RESET_TOPIC,
        DIAGNOSTICS_TOPIC,
    ]

    for topic in topics:
        assert topic.startswith("/")
        assert "//" not in topic