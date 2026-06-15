# Copyright 2026 Ahnaf Tahmid
from savo_realsense.models.stream_status import StreamStatus
from savo_realsense.utils.camera_checks import (
    all_required_streams_healthy,
    build_stream_status,
    low_rate_topics,
    missing_or_stale_topics,
    stream_is_healthy,
)


def test_build_stream_status_marks_seen_stream_healthy() -> None:
    status = build_stream_status(
        topic="/camera/camera/color/image_raw",
        seen=True,
        rate_hz=30.0,
        expected_hz=30.0,
        last_age_s=0.02,
        stale_timeout_s=0.50,
    )

    assert status.seen
    assert not status.stale
    assert status.ok


def test_build_stream_status_marks_old_stream_stale() -> None:
    status = build_stream_status(
        topic="/camera/camera/depth/image_rect_raw",
        seen=True,
        rate_hz=30.0,
        expected_hz=30.0,
        last_age_s=0.80,
        stale_timeout_s=0.50,
    )

    assert status.stale
    assert not status.ok


def test_build_stream_status_clamps_negative_values() -> None:
    status = build_stream_status(
        topic="/camera/camera/depth/image_rect_raw",
        seen=True,
        rate_hz=-10.0,
        expected_hz=-30.0,
        last_age_s=-1.0,
        stale_timeout_s=0.50,
    )

    assert status.rate_hz == 0.0
    assert status.expected_hz == 0.0
    assert status.last_age_s == 0.0


def test_stream_is_healthy_rejects_low_rate_stream() -> None:
    status = StreamStatus(
        topic="/camera/camera/color/image_raw",
        seen=True,
        stale=False,
        rate_hz=10.0,
        expected_hz=30.0,
        last_age_s=0.02,
    )

    assert status.ok
    assert status.below_expected_rate
    assert not stream_is_healthy(status)


def test_missing_or_stale_topics_returns_bad_topics() -> None:
    statuses = [
        StreamStatus("/color", True, False, 30.0, 30.0, 0.01),
        StreamStatus("/depth", True, True, 30.0, 30.0, 1.00),
        StreamStatus("/info", False, False, 0.0, 30.0, float("inf")),
    ]

    assert missing_or_stale_topics(statuses) == ["/depth", "/info"]


def test_low_rate_topics_returns_only_live_low_rate_topics() -> None:
    statuses = [
        StreamStatus("/color", True, False, 30.0, 30.0, 0.01),
        StreamStatus("/depth", True, False, 8.0, 30.0, 0.01),
        StreamStatus("/info", True, True, 8.0, 30.0, 1.00),
    ]

    assert low_rate_topics(statuses) == ["/depth"]


def test_all_required_streams_healthy_accepts_good_streams() -> None:
    statuses = [
        StreamStatus("/color", True, False, 30.0, 30.0, 0.01),
        StreamStatus("/depth", True, False, 30.0, 30.0, 0.01),
    ]

    assert all_required_streams_healthy(statuses)


def test_all_required_streams_healthy_rejects_stale_stream() -> None:
    statuses = [
        StreamStatus("/color", True, False, 30.0, 30.0, 0.01),
        StreamStatus("/depth", True, True, 30.0, 30.0, 1.00),
    ]

    assert not all_required_streams_healthy(statuses)
