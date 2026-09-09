# Copyright 2026 Ahnaf Tahmid
from savo_realsense.models.stream_status import StreamStatus


def test_stream_status_ok_when_seen_fresh_and_publishing() -> None:
    status = StreamStatus(
        topic="/camera/camera/color/image_raw",
        seen=True,
        stale=False,
        rate_hz=30.0,
        expected_hz=30.0,
        last_age_s=0.01,
    )

    assert status.ok


def test_stream_status_not_ok_when_not_seen() -> None:
    status = StreamStatus(
        topic="/camera/camera/color/image_raw",
        seen=False,
        stale=False,
        rate_hz=0.0,
        expected_hz=30.0,
        last_age_s=float("inf"),
    )

    assert not status.ok


def test_stream_status_not_ok_when_stale() -> None:
    status = StreamStatus(
        topic="/camera/camera/depth/image_rect_raw",
        seen=True,
        stale=True,
        rate_hz=30.0,
        expected_hz=30.0,
        last_age_s=1.0,
    )

    assert not status.ok


def test_stream_status_not_ok_when_rate_is_zero() -> None:
    status = StreamStatus(
        topic="/camera/camera/depth/image_rect_raw",
        seen=True,
        stale=False,
        rate_hz=0.0,
        expected_hz=30.0,
        last_age_s=0.01,
    )

    assert not status.ok


def test_camera_rate_uses_production_minimum_not_legacy_ratio() -> None:
    status = StreamStatus(
        topic="/camera/camera/color/image_raw",
        seen=True,
        stale=False,
        rate_hz=7.99,
        expected_hz=15.0,
        last_age_s=0.01,
    )

    assert status.below_expected_rate


def test_camera_rate_quality_boundaries() -> None:
    status = StreamStatus(
        topic="/camera/camera/color/image_raw",
        seen=True,
        stale=False,
        rate_hz=8.0,
        expected_hz=15.0,
        last_age_s=0.01,
    )

    assert not status.below_expected_rate
    assert status.rate_quality == "MINIMUM"

    assert StreamStatus(
        "/color", True, False, 12.0, 15.0, 0.01
    ).rate_quality == "GOOD"
    assert StreamStatus(
        "/color", True, False, 14.0, 15.0, 0.01
    ).rate_quality == "EXCELLENT"


def test_one_hz_pointcloud_is_below_minimum_even_when_fresh() -> None:
    status = StreamStatus(
        topic="/camera/camera/depth/color/points",
        seen=True,
        stale=False,
        rate_hz=1.0,
        expected_hz=0.0,
        last_age_s=0.01,
    )

    assert status.below_expected_rate
    assert not status.ok


def test_pointcloud_below_three_hz_is_degraded_when_fresh() -> None:
    status = StreamStatus(
        topic="/camera/camera/depth/color/points",
        seen=True,
        stale=False,
        rate_hz=2.73,
        expected_hz=8.0,
        last_age_s=0.37,
    )

    assert not status.ok
    assert status.below_expected_rate


def test_pointcloud_rate_quality_boundaries() -> None:
    """The optional 8 Hz cloud producer reports its own quality bands."""
    def make(rate: float) -> StreamStatus:
        return StreamStatus(
            "/camera/camera/depth/color/points",
            True,
            False,
            rate,
            8.0,
            0.01,
        )
    assert make(3.0).rate_quality == "MINIMUM"
    assert make(5.0).rate_quality == "GOOD"
    assert make(7.0).rate_quality == "EXCELLENT"
