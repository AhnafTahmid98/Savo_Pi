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


def test_stream_status_below_expected_rate_when_less_than_half_expected() -> None:
    status = StreamStatus(
        topic="/camera/camera/color/image_raw",
        seen=True,
        stale=False,
        rate_hz=10.0,
        expected_hz=30.0,
        last_age_s=0.01,
    )

    assert status.below_expected_rate


def test_stream_status_not_below_expected_rate_at_half_expected() -> None:
    status = StreamStatus(
        topic="/camera/camera/color/image_raw",
        seen=True,
        stale=False,
        rate_hz=15.0,
        expected_hz=30.0,
        last_age_s=0.01,
    )

    assert not status.below_expected_rate


def test_stream_status_ignores_rate_check_when_expected_rate_is_zero() -> None:
    status = StreamStatus(
        topic="/camera/camera/depth/color/points",
        seen=True,
        stale=False,
        rate_hz=1.0,
        expected_hz=0.0,
        last_age_s=0.01,
    )

    assert not status.below_expected_rate