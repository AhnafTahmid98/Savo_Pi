# Copyright 2026 Ahnaf Tahmid
from savo_realsense.models.camera_status import CameraStatus
from savo_realsense.models.stream_status import StreamStatus


def make_stream(
    seen: bool = True,
    stale: bool = False,
    rate_hz: float = 30.0,
    expected_hz: float = 30.0,
) -> StreamStatus:
    return StreamStatus(
        topic="/camera",
        seen=seen,
        stale=stale,
        rate_hz=rate_hz,
        expected_hz=expected_hz,
        last_age_s=0.01 if seen else float("inf"),
    )


def test_camera_status_ok_without_pointcloud_required() -> None:
    stream = make_stream()

    status = CameraStatus(
        color=stream,
        color_info=stream,
        depth=stream,
        depth_info=stream,
        pointcloud=None,
        require_pointcloud=False,
    )

    assert status.ok
    assert status.pointcloud_ok
    assert status.message == "RealSense streams OK"


def test_camera_status_fails_when_pointcloud_required_but_missing() -> None:
    stream = make_stream()

    status = CameraStatus(
        color=stream,
        color_info=stream,
        depth=stream,
        depth_info=stream,
        pointcloud=None,
        require_pointcloud=True,
    )

    assert not status.ok
    assert not status.pointcloud_ok
    assert status.message == "Unhealthy streams: pointcloud"


def test_camera_status_fails_when_depth_is_stale() -> None:
    good = make_stream()
    stale_depth = make_stream(stale=True)

    status = CameraStatus(
        color=good,
        color_info=good,
        depth=stale_depth,
        depth_info=good,
        pointcloud=None,
        require_pointcloud=False,
    )

    assert not status.ok
    assert not status.depth_ok
    assert status.message == "Unhealthy streams: depth"


def test_camera_status_reports_no_streams_detected() -> None:
    missing = make_stream(seen=False, rate_hz=0.0)

    status = CameraStatus(
        color=missing,
        color_info=missing,
        depth=missing,
        depth_info=missing,
        pointcloud=None,
        require_pointcloud=False,
    )

    assert not status.ok
    assert not status.any_stream_seen
    assert status.message == "No RealSense streams detected"


def test_camera_status_reports_multiple_unhealthy_streams() -> None:
    good = make_stream()
    missing = make_stream(seen=False, rate_hz=0.0)
    stale = make_stream(stale=True)

    status = CameraStatus(
        color=good,
        color_info=missing,
        depth=stale,
        depth_info=good,
        pointcloud=None,
        require_pointcloud=False,
    )

    assert not status.ok
    assert status.any_stream_seen
    assert status.message == "Unhealthy streams: color_info, depth"


def test_camera_status_ignores_unhealthy_pointcloud_when_not_required() -> None:
    stream = make_stream()
    bad_pointcloud = make_stream(seen=False, rate_hz=0.0, expected_hz=10.0)

    status = CameraStatus(
        color=stream,
        color_info=stream,
        depth=stream,
        depth_info=stream,
        pointcloud=bad_pointcloud,
        require_pointcloud=False,
    )

    assert status.ok
    assert status.pointcloud_ok
    assert status.message == "RealSense streams OK"


def test_camera_status_accepts_healthy_pointcloud_when_required() -> None:
    stream = make_stream()
    pointcloud = make_stream(rate_hz=10.0, expected_hz=10.0)

    status = CameraStatus(
        color=stream,
        color_info=stream,
        depth=stream,
        depth_info=stream,
        pointcloud=pointcloud,
        require_pointcloud=True,
    )

    assert status.ok
    assert status.pointcloud_ok
