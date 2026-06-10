"""Tests for visual odometry status models."""

from savo_vo.models.vo_status import VOState, VOStatus


def test_ok_status_is_healthy_and_usable() -> None:
    status = VOStatus.ok(
        tracking_quality=0.85,
        feature_count=320,
        age_s=0.02,
    )

    assert status.state == VOState.OK
    assert status.is_healthy
    assert status.is_usable
    assert status.tracking_quality == 0.85
    assert status.feature_count == 320
    assert status.age_s == 0.02


def test_degraded_status_is_usable_but_not_healthy() -> None:
    status = VOStatus.degraded(
        message="tracking quality is low",
        tracking_quality=0.40,
        feature_count=120,
        age_s=0.04,
    )

    assert status.state == VOState.DEGRADED
    assert not status.is_healthy
    assert status.is_usable
    assert status.message == "tracking quality is low"


def test_lost_status_is_not_usable() -> None:
    status = VOStatus.lost()

    assert status.state == VOState.LOST
    assert not status.is_healthy
    assert not status.is_usable


def test_stale_status_records_age() -> None:
    status = VOStatus.stale(age_s=0.75)

    assert status.state == VOState.STALE
    assert status.age_s == 0.75
    assert not status.is_healthy
    assert not status.is_usable


def test_error_status_is_not_usable() -> None:
    status = VOStatus.error("camera input failed")

    assert status.state == VOState.ERROR
    assert status.message == "camera input failed"
    assert not status.is_healthy
    assert not status.is_usable