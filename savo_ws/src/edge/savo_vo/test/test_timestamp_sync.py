"""Tests for timestamp synchronization helpers."""

import pytest

from savo_vo.core.timestamp_sync import (
    age_s,
    check_timestamp_sync,
    is_stale,
    is_timestamp_synced,
    timestamp_delta_s,
)


def test_timestamp_delta_uses_absolute_difference() -> None:
    assert timestamp_delta_s(10.0, 10.2) == pytest.approx(0.2)
    assert timestamp_delta_s(10.2, 10.0) == pytest.approx(0.2)


def test_is_timestamp_synced_accepts_values_inside_limit() -> None:
    assert is_timestamp_synced(
        first_timestamp_s=5.0,
        second_timestamp_s=5.04,
        max_delta_s=0.05,
    )


def test_is_timestamp_synced_rejects_values_outside_limit() -> None:
    assert not is_timestamp_synced(
        first_timestamp_s=5.0,
        second_timestamp_s=5.20,
        max_delta_s=0.05,
    )


def test_is_timestamp_synced_rejects_negative_limit() -> None:
    with pytest.raises(ValueError):
        is_timestamp_synced(
            first_timestamp_s=1.0,
            second_timestamp_s=1.0,
            max_delta_s=-0.1,
        )


def test_check_timestamp_sync_returns_clear_result() -> None:
    result = check_timestamp_sync(
        first_timestamp_s=1.0,
        second_timestamp_s=1.03,
        max_delta_s=0.05,
    )

    assert result.is_synced
    assert result.delta_s == pytest.approx(0.03)
    assert result.message == "timestamps are synchronized"


def test_check_timestamp_sync_reports_unsynced_result() -> None:
    result = check_timestamp_sync(
        first_timestamp_s=1.0,
        second_timestamp_s=1.20,
        max_delta_s=0.05,
    )

    assert not result.is_synced
    assert result.delta_s == pytest.approx(0.20)
    assert "exceeds limit" in result.message


def test_age_is_never_negative() -> None:
    assert age_s(timestamp_s=10.0, now_s=12.5) == pytest.approx(2.5)
    assert age_s(timestamp_s=12.5, now_s=10.0) == pytest.approx(0.0)


def test_is_stale_uses_timeout() -> None:
    assert is_stale(timestamp_s=10.0, now_s=10.6, stale_timeout_s=0.5)
    assert not is_stale(timestamp_s=10.0, now_s=10.4, stale_timeout_s=0.5)


def test_is_stale_rejects_negative_timeout() -> None:
    with pytest.raises(ValueError):
        is_stale(timestamp_s=10.0, now_s=10.0, stale_timeout_s=-0.1)