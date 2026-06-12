import math

import pytest

from savo_lidar.filters.scan_rate_filter import (
    check_scan_rate,
    minimum_allowed_rate,
    rate_to_period_s,
)


def test_minimum_allowed_rate_uses_tolerance_ratio():
    assert minimum_allowed_rate(5.5, tolerance_ratio=0.50) == 2.75


def test_minimum_allowed_rate_rejects_invalid_expected_rate():
    with pytest.raises(ValueError):
        minimum_allowed_rate(0.0)


def test_minimum_allowed_rate_rejects_invalid_tolerance_ratio():
    with pytest.raises(ValueError):
        minimum_allowed_rate(5.5, tolerance_ratio=1.5)


def test_check_scan_rate_reports_no_rate_measured():
    status = check_scan_rate(
        rate_hz=0.0,
        expected_hz=5.5,
        tolerance_ratio=0.50,
    )

    assert not status.ok
    assert status.rate_hz == 0.0
    assert status.expected_hz == 5.5
    assert status.min_allowed_hz == 2.75
    assert status.message == "no scan rate measured"


def test_check_scan_rate_reports_low_rate():
    status = check_scan_rate(
        rate_hz=1.0,
        expected_hz=5.5,
        tolerance_ratio=0.50,
    )

    assert not status.ok
    assert status.message == "LiDAR scan rate below expected range"


def test_check_scan_rate_reports_healthy_rate():
    status = check_scan_rate(
        rate_hz=5.0,
        expected_hz=5.5,
        tolerance_ratio=0.50,
    )

    assert status.ok
    assert status.message == "LiDAR scan rate healthy"


def test_rate_to_period_s_converts_rate_to_period():
    assert math.isclose(rate_to_period_s(10.0), 0.1)


def test_rate_to_period_s_rejects_invalid_rate():
    with pytest.raises(ValueError):
        rate_to_period_s(0.0)