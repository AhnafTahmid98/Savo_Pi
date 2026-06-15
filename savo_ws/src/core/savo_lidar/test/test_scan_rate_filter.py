# -*- coding: utf-8 -*-

import pytest

from savo_lidar.filters import (
    check_scan_rate,
    minimum_allowed_rate,
    rate_to_period_s,
)


def test_minimum_allowed_rate_uses_default_tolerance():
    assert minimum_allowed_rate(expected_hz=5.5) == pytest.approx(2.75)


def test_minimum_allowed_rate_uses_custom_tolerance():
    assert minimum_allowed_rate(expected_hz=10.0, tolerance_ratio=0.80) == pytest.approx(8.0)


def test_minimum_allowed_rate_rejects_zero_expected_rate():
    with pytest.raises(ValueError):
        minimum_allowed_rate(expected_hz=0.0)


def test_minimum_allowed_rate_rejects_negative_expected_rate():
    with pytest.raises(ValueError):
        minimum_allowed_rate(expected_hz=-1.0)


def test_minimum_allowed_rate_rejects_negative_tolerance():
    with pytest.raises(ValueError):
        minimum_allowed_rate(expected_hz=5.5, tolerance_ratio=-0.1)


def test_minimum_allowed_rate_rejects_tolerance_above_one():
    with pytest.raises(ValueError):
        minimum_allowed_rate(expected_hz=5.5, tolerance_ratio=1.1)


def test_check_scan_rate_reports_no_rate():
    status = check_scan_rate(rate_hz=0.0, expected_hz=5.5)

    assert not status.ok
    assert status.rate_hz == 0.0
    assert status.expected_hz == 5.5
    assert status.min_allowed_hz == pytest.approx(2.75)
    assert status.message == "no scan rate measured"


def test_check_scan_rate_clamps_negative_rate_to_zero():
    status = check_scan_rate(rate_hz=-5.0, expected_hz=5.5)

    assert not status.ok
    assert status.rate_hz == 0.0
    assert status.message == "no scan rate measured"


def test_check_scan_rate_reports_slow_rate():
    status = check_scan_rate(rate_hz=1.0, expected_hz=5.5)

    assert not status.ok
    assert status.rate_hz == 1.0
    assert status.expected_hz == 5.5
    assert status.min_allowed_hz == pytest.approx(2.75)
    assert status.message == "LiDAR scan rate below expected range"


def test_check_scan_rate_accepts_rate_at_minimum():
    status = check_scan_rate(rate_hz=2.75, expected_hz=5.5)

    assert status.ok
    assert status.rate_hz == 2.75
    assert status.message == "LiDAR scan rate healthy"


def test_check_scan_rate_accepts_healthy_rate():
    status = check_scan_rate(rate_hz=5.5, expected_hz=5.5)

    assert status.ok
    assert status.rate_hz == 5.5
    assert status.expected_hz == 5.5
    assert status.message == "LiDAR scan rate healthy"


def test_check_scan_rate_uses_custom_tolerance():
    status = check_scan_rate(
        rate_hz=7.0,
        expected_hz=10.0,
        tolerance_ratio=0.80,
    )

    assert not status.ok
    assert status.min_allowed_hz == pytest.approx(8.0)


def test_scan_rate_status_to_dict():
    status = check_scan_rate(rate_hz=5.5, expected_hz=5.5)
    data = status.to_dict()

    assert data["rate_hz"] == 5.5
    assert data["expected_hz"] == 5.5
    assert data["min_allowed_hz"] == pytest.approx(2.75)
    assert data["ok"] is True
    assert data["message"] == "LiDAR scan rate healthy"


def test_rate_to_period_s_accepts_positive_rate():
    assert rate_to_period_s(10.0) == pytest.approx(0.1)
    assert rate_to_period_s(5.0) == pytest.approx(0.2)


def test_rate_to_period_s_rejects_zero_rate():
    with pytest.raises(ValueError):
        rate_to_period_s(0.0)


def test_rate_to_period_s_rejects_negative_rate():
    with pytest.raises(ValueError):
        rate_to_period_s(-1.0)
