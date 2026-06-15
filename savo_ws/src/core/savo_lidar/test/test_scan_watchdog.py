# -*- coding: utf-8 -*-

import pytest

from savo_lidar.safety import ScanWatchdog, StaleScanPolicy
from savo_lidar.constants import STATUS_OK, STATUS_STALE, STATUS_WARN


def test_scan_watchdog_starts_stale_without_scan():
    watchdog = ScanWatchdog(timeout_s=1.0)

    assert watchdog.stale(now_s=10.0)
    assert watchdog.age_s(now_s=10.0) is None
    assert watchdog.status_message(now_s=10.0) == "no scan received"


def test_scan_watchdog_marks_scan_and_reports_fresh():
    watchdog = ScanWatchdog(timeout_s=1.0)
    watchdog.mark_scan(stamp_s=10.0)

    assert watchdog.scan_count == 1
    assert watchdog.last_scan_s == 10.0
    assert not watchdog.stale(now_s=10.5)
    assert watchdog.age_s(now_s=10.5) == 0.5
    assert watchdog.status_message(now_s=10.5) == "scan stream healthy | age_s=0.500"


def test_scan_watchdog_reports_stale_after_timeout():
    watchdog = ScanWatchdog(timeout_s=1.0)
    watchdog.mark_scan(stamp_s=10.0)

    assert watchdog.stale(now_s=11.5)
    assert watchdog.age_s(now_s=11.5) == 1.5
    assert watchdog.status_message(now_s=11.5) == "scan stream stale | age_s=1.500"


def test_scan_watchdog_increments_scan_count():
    watchdog = ScanWatchdog(timeout_s=1.0)

    watchdog.mark_scan(stamp_s=10.0)
    watchdog.mark_scan(stamp_s=10.1)
    watchdog.mark_scan(stamp_s=10.2)

    assert watchdog.scan_count == 3
    assert watchdog.last_scan_s == 10.2


def test_scan_watchdog_reset_clears_state():
    watchdog = ScanWatchdog(timeout_s=1.0)

    watchdog.mark_scan(stamp_s=10.0)
    watchdog.reset()

    assert watchdog.scan_count == 0
    assert watchdog.last_scan_s is None
    assert watchdog.stale(now_s=10.5)


def test_scan_watchdog_to_dict_contains_runtime_state():
    watchdog = ScanWatchdog(timeout_s=1.0)
    watchdog.mark_scan(stamp_s=10.0)

    data = watchdog.to_dict(now_s=10.5)

    assert data["timeout_s"] == 1.0
    assert data["last_scan_s"] == 10.0
    assert data["scan_count"] == 1
    assert data["age_s"] == 0.5
    assert data["stale"] is False
    assert data["message"] == "scan stream healthy | age_s=0.500"


def test_scan_watchdog_rejects_invalid_timeout():
    with pytest.raises(ValueError):
        ScanWatchdog(timeout_s=0.0)

    with pytest.raises(ValueError):
        ScanWatchdog(timeout_s=-1.0)


def test_stale_scan_policy_reports_missing_scan_as_stale():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=None, timeout_s=1.0)

    assert decision.stale
    assert decision.status == STATUS_STALE
    assert not decision.scan_ok
    assert decision.message == "no LiDAR scan received"


def test_stale_scan_policy_reports_fresh_scan():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=0.2, timeout_s=1.0)

    assert not decision.stale
    assert decision.status == STATUS_OK
    assert decision.scan_ok
    assert decision.message == "LiDAR scan fresh | age_s=0.200"


def test_stale_scan_policy_reports_warn_near_timeout():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=0.8, timeout_s=1.0)

    assert not decision.stale
    assert decision.status == STATUS_WARN
    assert decision.scan_ok
    assert decision.message == "LiDAR scan age near timeout | age_s=0.800"


def test_stale_scan_policy_reports_stale_after_timeout():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=1.2, timeout_s=1.0)

    assert decision.stale
    assert decision.status == STATUS_STALE
    assert not decision.scan_ok
    assert decision.message == "LiDAR scan stale | age_s=1.200"


def test_stale_scan_policy_clamps_negative_age_to_zero():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=-1.0, timeout_s=1.0)

    assert not decision.stale
    assert decision.status == STATUS_OK
    assert decision.scan_ok
    assert decision.message == "LiDAR scan fresh | age_s=0.000"


def test_stale_scan_policy_to_dict_contains_decision_fields():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=0.2, timeout_s=1.0)
    data = decision.to_dict()

    assert data["stale"] is False
    assert data["status"] == STATUS_OK
    assert data["scan_ok"] is True
    assert data["message"] == "LiDAR scan fresh | age_s=0.200"


def test_stale_scan_policy_rejects_invalid_warn_ratio():
    with pytest.raises(ValueError):
        StaleScanPolicy(warn_before_stale_ratio=-0.1)

    with pytest.raises(ValueError):
        StaleScanPolicy(warn_before_stale_ratio=1.1)


def test_stale_scan_policy_rejects_invalid_timeout():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    with pytest.raises(ValueError):
        policy.evaluate(last_scan_age_s=0.1, timeout_s=0.0)

    with pytest.raises(ValueError):
        policy.evaluate(last_scan_age_s=0.1, timeout_s=-1.0)
