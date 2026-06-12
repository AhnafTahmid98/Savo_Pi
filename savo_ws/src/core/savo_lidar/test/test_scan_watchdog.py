import pytest

from savo_lidar.safety.scan_watchdog import ScanWatchdog


def test_scan_watchdog_starts_stale_before_any_scan():
    watchdog = ScanWatchdog(timeout_s=1.0)

    assert watchdog.stale(now_s=10.0)
    assert watchdog.age_s(now_s=10.0) is None
    assert watchdog.scan_count == 0


def test_scan_watchdog_marks_scan_and_reports_fresh_state():
    watchdog = ScanWatchdog(timeout_s=1.0)
    watchdog.mark_scan(stamp_s=10.0)

    assert not watchdog.stale(now_s=10.5)
    assert watchdog.age_s(now_s=10.5) == 0.5
    assert watchdog.scan_count == 1


def test_scan_watchdog_reports_stale_after_timeout():
    watchdog = ScanWatchdog(timeout_s=1.0)
    watchdog.mark_scan(stamp_s=10.0)

    assert watchdog.stale(now_s=11.1)
    assert watchdog.age_s(now_s=11.1) == pytest.approx(1.1)


def test_scan_watchdog_reset_clears_state():
    watchdog = ScanWatchdog(timeout_s=1.0)
    watchdog.mark_scan(stamp_s=10.0)
    watchdog.reset()

    assert watchdog.scan_count == 0
    assert watchdog.last_scan_s is None
    assert watchdog.stale(now_s=20.0)


def test_scan_watchdog_status_message_before_scan():
    watchdog = ScanWatchdog(timeout_s=1.0)

    assert watchdog.status_message(now_s=10.0) == "no scan received"


def test_scan_watchdog_status_message_when_fresh():
    watchdog = ScanWatchdog(timeout_s=1.0)
    watchdog.mark_scan(stamp_s=10.0)

    assert watchdog.status_message(now_s=10.2) == "scan stream healthy | age_s=0.200"


def test_scan_watchdog_status_message_when_stale():
    watchdog = ScanWatchdog(timeout_s=1.0)
    watchdog.mark_scan(stamp_s=10.0)

    assert watchdog.status_message(now_s=11.5) == "scan stream stale | age_s=1.500"


def test_scan_watchdog_rejects_invalid_timeout():
    try:
        ScanWatchdog(timeout_s=0.0)
    except ValueError as exc:
        assert "timeout_s must be > 0.0" in str(exc)
    else:
        raise AssertionError("Expected ValueError for invalid timeout")