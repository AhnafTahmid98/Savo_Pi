from savo_lidar.constants import STATUS_OK, STATUS_STALE, STATUS_WARN
from savo_lidar.safety.stale_scan_policy import StaleScanPolicy


def test_stale_scan_policy_reports_stale_when_no_scan_received():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(
        last_scan_age_s=None,
        timeout_s=1.0,
    )

    assert decision.stale
    assert not decision.scan_ok
    assert decision.status == STATUS_STALE
    assert decision.message == "no LiDAR scan received"


def test_stale_scan_policy_reports_stale_after_timeout():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(
        last_scan_age_s=1.2,
        timeout_s=1.0,
    )

    assert decision.stale
    assert not decision.scan_ok
    assert decision.status == STATUS_STALE
    assert "LiDAR scan stale" in decision.message


def test_stale_scan_policy_reports_warn_near_timeout():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(
        last_scan_age_s=0.80,
        timeout_s=1.0,
    )

    assert not decision.stale
    assert decision.scan_ok
    assert decision.status == STATUS_WARN
    assert "near timeout" in decision.message


def test_stale_scan_policy_reports_ok_when_scan_is_fresh():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(
        last_scan_age_s=0.20,
        timeout_s=1.0,
    )

    assert not decision.stale
    assert decision.scan_ok
    assert decision.status == STATUS_OK
    assert "fresh" in decision.message


def test_stale_scan_policy_clamps_negative_age_to_zero():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(
        last_scan_age_s=-1.0,
        timeout_s=1.0,
    )

    assert not decision.stale
    assert decision.scan_ok
    assert decision.status == STATUS_OK
    assert "age_s=0.000" in decision.message


def test_stale_scan_policy_rejects_invalid_timeout():
    policy = StaleScanPolicy()

    try:
        policy.evaluate(
            last_scan_age_s=0.1,
            timeout_s=0.0,
        )
    except ValueError as exc:
        assert "timeout_s must be > 0.0" in str(exc)
    else:
        raise AssertionError("Expected ValueError for invalid timeout")


def test_stale_scan_policy_rejects_invalid_warn_ratio():
    try:
        StaleScanPolicy(warn_before_stale_ratio=1.5)
    except ValueError as exc:
        assert "warn_before_stale_ratio" in str(exc)
    else:
        raise AssertionError("Expected ValueError for invalid warn ratio")