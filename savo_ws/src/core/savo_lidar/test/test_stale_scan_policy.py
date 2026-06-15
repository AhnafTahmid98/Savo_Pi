# -*- coding: utf-8 -*-

import pytest

from savo_lidar.constants import STATUS_OK, STATUS_STALE, STATUS_WARN
from savo_lidar.safety import StaleScanDecision, StaleScanPolicy


def test_stale_scan_policy_reports_missing_scan_as_stale():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=None, timeout_s=1.0)

    assert isinstance(decision, StaleScanDecision)
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


def test_stale_scan_policy_reports_warning_near_timeout():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=0.75, timeout_s=1.0)

    assert not decision.stale
    assert decision.status == STATUS_WARN
    assert decision.scan_ok
    assert decision.message == "LiDAR scan age near timeout | age_s=0.750"


def test_stale_scan_policy_reports_stale_after_timeout():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=1.01, timeout_s=1.0)

    assert decision.stale
    assert decision.status == STATUS_STALE
    assert not decision.scan_ok
    assert decision.message == "LiDAR scan stale | age_s=1.010"


def test_stale_scan_policy_treats_exact_timeout_as_warning_not_stale():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=1.0, timeout_s=1.0)

    assert not decision.stale
    assert decision.status == STATUS_WARN
    assert decision.scan_ok
    assert decision.message == "LiDAR scan age near timeout | age_s=1.000"


def test_stale_scan_policy_clamps_negative_age_to_zero():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=-1.0, timeout_s=1.0)

    assert not decision.stale
    assert decision.status == STATUS_OK
    assert decision.scan_ok
    assert decision.message == "LiDAR scan fresh | age_s=0.000"


def test_stale_scan_policy_accepts_zero_warn_ratio():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.0)

    decision = policy.evaluate(last_scan_age_s=0.0, timeout_s=1.0)

    assert not decision.stale
    assert decision.status == STATUS_WARN
    assert decision.scan_ok


def test_stale_scan_policy_accepts_one_warn_ratio():
    policy = StaleScanPolicy(warn_before_stale_ratio=1.0)

    fresh = policy.evaluate(last_scan_age_s=0.9, timeout_s=1.0)
    warning = policy.evaluate(last_scan_age_s=1.0, timeout_s=1.0)

    assert fresh.status == STATUS_OK
    assert warning.status == STATUS_WARN


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


def test_stale_scan_decision_to_dict_contains_all_fields():
    policy = StaleScanPolicy(warn_before_stale_ratio=0.75)

    decision = policy.evaluate(last_scan_age_s=0.2, timeout_s=1.0)
    data = decision.to_dict()

    assert data == {
        "stale": False,
        "status": STATUS_OK,
        "scan_ok": True,
        "message": "LiDAR scan fresh | age_s=0.200",
    }
