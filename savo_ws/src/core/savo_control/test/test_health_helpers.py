#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for diagnostics health helpers."""

from __future__ import annotations

from savo_control.diagnostics import (
    HealthCheck,
    HealthReport,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
    bool_health_check,
    command_health_check,
    freshness_check,
    make_health_report,
    odom_health_check,
    safety_health_check,
    status_name_for_checks,
)


def test_health_check_status_ok():
    check = HealthCheck("cmd_vel", ok=True, value="fresh")

    assert check.status() == STATUS_OK
    assert check.to_dict() == {
        "name": "cmd_vel",
        "ok": True,
        "stale": False,
        "warning": False,
        "status": STATUS_OK,
        "value": "fresh",
        "note": "",
    }


def test_health_check_status_warn():
    check = HealthCheck("slowdown", ok=True, warning=True, value=0.5)

    assert check.status() == STATUS_WARN
    assert check.to_diagnostic_item().status == STATUS_WARN


def test_health_check_status_error():
    check = HealthCheck("safety_stop", ok=False, value=True)

    assert check.status() == STATUS_ERROR
    assert check.to_diagnostic_item().status == STATUS_ERROR


def test_health_check_status_stale_has_priority():
    check = HealthCheck("odom", ok=False, stale=True, warning=True)

    assert check.status() == STATUS_STALE
    assert check.to_diagnostic_item().status == STATUS_STALE


def test_health_report_ok():
    report = make_health_report(
        "savo_control",
        [
            HealthCheck("cmd_vel", ok=True),
            HealthCheck("odom", ok=True),
        ],
    )

    assert isinstance(report, HealthReport)
    assert report.name == "savo_control"
    assert report.overall == STATUS_OK
    assert report.ok() is True
    assert report.has_errors() is False
    assert report.has_stale() is False


def test_health_report_warn():
    report = make_health_report(
        "savo_control",
        [
            HealthCheck("cmd_vel", ok=True),
            HealthCheck("slowdown", ok=True, warning=True),
        ],
    )

    assert report.overall == STATUS_WARN
    assert report.ok() is False
    assert report.has_errors() is False
    assert report.has_stale() is False


def test_health_report_error_priority():
    report = make_health_report(
        "savo_control",
        [
            HealthCheck("odom", ok=True, stale=True),
            HealthCheck("safety_stop", ok=False),
        ],
    )

    assert report.overall == STATUS_ERROR
    assert report.ok() is False
    assert report.has_errors() is True
    assert report.has_stale() is True


def test_health_report_to_items_and_dict():
    report = make_health_report(
        "savo_control",
        [
            HealthCheck("cmd_vel", ok=True, value="manual"),
        ],
    )

    items = report.to_items()
    data = report.to_dict()

    assert len(items) == 1
    assert items[0].name == "cmd_vel"
    assert items[0].status == STATUS_OK

    assert data["name"] == "savo_control"
    assert data["overall"] == STATUS_OK
    assert data["ok"] is True
    assert data["checks"][0]["name"] == "cmd_vel"
    assert data["checks"][0]["value"] == "manual"


def test_health_report_status_text():
    report = make_health_report(
        "savo_control",
        [
            HealthCheck("cmd_vel", ok=True, value="manual"),
            HealthCheck("slowdown", ok=True, warning=True, value=0.5),
        ],
    )

    text = report.status_text()

    assert "savo_control: WARN" in text
    assert "cmd_vel: OK | value=manual" in text
    assert "slowdown: WARN | value=0.5" in text


def test_freshness_check_ok():
    check = freshness_check("odom", age_s=0.10, timeout_s=0.30)

    assert check.name == "odom"
    assert check.ok is True
    assert check.stale is False
    assert check.status() == STATUS_OK
    assert check.value == "0.100s"
    assert check.note == "timeout=0.300s"


def test_freshness_check_stale():
    check = freshness_check("odom", age_s=0.80, timeout_s=0.30, value="old")

    assert check.ok is False
    assert check.stale is True
    assert check.status() == STATUS_STALE
    assert check.value == "old"
    assert check.note == "timeout=0.300s"


def test_bool_health_check():
    ok = bool_health_check("config", ok=True, value="loaded")
    bad = bool_health_check("config", ok=False, value="missing", note="not found")

    assert ok.status() == STATUS_OK
    assert ok.value == "loaded"

    assert bad.status() == STATUS_ERROR
    assert bad.value == "missing"
    assert bad.note == "not found"


def test_safety_health_check_ok():
    check = safety_health_check(safety_stop=False, slowdown_factor=1.0)

    assert check.name == "safety_stop"
    assert check.ok is True
    assert check.warning is False
    assert check.status() == STATUS_OK
    assert check.value is False
    assert check.note == "slowdown=1.00"


def test_safety_health_check_slowdown_warning():
    check = safety_health_check(safety_stop=False, slowdown_factor=0.5)

    assert check.ok is True
    assert check.warning is True
    assert check.status() == STATUS_WARN
    assert check.value is False
    assert check.note == "slowdown=0.50"


def test_safety_health_check_error_when_active():
    check = safety_health_check(safety_stop=True)

    assert check.ok is False
    assert check.status() == STATUS_ERROR
    assert check.value is True
    assert check.note == "safety stop active"


def test_command_health_check_fresh():
    check = command_health_check(command_fresh=True, selected_source="manual")

    assert check.name == "command"
    assert check.ok is True
    assert check.stale is False
    assert check.status() == STATUS_OK
    assert check.value == "manual"
    assert check.note == "selected command source"


def test_command_health_check_stale():
    check = command_health_check(command_fresh=False, selected_source="auto")

    assert check.ok is False
    assert check.stale is True
    assert check.status() == STATUS_STALE
    assert check.value == "auto"


def test_odom_health_check_fresh():
    check = odom_health_check(odom_fresh=True, age_s=0.12)

    assert check.name == "odometry"
    assert check.ok is True
    assert check.stale is False
    assert check.status() == STATUS_OK
    assert check.value == "0.120s"
    assert check.note == "filtered odom freshness"


def test_odom_health_check_stale_without_age():
    check = odom_health_check(odom_fresh=False)

    assert check.ok is False
    assert check.stale is True
    assert check.status() == STATUS_STALE
    assert check.value == ""


def test_status_name_for_checks():
    assert status_name_for_checks([HealthCheck("cmd", ok=True)]) == STATUS_OK

    assert status_name_for_checks(
        [
            HealthCheck("cmd", ok=True),
            HealthCheck("slowdown", ok=True, warning=True),
        ]
    ) == STATUS_WARN

    assert status_name_for_checks(
        [
            HealthCheck("cmd", ok=True),
            HealthCheck("odom", ok=False, stale=True),
        ]
    ) == STATUS_STALE

    assert status_name_for_checks(
        [
            HealthCheck("safety", ok=False),
        ]
    ) == STATUS_ERROR


def test_status_name_for_empty_checks_is_unknown():
    assert status_name_for_checks([]) == STATUS_UNKNOWN
