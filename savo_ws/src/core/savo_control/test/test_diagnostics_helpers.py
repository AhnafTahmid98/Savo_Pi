#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for diagnostics helper utilities."""

from __future__ import annotations

from savo_control.diagnostics import (
    DiagnosticItem,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
    format_bool,
    format_float,
    format_key_values,
    format_report,
    format_seconds,
    format_section,
    status_from_age,
    status_from_bool,
    summarize_items,
)


def test_diagnostic_item_normalizes_known_statuses():
    assert DiagnosticItem("cmd", "ok").normalized_status() == STATUS_OK
    assert DiagnosticItem("cmd", "warn").normalized_status() == STATUS_WARN
    assert DiagnosticItem("cmd", "error").normalized_status() == STATUS_ERROR
    assert DiagnosticItem("cmd", "stale").normalized_status() == STATUS_STALE


def test_diagnostic_item_unknown_status_becomes_unknown():
    assert DiagnosticItem("cmd", "bad").normalized_status() == STATUS_UNKNOWN
    assert DiagnosticItem("cmd", "").normalized_status() == STATUS_UNKNOWN


def test_diagnostic_item_is_ok():
    assert DiagnosticItem("cmd", STATUS_OK).is_ok() is True
    assert DiagnosticItem("cmd", STATUS_WARN).is_ok() is False
    assert DiagnosticItem("cmd", STATUS_ERROR).is_ok() is False


def test_diagnostic_item_to_line():
    assert DiagnosticItem("cmd_vel", STATUS_OK).to_line() == "cmd_vel: OK"

    item = DiagnosticItem("odom", STATUS_STALE, "0.80s", "timeout")
    assert item.to_line() == "odom: STALE | value=0.80s | note=timeout"


def test_status_from_bool():
    assert status_from_bool(True) == STATUS_OK
    assert status_from_bool(False) == STATUS_ERROR
    assert status_from_bool(True, stale=True) == STATUS_STALE
    assert status_from_bool(False, stale=True) == STATUS_STALE


def test_status_from_age():
    assert status_from_age(0.10, 0.50) == STATUS_OK
    assert status_from_age(0.50, 0.50) == STATUS_OK
    assert status_from_age(0.51, 0.50) == STATUS_STALE
    assert status_from_age(-1.0, 0.50) == STATUS_UNKNOWN


def test_format_helpers():
    assert format_seconds(0.12345) == "0.123s"
    assert format_float(1.23456) == "1.235"
    assert format_float(1.23456, unit="m", digits=2) == "1.23m"
    assert format_bool(True) == "true"
    assert format_bool(False) == "false"


def test_format_key_values():
    text = format_key_values(
        {
            "mode": "AUTO",
            "safety_stop": False,
            "vx": 0.1,
        }
    )

    assert text == "mode=AUTO; safety_stop=False; vx=0.1"


def test_format_section():
    assert format_section("Control", []) == "Control"

    text = format_section("Control", ["cmd_vel: OK", "", "odom: STALE"])
    assert text == "Control\ncmd_vel: OK\nodom: STALE"


def test_summarize_items_empty_is_unknown():
    assert summarize_items([]) == STATUS_UNKNOWN


def test_summarize_items_priority():
    assert summarize_items(
        [
            DiagnosticItem("cmd", STATUS_OK),
            DiagnosticItem("odom", STATUS_OK),
        ]
    ) == STATUS_OK

    assert summarize_items(
        [
            DiagnosticItem("cmd", STATUS_OK),
            DiagnosticItem("odom", STATUS_WARN),
        ]
    ) == STATUS_WARN

    assert summarize_items(
        [
            DiagnosticItem("cmd", STATUS_OK),
            DiagnosticItem("odom", STATUS_STALE),
        ]
    ) == STATUS_STALE

    assert summarize_items(
        [
            DiagnosticItem("cmd", STATUS_STALE),
            DiagnosticItem("odom", STATUS_ERROR),
        ]
    ) == STATUS_ERROR


def test_format_report():
    items = [
        DiagnosticItem("cmd_vel", STATUS_OK, "fresh"),
        DiagnosticItem("odom", STATUS_STALE, "0.80s"),
    ]

    report = format_report("savo_control", items)

    assert "savo_control: STALE" in report
    assert "cmd_vel: OK | value=fresh" in report
    assert "odom: STALE | value=0.80s" in report
