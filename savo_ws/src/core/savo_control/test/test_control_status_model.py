#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for the pure Python control-status model."""

from __future__ import annotations

from savo_control.models import (
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_STOPPED,
    STATUS_UNKNOWN,
    STATUS_WARN,
    ControlMode,
    ControlStatus,
    TwistCommand,
    control_status_from_values,
)


def test_default_status_is_safe_unknown_stop():
    status = ControlStatus()

    assert status.status == STATUS_UNKNOWN
    assert status.normalized_status() == STATUS_UNKNOWN
    assert status.mode == ControlMode.STOP
    assert status.selected_source == "none"
    assert status.safety_stop is False
    assert status.healthy() is False
    assert status.blocked() is False


def test_stopped_factory():
    status = ControlStatus.stopped(reason="startup")

    assert status.status == STATUS_STOPPED
    assert status.mode == ControlMode.STOP
    assert status.selected_source == "none"
    assert status.note == "startup"
    assert status.healthy() is False
    assert status.blocked() is False


def test_ok_factory_accepts_text_mode():
    status = ControlStatus.ok(mode="MANUAL", selected_source="manual")

    assert status.status == STATUS_OK
    assert status.mode == ControlMode.MANUAL
    assert status.selected_source == "manual"
    assert status.healthy() is True
    assert status.blocked() is False


def test_normalized_status_accepts_known_statuses():
    assert ControlStatus(status="ok").normalized_status() == STATUS_OK
    assert ControlStatus(status="warn").normalized_status() == STATUS_WARN
    assert ControlStatus(status="error").normalized_status() == STATUS_ERROR
    assert ControlStatus(status="stale").normalized_status() == STATUS_STALE
    assert ControlStatus(status="stopped").normalized_status() == STATUS_STOPPED


def test_normalized_status_rejects_unknown_status():
    assert ControlStatus(status="bad").normalized_status() == STATUS_UNKNOWN
    assert ControlStatus(status="").normalized_status() == STATUS_UNKNOWN


def test_healthy_requires_ok_no_safety_and_fresh_data():
    assert ControlStatus(status=STATUS_OK).healthy() is True

    assert ControlStatus(status=STATUS_WARN).healthy() is False
    assert ControlStatus(status=STATUS_ERROR).healthy() is False
    assert ControlStatus(status=STATUS_OK, safety_stop=True).healthy() is False
    assert ControlStatus(status=STATUS_OK, command_fresh=False).healthy() is False
    assert ControlStatus(status=STATUS_OK, odom_fresh=False).healthy() is False


def test_blocked_when_safety_stop_stuck_or_error():
    assert ControlStatus(status=STATUS_OK).blocked() is False
    assert ControlStatus(status=STATUS_OK, safety_stop=True).blocked() is True
    assert ControlStatus(status=STATUS_OK, stuck_detected=True).blocked() is True
    assert ControlStatus(status=STATUS_ERROR).blocked() is True


def test_to_dict_contains_core_status_fields():
    status = ControlStatus(
        status=STATUS_OK,
        mode=ControlMode.AUTO,
        selected_source="auto",
        cmd_vel=TwistCommand(vx=0.1, source="cmd_vel"),
        cmd_vel_mux=TwistCommand(vx=0.08, source="mux"),
        cmd_vel_safe=TwistCommand(vx=0.05, source="safe"),
        safety_stop=False,
        slowdown_factor=0.50,
        recovery_active=True,
        stuck_detected=False,
        odom_fresh=True,
        command_fresh=True,
        note="test",
    )

    data = status.to_dict()

    assert data["status"] == STATUS_OK
    assert data["mode"] == "AUTO"
    assert data["selected_source"] == "auto"
    assert data["cmd_vel"]["vx"] == 0.1
    assert data["cmd_vel_mux"]["vx"] == 0.08
    assert data["cmd_vel_safe"]["vx"] == 0.05
    assert data["safety_stop"] is False
    assert data["slowdown_factor"] == 0.50
    assert data["recovery_active"] is True
    assert data["stuck_detected"] is False
    assert data["odom_fresh"] is True
    assert data["command_fresh"] is True
    assert data["note"] == "test"


def test_status_text_without_note():
    status = ControlStatus(
        status=STATUS_OK,
        mode=ControlMode.NAV,
        selected_source="nav",
        safety_stop=False,
        slowdown_factor=0.75,
        recovery_active=False,
        stuck_detected=False,
    )

    assert status.status_text() == (
        "status=OK; mode=NAV; source=nav; safety_stop=false; "
        "slowdown=0.75; recovery=false; stuck=false"
    )


def test_status_text_with_note():
    status = ControlStatus(
        status=STATUS_STOPPED,
        mode=ControlMode.STOP,
        selected_source="none",
        note="startup",
    )

    assert status.status_text() == (
        "status=STOPPED; mode=STOP; source=none; safety_stop=false; "
        "slowdown=1.00; recovery=false; stuck=false; note=startup"
    )


def test_control_status_from_values():
    status = control_status_from_values(
        status="OK",
        mode="AUTO",
        selected_source="auto",
        safety_stop=True,
        slowdown_factor="0.5",
        recovery_active=True,
        stuck_detected=False,
        odom_fresh=True,
        command_fresh=False,
        note="safety",
    )

    assert status.status == "OK"
    assert status.mode == ControlMode.AUTO
    assert status.selected_source == "auto"
    assert status.safety_stop is True
    assert status.slowdown_factor == 0.5
    assert status.recovery_active is True
    assert status.stuck_detected is False
    assert status.odom_fresh is True
    assert status.command_fresh is False
    assert status.note == "safety"
    assert status.healthy() is False
    assert status.blocked() is True


def test_control_status_from_values_defaults_bad_mode_to_stop():
    status = control_status_from_values(
        status="OK",
        mode="bad",
        selected_source="none",
    )

    assert status.mode == ControlMode.STOP
