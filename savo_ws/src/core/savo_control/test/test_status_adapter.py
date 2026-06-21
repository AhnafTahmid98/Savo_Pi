#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for status adapters."""

from __future__ import annotations

import json

from savo_control.adapters import (
    compact_control_status,
    compact_distance_status,
    compact_recovery_status,
    compact_stuck_status,
    control_status_from_dict,
    status_to_json,
    status_to_json_msg,
    status_to_string_msg,
    status_to_text,
)
from savo_control.models import (
    ControlMode,
    ControlStatus,
    DistanceApproachState,
    RecoveryState,
    StuckDetectorState,
    TwistCommand,
)


class StringMsg:
    data = ""


def test_status_to_text_uses_status_text_method():
    status = ControlStatus.ok(mode="AUTO", selected_source="auto")

    text = status_to_text(status)

    assert "status=OK" in text
    assert "mode=AUTO" in text
    assert "source=auto" in text


def test_status_to_text_uses_dict_fallback():
    text = status_to_text(
        {
            "status": "OK",
            "mode": "AUTO",
            "source": "auto",
        }
    )

    assert text == "status=OK; mode=AUTO; source=auto"


def test_status_to_text_uses_string_fallback():
    assert status_to_text("hello") == "hello"


def test_status_to_json_from_model():
    status = ControlStatus.ok(mode="MANUAL", selected_source="manual")

    data = json.loads(status_to_json(status))

    assert data["status"] == "OK"
    assert data["mode"] == "MANUAL"
    assert data["selected_source"] == "manual"


def test_status_to_json_from_dict():
    data = json.loads(
        status_to_json(
            {
                "status": "OK",
                "mode": "AUTO",
            }
        )
    )

    assert data == {
        "mode": "AUTO",
        "status": "OK",
    }


def test_status_to_json_from_plain_value():
    data = json.loads(status_to_json("READY"))

    assert data == {
        "status": "READY",
    }


def test_status_to_string_msg_with_fake_type():
    status = ControlStatus.ok(mode="NAV", selected_source="nav")

    msg = status_to_string_msg(status, msg_type=StringMsg)

    assert isinstance(msg, StringMsg)
    assert "status=OK" in msg.data
    assert "mode=NAV" in msg.data


def test_status_to_json_msg_with_fake_type():
    status = ControlStatus.ok(mode="AUTO", selected_source="auto")

    msg = status_to_json_msg(status, msg_type=StringMsg)
    data = json.loads(msg.data)

    assert isinstance(msg, StringMsg)
    assert data["status"] == "OK"
    assert data["mode"] == "AUTO"
    assert data["selected_source"] == "auto"


def test_control_status_from_dict():
    status = control_status_from_dict(
        {
            "status": "OK",
            "mode": "MANUAL",
            "selected_source": "manual",
            "safety_stop": False,
            "slowdown_factor": 0.75,
            "recovery_active": True,
            "stuck_detected": False,
            "odom_fresh": True,
            "command_fresh": True,
            "note": "test",
        }
    )

    assert status.status == "OK"
    assert status.mode == ControlMode.MANUAL
    assert status.selected_source == "manual"
    assert status.safety_stop is False
    assert status.slowdown_factor == 0.75
    assert status.recovery_active is True
    assert status.stuck_detected is False
    assert status.note == "test"


def test_control_status_from_dict_accepts_source_alias():
    status = control_status_from_dict(
        {
            "status": "OK",
            "mode": "AUTO",
            "source": "auto",
        }
    )

    assert status.mode == ControlMode.AUTO
    assert status.selected_source == "auto"


def test_compact_control_status():
    status = ControlStatus.ok(mode="AUTO", selected_source="auto")

    data = compact_control_status(status)

    assert data == {
        "status": "OK",
        "mode": "AUTO",
        "source": "auto",
        "safety_stop": False,
        "slowdown": 1.0,
        "recovery": False,
        "stuck": False,
        "healthy": True,
        "blocked": False,
    }


def test_compact_control_status_blocked():
    status = ControlStatus(
        status="OK",
        mode=ControlMode.AUTO,
        selected_source="auto",
        safety_stop=True,
    )

    data = compact_control_status(status)

    assert data["healthy"] is False
    assert data["blocked"] is True
    assert data["safety_stop"] is True


def test_compact_distance_status():
    status = DistanceApproachState(
        state="RUNNING",
        distance_m=0.80,
        target_distance_m=0.60,
        error_m=0.20,
        command=TwistCommand(vx=0.10, source="distance_approach"),
        safety_stop=False,
        stale=False,
        valid=True,
    )

    data = compact_distance_status(status)

    assert data == {
        "state": "RUNNING",
        "distance_m": 0.80,
        "target_m": 0.60,
        "error_m": 0.20,
        "vx": 0.10,
        "safety_stop": False,
        "stale": False,
        "valid": True,
    }


def test_compact_recovery_status():
    status = RecoveryState.requested(reason="stuck")

    data = compact_recovery_status(status)

    assert data["phase"] == "REQUESTED"
    assert data["active"] is True
    assert data["request"] is True
    assert data["attempt"] == 0
    assert data["max_attempts"] == 3
    assert data["vx"] == 0.0
    assert data["wz"] == 0.0
    assert data["reason"] == "stuck"
    assert data["elapsed_s"] == 0.0


def test_compact_stuck_status():
    status = StuckDetectorState.stuck(
        cmd_vel_safe=TwistCommand(vx=0.10, source="safe"),
        stuck_duration_s=2.5,
        reason="no_motion",
    )

    data = compact_stuck_status(status)

    assert data == {
        "state": "STUCK",
        "stuck": True,
        "can_request_recovery": True,
        "safety_stop": False,
        "cmd_stale": False,
        "odom_stale": False,
        "stuck_duration_s": 2.5,
        "reason": "no_motion",
    }
