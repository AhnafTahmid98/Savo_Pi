# -*- coding: utf-8 -*-

"""Adapters for status models and ROS-friendly payloads."""

from __future__ import annotations

import json
from typing import Any

from savo_control.models import (
    ControlStatus,
    DistanceApproachState,
    RecoveryState,
    StuckDetectorState,
)


def status_to_json(status: Any, *, indent: int | None = None) -> str:
    if hasattr(status, "to_dict"):
        payload = status.to_dict()
    elif isinstance(status, dict):
        payload = status
    else:
        payload = {"status": str(status)}

    return json.dumps(payload, indent=indent, sort_keys=True)


def status_to_text(status: Any) -> str:
    if hasattr(status, "status_text"):
        return status.status_text()

    if hasattr(status, "to_dict"):
        return _dict_to_text(status.to_dict())

    if isinstance(status, dict):
        return _dict_to_text(status)

    return str(status)


def status_to_string_msg(status: Any, msg_type: Any | None = None) -> Any:
    if msg_type is None:
        try:
            from std_msgs.msg import String as msg_type
        except Exception as exc:
            raise RuntimeError("std_msgs is required to create String messages") from exc

    msg = msg_type()
    msg.data = status_to_text(status)
    return msg


def status_to_json_msg(status: Any, msg_type: Any | None = None) -> Any:
    if msg_type is None:
        try:
            from std_msgs.msg import String as msg_type
        except Exception as exc:
            raise RuntimeError("std_msgs is required to create String messages") from exc

    msg = msg_type()
    msg.data = status_to_json(status)
    return msg


def control_status_from_dict(data: dict[str, Any]) -> ControlStatus:
    from savo_control.models import control_status_from_values

    return control_status_from_values(
        status=str(data.get("status", "UNKNOWN")),
        mode=data.get("mode", "STOP"),
        selected_source=str(data.get("selected_source", data.get("source", "none"))),
        safety_stop=bool(data.get("safety_stop", False)),
        slowdown_factor=float(data.get("slowdown_factor", 1.0)),
        recovery_active=bool(data.get("recovery_active", False)),
        stuck_detected=bool(data.get("stuck_detected", False)),
        odom_fresh=bool(data.get("odom_fresh", True)),
        command_fresh=bool(data.get("command_fresh", True)),
        note=str(data.get("note", "")),
    )


def compact_control_status(status: ControlStatus) -> dict[str, Any]:
    return {
        "status": status.normalized_status(),
        "mode": status.mode.value,
        "source": status.selected_source,
        "safety_stop": status.safety_stop,
        "slowdown": status.slowdown_factor,
        "recovery": status.recovery_active,
        "stuck": status.stuck_detected,
        "healthy": status.healthy(),
        "blocked": status.blocked(),
    }


def compact_distance_status(status: DistanceApproachState) -> dict[str, Any]:
    return {
        "state": status.state,
        "distance_m": status.distance_m,
        "target_m": status.target_distance_m,
        "error_m": status.error_m,
        "vx": status.command.sanitized().vx,
        "safety_stop": status.safety_stop,
        "stale": status.stale,
        "valid": status.valid,
    }


def compact_recovery_status(status: RecoveryState) -> dict[str, Any]:
    return {
        "phase": status.phase.value,
        "active": status.active,
        "request": status.request,
        "attempt": status.attempt,
        "max_attempts": status.max_attempts,
        "vx": status.command.sanitized().vx,
        "wz": status.command.sanitized().wz,
        "reason": status.reason,
        "elapsed_s": status.elapsed_s,
    }


def compact_stuck_status(status: StuckDetectorState) -> dict[str, Any]:
    return {
        "state": status.state.value,
        "stuck": status.stuck_detected(),
        "can_request_recovery": status.can_request_recovery(),
        "safety_stop": status.safety_stop,
        "cmd_stale": status.cmd_stale,
        "odom_stale": status.odom_stale,
        "stuck_duration_s": status.stuck_duration_s,
        "reason": status.reason,
    }


def _dict_to_text(data: dict[str, Any]) -> str:
    return "; ".join(f"{key}={value}" for key, value in data.items())


__all__ = [
    "compact_control_status",
    "compact_distance_status",
    "compact_recovery_status",
    "compact_stuck_status",
    "control_status_from_dict",
    "status_to_json",
    "status_to_json_msg",
    "status_to_string_msg",
    "status_to_text",
]
