#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/ros/adapters.py
--------------------------------------
Professional ROS2 Jazzy adapter helpers for `savo_base`.

Purpose
-------
This module provides small, reusable conversion helpers ("adapters") between:
- internal `savo_base.models.*` dataclasses
- ROS messages (std_msgs / geometry_msgs)
- JSON string payloads used by lightweight state topics

Why this module exists
----------------------
Keeps node code clean and consistent:
- `base_driver_node.py`
- `base_state_publisher_node.py`
- `base_watchdog_node.py`
- `base_heartbeat_node.py`
- `base_diag_runner_node.py`

Design principles
-----------------
- No hardware access here
- No ROS node initialization here
- Safe defaults + tolerant parsing
- JSON outputs are compact and dashboard-friendly
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, is_dataclass
from typing import Any, Dict, Optional, Tuple

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

# Internal models (created earlier in your savo_base package)
from savo_base.models.wheel_command import WheelCommand
from savo_base.models.base_state import BaseState
from savo_base.models.watchdog_state import WatchdogState
from savo_base.models.motor_board_status import MotorBoardStatus


# =============================================================================
# Generic helpers
# =============================================================================
def monotonic_s() -> float:
    """Return monotonic time in seconds."""
    return float(time.monotonic())


def wall_time_s() -> float:
    """Return wall-clock unix time in seconds."""
    return float(time.time())


def safe_json_loads(text: str) -> Optional[Dict[str, Any]]:
    """
    Parse JSON string into dict. Returns None on error or non-dict root.
    """
    try:
        obj = json.loads(text)
        return obj if isinstance(obj, dict) else None
    except Exception:
        return None


def compact_json(data: Dict[str, Any]) -> str:
    """
    Compact JSON string for ROS String topics.
    """
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"), default=str)


def pretty_json(data: Dict[str, Any]) -> str:
    """
    Pretty JSON string (useful for debugging/logging).
    """
    return json.dumps(data, ensure_ascii=False, indent=2, default=str)


def dataclass_to_dict(obj: Any) -> Dict[str, Any]:
    """
    Convert dataclass instance to dict safely.
    """
    if is_dataclass(obj):
        return asdict(obj)
    raise TypeError(f"Expected dataclass instance, got {type(obj).__name__}")


# =============================================================================
# Twist <-> internal command adapters
# =============================================================================
def twist_to_planar_components(msg: Twist) -> Tuple[float, float, float]:
    """
    Extract planar robot motion components from geometry_msgs/Twist.

    Returns
    -------
    (vx, vy, wz)
      vx = linear.x
      vy = linear.y
      wz = angular.z
    """
    return float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)


def planar_components_to_twist(vx: float, vy: float, wz: float) -> Twist:
    """
    Build geometry_msgs/Twist from planar components.
    """
    msg = Twist()
    msg.linear.x = float(vx)
    msg.linear.y = float(vy)
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = float(wz)
    return msg


def twist_to_wheel_command(
    msg: Twist,
    *,
    stamp_mono_s: Optional[float] = None,
    source: str = "ros:/cmd_vel_safe",
) -> WheelCommand:
    """
    Convert Twist to internal WheelCommand (command frame only).

    Notes
    -----
    This helper stores the *robot command intent* (vx, vy, wz).
    Actual wheel mixing (mecanum kinematics) is handled elsewhere
    (e.g., `savo_base.kinematics.mecanum`).
    """
    vx, vy, wz = twist_to_planar_components(msg)
    ts = monotonic_s() if stamp_mono_s is None else float(stamp_mono_s)

    # Supports the dataclass structure we created earlier for WheelCommand
    # (vx, vy, wz + metadata fields).
    return WheelCommand(
        vx=vx,
        vy=vy,
        wz=wz,
        stamp_mono_s=ts,
        source=source,
    )


def wheel_command_to_twist(cmd: WheelCommand) -> Twist:
    """
    Convert internal WheelCommand back to geometry_msgs/Twist.
    """
    return planar_components_to_twist(cmd.vx, cmd.vy, cmd.wz)


# =============================================================================
# std_msgs adapters
# =============================================================================
def bool_to_msg(value: bool) -> Bool:
    msg = Bool()
    msg.data = bool(value)
    return msg


def string_to_msg(text: str) -> String:
    msg = String()
    msg.data = str(text)
    return msg


def json_to_string_msg(data: Dict[str, Any]) -> String:
    msg = String()
    msg.data = compact_json(data)
    return msg


# =============================================================================
# Internal model -> JSON payload adapters (for state topics)
# =============================================================================
def wheel_command_to_dict(cmd: WheelCommand) -> Dict[str, Any]:
    """
    Dataclass -> dict adapter for WheelCommand with a stable JSON schema.
    """
    d = dataclass_to_dict(cmd)
    d.setdefault("model", "WheelCommand")
    return d


def motor_board_status_to_dict(status: MotorBoardStatus) -> Dict[str, Any]:
    """
    Dataclass -> dict adapter for MotorBoardStatus with a stable JSON schema.
    """
    d = dataclass_to_dict(status)
    d.setdefault("model", "MotorBoardStatus")
    return d


def watchdog_state_to_dict(state: WatchdogState) -> Dict[str, Any]:
    """
    Dataclass -> dict adapter for WatchdogState with a stable JSON schema.
    """
    d = dataclass_to_dict(state)
    d.setdefault("model", "WatchdogState")
    return d


def base_state_to_dict(state: BaseState) -> Dict[str, Any]:
    """
    Dataclass -> dict adapter for BaseState with a stable JSON schema.
    """
    d = dataclass_to_dict(state)
    d.setdefault("model", "BaseState")
    return d


def base_state_to_json(state: BaseState) -> str:
    return compact_json(base_state_to_dict(state))


def watchdog_state_to_json(state: WatchdogState) -> str:
    return compact_json(watchdog_state_to_dict(state))


def motor_board_status_to_json(status: MotorBoardStatus) -> str:
    return compact_json(motor_board_status_to_dict(status))


def wheel_command_to_json(cmd: WheelCommand) -> str:
    return compact_json(wheel_command_to_dict(cmd))


# =============================================================================
# JSON -> internal model adapters (tolerant parsing for ROS String topics)
# =============================================================================
def json_str_to_base_state(text: str) -> Optional[BaseState]:
    """
    Parse a JSON string into BaseState (best-effort/tolerant).

    Returns None if parsing fails or required fields are missing.

    Note:
    This expects your earlier `BaseState` dataclass fields. If your exact
    dataclass evolves, update mapping below in one place (this file).
    """
    d = safe_json_loads(text)
    if d is None:
        return None

    try:
        return BaseState(
            mode=str(d.get("mode", "UNKNOWN")),
            enabled=bool(d.get("enabled", False)),
            estop_active=bool(d.get("estop_active", False)),
            watchdog_trip=bool(d.get("watchdog_trip", False)),
            last_cmd_age_s=float(d.get("last_cmd_age_s", 0.0)),
            cmd_source=str(d.get("cmd_source", "")),
            vx_cmd=float(d.get("vx_cmd", 0.0)),
            vy_cmd=float(d.get("vy_cmd", 0.0)),
            wz_cmd=float(d.get("wz_cmd", 0.0)),
            stamp_mono_s=float(d.get("stamp_mono_s", 0.0)),
        )
    except Exception:
        return None


def json_str_to_watchdog_state(text: str) -> Optional[WatchdogState]:
    d = safe_json_loads(text)
    if d is None:
        return None

    try:
        return WatchdogState(
            tripped=bool(d.get("tripped", False)),
            reason=str(d.get("reason", "")),
            timeout_s=float(d.get("timeout_s", 0.0)),
            last_cmd_age_s=float(d.get("last_cmd_age_s", 0.0)),
            last_cmd_seen=bool(d.get("last_cmd_seen", False)),
            stamp_mono_s=float(d.get("stamp_mono_s", 0.0)),
        )
    except Exception:
        return None


def json_str_to_motor_board_status(text: str) -> Optional[MotorBoardStatus]:
    d = safe_json_loads(text)
    if d is None:
        return None

    try:
        return MotorBoardStatus(
            connected=bool(d.get("connected", False)),
            dry_run=bool(d.get("dry_run", False)),
            driver_name=str(d.get("driver_name", "")),
            i2c_bus=int(d.get("i2c_bus", 1)),
            i2c_addr=int(d.get("i2c_addr", 0x40)),
            last_error=str(d.get("last_error", "")),
            stamp_mono_s=float(d.get("stamp_mono_s", 0.0)),
        )
    except Exception:
        return None


# =============================================================================
# Diagnostics payload helpers (common format for node event/state topics)
# =============================================================================
def make_event_payload(
    *,
    node: str,
    event_type: str,
    message: str,
    details: Optional[Dict[str, Any]] = None,
    robot_name: str = "Robot Savo",
) -> Dict[str, Any]:
    """
    Standard event payload format for ROS String(JSON) topics.
    """
    return {
        "node": node,
        "robot_name": robot_name,
        "event_type": event_type,
        "message": message,
        "time_unix_s": wall_time_s(),
        "time_mono_s": monotonic_s(),
        "details": details or {},
    }


def make_state_payload(
    *,
    node: str,
    status_level: str,
    fields: Optional[Dict[str, Any]] = None,
    robot_name: str = "Robot Savo",
) -> Dict[str, Any]:
    """
    Standard state payload format for ROS String(JSON) topics.
    """
    return {
        "node": node,
        "robot_name": robot_name,
        "status_level": str(status_level),
        "time_unix_s": wall_time_s(),
        "time_mono_s": monotonic_s(),
        "fields": fields or {},
    }


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    # generic
    "monotonic_s",
    "wall_time_s",
    "safe_json_loads",
    "compact_json",
    "pretty_json",
    "dataclass_to_dict",
    # twist / command
    "twist_to_planar_components",
    "planar_components_to_twist",
    "twist_to_wheel_command",
    "wheel_command_to_twist",
    # std_msgs
    "bool_to_msg",
    "string_to_msg",
    "json_to_string_msg",
    # model adapters
    "wheel_command_to_dict",
    "motor_board_status_to_dict",
    "watchdog_state_to_dict",
    "base_state_to_dict",
    "wheel_command_to_json",
    "motor_board_status_to_json",
    "watchdog_state_to_json",
    "base_state_to_json",
    "json_str_to_base_state",
    "json_str_to_watchdog_state",
    "json_str_to_motor_board_status",
    # common payloads
    "make_event_payload",
    "make_state_payload",
]