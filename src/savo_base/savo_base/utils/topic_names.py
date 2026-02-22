# -*- coding: utf-8 -*-
"""
Robot SAVO — savo_base/utils/topic_names.py
-------------------------------------------
Centralized ROS topic-name constants and helpers for `savo_base`.

Why this file exists
--------------------
- Prevents hard-coded topic strings across multiple nodes
- Keeps naming consistent during bringup/refactors
- Makes launch/config overrides easier to audit

Design goals
------------
- Zero ROS dependency (pure Python utility)
- Safe defaults matching current Robot Savo contracts
- Small helper functions for namespacing/normalization
- Prefer `savo_base.constants` as single source of truth (with fallback)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Optional


# =============================================================================
# Optional bridge to package constants (preferred single source of truth)
# =============================================================================
try:
    from savo_base.constants import (
        TOPIC_CMD_VEL as _TOPIC_CMD_VEL,
        TOPIC_CMD_VEL_SAFE as _TOPIC_CMD_VEL_SAFE,
        TOPIC_SAFETY_STOP as _TOPIC_SAFETY_STOP,
        TOPIC_SAFETY_SLOWDOWN_FACTOR as _TOPIC_SAFETY_SLOWDOWN_FACTOR,
        TOPIC_SAVO_BASE_WATCHDOG_STATE as _TOPIC_SAVO_BASE_WATCHDOG_STATE,
        TOPIC_SAVO_BASE_BASE_STATE as _TOPIC_SAVO_BASE_BASE_STATE,
    )
    _HAS_PACKAGE_CONSTANTS = True
except Exception:
    _HAS_PACKAGE_CONSTANTS = False
    _TOPIC_CMD_VEL = "/cmd_vel"
    _TOPIC_CMD_VEL_SAFE = "/cmd_vel_safe"
    _TOPIC_SAFETY_STOP = "/safety/stop"
    _TOPIC_SAFETY_SLOWDOWN_FACTOR = "/safety/slowdown_factor"
    _TOPIC_SAVO_BASE_WATCHDOG_STATE = "/savo_base/watchdog_state"
    _TOPIC_SAVO_BASE_BASE_STATE = "/savo_base/base_state"


# =============================================================================
# Low-level helpers (pure string ops)
# =============================================================================
def ensure_leading_slash(name: str) -> str:
    """Return topic with a leading '/'."""
    s = str(name or "").strip()
    if not s:
        return "/"
    return s if s.startswith("/") else f"/{s}"


def strip_trailing_slash(name: str) -> str:
    """Remove trailing slash unless the topic is root ('/')."""
    s = ensure_leading_slash(name)
    if s != "/" and s.endswith("/"):
        return s[:-1]
    return s


def normalize_topic_name(name: str) -> str:
    """
    Normalize a topic string:
    - ensure leading slash
    - collapse repeated slashes
    - remove trailing slash (except root)
    """
    s = ensure_leading_slash(name)
    while "//" in s:
        s = s.replace("//", "/")
    return strip_trailing_slash(s)


def join_topic(*parts: str) -> str:
    """
    Join topic path fragments safely.

    Example:
        join_topic("/savo_base", "watchdog_state") -> "/savo_base/watchdog_state"
        join_topic("safety", "stop") -> "/safety/stop"
    """
    cleaned = []
    for p in parts:
        if p is None:
            continue
        text = str(p).strip()
        if not text:
            continue
        cleaned.append(text.strip("/"))
    if not cleaned:
        return "/"
    return normalize_topic_name("/" + "/".join(cleaned))


# =============================================================================
# Canonical topic constants (Robot Savo contracts)
# =============================================================================
# Motion command chain
CMD_VEL = _TOPIC_CMD_VEL
CMD_VEL_SAFE = _TOPIC_CMD_VEL_SAFE

# Safety topics (aligned with savo_perception / safety gate pipeline)
SAFETY_STOP = _TOPIC_SAFETY_STOP
SAFETY_SLOWDOWN_FACTOR = _TOPIC_SAFETY_SLOWDOWN_FACTOR

# Base node lightweight JSON telemetry outputs
SAVO_BASE_WATCHDOG_STATE = _TOPIC_SAVO_BASE_WATCHDOG_STATE
SAVO_BASE_BASE_STATE = _TOPIC_SAVO_BASE_BASE_STATE

# Optional future/diagnostic topics (kept here for consistency)
SAVO_BASE_HEARTBEAT = "/savo_base/heartbeat"
SAVO_BASE_DIAG = "/savo_base/diag"
SAVO_BASE_MOTOR_STATUS = "/savo_base/motor_board_status"

# Optional wheel/base execution diagnostics (future-ready)
SAVO_BASE_WHEEL_COMMAND = "/savo_base/wheel_command"
SAVO_BASE_MOTION_PERMISSION = "/savo_base/motion_permission"

# Common localization/control inputs that `savo_base` may observe or publish around
ODOM_WHEEL = "/wheel/odom"
ODOM_FILTERED = "/odometry/filtered"


# =============================================================================
# Structured topic groups (easy import/use in nodes & tests)
# =============================================================================
@dataclass(frozen=True)
class BaseDriverTopics:
    """Topic contract for `base_driver_node.py` defaults."""
    cmd_vel_safe: str = CMD_VEL_SAFE
    safety_stop: str = SAFETY_STOP
    safety_slowdown_factor: str = SAFETY_SLOWDOWN_FACTOR
    watchdog_state: str = SAVO_BASE_WATCHDOG_STATE
    base_state: str = SAVO_BASE_BASE_STATE


@dataclass(frozen=True)
class BaseStatusTopics:
    """Topic contract for status/diagnostics publishers in `savo_base`."""
    heartbeat: str = SAVO_BASE_HEARTBEAT
    diag: str = SAVO_BASE_DIAG
    motor_status: str = SAVO_BASE_MOTOR_STATUS
    watchdog_state: str = SAVO_BASE_WATCHDOG_STATE
    base_state: str = SAVO_BASE_BASE_STATE


@dataclass(frozen=True)
class SafetyTopics:
    """Shared safety topics used across perception/control/base."""
    stop: str = SAFETY_STOP
    slowdown_factor: str = SAFETY_SLOWDOWN_FACTOR
    cmd_vel_in: str = CMD_VEL
    cmd_vel_out: str = CMD_VEL_SAFE


# Frozen default bundles
BASE_DRIVER_TOPICS = BaseDriverTopics()
BASE_STATUS_TOPICS = BaseStatusTopics()
SAFETY_TOPICS = SafetyTopics()


# =============================================================================
# Namespace utilities (for simulations, multi-robot, testing)
# =============================================================================
def topic_in_namespace(topic: str, namespace: Optional[str]) -> str:
    """
    Apply a ROS namespace to an absolute topic.

    Example:
        topic_in_namespace("/cmd_vel_safe", "/robot_savo")
        -> "/robot_savo/cmd_vel_safe"

    Rules:
    - If namespace is empty/None -> normalized original topic
    - Topic is treated as absolute contract name and re-rooted under namespace
    """
    t = normalize_topic_name(topic)
    if not namespace:
        return t

    ns = normalize_topic_name(namespace)
    if ns == "/":
        return t

    # Re-root absolute topic under namespace by removing leading '/'
    return join_topic(ns, t.lstrip("/"))


def remap_topic_dict(
    topic_map: Mapping[str, str],
    namespace: Optional[str] = None,
    overrides: Optional[Mapping[str, str]] = None,
) -> Dict[str, str]:
    """
    Build a resolved topic dict from a logical-name->topic map.

    Parameters
    ----------
    topic_map:
        Dict like {"cmd": "/cmd_vel_safe", "stop": "/safety/stop"}
    namespace:
        Optional namespace to apply to all topics
    overrides:
        Optional logical-name->topic overrides (already-final or custom)

    Returns
    -------
    Dict[str, str]
        Resolved, normalized topics.
    """
    ov = dict(overrides or {})
    out: Dict[str, str] = {}
    for key, topic in topic_map.items():
        if key in ov:
            out[key] = normalize_topic_name(ov[key])
        else:
            out[key] = topic_in_namespace(topic, namespace)
    return out


def prefixed_topic_bundle(prefix: str) -> Dict[str, str]:
    """
    Return a compact topic bundle under a custom prefix.

    Example:
        prefixed_topic_bundle("/savo_base")
        -> {
             "heartbeat": "/savo_base/heartbeat",
             "diag": "/savo_base/diag",
             ...
           }
    """
    p = normalize_topic_name(prefix)
    return {
        "heartbeat": join_topic(p, "heartbeat"),
        "diag": join_topic(p, "diag"),
        "watchdog_state": join_topic(p, "watchdog_state"),
        "base_state": join_topic(p, "base_state"),
        "motor_board_status": join_topic(p, "motor_board_status"),
        "wheel_command": join_topic(p, "wheel_command"),
        "motion_permission": join_topic(p, "motion_permission"),
    }


# =============================================================================
# Convenience exports for node defaults / parameter declarations
# =============================================================================
DEFAULT_BASE_DRIVER_TOPIC_MAP: Dict[str, str] = {
    "cmd_topic": CMD_VEL_SAFE,
    "safety_stop_topic": SAFETY_STOP,
    "slowdown_topic": SAFETY_SLOWDOWN_FACTOR,
    "watchdog_state_topic": SAVO_BASE_WATCHDOG_STATE,
    "base_state_topic": SAVO_BASE_BASE_STATE,
}

DEFAULT_SAFETY_GATE_TOPIC_MAP: Dict[str, str] = {
    "cmd_in": CMD_VEL,
    "cmd_out": CMD_VEL_SAFE,
    "stop": SAFETY_STOP,
    "slowdown_factor": SAFETY_SLOWDOWN_FACTOR,
}


# =============================================================================
# Validation helpers (useful in tests / config sanity checks)
# =============================================================================
def validate_topic_names(topics: Iterable[str]) -> bool:
    """
    Basic topic-name sanity check.

    This is intentionally lightweight and not a full ROS name validator.
    Returns True if all names:
    - are non-empty after normalization
    - start with '/'
    - do not contain spaces
    """
    for t in topics:
        n = normalize_topic_name(t)
        if not n or not n.startswith("/"):
            return False
        if " " in n:
            return False
    return True


__all__ = [
    # helpers
    "ensure_leading_slash",
    "strip_trailing_slash",
    "normalize_topic_name",
    "join_topic",
    "topic_in_namespace",
    "remap_topic_dict",
    "prefixed_topic_bundle",
    "validate_topic_names",
    # constants
    "CMD_VEL",
    "CMD_VEL_SAFE",
    "SAFETY_STOP",
    "SAFETY_SLOWDOWN_FACTOR",
    "SAVO_BASE_WATCHDOG_STATE",
    "SAVO_BASE_BASE_STATE",
    "SAVO_BASE_HEARTBEAT",
    "SAVO_BASE_DIAG",
    "SAVO_BASE_MOTOR_STATUS",
    "SAVO_BASE_WHEEL_COMMAND",
    "SAVO_BASE_MOTION_PERMISSION",
    "ODOM_WHEEL",
    "ODOM_FILTERED",
    # dataclasses / bundles
    "BaseDriverTopics",
    "BaseStatusTopics",
    "SafetyTopics",
    "BASE_DRIVER_TOPICS",
    "BASE_STATUS_TOPICS",
    "SAFETY_TOPICS",
    "DEFAULT_BASE_DRIVER_TOPIC_MAP",
    "DEFAULT_SAFETY_GATE_TOPIC_MAP",
]