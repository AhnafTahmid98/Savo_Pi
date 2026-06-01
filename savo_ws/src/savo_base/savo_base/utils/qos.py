#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/utils/qos.py
-----------------------------------
Professional ROS2 Jazzy QoS helpers for `savo_base`.

Purpose
-------
Provide a single place for QoS profile creation so all `savo_base` nodes use
consistent QoS settings for:
- command topics (/cmd_vel_safe)
- safety topics (/safety/stop, /safety/slowdown_factor)
- state/diagnostic topics (/savo_base/base_state, /savo_base/watchdog_state)
- optional latched status/config topics

Design goals
------------
- ROS2 Jazzy compatible
- explicit and readable
- easy reuse across Python nodes
- safe fallbacks if imported outside ROS2 (unit tests / lint environments)

Notes for Robot Savo
--------------------
- Command topics are typically RELIABLE + KEEP_LAST.
- Safety sensor-like topics are often BEST_EFFORT + KEEP_LAST for compatibility.
- JSON state summaries can be RELIABLE + KEEP_LAST.
- "Latched" behavior in ROS2 = TRANSIENT_LOCAL durability.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any, Dict, Optional


# =============================================================================
# Optional ROS imports (keep module importable even without ROS env)
# =============================================================================

try:
    from rclpy.qos import (
        QoSProfile,
        ReliabilityPolicy,
        HistoryPolicy,
        DurabilityPolicy,
        LivelinessPolicy,
    )
    _HAS_RCLPY_QOS = True
except Exception:
    QoSProfile = Any  # type: ignore
    ReliabilityPolicy = Any  # type: ignore
    HistoryPolicy = Any  # type: ignore
    DurabilityPolicy = Any  # type: ignore
    LivelinessPolicy = Any  # type: ignore
    _HAS_RCLPY_QOS = False


# =============================================================================
# Fallback lightweight profile (for tests / static imports)
# =============================================================================

@dataclass
class QoSProfileFallback:
    """
    Minimal fallback structure used when rclpy is unavailable.

    This is NOT used for real ROS runtime publishing/subscribing, but it helps
    unit tests and tooling import the module without failing.
    """
    depth: int = 10
    reliability: str = "reliable"
    history: str = "keep_last"
    durability: str = "volatile"
    liveliness: str = "automatic"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


# =============================================================================
# Internal helpers
# =============================================================================

def _clamp_depth(depth: int) -> int:
    d = int(depth)
    if d < 1:
        return 1
    if d > 10000:
        return 10000
    return d


def _make_profile(
    *,
    depth: int,
    reliability: str,
    history: str = "keep_last",
    durability: str = "volatile",
    liveliness: str = "automatic",
):
    """
    Create a ROS2 QoSProfile if rclpy is available; otherwise return fallback.
    """
    depth = _clamp_depth(depth)

    if not _HAS_RCLPY_QOS:
        return QoSProfileFallback(
            depth=depth,
            reliability=reliability,
            history=history,
            durability=durability,
            liveliness=liveliness,
        )

    # Map strings -> enums (explicit for readability)
    rel_map = {
        "reliable": ReliabilityPolicy.RELIABLE,
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
    }
    hist_map = {
        "keep_last": HistoryPolicy.KEEP_LAST,
        "keep_all": HistoryPolicy.KEEP_ALL,
    }
    dur_map = {
        "volatile": DurabilityPolicy.VOLATILE,
        "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
    }
    live_map = {
        "automatic": LivelinessPolicy.AUTOMATIC,
        "manual_by_topic": LivelinessPolicy.MANUAL_BY_TOPIC,
    }

    return QoSProfile(
        depth=depth,
        reliability=rel_map[reliability],
        history=hist_map[history],
        durability=dur_map[durability],
        liveliness=live_map[liveliness],
    )


# =============================================================================
# Standard Savo Base QoS factories
# =============================================================================

def qos_cmd(depth: int = 10):
    """
    QoS for command topics (e.g., /cmd_vel_safe).

    Why RELIABLE:
    - Command loss can cause inconsistent motion behavior.
    - Works well with base execution / control chains.

    Recommended use:
    - Twist subscriptions in base_driver_node
    - command publishers in teleop/control nodes
    """
    return _make_profile(
        depth=depth,
        reliability="reliable",
        history="keep_last",
        durability="volatile",
        liveliness="automatic",
    )


def qos_safety_sensor(depth: int = 10):
    """
    QoS for safety/sensor-like topics (e.g., /safety/stop, /safety/slowdown_factor).

    Why BEST_EFFORT:
    - Common compatibility choice for high-rate sensor-ish streams
    - Matches typical ROS2 perception pipelines
    - Your perception stack has already required QoS compatibility tuning

    Recommended use:
    - safety stop subscription (Bool)
    - slowdown factor subscription (Float32)
    """
    return _make_profile(
        depth=depth,
        reliability="best_effort",
        history="keep_last",
        durability="volatile",
        liveliness="automatic",
    )


def qos_state(depth: int = 10):
    """
    QoS for state/diagnostic summary topics (JSON strings, low-rate status).

    Why RELIABLE:
    - Dashboards and debugging tools benefit from dependable delivery
    - These are low-rate topics, so reliable overhead is negligible
    """
    return _make_profile(
        depth=depth,
        reliability="reliable",
        history="keep_last",
        durability="volatile",
        liveliness="automatic",
    )


def qos_latched_state(depth: int = 1):
    """
    QoS for latched status/config topics (TRANSIENT_LOCAL).

    New subscribers receive the most recent message.
    Useful for:
    - static config summaries
    - one-shot board info / capability announcements
    - version/status snapshot topics
    """
    return _make_profile(
        depth=depth,
        reliability="reliable",
        history="keep_last",
        durability="transient_local",
        liveliness="automatic",
    )


def qos_best_effort_debug(depth: int = 10):
    """
    QoS for debug/telemetry streams where dropping occasional messages is okay.
    """
    return _make_profile(
        depth=depth,
        reliability="best_effort",
        history="keep_last",
        durability="volatile",
        liveliness="automatic",
    )


# =============================================================================
# Named topic convenience helpers (optional, but useful for consistency)
# =============================================================================

def qos_for_topic(topic_name: str, *, default_depth: int = 10):
    """
    Return a recommended QoS profile based on common Robot Savo topic names.

    This is a convenience helper; explicit QoS functions are still preferred
    when you want clarity in node code.
    """
    t = str(topic_name).strip()

    # Base command path
    if t in ("/cmd_vel", "/cmd_vel_safe"):
        return qos_cmd(default_depth)

    # Safety topics from savo_perception
    if t in ("/safety/stop", "/safety/slowdown_factor", "/safety/slowdown_vx", "/safety/slowdown_vy"):
        return qos_safety_sensor(default_depth)

    # Common state/diag topics in savo_base
    if t.startswith("/savo_base/"):
        return qos_state(default_depth)

    # Generic fallback (safe, reliable)
    return qos_state(default_depth)


# =============================================================================
# Introspection / diagnostics helpers
# =============================================================================

def qos_profile_to_dict(profile: Any) -> Dict[str, Any]:
    """
    Convert a QoS profile (real or fallback) into a simple dict for logs/tests.
    """
    if isinstance(profile, QoSProfileFallback):
        return profile.to_dict()

    # Best-effort for real ROS QoSProfile (enum values may vary by ROS version)
    out: Dict[str, Any] = {}
    try:
        out["depth"] = int(getattr(profile, "depth", 0))
    except Exception:
        out["depth"] = None

    for field in ("reliability", "history", "durability", "liveliness"):
        try:
            val = getattr(profile, field, None)
            # Enum-ish objects often have .name
            name = getattr(val, "name", None)
            out[field] = str(name if name is not None else val)
        except Exception:
            out[field] = None
    return out


def make_qos_catalog() -> Dict[str, Dict[str, Any]]:
    """
    Return a ready-to-log catalog of standard `savo_base` QoS presets.
    Useful for startup diagnostics.
    """
    return {
        "cmd": qos_profile_to_dict(qos_cmd()),
        "safety_sensor": qos_profile_to_dict(qos_safety_sensor()),
        "state": qos_profile_to_dict(qos_state()),
        "latched_state": qos_profile_to_dict(qos_latched_state()),
        "debug_best_effort": qos_profile_to_dict(qos_best_effort_debug()),
    }


# =============================================================================
# Public exports
# =============================================================================

__all__ = [
    "QoSProfileFallback",
    "qos_cmd",
    "qos_safety_sensor",
    "qos_state",
    "qos_latched_state",
    "qos_best_effort_debug",
    "qos_for_topic",
    "qos_profile_to_dict",
    "make_qos_catalog",
]