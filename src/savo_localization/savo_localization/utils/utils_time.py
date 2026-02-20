#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_localization/utils/utils_time.py
--------------------------------------------------
Professional time utilities for ROS2 localization nodes.

Design goals:
- Keep time handling consistent across imu_node / wheel_odom_node / fallback nodes
- Provide robust dt computation (handles clock jumps, sim time, startup)
- Provide "stale" checks for sensor streams
- Keep this module small and dependable

Notes:
- Supports both ROS time (node clock + message stamps) and non-ROS monotonic time.
- ROS message imports are lazy to keep this module importable in non-ROS unit tests.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional, Tuple


# ---------------------------
# Generic (non-ROS) helpers
# ---------------------------

def monotonic_now_s() -> float:
    """Monotonic clock in seconds (never goes backwards)."""
    return time.monotonic()


def wall_now_s() -> float:
    """Wall clock time in seconds since epoch."""
    return time.time()


def clamp_dt(
    dt: float,
    *,
    min_dt: float = 1e-4,
    max_dt: float = 0.5,
    fallback_dt: float = 0.02,
) -> float:
    """
    Clamp dt to a safe range.

    - If dt is invalid (<=0 or NaN), returns fallback_dt.
    - If dt is too small, returns min_dt.
    - If dt is too large, returns max_dt (prevents integration blowups).

    Defaults are conservative for 20–100 Hz sensor loops.
    """
    try:
        if not (dt > 0.0):
            return fallback_dt
        if dt < min_dt:
            return min_dt
        if dt > max_dt:
            return max_dt
        return dt
    except Exception:
        return fallback_dt


def is_stale(age_s: float, stale_timeout_s: float) -> bool:
    """True if age exceeds stale timeout (or timeout <= 0 treated as never stale)."""
    if stale_timeout_s is None or stale_timeout_s <= 0.0:
        return False
    return age_s > stale_timeout_s


# ---------------------------
# ROS time conversions (lazy imports)
# ---------------------------

def ros_time_to_float_s(t: Any) -> float:
    """
    Convert a ROS Time-like object to float seconds.

    Supports:
    - rclpy.time.Time (has .nanoseconds)
    - builtin_interfaces.msg.Time (sec, nanosec)
    - Anything with attributes {sec,nanosec} or {nanoseconds}
    """
    if t is None:
        raise ValueError("ros_time_to_float_s got None")

    # rclpy.time.Time
    ns = getattr(t, "nanoseconds", None)
    if isinstance(ns, int):
        return ns * 1e-9

    # builtin_interfaces.msg.Time
    sec = getattr(t, "sec", None)
    nsec = getattr(t, "nanosec", None)
    if isinstance(sec, int) and isinstance(nsec, int):
        return float(sec) + float(nsec) * 1e-9

    raise TypeError(f"Unsupported time object: {type(t).__name__}")


def float_s_to_ros_time_msg(stamp_s: float) -> Any:
    """
    Convert float seconds to builtin_interfaces.msg.Time.

    Returns:
        builtin_interfaces.msg.Time
    """
    try:
        from builtin_interfaces.msg import Time as RosTimeMsg  # type: ignore
    except Exception as e:
        raise RuntimeError("builtin_interfaces.msg.Time is not available") from e

    if stamp_s < 0:
        stamp_s = 0.0

    sec = int(stamp_s)
    nanosec = int(round((stamp_s - sec) * 1e9))
    # Normalize
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    if nanosec < 0:
        nanosec = 0

    msg = RosTimeMsg()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


def node_now_float_s(node: Any) -> float:
    """
    Get current ROS time (node clock) as float seconds.

    Works with sim time if enabled on the node.
    """
    now = node.get_clock().now()
    return ros_time_to_float_s(now)


# ---------------------------
# dt trackers
# ---------------------------

@dataclass
class DtTracker:
    """
    Track dt between successive events using a monotonic clock or ROS time.

    Typical usage (monotonic):
        dt = tracker.step()

    Typical usage (ROS time):
        now_s = node_now_float_s(self)
        dt = tracker.step(now_s)

    If you pass explicit_now_s, the tracker uses that; otherwise it uses time.monotonic().
    """
    last_s: Optional[float] = None

    def reset(self) -> None:
        self.last_s = None

    def step(
        self,
        explicit_now_s: Optional[float] = None,
        *,
        min_dt: float = 1e-4,
        max_dt: float = 0.5,
        fallback_dt: float = 0.02,
    ) -> float:
        now_s = explicit_now_s if explicit_now_s is not None else monotonic_now_s()

        if self.last_s is None:
            self.last_s = now_s
            return fallback_dt

        raw_dt = now_s - self.last_s
        self.last_s = now_s
        return clamp_dt(raw_dt, min_dt=min_dt, max_dt=max_dt, fallback_dt=fallback_dt)


@dataclass
class StaleTracker:
    """
    Track the last update time for a stream and compute age/stale status.

    Use case:
        st = StaleTracker()
        st.mark(now_s)
        age = st.age(now_s)
        if st.is_stale(now_s, stale_timeout_s): ...
    """
    last_update_s: Optional[float] = None

    def reset(self) -> None:
        self.last_update_s = None

    def mark(self, now_s: float) -> None:
        self.last_update_s = now_s

    def age(self, now_s: float) -> float:
        if self.last_update_s is None:
            return float("inf")
        return max(0.0, now_s - self.last_update_s)

    def is_stale(self, now_s: float, stale_timeout_s: float) -> bool:
        return is_stale(self.age(now_s), stale_timeout_s)


# ---------------------------
# Sensor timestamp helpers
# ---------------------------

def msg_stamp_to_float_s(msg: Any, *, fallback_now_s: Optional[float] = None) -> float:
    """
    Extract msg.header.stamp -> float seconds.

    If stamp is zero/unset and fallback_now_s is provided, returns fallback_now_s.
    """
    header = getattr(msg, "header", None)
    if header is None:
        raise AttributeError("Message has no 'header' attribute")

    stamp = getattr(header, "stamp", None)
    if stamp is None:
        raise AttributeError("Message header has no 'stamp' attribute")

    # Detect "zero" stamp (common when someone forgets to stamp)
    sec = getattr(stamp, "sec", 0)
    nsec = getattr(stamp, "nanosec", 0)
    if isinstance(sec, int) and isinstance(nsec, int) and sec == 0 and nsec == 0:
        if fallback_now_s is not None:
            return fallback_now_s

    return ros_time_to_float_s(stamp)


def age_from_msg_stamp_s(
    now_s: float,
    msg: Any,
    *,
    clamp_nonnegative: bool = True,
    fallback_to_inf_if_missing: bool = True,
) -> float:
    """
    Compute age (seconds) of a stamped message relative to now_s.

    If the message has a missing stamp:
      - returns +inf if fallback_to_inf_if_missing else raises.
    """
    try:
        stamp_s = msg_stamp_to_float_s(msg)
    except Exception:
        if fallback_to_inf_if_missing:
            return float("inf")
        raise

    age = now_s - stamp_s
    if clamp_nonnegative and age < 0.0:
        age = 0.0
    return age