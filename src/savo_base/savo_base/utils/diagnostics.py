#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/utils/diagnostics.py
-------------------------------------------
Lightweight diagnostics helpers for `savo_base` (ROS2 Jazzy friendly).

Purpose
-------
Provide small, reusable utilities for:
- status level normalization (OK/WARN/ERROR)
- safe exception formatting
- monotonic timing helpers
- counters / rate estimation
- JSON-safe diagnostic payload building

Design goals
------------
- Dependency-free (standard library only)
- Safe in real robot runtime loops
- Easy to use from nodes, drivers, and safety policies
- Human-readable outputs for dashboards/logs
"""

from __future__ import annotations

import json
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


# =============================================================================
# Time helpers
# =============================================================================

def now_mono() -> float:
    """
    Return monotonic time in seconds.

    Use for watchdogs, loop timing, and age calculations.
    """
    return float(time.monotonic())


def now_wall() -> float:
    """
    Return wall-clock UNIX timestamp in seconds.
    """
    return float(time.time())


def age_s(ts_mono: float, now_mono_s: Optional[float] = None) -> float:
    """
    Compute age in seconds from a monotonic timestamp.
    Returns +inf if timestamp is <= 0.
    """
    if ts_mono is None or float(ts_mono) <= 0.0:
        return float("inf")
    n = now_mono() if now_mono_s is None else float(now_mono_s)
    return max(0.0, n - float(ts_mono))


# =============================================================================
# Status level helpers
# =============================================================================

STATUS_OK = "OK"
STATUS_WARN = "WARN"
STATUS_ERROR = "ERROR"
STATUS_STALE = "STALE"
STATUS_SAFETY_STOP = "SAFETY_STOP"
STATUS_UNKNOWN = "UNKNOWN"


def normalize_status_level(level: Any) -> str:
    """
    Normalize a status level string to a known uppercase value.
    Unknown values become 'UNKNOWN'.
    """
    if level is None:
        return STATUS_UNKNOWN
    s = str(level).strip().upper()
    known = {
        STATUS_OK,
        STATUS_WARN,
        STATUS_ERROR,
        STATUS_STALE,
        STATUS_SAFETY_STOP,
        STATUS_UNKNOWN,
    }
    return s if s in known else STATUS_UNKNOWN


def worst_status(*levels: Any) -> str:
    """
    Return the most severe status among inputs.

    Severity order (low -> high):
      OK < WARN < STALE < SAFETY_STOP < ERROR < UNKNOWN
    """
    rank = {
        STATUS_OK: 0,
        STATUS_WARN: 1,
        STATUS_STALE: 2,
        STATUS_SAFETY_STOP: 3,
        STATUS_ERROR: 4,
        STATUS_UNKNOWN: 5,
    }
    if not levels:
        return STATUS_UNKNOWN
    normed = [normalize_status_level(x) for x in levels]
    return max(normed, key=lambda s: rank.get(s, 999))


# =============================================================================
# Exception formatting helpers
# =============================================================================

def exception_message(exc: BaseException) -> str:
    """
    Compact exception string for logs and diagnostics payloads.
    """
    return f"{exc.__class__.__name__}: {exc}"


def exception_trace(exc: BaseException, *, limit: Optional[int] = None) -> str:
    """
    Full traceback as string (optionally limited).
    Use sparingly in high-rate loops.
    """
    tb = traceback.format_exception(type(exc), exc, exc.__traceback__, limit=limit)
    return "".join(tb).strip()


# =============================================================================
# JSON-safe conversion helpers
# =============================================================================

def json_safe(value: Any) -> Any:
    """
    Convert common runtime values into JSON-safe structures.

    Rules:
    - primitive JSON types pass through
    - dict/list/tuple/set are recursively converted
    - dataclasses with __dict__ are converted to dict
    - unknown objects become str(value)
    """
    if value is None or isinstance(value, (bool, int, float, str)):
        return value

    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}

    if isinstance(value, (list, tuple, set)):
        return [json_safe(v) for v in value]

    # dataclass or general object
    if hasattr(value, "__dict__"):
        try:
            return {str(k): json_safe(v) for k, v in vars(value).items()}
        except Exception:
            return str(value)

    return str(value)


def dumps_compact(payload: Dict[str, Any], *, ensure_ascii: bool = False) -> str:
    """
    Compact JSON dump for ROS String diagnostics topics.
    """
    return json.dumps(json_safe(payload), ensure_ascii=ensure_ascii, separators=(",", ":"))


def dumps_pretty(payload: Dict[str, Any], *, ensure_ascii: bool = False) -> str:
    """
    Pretty JSON dump (useful for file logging or debugging).
    """
    return json.dumps(json_safe(payload), ensure_ascii=ensure_ascii, indent=2)


# =============================================================================
# Rate / loop diagnostics
# =============================================================================

@dataclass
class RateTracker:
    """
    Track event timing and estimate loop/event rate.

    Example
    -------
    rt = RateTracker("exec_loop")
    ...
    rt.tick()
    hz = rt.hz_estimate
    """

    name: str = "rate_tracker"
    alpha: float = 0.2  # EMA smoothing factor for dt
    count: int = 0
    first_t_mono: float = 0.0
    last_t_mono: float = 0.0
    ema_dt_s: float = 0.0
    min_dt_s: float = float("inf")
    max_dt_s: float = 0.0

    def reset(self) -> None:
        self.count = 0
        self.first_t_mono = 0.0
        self.last_t_mono = 0.0
        self.ema_dt_s = 0.0
        self.min_dt_s = float("inf")
        self.max_dt_s = 0.0

    def tick(self, t_mono: Optional[float] = None) -> float:
        """
        Record one event tick.
        Returns the measured dt (0 for first tick).
        """
        t = now_mono() if t_mono is None else float(t_mono)
        if self.count == 0:
            self.first_t_mono = t
            self.last_t_mono = t
            self.count = 1
            return 0.0

        dt = max(0.0, t - self.last_t_mono)
        self.last_t_mono = t
        self.count += 1

        if dt > 0.0:
            if self.ema_dt_s <= 0.0:
                self.ema_dt_s = dt
            else:
                a = max(0.0, min(1.0, float(self.alpha)))
                self.ema_dt_s = (a * dt) + ((1.0 - a) * self.ema_dt_s)

            if dt < self.min_dt_s:
                self.min_dt_s = dt
            if dt > self.max_dt_s:
                self.max_dt_s = dt

        return dt

    @property
    def uptime_s(self) -> float:
        if self.count <= 0 or self.first_t_mono <= 0.0:
            return 0.0
        end_t = self.last_t_mono if self.last_t_mono > 0.0 else now_mono()
        return max(0.0, end_t - self.first_t_mono)

    @property
    def avg_hz(self) -> float:
        """
        Average rate since first tick.
        """
        if self.count <= 1:
            return 0.0
        up = self.uptime_s
        if up <= 0.0:
            return 0.0
        # count-1 intervals over uptime
        return float(self.count - 1) / up

    @property
    def hz_estimate(self) -> float:
        """
        EMA-based rate estimate (reacts faster than avg_hz).
        """
        if self.ema_dt_s <= 0.0:
            return 0.0
        return 1.0 / self.ema_dt_s

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "count": self.count,
            "uptime_s": self.uptime_s,
            "avg_hz": self.avg_hz,
            "hz_estimate": self.hz_estimate,
            "ema_dt_s": self.ema_dt_s,
            "min_dt_s": (None if self.min_dt_s == float("inf") else self.min_dt_s),
            "max_dt_s": self.max_dt_s,
        }


# =============================================================================
# Counters and heartbeat-style diagnostics
# =============================================================================

@dataclass
class EventCounters:
    """
    Generic counters for node/driver diagnostics.

    Add your own counters via `inc("name")`.
    """
    values: Dict[str, int] = field(default_factory=dict)

    def inc(self, name: str, amount: int = 1) -> int:
        key = str(name)
        self.values[key] = int(self.values.get(key, 0)) + int(amount)
        return self.values[key]

    def get(self, name: str, default: int = 0) -> int:
        return int(self.values.get(str(name), int(default)))

    def set(self, name: str, value: int) -> None:
        self.values[str(name)] = int(value)

    def reset(self, name: Optional[str] = None) -> None:
        if name is None:
            self.values.clear()
        else:
            self.values.pop(str(name), None)

    def to_dict(self) -> Dict[str, int]:
        return {str(k): int(v) for k, v in self.values.items()}


@dataclass
class HeartbeatState:
    """
    Lightweight heartbeat / liveness helper for diagnostics topics.

    Typical use:
      hb = HeartbeatState(name="base_driver")
      hb.beat()
      payload = hb.to_dict()
    """
    name: str = "heartbeat"
    seq: int = 0
    started_mono: float = field(default_factory=now_mono)
    last_beat_mono: float = 0.0
    note: str = ""

    def beat(self, note: str = "") -> None:
        self.seq += 1
        self.last_beat_mono = now_mono()
        if note:
            self.note = str(note)

    def is_alive(self, timeout_s: float, now_mono_s: Optional[float] = None) -> bool:
        if self.last_beat_mono <= 0.0:
            return False
        return age_s(self.last_beat_mono, now_mono_s=now_mono_s) < max(0.0, float(timeout_s))

    def to_dict(self, *, timeout_s: Optional[float] = None) -> Dict[str, Any]:
        n = now_mono()
        payload = {
            "name": self.name,
            "seq": self.seq,
            "uptime_s": max(0.0, n - self.started_mono),
            "last_beat_age_s": age_s(self.last_beat_mono, now_mono_s=n),
            "note": self.note,
        }
        if timeout_s is not None:
            payload["timeout_s"] = float(timeout_s)
            payload["alive"] = self.is_alive(float(timeout_s), now_mono_s=n)
        return payload


# =============================================================================
# Diagnostic payload builder
# =============================================================================

def make_diag_payload(
    *,
    node: str,
    status_level: str,
    summary: str,
    details: Optional[Dict[str, Any]] = None,
    counters: Optional[Dict[str, Any]] = None,
    timestamps: bool = True,
) -> Dict[str, Any]:
    """
    Build a consistent diagnostics payload dict.

    Parameters
    ----------
    node : str
        Source node/module name.
    status_level : str
        One of OK/WARN/ERROR/STALE/SAFETY_STOP (normalized internally).
    summary : str
        Human-readable status summary.
    details : dict, optional
        Extra structured diagnostic fields.
    counters : dict, optional
        Optional counters map.
    timestamps : bool
        Include wall and monotonic timestamps.
    """
    payload: Dict[str, Any] = {
        "node": str(node),
        "status_level": normalize_status_level(status_level),
        "summary": str(summary),
    }

    if timestamps:
        payload["time"] = {
            "wall_unix_s": now_wall(),
            "mono_s": now_mono(),
        }

    if details:
        payload["details"] = json_safe(details)

    if counters:
        payload["counters"] = json_safe(counters)

    return payload


def make_error_diag(
    *,
    node: str,
    exc: BaseException,
    summary: Optional[str] = None,
    include_trace: bool = False,
    details: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Convenience helper for exception-based diagnostics payloads.
    """
    d = dict(details or {})
    d["exception"] = {
        "type": exc.__class__.__name__,
        "message": str(exc),
    }
    if include_trace:
        d["traceback"] = exception_trace(exc)

    return make_diag_payload(
        node=node,
        status_level=STATUS_ERROR,
        summary=summary or exception_message(exc),
        details=d,
    )