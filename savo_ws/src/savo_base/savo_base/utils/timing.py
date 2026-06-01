#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/utils/timing.py
--------------------------------------
Professional timing utilities for ROS2 Jazzy / real robot testing.

Purpose
-------
Centralized, ROS-independent timing helpers used across `savo_base` modules for:
- monotonic timestamps (safe for watchdogs / loop timing)
- wall-clock UTC timestamps (logs / diagnostics)
- elapsed-time checks
- stale-data checks
- periodic execution gating
- lightweight stopwatch / scope timing

Why this file exists
--------------------
Real robot safety logic must use monotonic time for correctness.
Wall-clock time can jump (NTP sync, manual clock changes), but monotonic time does not.

Recommended usage
-----------------
- Watchdogs, staleness, command age -> use monotonic helpers
- Log messages / JSON diagnostics timestamps -> use wall-clock UTC helpers
"""

from __future__ import annotations

import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Any, Dict, Optional


# =============================================================================
# Core clock functions
# =============================================================================

def now_mono_s() -> float:
    """
    Current monotonic time in seconds (float).

    Use for:
    - watchdogs
    - stale checks
    - dt calculations
    - loop timing
    """
    return float(time.monotonic())


def now_wall_s() -> float:
    """
    Current wall-clock Unix timestamp in seconds (float).

    Use for:
    - log timestamps
    - telemetry timestamps
    - human-facing records
    """
    return float(time.time())


def now_wall_ms() -> int:
    """
    Current wall-clock Unix timestamp in milliseconds (int).
    """
    return int(round(time.time() * 1000.0))


def utc_now_iso(timespec: str = "milliseconds") -> str:
    """
    UTC timestamp in ISO8601 format with trailing 'Z'.

    Examples
    --------
    - 2026-02-22T19:42:10.123Z
    - 2026-02-22T19:42:10Z
    """
    dt = datetime.now(timezone.utc)
    text = dt.isoformat(timespec=timespec)
    return text.replace("+00:00", "Z")


# =============================================================================
# Elapsed / age helpers
# =============================================================================

def elapsed_s(since_mono_s: float, now_s: Optional[float] = None) -> float:
    """
    Elapsed monotonic seconds since `since_mono_s`.

    Returns non-negative value (clamped at 0.0).
    """
    if now_s is None:
        now_s = now_mono_s()
    dt = float(now_s) - float(since_mono_s)
    return dt if dt > 0.0 else 0.0


def age_s(timestamp_mono_s: float, now_s: Optional[float] = None) -> float:
    """
    Alias of `elapsed_s(...)` for semantic readability.

    Example
    -------
    cmd_age = age_s(last_cmd_rx_mono)
    """
    return elapsed_s(timestamp_mono_s, now_s=now_s)


def is_stale(
    timestamp_mono_s: float,
    timeout_s: float,
    *,
    now_s: Optional[float] = None,
    treat_nonpositive_as_stale: bool = True,
) -> bool:
    """
    Check if a monotonic timestamp is stale.

    Parameters
    ----------
    timestamp_mono_s:
        The event time (monotonic seconds).
    timeout_s:
        Stale threshold in seconds.
    now_s:
        Optional current monotonic time for batched checks.
    treat_nonpositive_as_stale:
        If True, timestamp <= 0 is considered stale/uninitialized.
    """
    ts = float(timestamp_mono_s)
    if ts <= 0.0 and treat_nonpositive_as_stale:
        return True
    to = max(0.0, float(timeout_s))
    return age_s(ts, now_s=now_s) >= to


def remaining_s(
    timestamp_mono_s: float,
    timeout_s: float,
    *,
    now_s: Optional[float] = None,
) -> float:
    """
    Remaining time before a timestamp becomes stale (seconds).

    Returns
    -------
    float
        Positive if still fresh, 0.0 if expired/stale.
    """
    to = max(0.0, float(timeout_s))
    rem = to - age_s(timestamp_mono_s, now_s=now_s)
    return rem if rem > 0.0 else 0.0


def timeout_ratio(
    timestamp_mono_s: float,
    timeout_s: float,
    *,
    now_s: Optional[float] = None,
    inf_if_invalid: bool = True,
) -> float:
    """
    Age/timeout ratio for diagnostics.

    - 0.0   -> just received
    - 0.5   -> halfway to timeout
    - 1.0+  -> stale
    """
    to = float(timeout_s)
    if to <= 0.0:
        return float("inf") if inf_if_invalid else 0.0
    return age_s(timestamp_mono_s, now_s=now_s) / to


# =============================================================================
# Stopwatch / scoped timing
# =============================================================================

@dataclass
class StopwatchSample:
    """
    One timing sample from Stopwatch.
    """
    name: str
    started_mono_s: float
    ended_mono_s: float
    elapsed_s: float
    wall_utc_iso: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class Stopwatch:
    """
    Lightweight monotonic stopwatch for timing operations.

    Example
    -------
    >>> sw = Stopwatch("board_write")
    >>> # do work...
    >>> sample = sw.stop()
    >>> print(sample.elapsed_s)
    """

    def __init__(self, name: str = "stopwatch", *, auto_start: bool = True) -> None:
        self._name = str(name)
        self._started = False
        self._t0 = 0.0
        self._t1 = 0.0
        if auto_start:
            self.start()

    @property
    def name(self) -> str:
        return self._name

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> "Stopwatch":
        self._t0 = now_mono_s()
        self._t1 = 0.0
        self._started = True
        return self

    def restart(self) -> "Stopwatch":
        return self.start()

    def peek_elapsed_s(self) -> float:
        if not self._started:
            return 0.0
        end = self._t1 if self._t1 > 0.0 else now_mono_s()
        dt = end - self._t0
        return dt if dt > 0.0 else 0.0

    def stop(self) -> StopwatchSample:
        if not self._started:
            # Return zero-length sample but remain safe
            return StopwatchSample(
                name=self._name,
                started_mono_s=0.0,
                ended_mono_s=0.0,
                elapsed_s=0.0,
                wall_utc_iso=utc_now_iso(),
            )
        self._t1 = now_mono_s()
        self._started = False
        return StopwatchSample(
            name=self._name,
            started_mono_s=self._t0,
            ended_mono_s=self._t1,
            elapsed_s=max(0.0, self._t1 - self._t0),
            wall_utc_iso=utc_now_iso(),
        )


class ScopeTimer:
    """
    Context-manager timer for scoped timing blocks.

    Example
    -------
    >>> with ScopeTimer("mix_and_scale") as t:
    ...     do_work()
    >>> print(t.elapsed_s)

    Notes
    -----
    - Keeps timing only (no logging side effects).
    - Safe to use in tests and production code.
    """

    def __init__(self, name: str = "scope_timer") -> None:
        self.name = str(name)
        self.started_mono_s: float = 0.0
        self.ended_mono_s: float = 0.0
        self.elapsed_s: float = 0.0
        self.wall_utc_iso: str = ""

    def __enter__(self) -> "ScopeTimer":
        self.started_mono_s = now_mono_s()
        self.ended_mono_s = 0.0
        self.elapsed_s = 0.0
        self.wall_utc_iso = ""
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.ended_mono_s = now_mono_s()
        self.elapsed_s = max(0.0, self.ended_mono_s - self.started_mono_s)
        self.wall_utc_iso = utc_now_iso()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "started_mono_s": self.started_mono_s,
            "ended_mono_s": self.ended_mono_s,
            "elapsed_s": self.elapsed_s,
            "wall_utc_iso": self.wall_utc_iso,
        }


# =============================================================================
# Periodic trigger helper (for loops / diagnostics)
# =============================================================================

class PeriodicTrigger:
    """
    Fire-at-most-once-per-period helper using monotonic time.

    Useful for:
    - throttled warnings (e.g. log every 1s)
    - low-rate diagnostic publishing inside faster loops
    - periodic checks without ROS timers

    Example
    -------
    >>> trig = PeriodicTrigger(period_s=1.0, fire_immediately=True)
    >>> if trig.ready():
    ...     print("1 Hz message")
    """

    def __init__(
        self,
        period_s: float,
        *,
        fire_immediately: bool = True,
        align_from_now: bool = True,
    ) -> None:
        p = float(period_s)
        if p <= 0.0:
            raise ValueError("PeriodicTrigger period_s must be > 0")
        self._period_s = p
        self._last_fire_mono_s = 0.0
        self._initialized = False
        self._fire_immediately = bool(fire_immediately)
        self._align_from_now = bool(align_from_now)

    @property
    def period_s(self) -> float:
        return self._period_s

    def reset(self) -> None:
        self._last_fire_mono_s = 0.0
        self._initialized = False

    def ready(self, *, now_s: Optional[float] = None) -> bool:
        """
        Returns True if period elapsed, and consumes the trigger.
        """
        if now_s is None:
            now_s = now_mono_s()

        if not self._initialized:
            self._initialized = True
            if self._fire_immediately:
                self._last_fire_mono_s = float(now_s)
                return True
            self._last_fire_mono_s = float(now_s)
            return False

        dt = float(now_s) - self._last_fire_mono_s
        if dt >= self._period_s:
            if self._align_from_now:
                self._last_fire_mono_s = float(now_s)
            else:
                self._last_fire_mono_s += self._period_s
                if self._last_fire_mono_s > float(now_s):
                    self._last_fire_mono_s = float(now_s)
            return True
        return False

    def time_until_ready_s(self, *, now_s: Optional[float] = None) -> float:
        """
        Remaining time until next ready() would return True.
        """
        if now_s is None:
            now_s = now_mono_s()
        if not self._initialized:
            return 0.0 if self._fire_immediately else self._period_s
        rem = self._period_s - (float(now_s) - self._last_fire_mono_s)
        return rem if rem > 0.0 else 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "period_s": self._period_s,
            "initialized": self._initialized,
            "last_fire_mono_s": self._last_fire_mono_s,
            "time_until_ready_s": self.time_until_ready_s(),
        }


# =============================================================================
# Convenience factories
# =============================================================================

def make_stopwatch(name: str = "stopwatch", *, auto_start: bool = True) -> Stopwatch:
    return Stopwatch(name=name, auto_start=auto_start)


def make_periodic_trigger(
    period_s: float,
    *,
    fire_immediately: bool = True,
    align_from_now: bool = True,
) -> PeriodicTrigger:
    return PeriodicTrigger(
        period_s=period_s,
        fire_immediately=fire_immediately,
        align_from_now=align_from_now,
    )


# =============================================================================
# Public exports
# =============================================================================

__all__ = [
    # clocks
    "now_mono_s",
    "now_wall_s",
    "now_wall_ms",
    "utc_now_iso",
    # elapsed / age / stale
    "elapsed_s",
    "age_s",
    "is_stale",
    "remaining_s",
    "timeout_ratio",
    # timing objects
    "StopwatchSample",
    "Stopwatch",
    "ScopeTimer",
    "PeriodicTrigger",
    # factories
    "make_stopwatch",
    "make_periodic_trigger",
]