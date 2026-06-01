#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/utils/ratekeeper.py
------------------------------------------
Professional timing helper for fixed-rate loops (ROS2 Jazzy friendly).

Purpose
-------
Provide a small, deterministic utility to keep software loops running near a
target frequency (Hz) while exposing timing diagnostics useful for real robot
testing.

Typical use-cases in `savo_base`
--------------------------------
- Non-ROS hardware polling loops (motor tests, board diagnostics)
- Internal worker loops in nodes (if not using ROS timers)
- Safety/diagnostic routines where loop timing quality matters

Design goals
------------
- Monotonic-clock based (safe against system time changes)
- Low overhead
- Jitter/overrun statistics for diagnostics
- Optional "reset on overrun" behavior
- Clean import without ROS dependency

Important note
--------------
In ROS2 nodes, prefer `create_timer(...)` for normal callback scheduling.
Use this utility when you intentionally need a manual loop (e.g. hardware driver
threads, non-ROS scripts, or tight bringup tools).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, asdict
from typing import Any, Dict, Optional


# =============================================================================
# Helpers
# =============================================================================

def _now_mono() -> float:
    """Monotonic time in seconds."""
    return float(time.monotonic())


def _clamp_hz(hz: float) -> float:
    """Clamp Hz to a sane positive range."""
    val = float(hz)
    if val <= 0.0:
        raise ValueError("RateKeeper hz must be > 0")
    if val > 10_000.0:
        return 10_000.0
    return val


# =============================================================================
# Data models
# =============================================================================

@dataclass
class RateStats:
    """
    Runtime timing statistics for a RateKeeper instance.

    Fields
    ------
    target_hz:
        Configured target frequency.
    target_period_s:
        Desired loop period (1 / target_hz).
    cycles:
        Number of completed `sleep()` cycles.
    overrun_count:
        Number of times loop execution exceeded the target period.
    total_sleep_s:
        Cumulative time spent sleeping.
    total_overrun_s:
        Cumulative overrun time (execution late beyond target wakeup).
    last_dt_s:
        Time between consecutive cycle boundaries.
    last_sleep_s:
        Sleep duration used in the most recent cycle (0 for overrun cycles).
    last_overrun_s:
        Overrun duration in the most recent cycle (0 if no overrun).
    avg_dt_s:
        Running average loop period (cycle-to-cycle).
    max_dt_s:
        Largest observed loop period.
    min_dt_s:
        Smallest observed loop period.
    """
    target_hz: float
    target_period_s: float
    cycles: int = 0
    overrun_count: int = 0
    total_sleep_s: float = 0.0
    total_overrun_s: float = 0.0
    last_dt_s: float = 0.0
    last_sleep_s: float = 0.0
    last_overrun_s: float = 0.0
    avg_dt_s: float = 0.0
    max_dt_s: float = 0.0
    min_dt_s: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


# =============================================================================
# RateKeeper
# =============================================================================

class RateKeeper:
    """
    Fixed-rate loop keeper using a monotonic clock.

    Example
    -------
    >>> rk = RateKeeper(30.0)
    >>> while running:
    ...     do_work()
    ...     rk.sleep()

    Behavior
    --------
    - Tracks a "next wake time" boundary.
    - Sleeps until next boundary when possible.
    - Detects overruns when work takes too long.
    - Can optionally reset schedule after overrun to avoid drift accumulation.
    """

    def __init__(
        self,
        hz: float,
        *,
        name: str = "ratekeeper",
        reset_on_overrun: bool = True,
        warn_overrun_ratio: float = 1.25,
        max_catchup_cycles: int = 4,
        auto_start: bool = True,
    ) -> None:
        """
        Parameters
        ----------
        hz:
            Target frequency in Hz.
        name:
            Diagnostic label.
        reset_on_overrun:
            If True, reset schedule to "now + period" after overrun.
            If False, keep advancing schedule by one period (catch-up style).
        warn_overrun_ratio:
            Threshold ratio to flag "significant" overrun in diagnostics.
            Example: 1.25 means overrun if dt > 1.25 * target_period_s.
        max_catchup_cycles:
            When reset_on_overrun=False, cap fast-forward steps after long stalls.
        auto_start:
            If True, initialize timing baseline immediately.
        """
        self._name = str(name)
        self._target_hz = _clamp_hz(hz)
        self._period_s = 1.0 / self._target_hz
        self._reset_on_overrun = bool(reset_on_overrun)
        self._warn_overrun_ratio = max(1.0, float(warn_overrun_ratio))
        self._max_catchup_cycles = max(1, int(max_catchup_cycles))

        self._started = False
        self._t_start_mono = 0.0
        self._t_last_cycle_mono = 0.0
        self._t_next_wakeup_mono = 0.0

        self._stats = RateStats(
            target_hz=self._target_hz,
            target_period_s=self._period_s,
        )

        if auto_start:
            self.start()

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def start(self) -> None:
        """
        Start or restart timing schedule from current monotonic time.
        """
        now = _now_mono()
        self._started = True
        self._t_start_mono = now
        self._t_last_cycle_mono = now
        self._t_next_wakeup_mono = now + self._period_s

        # Keep cumulative stats unless user explicitly resets
        self._stats.last_dt_s = 0.0
        self._stats.last_sleep_s = 0.0
        self._stats.last_overrun_s = 0.0

    def reset(self, *, reset_stats: bool = False) -> None:
        """
        Reset schedule baseline. Optionally reset accumulated statistics.
        """
        if reset_stats:
            self._stats = RateStats(
                target_hz=self._target_hz,
                target_period_s=self._period_s,
            )
        self.start()

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    @property
    def hz(self) -> float:
        return self._target_hz

    @property
    def period_s(self) -> float:
        return self._period_s

    def set_rate(self, hz: float, *, preserve_stats: bool = True) -> None:
        """
        Update target frequency and restart schedule.

        Parameters
        ----------
        hz:
            New target frequency in Hz.
        preserve_stats:
            If False, clears accumulated stats.
        """
        self._target_hz = _clamp_hz(hz)
        self._period_s = 1.0 / self._target_hz

        if not preserve_stats:
            self._stats = RateStats(
                target_hz=self._target_hz,
                target_period_s=self._period_s,
            )
        else:
            self._stats.target_hz = self._target_hz
            self._stats.target_period_s = self._period_s

        self.start()

    # -------------------------------------------------------------------------
    # Core API
    # -------------------------------------------------------------------------

    def sleep(self) -> float:
        """
        Keep loop timing near the configured rate.

        Returns
        -------
        float
            The actual sleep duration used (seconds). Returns 0.0 on overrun.

        Notes
        -----
        Call this once per loop iteration, typically after the loop's work.
        """
        if not self._started:
            self.start()

        now = _now_mono()
        sleep_s = self._t_next_wakeup_mono - now

        if sleep_s > 0.0:
            time.sleep(sleep_s)
            actual_sleep = sleep_s
            wake = _now_mono()
            overrun_s = 0.0
        else:
            # Overrun (work exceeded allotted period)
            actual_sleep = 0.0
            wake = now
            overrun_s = abs(sleep_s)

        # Cycle-to-cycle dt
        dt_s = max(0.0, wake - self._t_last_cycle_mono)
        self._t_last_cycle_mono = wake

        # Update schedule
        if overrun_s > 0.0:
            self._stats.overrun_count += 1
            self._stats.total_overrun_s += overrun_s

            if self._reset_on_overrun:
                self._t_next_wakeup_mono = wake + self._period_s
            else:
                # Catch-up mode, but bounded
                next_wakeup = self._t_next_wakeup_mono + self._period_s
                steps = 0
                while next_wakeup <= wake and steps < self._max_catchup_cycles:
                    next_wakeup += self._period_s
                    steps += 1
                if next_wakeup <= wake:
                    next_wakeup = wake + self._period_s
                self._t_next_wakeup_mono = next_wakeup
        else:
            self._t_next_wakeup_mono += self._period_s
            self._stats.total_sleep_s += actual_sleep

        # Stats bookkeeping
        self._stats.cycles += 1
        self._stats.last_dt_s = dt_s
        self._stats.last_sleep_s = actual_sleep
        self._stats.last_overrun_s = overrun_s

        if self._stats.cycles == 1:
            self._stats.avg_dt_s = dt_s
            self._stats.min_dt_s = dt_s
            self._stats.max_dt_s = dt_s
        else:
            n = float(self._stats.cycles)
            self._stats.avg_dt_s = ((self._stats.avg_dt_s * (n - 1.0)) + dt_s) / n
            if dt_s < self._stats.min_dt_s:
                self._stats.min_dt_s = dt_s
            if dt_s > self._stats.max_dt_s:
                self._stats.max_dt_s = dt_s

        return actual_sleep

    # -------------------------------------------------------------------------
    # Diagnostics / state
    # -------------------------------------------------------------------------

    def is_started(self) -> bool:
        return self._started

    def uptime_s(self) -> float:
        if not self._started:
            return 0.0
        return max(0.0, _now_mono() - self._t_start_mono)

    def next_wakeup_in_s(self) -> float:
        """
        Seconds until next scheduled wakeup (can be negative if already late).
        """
        if not self._started:
            return 0.0
        return self._t_next_wakeup_mono - _now_mono()

    def overrun_ratio_last(self) -> float:
        """
        last_dt / target_period. >1.0 indicates slower than target period.
        """
        if self._period_s <= 0.0:
            return 0.0
        return self._stats.last_dt_s / self._period_s if self._stats.last_dt_s > 0.0 else 0.0

    def is_overrun_last(self) -> bool:
        return self._stats.last_overrun_s > 0.0

    def is_significant_overrun_last(self) -> bool:
        return self.overrun_ratio_last() >= self._warn_overrun_ratio

    def stats(self) -> RateStats:
        """
        Return a snapshot copy of current statistics.
        """
        s = self._stats
        return RateStats(
            target_hz=s.target_hz,
            target_period_s=s.target_period_s,
            cycles=s.cycles,
            overrun_count=s.overrun_count,
            total_sleep_s=s.total_sleep_s,
            total_overrun_s=s.total_overrun_s,
            last_dt_s=s.last_dt_s,
            last_sleep_s=s.last_sleep_s,
            last_overrun_s=s.last_overrun_s,
            avg_dt_s=s.avg_dt_s,
            max_dt_s=s.max_dt_s,
            min_dt_s=s.min_dt_s,
        )

    def to_dict(self) -> Dict[str, Any]:
        """
        Ready-to-log / dashboard-friendly dict.
        """
        st = self.stats()
        return {
            "name": self._name,
            "started": self._started,
            "config": {
                "target_hz": self._target_hz,
                "target_period_s": self._period_s,
                "reset_on_overrun": self._reset_on_overrun,
                "warn_overrun_ratio": self._warn_overrun_ratio,
                "max_catchup_cycles": self._max_catchup_cycles,
            },
            "runtime": {
                "uptime_s": self.uptime_s(),
                "next_wakeup_in_s": self.next_wakeup_in_s(),
                "last_overrun": self.is_overrun_last(),
                "significant_overrun_last": self.is_significant_overrun_last(),
                "overrun_ratio_last": self.overrun_ratio_last(),
            },
            "stats": st.to_dict(),
        }

    def summary(self) -> str:
        """
        Compact human-readable summary for logs.
        """
        st = self._stats
        return (
            f"[{self._name}] "
            f"target={self._target_hz:.2f}Hz "
            f"cycles={st.cycles} "
            f"dt_last={st.last_dt_s:.4f}s "
            f"avg={st.avg_dt_s:.4f}s "
            f"overruns={st.overrun_count} "
            f"last_overrun={st.last_overrun_s:.4f}s"
        )


# =============================================================================
# Optional convenience factory
# =============================================================================

def make_ratekeeper(
    hz: float,
    *,
    name: str = "ratekeeper",
    reset_on_overrun: bool = True,
    warn_overrun_ratio: float = 1.25,
    max_catchup_cycles: int = 4,
    auto_start: bool = True,
) -> RateKeeper:
    """
    Convenience factory for consistency with other `savo_base.utils` helpers.
    """
    return RateKeeper(
        hz=hz,
        name=name,
        reset_on_overrun=reset_on_overrun,
        warn_overrun_ratio=warn_overrun_ratio,
        max_catchup_cycles=max_catchup_cycles,
        auto_start=auto_start,
    )


# =============================================================================
# Public exports
# =============================================================================

__all__ = [
    "RateStats",
    "RateKeeper",
    "make_ratekeeper",
]