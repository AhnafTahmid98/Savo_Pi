#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS-independent diagnostics helpers: freshness checks, status text formatting, diagnostic levels."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class DiagnosticLevel(str, Enum):
    """Simple diagnostic level labels."""

    OK = "OK"
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"
    STALE = "STALE"
    SAFETY_STOP = "SAFETY_STOP"
    DISABLED = "DISABLED"
    UNKNOWN = "UNKNOWN"


@dataclass
class StampSample:
    """Base sample with monotonic timestamp."""

    stamp: Optional[float] = None

    def age_s(self, now_s: Optional[float] = None) -> Optional[float]:
        if self.stamp is None:
            return None

        now = time.monotonic() if now_s is None else safe_float(now_s)
        return max(0.0, now - self.stamp)

    def is_fresh(self, timeout_s: float, now_s: Optional[float] = None) -> bool:
        age = self.age_s(now_s)
        if age is None:
            return False

        return age <= max(0.0, safe_float(timeout_s))

    def is_stale(self, timeout_s: float, now_s: Optional[float] = None) -> bool:
        return not self.is_fresh(timeout_s, now_s)


@dataclass
class TwistSample:
    """Twist-like sample with timestamp."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    stamp: Optional[float] = None

    @property
    def nonzero(self) -> bool:
        return (
            abs(self.vx) > 1.0e-4
            or abs(self.vy) > 1.0e-4
            or abs(self.wz) > 1.0e-4
        )

    def age_s(self, now_s: Optional[float] = None) -> Optional[float]:
        if self.stamp is None:
            return None

        now = time.monotonic() if now_s is None else safe_float(now_s)
        return max(0.0, now - self.stamp)

    def is_fresh(self, timeout_s: float, now_s: Optional[float] = None) -> bool:
        age = self.age_s(now_s)
        if age is None:
            return False

        return age <= max(0.0, safe_float(timeout_s))


@dataclass
class BoolSample:
    """Boolean sample with timestamp."""

    value: Optional[bool] = None
    stamp: Optional[float] = None

    def age_s(self, now_s: Optional[float] = None) -> Optional[float]:
        if self.stamp is None:
            return None

        now = time.monotonic() if now_s is None else safe_float(now_s)
        return max(0.0, now - self.stamp)

    def is_fresh(self, timeout_s: float, now_s: Optional[float] = None) -> bool:
        age = self.age_s(now_s)
        if age is None:
            return False

        return age <= max(0.0, safe_float(timeout_s))


@dataclass
class ScalarSample:
    """Scalar float sample with timestamp."""

    value: Optional[float] = None
    stamp: Optional[float] = None

    def age_s(self, now_s: Optional[float] = None) -> Optional[float]:
        if self.stamp is None:
            return None

        now = time.monotonic() if now_s is None else safe_float(now_s)
        return max(0.0, now - self.stamp)

    def is_fresh(self, timeout_s: float, now_s: Optional[float] = None) -> bool:
        age = self.age_s(now_s)
        if age is None:
            return False

        return age <= max(0.0, safe_float(timeout_s))


@dataclass
class StringSample:
    """String sample with timestamp."""

    value: str = ""
    stamp: Optional[float] = None

    def age_s(self, now_s: Optional[float] = None) -> Optional[float]:
        if self.stamp is None:
            return None

        now = time.monotonic() if now_s is None else safe_float(now_s)
        return max(0.0, now - self.stamp)

    def is_fresh(self, timeout_s: float, now_s: Optional[float] = None) -> bool:
        age = self.age_s(now_s)
        if age is None:
            return False

        return age <= max(0.0, safe_float(timeout_s))


@dataclass
class OdomSample:
    """Compact odometry status sample."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_speed: float = 0.0
    angular_speed: float = 0.0
    stamp: Optional[float] = None

    def age_s(self, now_s: Optional[float] = None) -> Optional[float]:
        if self.stamp is None:
            return None

        now = time.monotonic() if now_s is None else safe_float(now_s)
        return max(0.0, now - self.stamp)

    def is_fresh(self, timeout_s: float, now_s: Optional[float] = None) -> bool:
        age = self.age_s(now_s)
        if age is None:
            return False

        return age <= max(0.0, safe_float(timeout_s))


def now_s() -> float:
    """Return monotonic time in seconds."""
    return time.monotonic()


def safe_float(value: float, default: float = 0.0) -> float:
    """Return a finite float, otherwise default."""
    try:
        value_f = float(value)
    except (TypeError, ValueError):
        return float(default)

    if not math.isfinite(value_f):
        return float(default)

    return value_f


def is_finite_number(value: float) -> bool:
    """Return True if value is finite."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def age_text(stamp: Optional[float], now_value: Optional[float] = None) -> str:
    """Format age from monotonic timestamp."""
    if stamp is None:
        return "never"

    now = now_s() if now_value is None else safe_float(now_value)
    age = max(0.0, now - stamp)

    return f"{age:.2f}s"


def freshness_text(
    stamp: Optional[float],
    timeout_s: float,
    now_value: Optional[float] = None,
) -> str:
    """Return FRESH/STALE with age."""
    if stamp is None:
        return "STALE age=never"

    now = now_s() if now_value is None else safe_float(now_value)
    age = max(0.0, now - stamp)
    timeout = max(0.0, safe_float(timeout_s))

    state = "FRESH" if age <= timeout else "STALE"
    return f"{state} age={age:.2f}s"


def is_fresh(
    stamp: Optional[float],
    timeout_s: float,
    now_value: Optional[float] = None,
) -> bool:
    """Return True if timestamp exists and age <= timeout."""
    if stamp is None:
        return False

    now = now_s() if now_value is None else safe_float(now_value)
    age = max(0.0, now - stamp)

    return age <= max(0.0, safe_float(timeout_s))


def bool_text(
    sample: BoolSample,
    timeout_s: float,
    *,
    stale_text: str = "STALE",
) -> str:
    """Format BoolSample for dashboard/status output."""
    if not sample.is_fresh(timeout_s):
        return f"{stale_text} age={age_text(sample.stamp)}"

    return f"{sample.value} age={age_text(sample.stamp)}"


def scalar_text(
    sample: ScalarSample,
    timeout_s: float,
    *,
    precision: int = 3,
    unit: str = "",
    stale_text: str = "STALE",
) -> str:
    """Format ScalarSample for dashboard/status output."""
    if not sample.is_fresh(timeout_s) or sample.value is None:
        return f"{stale_text} age={age_text(sample.stamp)}"

    suffix = f" {unit}" if unit else ""
    return f"{sample.value:.{precision}f}{suffix} age={age_text(sample.stamp)}"


def string_text(
    sample: StringSample,
    timeout_s: float,
    *,
    max_len: int = 120,
    stale_text: str = "STALE",
) -> str:
    """Format StringSample for dashboard/status output."""
    if not sample.is_fresh(timeout_s):
        return f"{stale_text} age={age_text(sample.stamp)}"

    value = sample.value if sample.value else ""
    if max_len > 3 and len(value) > max_len:
        value = value[: max_len - 3] + "..."

    return f"{value} age={age_text(sample.stamp)}"


def twist_text(
    sample: TwistSample,
    timeout_s: float,
    *,
    stale_text: str = "STALE",
    show_age: bool = True,
) -> str:
    """Format TwistSample as vx/vy/wz."""
    if not sample.is_fresh(timeout_s):
        return f"{stale_text} age={age_text(sample.stamp)}"

    text = f"vx={sample.vx:+.2f} vy={sample.vy:+.2f} wz={sample.wz:+.2f}"
    if show_age:
        text += f" age={age_text(sample.stamp)}"
    return text


def odom_text(
    sample: OdomSample,
    timeout_s: float,
    *,
    stale_text: str = "STALE",
) -> str:
    """Format OdomSample."""
    if not sample.is_fresh(timeout_s):
        return f"{stale_text} age={age_text(sample.stamp)}"

    return (
        f"x={sample.x:+.2f} y={sample.y:+.2f} yaw={sample.yaw:+.2f} "
        f"lin={sample.linear_speed:.3f} ang={sample.angular_speed:.3f} "
        f"age={age_text(sample.stamp)}"
    )


def compact_twist_text(sample: TwistSample, timeout_s: float) -> str:
    """Compact TwistSample for one-line status messages."""
    if not sample.is_fresh(timeout_s):
        return "STALE"

    return f"{sample.vx:+.2f},{sample.vy:+.2f},{sample.wz:+.2f}"


def topic_state_text(
    name: str,
    stamp: Optional[float],
    timeout_s: float,
) -> str:
    """Format simple topic state string."""
    state = "OK" if is_fresh(stamp, timeout_s) else "STALE"
    return f"{name}={state} age={age_text(stamp)}"


def level_from_freshness(
    required_stamps: list[Optional[float]],
    timeout_s: float,
) -> DiagnosticLevel:
    """
    Return OK if all required stamps are fresh, otherwise STALE.
    """
    for stamp in required_stamps:
        if not is_fresh(stamp, timeout_s):
            return DiagnosticLevel.STALE
    return DiagnosticLevel.OK


def level_for_command_chain(
    *,
    source_active: bool,
    mux_stamp: Optional[float],
    cmd_stamp: Optional[float],
    safe_stamp: Optional[float],
    safety_stop: Optional[bool],
    timeout_s: float,
) -> DiagnosticLevel:
    """
    Determine high-level diagnostic level for control command chain.

    Logic:
        - safety_stop True -> SAFETY_STOP
        - source active but mux stale -> WARN
        - mux fresh but cmd stale -> WARN
        - cmd fresh but safe stale -> WARN
        - otherwise OK
    """
    if safety_stop is True:
        return DiagnosticLevel.SAFETY_STOP

    if source_active and not is_fresh(mux_stamp, timeout_s):
        return DiagnosticLevel.WARN

    if is_fresh(mux_stamp, timeout_s) and not is_fresh(cmd_stamp, timeout_s):
        return DiagnosticLevel.WARN

    if is_fresh(cmd_stamp, timeout_s) and not is_fresh(safe_stamp, timeout_s):
        return DiagnosticLevel.WARN

    return DiagnosticLevel.OK


def make_key_value_status(**kwargs) -> str:
    """
    Build compact key=value; key=value status string.

    Example:
        make_key_value_status(level="OK", mode="MANUAL")
        -> "level=OK; mode=MANUAL"
    """
    parts: list[str] = []
    for key, value in kwargs.items():
        parts.append(f"{key}={value}")
    return "; ".join(parts)


def yes_no(value: Optional[bool]) -> str:
    """Format bool as yes/no/unknown."""
    if value is None:
        return "unknown"
    return "yes" if value else "no"


def true_false_stale(
    sample: BoolSample,
    timeout_s: float,
) -> str:
    """Return True/False/STALE for BoolSample."""
    if not sample.is_fresh(timeout_s):
        return "STALE"
    return "True" if sample.value else "False"


def status_banner(title: str, width: int = 78, char: str = "=") -> list[str]:
    """Create a small text banner."""
    width = max(10, int(width))
    char = char[0] if char else "="

    title = f" {title.strip()} "
    line = char * width

    if len(title) >= width:
        return [line, title[:width], line]

    left = (width - len(title)) // 2
    right = width - len(title) - left

    return [line, f"{char * left}{title}{char * right}", line]