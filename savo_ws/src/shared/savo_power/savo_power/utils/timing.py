"""Timing helpers for Robot Savo power Python tools."""

from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import UTC, datetime

from savo_power import constants as c


def monotonic_time_s() -> float:
    """Return monotonic time in seconds."""

    return time.monotonic()


def wall_time_s() -> float:
    """Return wall-clock time in seconds since epoch."""

    return time.time()


def utc_now() -> datetime:
    """Return current UTC datetime."""

    return datetime.now(UTC)


def utc_now_iso() -> str:
    """Return current UTC time in ISO-8601 text form."""

    return utc_now().isoformat()


def sleep_s(duration_s: float) -> None:
    """Sleep for non-negative seconds."""

    delay = max(0.0, float(duration_s))

    if delay > 0.0:
        time.sleep(delay)


def clamp_nonnegative_s(value: float | int | None) -> float:
    """Clamp optional seconds value to non-negative float."""

    if value is None:
        return 0.0

    return max(0.0, float(value))


def rate_to_period_s(rate_hz: float | int | None, *, default_hz: float = 1.0) -> float:
    """Convert rate in Hz to period in seconds."""

    if rate_hz is None:
        rate = float(default_hz)
    else:
        rate = float(rate_hz)

    if rate <= 0.0:
        rate = float(default_hz)

    if rate <= 0.0:
        rate = 1.0

    return 1.0 / rate


def period_to_rate_hz(period_s: float | int | None, *, default_period_s: float = 1.0) -> float:
    """Convert period in seconds to rate in Hz."""

    if period_s is None:
        period = float(default_period_s)
    else:
        period = float(period_s)

    if period <= 0.0:
        period = float(default_period_s)

    if period <= 0.0:
        period = 1.0

    return 1.0 / period


def elapsed_s_since(start_time_s: float | int | None, *, now_s: float | None = None) -> float:
    """Return elapsed monotonic seconds since start."""

    if start_time_s is None:
        return 0.0

    now = monotonic_time_s() if now_s is None else float(now_s)

    return max(0.0, now - float(start_time_s))


def age_s(timestamp_s: float | int | None, *, now_s: float | None = None) -> float:
    """Return age in seconds for a monotonic timestamp."""

    return elapsed_s_since(timestamp_s, now_s=now_s)


def is_stale(
    timestamp_s: float | int | None,
    *,
    timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S,
    now_s: float | None = None,
) -> bool:
    """Return True when timestamp is missing or older than timeout."""

    if timestamp_s is None:
        return True

    return age_s(timestamp_s, now_s=now_s) > max(0.0, float(timeout_s))


def deadline_time_s(timeout_s: float | int, *, start_s: float | None = None) -> float:
    """Return monotonic deadline time."""

    start = monotonic_time_s() if start_s is None else float(start_s)
    return start + clamp_nonnegative_s(timeout_s)


def time_until_s(deadline_s: float | int, *, now_s: float | None = None) -> float:
    """Return non-negative seconds until deadline."""

    now = monotonic_time_s() if now_s is None else float(now_s)
    return max(0.0, float(deadline_s) - now)


def deadline_expired(deadline_s: float | int, *, now_s: float | None = None) -> bool:
    """Return True when deadline has passed."""

    now = monotonic_time_s() if now_s is None else float(now_s)
    return now >= float(deadline_s)


def format_duration_s(value_s: float | int | None, *, precision: int = 2) -> str:
    """Format duration seconds."""

    if value_s is None:
        return "unknown"

    return f"{float(value_s):.{int(precision)}f} s"


def format_rate_hz(value_hz: float | int | None, *, precision: int = 2) -> str:
    """Format rate in Hz."""

    if value_hz is None:
        return "unknown"

    return f"{float(value_hz):.{int(precision)}f} Hz"


@dataclass
class Stopwatch:
    """Simple monotonic stopwatch."""

    start_time_s: float = 0.0

    @classmethod
    def started(cls) -> "Stopwatch":
        return cls(start_time_s=monotonic_time_s())

    def restart(self) -> None:
        self.start_time_s = monotonic_time_s()

    def elapsed_s(self, *, now_s: float | None = None) -> float:
        return elapsed_s_since(self.start_time_s, now_s=now_s)

    def elapsed_text(self, *, precision: int = 2) -> str:
        return format_duration_s(self.elapsed_s(), precision=precision)


@dataclass
class TimestampTracker:
    """Track latest update timestamp for stale checks."""

    timestamp_s: float | None = None

    @property
    def seen(self) -> bool:
        return self.timestamp_s is not None

    def mark_now(self) -> float:
        self.timestamp_s = monotonic_time_s()
        return self.timestamp_s

    def clear(self) -> None:
        self.timestamp_s = None

    def age_s(self, *, now_s: float | None = None) -> float:
        return age_s(self.timestamp_s, now_s=now_s)

    def is_stale(
        self,
        *,
        timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S,
        now_s: float | None = None,
    ) -> bool:
        return is_stale(
            self.timestamp_s,
            timeout_s=timeout_s,
            now_s=now_s,
        )


@dataclass
class Deadline:
    """Simple deadline helper."""

    deadline_s: float

    @classmethod
    def from_timeout(
        cls,
        timeout_s: float | int,
        *,
        start_s: float | None = None,
    ) -> "Deadline":
        return cls(
            deadline_s=deadline_time_s(
                timeout_s,
                start_s=start_s,
            )
        )

    def remaining_s(self, *, now_s: float | None = None) -> float:
        return time_until_s(self.deadline_s, now_s=now_s)

    def expired(self, *, now_s: float | None = None) -> bool:
        return deadline_expired(self.deadline_s, now_s=now_s)

    def remaining_text(self, *, precision: int = 2) -> str:
        return format_duration_s(
            self.remaining_s(),
            precision=precision,
        )


@dataclass
class LoopTimer:
    """Helper for simple loop timing diagnostics."""

    last_time_s: float | None = None
    period_s: float = 0.0

    @classmethod
    def from_rate(cls, rate_hz: float | int | None) -> "LoopTimer":
        return cls(
            last_time_s=None,
            period_s=rate_to_period_s(rate_hz),
        )

    def mark(self, *, now_s: float | None = None) -> float:
        now = monotonic_time_s() if now_s is None else float(now_s)

        if self.last_time_s is None:
            self.period_s = 0.0
        else:
            self.period_s = max(0.0, now - self.last_time_s)

        self.last_time_s = now
        return self.period_s

    @property
    def rate_hz(self) -> float:
        return period_to_rate_hz(self.period_s) if self.period_s > 0.0 else 0.0

    def period_text(self, *, precision: int = 2) -> str:
        return format_duration_s(self.period_s, precision=precision)

    def rate_text(self, *, precision: int = 2) -> str:
        return format_rate_hz(self.rate_hz, precision=precision)


__all__ = [
    "Deadline",
    "LoopTimer",
    "Stopwatch",
    "TimestampTracker",
    "age_s",
    "clamp_nonnegative_s",
    "deadline_expired",
    "deadline_time_s",
    "elapsed_s_since",
    "format_duration_s",
    "format_rate_hz",
    "is_stale",
    "monotonic_time_s",
    "period_to_rate_hz",
    "rate_to_period_s",
    "sleep_s",
    "time_until_s",
    "utc_now",
    "utc_now_iso",
    "wall_time_s",
]
