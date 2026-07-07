"""Loop rate helpers for Robot Savo power Python tools."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from savo_power import constants as c
from savo_power.utils.clamp import clamp_publish_rate_hz
from savo_power.utils.timing import (
    format_duration_s,
    format_rate_hz,
    monotonic_time_s,
    period_to_rate_hz,
    rate_to_period_s,
    sleep_s,
)


ClockFn = Callable[[], float]
SleepFn = Callable[[float], None]


@dataclass(frozen=True)
class RateKeeperConfig:
    """Configuration for RateKeeper."""

    rate_hz: float = c.DEFAULT_PUBLISH_RATE_HZ
    catch_up: bool = False
    min_sleep_s: float = 0.0

    @property
    def normalized_rate_hz(self) -> float:
        return clamp_publish_rate_hz(self.rate_hz)

    @property
    def period_s(self) -> float:
        return rate_to_period_s(self.normalized_rate_hz)


@dataclass(frozen=True)
class RateKeeperSnapshot:
    """Small immutable snapshot of ratekeeper state."""

    rate_hz: float
    period_s: float
    tick_count: int
    overrun_count: int
    last_period_s: float
    remaining_s: float
    behind_s: float

    @property
    def effective_rate_hz(self) -> float:
        if self.last_period_s <= 0.0:
            return 0.0

        return period_to_rate_hz(self.last_period_s)

    def to_dict(self) -> dict[str, object]:
        return {
            "rate_hz": self.rate_hz,
            "period_s": self.period_s,
            "tick_count": self.tick_count,
            "overrun_count": self.overrun_count,
            "last_period_s": self.last_period_s,
            "remaining_s": self.remaining_s,
            "behind_s": self.behind_s,
            "effective_rate_hz": self.effective_rate_hz,
        }

    def format_line(self) -> str:
        return (
            f"rate={format_rate_hz(self.rate_hz)} "
            f"period={format_duration_s(self.period_s)} "
            f"ticks={self.tick_count} "
            f"overruns={self.overrun_count} "
            f"last_period={format_duration_s(self.last_period_s)} "
            f"remaining={format_duration_s(self.remaining_s)} "
            f"behind={format_duration_s(self.behind_s)}"
        )


class RateKeeper:
    """Small ROS-free rate helper for Python loops."""

    def __init__(
        self,
        rate_hz: float | None = None,
        *,
        config: RateKeeperConfig | None = None,
        clock: ClockFn | None = None,
        sleeper: SleepFn | None = None,
    ) -> None:
        if config is None:
            config = RateKeeperConfig(
                rate_hz=(
                    c.DEFAULT_PUBLISH_RATE_HZ
                    if rate_hz is None
                    else float(rate_hz)
                )
            )

        self._config = config
        self._clock = clock or monotonic_time_s
        self._sleeper = sleeper or sleep_s

        now = self._clock()

        self._last_tick_s = now
        self._next_tick_s = now + self.period_s

        self._tick_count = 0
        self._overrun_count = 0
        self._last_period_s = 0.0

    @property
    def config(self) -> RateKeeperConfig:
        return self._config

    @property
    def rate_hz(self) -> float:
        return self._config.normalized_rate_hz

    @property
    def period_s(self) -> float:
        return self._config.period_s

    @property
    def tick_count(self) -> int:
        return self._tick_count

    @property
    def overrun_count(self) -> int:
        return self._overrun_count

    @property
    def last_period_s(self) -> float:
        return self._last_period_s

    @property
    def next_tick_s(self) -> float:
        return self._next_tick_s

    def reset(self) -> None:
        """Reset internal timing state."""

        now = self._clock()

        self._last_tick_s = now
        self._next_tick_s = now + self.period_s

        self._tick_count = 0
        self._overrun_count = 0
        self._last_period_s = 0.0

    def set_rate_hz(self, rate_hz: float) -> None:
        """Change loop rate and reset timing."""

        self._config = RateKeeperConfig(
            rate_hz=float(rate_hz),
            catch_up=self._config.catch_up,
            min_sleep_s=self._config.min_sleep_s,
        )
        self.reset()

    def now_s(self) -> float:
        """Return current time from configured clock."""

        return self._clock()

    def remaining_s(self, *, now_s: float | None = None) -> float:
        """Return seconds remaining until next tick."""

        now = self._clock() if now_s is None else float(now_s)
        return max(0.0, self._next_tick_s - now)

    def behind_s(self, *, now_s: float | None = None) -> float:
        """Return seconds behind schedule."""

        now = self._clock() if now_s is None else float(now_s)
        return max(0.0, now - self._next_tick_s)

    def ready(self, *, now_s: float | None = None) -> bool:
        """Return True when next tick time has arrived."""

        now = self._clock() if now_s is None else float(now_s)
        return now >= self._next_tick_s

    def mark(self, *, now_s: float | None = None) -> float:
        """Mark one completed loop tick and update schedule."""

        now = self._clock() if now_s is None else float(now_s)

        self._last_period_s = max(0.0, now - self._last_tick_s)
        self._last_tick_s = now
        self._tick_count += 1

        if now > self._next_tick_s:
            self._overrun_count += 1

        if self._config.catch_up:
            while self._next_tick_s <= now:
                self._next_tick_s += self.period_s
        else:
            self._next_tick_s = now + self.period_s

        return self._last_period_s

    def sleep(self) -> float:
        """Sleep until next tick and mark the loop.

        Returns the requested sleep duration.
        """

        remaining = self.remaining_s()

        if remaining > max(0.0, float(self._config.min_sleep_s)):
            self._sleeper(remaining)
            slept = remaining
        else:
            slept = 0.0

        self.mark()
        return slept

    def wait(self) -> float:
        """Alias for sleep()."""

        return self.sleep()

    def snapshot(self) -> RateKeeperSnapshot:
        """Return current state snapshot."""

        return RateKeeperSnapshot(
            rate_hz=self.rate_hz,
            period_s=self.period_s,
            tick_count=self._tick_count,
            overrun_count=self._overrun_count,
            last_period_s=self._last_period_s,
            remaining_s=self.remaining_s(),
            behind_s=self.behind_s(),
        )


class FakeClock:
    """Small controllable clock useful for tests."""

    def __init__(self, start_s: float = 0.0) -> None:
        self._now_s = float(start_s)

    @property
    def now_s(self) -> float:
        return self._now_s

    def __call__(self) -> float:
        return self._now_s

    def advance(self, duration_s: float) -> None:
        self._now_s += max(0.0, float(duration_s))

    def sleep(self, duration_s: float) -> None:
        self.advance(duration_s)


def make_ratekeeper(
    rate_hz: float = c.DEFAULT_PUBLISH_RATE_HZ,
    *,
    catch_up: bool = False,
    min_sleep_s: float = 0.0,
) -> RateKeeper:
    """Create RateKeeper with normalized config."""

    return RateKeeper(
        config=RateKeeperConfig(
            rate_hz=rate_hz,
            catch_up=catch_up,
            min_sleep_s=min_sleep_s,
        )
    )


def sleep_at_rate(rate_hz: float) -> float:
    """Sleep once for one period of rate_hz and return sleep duration."""

    duration_s = rate_to_period_s(
        clamp_publish_rate_hz(rate_hz),
    )
    sleep_s(duration_s)
    return duration_s


def should_run_periodic(
    last_run_s: float | None,
    *,
    rate_hz: float,
    now_s: float,
) -> bool:
    """Return True if enough time passed for a periodic action."""

    if last_run_s is None:
        return True

    period_s = rate_to_period_s(
        clamp_publish_rate_hz(rate_hz),
    )

    return float(now_s) - float(last_run_s) >= period_s


__all__ = [
    "ClockFn",
    "FakeClock",
    "RateKeeper",
    "RateKeeperConfig",
    "RateKeeperSnapshot",
    "SleepFn",
    "make_ratekeeper",
    "should_run_periodic",
    "sleep_at_rate",
]
