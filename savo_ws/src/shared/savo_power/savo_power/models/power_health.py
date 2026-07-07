"""Power health models for Robot Savo Python diagnostics and fallbacks."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from savo_power import constants as c
from savo_power.models.power_status import (
    PowerState,
    normalize_power_state,
)


class PowerHealthLevel(str, Enum):
    """Normalized power health levels."""

    OK = c.HEALTH_OK
    WARN = c.HEALTH_WARN
    ERROR = c.HEALTH_ERROR
    UNKNOWN = c.HEALTH_UNKNOWN


@dataclass(frozen=True)
class PowerHealthConfig:
    """Configuration for evaluating power health."""

    stale_timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S

    missing_status_is_error: bool = True
    stale_status_is_error: bool = True
    unknown_state_is_error: bool = True


@dataclass(frozen=True)
class PowerHealthInput:
    """Input for health evaluation."""

    seen: bool = False
    state: PowerState = PowerState.UNKNOWN

    age_s: float = 0.0

    status_text: str = ""


@dataclass(frozen=True)
class PowerHealthResult:
    """Health evaluation result."""

    level: PowerHealthLevel = PowerHealthLevel.UNKNOWN
    state: PowerState = PowerState.UNKNOWN

    missing: bool = True
    stale: bool = False

    reason: str = ""
    text: str = ""

    @property
    def level_text(self) -> str:
        return self.level.value

    @property
    def state_text(self) -> str:
        return self.state.value

    @property
    def is_ok(self) -> bool:
        return self.level == PowerHealthLevel.OK

    @property
    def needs_attention(self) -> bool:
        return self.level in {
            PowerHealthLevel.WARN,
            PowerHealthLevel.ERROR,
            PowerHealthLevel.UNKNOWN,
        }


def normalize_health_level(
    value: str | PowerHealthLevel | None,
) -> PowerHealthLevel:
    """Convert text or enum input into a PowerHealthLevel."""

    if isinstance(value, PowerHealthLevel):
        return value

    if value is None:
        return PowerHealthLevel.UNKNOWN

    normalized = str(value).strip().upper()

    for level in PowerHealthLevel:
        if normalized == level.value:
            return level

    return PowerHealthLevel.UNKNOWN


def health_level_from_power_state(
    state: str | PowerState | None,
    *,
    unknown_state_is_error: bool = True,
) -> PowerHealthLevel:
    """Convert power state into health level."""

    normalized = normalize_power_state(state)

    if normalized in {
        PowerState.OK,
        PowerState.FULL,
        PowerState.CHARGING,
    }:
        return PowerHealthLevel.OK

    if normalized == PowerState.LOW:
        return PowerHealthLevel.WARN

    if normalized == PowerState.UNKNOWN and not unknown_state_is_error:
        return PowerHealthLevel.UNKNOWN

    return PowerHealthLevel.ERROR


def health_reason_for_state(state: str | PowerState | None) -> str:
    """Return a stable reason string for a power state."""

    normalized = normalize_power_state(state)

    if normalized in {
        PowerState.OK,
        PowerState.FULL,
        PowerState.CHARGING,
    }:
        return "power_ok"

    if normalized == PowerState.LOW:
        return "power_low"

    if normalized == PowerState.CRITICAL:
        return "power_critical"

    if normalized == PowerState.ERROR:
        return "power_error"

    if normalized == PowerState.STALE:
        return "power_stale"

    return "power_unknown"


def _contains(text: str, token: str) -> bool:
    return token in text


def state_from_status_text(status_text: str | None) -> PowerState:
    """Extract overall power state from status/dashboard text."""

    if not status_text:
        return PowerState.UNKNOWN

    lower = str(status_text).strip().lower()

    if (
        _contains(lower, "overall=critical")
        or _contains(lower, "state=critical")
        or _contains(lower, "overall power: critical")
        or _contains(lower, " critical")
    ):
        return PowerState.CRITICAL

    if (
        _contains(lower, "overall=error")
        or _contains(lower, "state=error")
        or _contains(lower, "overall power: error")
        or _contains(lower, " error")
        or _contains(lower, "error:")
    ):
        return PowerState.ERROR

    if (
        _contains(lower, "overall=stale")
        or _contains(lower, "state=stale")
        or _contains(lower, "overall power: stale")
        or _contains(lower, " stale")
    ):
        return PowerState.STALE

    if (
        _contains(lower, "overall=low")
        or _contains(lower, "state=low")
        or _contains(lower, "overall power: low")
        or _contains(lower, " low")
    ):
        return PowerState.LOW

    if (
        _contains(lower, "overall=unknown")
        or _contains(lower, "state=unknown")
        or _contains(lower, "overall power: unknown")
        or _contains(lower, " unknown")
    ):
        return PowerState.UNKNOWN

    if (
        _contains(lower, "overall=charging")
        or _contains(lower, "state=charging")
        or _contains(lower, "overall power: charging")
        or _contains(lower, " charging")
    ):
        return PowerState.CHARGING

    if (
        _contains(lower, "overall=full")
        or _contains(lower, "state=full")
        or _contains(lower, "overall power: full")
        or _contains(lower, " full")
    ):
        return PowerState.FULL

    if (
        _contains(lower, "overall=ok")
        or _contains(lower, "state=ok")
        or _contains(lower, "overall power: ok")
        or _contains(lower, " ok")
    ):
        return PowerState.OK

    return PowerState.UNKNOWN


def make_health_text(
    result: PowerHealthResult,
    input_data: PowerHealthInput,
) -> str:
    """Create stable compact health text."""

    parts = [
        f"level={result.level.value}",
        f"state={result.state.value}",
        f"reason={result.reason}",
    ]

    if result.stale:
        parts.append(f"age_s={int(input_data.age_s)}")

    if input_data.status_text and not result.missing and not result.stale:
        parts.append(f'status="{input_data.status_text}"')

    return " ".join(parts)


class PowerHealth:
    """Reusable health evaluator for Python diagnostics."""

    def __init__(self, config: PowerHealthConfig | None = None) -> None:
        self._config = config or PowerHealthConfig()

    @property
    def config(self) -> PowerHealthConfig:
        return self._config

    def set_config(self, config: PowerHealthConfig) -> None:
        self._config = config

    def evaluate(self, input_data: PowerHealthInput) -> PowerHealthResult:
        missing = not bool(input_data.seen)

        if missing:
            level = (
                PowerHealthLevel.ERROR
                if self._config.missing_status_is_error
                else PowerHealthLevel.UNKNOWN
            )
            result = PowerHealthResult(
                level=level,
                state=PowerState.STALE,
                missing=True,
                stale=False,
                reason="missing_power_status",
            )
            return PowerHealthResult(
                level=result.level,
                state=result.state,
                missing=result.missing,
                stale=result.stale,
                reason=result.reason,
                text=make_health_text(result, input_data),
            )

        stale = float(input_data.age_s) > float(self._config.stale_timeout_s)

        if stale:
            level = (
                PowerHealthLevel.ERROR
                if self._config.stale_status_is_error
                else PowerHealthLevel.WARN
            )
            result = PowerHealthResult(
                level=level,
                state=PowerState.STALE,
                missing=False,
                stale=True,
                reason="stale_power_status",
            )
            return PowerHealthResult(
                level=result.level,
                state=result.state,
                missing=result.missing,
                stale=result.stale,
                reason=result.reason,
                text=make_health_text(result, input_data),
            )

        state = normalize_power_state(input_data.state)
        level = health_level_from_power_state(
            state,
            unknown_state_is_error=self._config.unknown_state_is_error,
        )

        result = PowerHealthResult(
            level=level,
            state=state,
            missing=False,
            stale=False,
            reason=health_reason_for_state(state),
        )

        return PowerHealthResult(
            level=result.level,
            state=result.state,
            missing=result.missing,
            stale=result.stale,
            reason=result.reason,
            text=make_health_text(result, input_data),
        )

    def evaluate_status_text(
        self,
        seen: bool,
        age_s: float,
        status_text: str,
    ) -> PowerHealthResult:
        input_data = PowerHealthInput(
            seen=seen,
            age_s=max(0.0, float(age_s)),
            status_text=status_text,
            state=state_from_status_text(status_text),
        )

        return self.evaluate(input_data)


def evaluate_power_health(
    input_data: PowerHealthInput,
    config: PowerHealthConfig | None = None,
) -> PowerHealthResult:
    """Convenience wrapper for one-shot health evaluation."""

    return PowerHealth(config).evaluate(input_data)


def evaluate_status_text_health(
    *,
    seen: bool,
    age_s: float,
    status_text: str,
    config: PowerHealthConfig | None = None,
) -> PowerHealthResult:
    """Convenience wrapper for one-shot status text health evaluation."""

    return PowerHealth(config).evaluate_status_text(
        seen=seen,
        age_s=age_s,
        status_text=status_text,
    )


__all__ = [
    "PowerHealth",
    "PowerHealthConfig",
    "PowerHealthInput",
    "PowerHealthLevel",
    "PowerHealthResult",
    "evaluate_power_health",
    "evaluate_status_text_health",
    "health_level_from_power_state",
    "health_reason_for_state",
    "make_health_text",
    "normalize_health_level",
    "state_from_status_text",
]
