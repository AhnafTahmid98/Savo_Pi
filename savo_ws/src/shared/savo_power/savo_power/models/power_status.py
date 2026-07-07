"""Power status models for Robot Savo Python diagnostics and fallbacks."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from savo_power import constants as c


class PowerState(str, Enum):
    """Normalized power state values."""

    OK = c.STATE_OK
    LOW = c.STATE_LOW
    CRITICAL = c.STATE_CRITICAL
    CHARGING = c.STATE_CHARGING
    FULL = c.STATE_FULL
    ERROR = c.STATE_ERROR
    STALE = c.STATE_STALE
    UNKNOWN = c.STATE_UNKNOWN


class BatterySource(str, Enum):
    """Normalized power source names."""

    CORE_UPS = c.CORE_UPS_SOURCE
    EDGE_UPS = c.EDGE_UPS_SOURCE
    BASE_BATTERY = c.BASE_BATTERY_SOURCE
    UNKNOWN = c.UNKNOWN_SOURCE


_STATE_SEVERITY: dict[PowerState, int] = {
    PowerState.OK: 0,
    PowerState.FULL: 0,
    PowerState.CHARGING: 0,
    PowerState.UNKNOWN: 1,
    PowerState.LOW: 2,
    PowerState.STALE: 3,
    PowerState.ERROR: 4,
    PowerState.CRITICAL: 5,
}


def normalize_power_state(value: str | PowerState | None) -> PowerState:
    """Convert text or enum input into a PowerState."""

    if isinstance(value, PowerState):
        return value

    if value is None:
        return PowerState.UNKNOWN

    normalized = str(value).strip().lower()

    for state in PowerState:
        if normalized == state.value:
            return state

    return PowerState.UNKNOWN


def normalize_battery_source(value: str | BatterySource | None) -> BatterySource:
    """Convert text or enum input into a BatterySource."""

    if isinstance(value, BatterySource):
        return value

    if value is None:
        return BatterySource.UNKNOWN

    normalized = str(value).strip().lower()

    for source in BatterySource:
        if normalized == source.value:
            return source

    return BatterySource.UNKNOWN


def power_state_text(state: str | PowerState | None) -> str:
    """Return normalized power state text."""

    return normalize_power_state(state).value


def battery_source_text(source: str | BatterySource | None) -> str:
    """Return normalized source text."""

    return normalize_battery_source(source).value


def source_label(source: str | BatterySource | None) -> str:
    """Return a human-readable source label."""

    normalized = normalize_battery_source(source)

    if normalized == BatterySource.CORE_UPS:
        return "Core UPS"

    if normalized == BatterySource.EDGE_UPS:
        return "Edge UPS"

    if normalized == BatterySource.BASE_BATTERY:
        return "Base battery"

    return "Unknown source"


def is_fault_state(state: str | PowerState | None) -> bool:
    """Return True when a power state needs attention."""

    normalized = normalize_power_state(state)
    return normalized in {
        PowerState.LOW,
        PowerState.CRITICAL,
        PowerState.ERROR,
        PowerState.STALE,
        PowerState.UNKNOWN,
    }


def is_safe_state(state: str | PowerState | None) -> bool:
    """Return True when a power state allows normal operation."""

    normalized = normalize_power_state(state)
    return normalized in {
        PowerState.OK,
        PowerState.CHARGING,
        PowerState.FULL,
    }


def requires_operator_attention(state: str | PowerState | None) -> bool:
    """Return True when the operator should inspect the power system."""

    normalized = normalize_power_state(state)
    return normalized in {
        PowerState.LOW,
        PowerState.CRITICAL,
        PowerState.ERROR,
        PowerState.STALE,
    }


def allows_normal_operation(state: str | PowerState | None) -> bool:
    """Return True when power state does not block normal operation."""

    return is_safe_state(state)


def more_severe_state(
    left: str | PowerState | None,
    right: str | PowerState | None,
) -> PowerState:
    """Return the more severe of two power states."""

    left_state = normalize_power_state(left)
    right_state = normalize_power_state(right)

    if _STATE_SEVERITY[right_state] > _STATE_SEVERITY[left_state]:
        return right_state

    return left_state


@dataclass(frozen=True)
class PowerSourceStatus:
    """Status of one expected or optional power source."""

    source: BatterySource = BatterySource.UNKNOWN
    state: PowerState = PowerState.UNKNOWN

    expected: bool = True
    seen: bool = False
    stale: bool = False

    age_s: float = 0.0
    text: str = ""

    @property
    def source_text(self) -> str:
        return self.source.value

    @property
    def state_text(self) -> str:
        return self.state.value

    @property
    def label(self) -> str:
        return source_label(self.source)

    @property
    def needs_attention(self) -> bool:
        return self.expected and (
            not self.seen or self.stale or is_fault_state(self.state)
        )


@dataclass(frozen=True)
class PowerStatusSummary:
    """Aggregated power status across Robot Savo power sources."""

    overall_state: PowerState = PowerState.UNKNOWN

    core_ups: PowerSourceStatus = PowerSourceStatus(
        source=BatterySource.CORE_UPS,
    )
    edge_ups: PowerSourceStatus = PowerSourceStatus(
        source=BatterySource.EDGE_UPS,
    )
    base_battery: PowerSourceStatus = PowerSourceStatus(
        source=BatterySource.BASE_BATTERY,
    )

    status_text: str = ""
    dashboard_text: str = ""
    shutdown_requested: bool = False

    @property
    def overall_state_text(self) -> str:
        return self.overall_state.value

    @property
    def has_fault(self) -> bool:
        return (
            is_fault_state(self.overall_state)
            or self.core_ups.needs_attention
            or self.edge_ups.needs_attention
            or self.base_battery.needs_attention
        )


def missing_source_status(source: str | BatterySource) -> PowerSourceStatus:
    """Create status for an expected source that has not reported yet."""

    return PowerSourceStatus(
        source=normalize_battery_source(source),
        state=PowerState.UNKNOWN,
        expected=True,
        seen=False,
        stale=False,
        text="missing",
    )


def not_expected_source_status(source: str | BatterySource) -> PowerSourceStatus:
    """Create status for a source that is disabled by configuration."""

    return PowerSourceStatus(
        source=normalize_battery_source(source),
        state=PowerState.OK,
        expected=False,
        seen=False,
        stale=False,
        text="not expected",
    )


def source_status_from_state(
    source: str | BatterySource,
    state: str | PowerState,
    *,
    expected: bool = True,
    seen: bool = True,
    stale: bool = False,
    age_s: float = 0.0,
    text: str = "",
) -> PowerSourceStatus:
    """Create a normalized PowerSourceStatus."""

    normalized_source = normalize_battery_source(source)
    normalized_state = normalize_power_state(state)

    return PowerSourceStatus(
        source=normalized_source,
        state=normalized_state,
        expected=expected,
        seen=seen,
        stale=stale,
        age_s=max(0.0, float(age_s)),
        text=text or normalized_state.value,
    )


def aggregate_overall_state(
    *states: str | PowerState | None,
) -> PowerState:
    """Aggregate many states into one overall state."""

    overall = PowerState.OK

    for state in states:
        overall = more_severe_state(overall, state)

    return overall


__all__ = [
    "BatterySource",
    "PowerSourceStatus",
    "PowerState",
    "PowerStatusSummary",
    "aggregate_overall_state",
    "allows_normal_operation",
    "battery_source_text",
    "is_fault_state",
    "is_safe_state",
    "missing_source_status",
    "more_severe_state",
    "normalize_battery_source",
    "normalize_power_state",
    "not_expected_source_status",
    "power_state_text",
    "requires_operator_attention",
    "source_label",
    "source_status_from_state",
]
