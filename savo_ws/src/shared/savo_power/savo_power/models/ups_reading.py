"""UPS HAT reading model and conversion helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from savo_power import constants as c
from savo_power.models.power_status import (
    BatterySource,
    PowerState,
    battery_source_text,
    normalize_battery_source,
    source_label,
)


def swap_word_bytes(value: int) -> int:
    """Swap low/high bytes of a 16-bit SMBus word."""

    value = int(value) & 0xFFFF
    return ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8)


def ups_voltage_from_raw_word(raw_word: int) -> float:
    """Convert raw UPS voltage register word to volts."""

    swapped = swap_word_bytes(raw_word)
    return float(swapped) * 1.25 / 1000.0 / 16.0


def ups_capacity_from_raw_word(raw_word: int) -> float:
    """Convert raw UPS capacity register word to percent."""

    swapped = swap_word_bytes(raw_word)
    return float(swapped) / 256.0


def is_valid_percentage(value: float | None) -> bool:
    """Return True when a percentage is inside 0..100."""

    if value is None:
        return False

    return 0.0 <= float(value) <= 100.0


def is_ups_source(source: str | BatterySource | None) -> bool:
    """Return True if the source is core UPS or edge UPS."""

    normalized = normalize_battery_source(source)
    return normalized in {
        BatterySource.CORE_UPS,
        BatterySource.EDGE_UPS,
    }


def evaluate_ups_state(
    voltage_v: float | None,
    capacity_pct: float | None = None,
    *,
    low_voltage_v: float = c.UPS_LOW_VOLTAGE,
    critical_voltage_v: float = c.UPS_CRITICAL_VOLTAGE,
    full_capacity_pct: float = c.FULL_CAPACITY_PERCENT,
    error: str = "",
) -> PowerState:
    """Evaluate UPS state using voltage and optional capacity."""

    if error:
        return PowerState.ERROR

    if voltage_v is None:
        return PowerState.UNKNOWN

    voltage = float(voltage_v)

    if voltage <= critical_voltage_v:
        return PowerState.CRITICAL

    if voltage <= low_voltage_v:
        return PowerState.LOW

    if (
        capacity_pct is not None
        and is_valid_percentage(capacity_pct)
        and float(capacity_pct) >= full_capacity_pct
    ):
        return PowerState.FULL

    return PowerState.OK


@dataclass(frozen=True)
class UpsReading:
    """Normalized UPS HAT reading."""

    source: BatterySource = BatterySource.UNKNOWN
    state: PowerState = PowerState.UNKNOWN

    ok: bool = False

    voltage_v: float | None = None
    capacity_pct: float | None = None

    raw_voltage_word: int | None = None
    raw_capacity_word: int | None = None

    error_message: str = ""

    @property
    def source_text(self) -> str:
        return battery_source_text(self.source)

    @property
    def state_text(self) -> str:
        return self.state.value

    @property
    def label(self) -> str:
        return source_label(self.source)

    @property
    def has_voltage(self) -> bool:
        return self.voltage_v is not None

    @property
    def has_capacity(self) -> bool:
        return self.capacity_pct is not None

    @property
    def has_valid_capacity(self) -> bool:
        return is_valid_percentage(self.capacity_pct)

    @property
    def has_error(self) -> bool:
        return bool(self.error_message) or self.state == PowerState.ERROR or not self.ok

    def with_state(self, state: str | PowerState) -> "UpsReading":
        normalized_state = PowerState(state) if isinstance(state, str) else state
        return UpsReading(
            source=self.source,
            state=normalized_state,
            ok=normalized_state not in {
                PowerState.ERROR,
                PowerState.UNKNOWN,
                PowerState.STALE,
            },
            voltage_v=self.voltage_v,
            capacity_pct=self.capacity_pct,
            raw_voltage_word=self.raw_voltage_word,
            raw_capacity_word=self.raw_capacity_word,
            error_message=self.error_message,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "source": self.source.value,
            "state": self.state.value,
            "ok": self.ok,
            "voltage_v": self.voltage_v,
            "capacity_pct": self.capacity_pct,
            "raw_voltage_word": self.raw_voltage_word,
            "raw_capacity_word": self.raw_capacity_word,
            "error_message": self.error_message,
        }

    def format_line(self) -> str:
        voltage_text = "n/a V"
        if self.voltage_v is not None:
            voltage_text = f"{self.voltage_v:.2f} V"

        line = f"{self.label} {self.state.value}: {voltage_text}"

        if self.capacity_pct is not None:
            line += f", capacity {self.capacity_pct:.1f}%"

        if self.error_message:
            line += f", error: {self.error_message}"
        elif not self.ok:
            line += ", not ok"

        return line


def make_ups_reading(
    source: str | BatterySource,
    voltage_v: float | None,
    capacity_pct: float | None = None,
    *,
    raw_voltage_word: int | None = None,
    raw_capacity_word: int | None = None,
) -> UpsReading:
    """Create a normalized UPS reading and evaluate its state."""

    normalized_source = normalize_battery_source(source)

    if not is_ups_source(normalized_source):
        return make_ups_error(
            normalized_source,
            "UPS reading source must be core_ups or edge_ups",
        )

    state = evaluate_ups_state(
        voltage_v=voltage_v,
        capacity_pct=capacity_pct,
    )

    return UpsReading(
        source=normalized_source,
        state=state,
        ok=state not in {
            PowerState.ERROR,
            PowerState.UNKNOWN,
            PowerState.STALE,
        },
        voltage_v=None if voltage_v is None else float(voltage_v),
        capacity_pct=None if capacity_pct is None else float(capacity_pct),
        raw_voltage_word=raw_voltage_word,
        raw_capacity_word=raw_capacity_word,
    )


def make_ups_reading_from_raw(
    source: str | BatterySource,
    raw_voltage_word: int,
    raw_capacity_word: int,
) -> UpsReading:
    """Create a UPS reading from raw SMBus register words."""

    voltage_v = ups_voltage_from_raw_word(raw_voltage_word)
    capacity_pct = ups_capacity_from_raw_word(raw_capacity_word)

    return make_ups_reading(
        source=source,
        voltage_v=voltage_v,
        capacity_pct=capacity_pct,
        raw_voltage_word=int(raw_voltage_word) & 0xFFFF,
        raw_capacity_word=int(raw_capacity_word) & 0xFFFF,
    )


def make_ups_error(
    source: str | BatterySource | None,
    error_message: str,
) -> UpsReading:
    """Create a UPS error reading."""

    normalized_source = normalize_battery_source(source)

    return UpsReading(
        source=normalized_source,
        state=PowerState.ERROR,
        ok=False,
        error_message=str(error_message),
    )


__all__ = [
    "UpsReading",
    "evaluate_ups_state",
    "is_ups_source",
    "is_valid_percentage",
    "make_ups_error",
    "make_ups_reading",
    "make_ups_reading_from_raw",
    "swap_word_bytes",
    "ups_capacity_from_raw_word",
    "ups_voltage_from_raw_word",
]
