"""Freenove/base battery reading model and ADS7830 conversion helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from savo_power import constants as c
from savo_power.models.power_status import (
    BatterySource,
    PowerState,
    battery_source_text,
    source_label,
)


def is_valid_ads7830_channel(channel: int) -> bool:
    """Return True when ADS7830 channel is in range 0..7."""

    return 0 <= int(channel) <= 7


def ads7830_channel_command(channel: int) -> int:
    """Return ADS7830 command byte for a single-ended channel read."""

    if not is_valid_ads7830_channel(channel):
        raise ValueError("ADS7830 channel must be in range 0..7")

    channel = int(channel)

    return int(
        c.ADS7830_CMD_BASE
        | (((channel << 2) | (channel >> 1)) & 0x07) << 4
    ) & 0xFF


def normalize_pcb_version(pcb_version: str | None) -> str:
    """Normalize ADS7830/Freenove PCB version text."""

    if pcb_version is None:
        return c.ADS7830_DEFAULT_PCB_VERSION

    normalized = str(pcb_version).strip().lower()

    if normalized in {c.ADS7830_PCB_V1, "1", "pcb_v1", "pcb1"}:
        return c.ADS7830_PCB_V1

    if normalized in {c.ADS7830_PCB_V2, "2", "pcb_v2", "pcb2"}:
        return c.ADS7830_PCB_V2

    return c.ADS7830_DEFAULT_PCB_VERSION


def ads7830_reference_voltage(pcb_version: str | None = None) -> float:
    """Return ADS7830 reference voltage for the Freenove PCB version."""

    version = normalize_pcb_version(pcb_version)

    if version == c.ADS7830_PCB_V1:
        return 3.3

    return 5.2


def ads7830_battery_multiplier(pcb_version: str | None = None) -> float:
    """Return battery divider multiplier for the Freenove PCB version."""

    version = normalize_pcb_version(pcb_version)

    if version == c.ADS7830_PCB_V1:
        return 3.0

    return 2.0


def ads7830_adc_voltage_from_byte(
    raw_value: int,
    pcb_version: str | None = None,
) -> float:
    """Convert ADS7830 raw byte into ADC input voltage."""

    raw = int(raw_value) & 0xFF

    return raw / 255.0 * ads7830_reference_voltage(pcb_version)


def ads7830_battery_voltage_from_byte(
    raw_value: int,
    pcb_version: str | None = None,
) -> float:
    """Convert ADS7830 raw byte into estimated battery voltage."""

    return (
        ads7830_adc_voltage_from_byte(raw_value, pcb_version)
        * ads7830_battery_multiplier(pcb_version)
    )


def estimate_linear_soc_pct(
    voltage_v: float,
    *,
    empty_voltage_v: float = c.BASE_BATTERY_EMPTY_VOLTAGE,
    full_voltage_v: float = c.BASE_BATTERY_FULL_VOLTAGE,
) -> float:
    """Estimate state-of-charge using a simple clamped linear model."""

    voltage = float(voltage_v)

    if voltage <= empty_voltage_v:
        return 0.0

    if voltage >= full_voltage_v:
        return 100.0

    return (voltage - empty_voltage_v) / (full_voltage_v - empty_voltage_v) * 100.0


def is_valid_percentage(value: float | None) -> bool:
    """Return True when a percentage is inside 0..100."""

    if value is None:
        return False

    return 0.0 <= float(value) <= 100.0


def evaluate_base_battery_state(
    voltage_v: float | None,
    soc_pct: float | None = None,
    *,
    empty_voltage_v: float = c.BASE_BATTERY_EMPTY_VOLTAGE,
    low_voltage_v: float = c.BASE_BATTERY_LOW_VOLTAGE,
    full_voltage_v: float = c.BASE_BATTERY_FULL_VOLTAGE,
    low_soc_pct: float = c.BASE_BATTERY_LOW_SOC,
    full_soc_pct: float = c.BASE_BATTERY_FULL_SOC,
    error: str = "",
) -> PowerState:
    """Evaluate Freenove/base battery state."""

    if error:
        return PowerState.ERROR

    has_voltage = voltage_v is not None
    has_soc = soc_pct is not None and is_valid_percentage(soc_pct)

    if not has_voltage and not has_soc:
        return PowerState.UNKNOWN

    if has_voltage and float(voltage_v) <= empty_voltage_v:
        return PowerState.CRITICAL

    if has_voltage and float(voltage_v) <= low_voltage_v:
        return PowerState.LOW

    if has_soc and float(soc_pct) <= low_soc_pct:
        return PowerState.LOW

    if has_voltage and float(voltage_v) >= full_voltage_v:
        return PowerState.FULL

    if has_soc and float(soc_pct) >= full_soc_pct:
        return PowerState.FULL

    return PowerState.OK


@dataclass(frozen=True)
class KitBatteryReading:
    """Normalized Freenove/base battery reading."""

    source: BatterySource = BatterySource.BASE_BATTERY
    state: PowerState = PowerState.UNKNOWN

    ok: bool = False

    voltage_v: float | None = None
    soc_pct: float | None = None
    adc_voltage_v: float | None = None

    raw_byte: int | None = None
    channel: int = c.ADS7830_DEFAULT_CHANNEL
    pcb_version: str = c.ADS7830_DEFAULT_PCB_VERSION

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
    def has_soc(self) -> bool:
        return self.soc_pct is not None

    @property
    def has_valid_soc(self) -> bool:
        return is_valid_percentage(self.soc_pct)

    @property
    def has_error(self) -> bool:
        return bool(self.error_message) or self.state == PowerState.ERROR or not self.ok

    def to_dict(self) -> dict[str, Any]:
        return {
            "source": self.source.value,
            "state": self.state.value,
            "ok": self.ok,
            "voltage_v": self.voltage_v,
            "soc_pct": self.soc_pct,
            "adc_voltage_v": self.adc_voltage_v,
            "raw_byte": self.raw_byte,
            "channel": self.channel,
            "pcb_version": self.pcb_version,
            "error_message": self.error_message,
        }

    def format_line(self) -> str:
        voltage_text = "n/a V"
        if self.voltage_v is not None:
            voltage_text = f"{self.voltage_v:.2f} V"

        line = f"{self.label} {self.state.value}: {voltage_text}"

        if self.soc_pct is not None:
            line += f", SoC {self.soc_pct:.1f}%"

        if self.error_message:
            line += f", error: {self.error_message}"
        elif not self.ok:
            line += ", not ok"

        return line


def make_kit_battery_reading(
    voltage_v: float | None,
    soc_pct: float | None = None,
    *,
    adc_voltage_v: float | None = None,
    raw_byte: int | None = None,
    channel: int = c.ADS7830_DEFAULT_CHANNEL,
    pcb_version: str | None = c.ADS7830_DEFAULT_PCB_VERSION,
) -> KitBatteryReading:
    """Create a normalized Freenove/base battery reading."""

    normalized_pcb = normalize_pcb_version(pcb_version)

    if not is_valid_ads7830_channel(channel):
        return make_kit_battery_error(
            f"ADS7830 channel must be in range 0..7, got {channel}",
            channel=channel,
            pcb_version=normalized_pcb,
        )

    state = evaluate_base_battery_state(
        voltage_v=voltage_v,
        soc_pct=soc_pct,
    )

    return KitBatteryReading(
        source=BatterySource.BASE_BATTERY,
        state=state,
        ok=state not in {
            PowerState.ERROR,
            PowerState.UNKNOWN,
            PowerState.STALE,
        },
        voltage_v=None if voltage_v is None else float(voltage_v),
        soc_pct=None if soc_pct is None else float(soc_pct),
        adc_voltage_v=None if adc_voltage_v is None else float(adc_voltage_v),
        raw_byte=None if raw_byte is None else int(raw_byte) & 0xFF,
        channel=int(channel),
        pcb_version=normalized_pcb,
    )


def make_kit_battery_reading_from_raw(
    raw_byte: int,
    *,
    channel: int = c.ADS7830_DEFAULT_CHANNEL,
    pcb_version: str | None = c.ADS7830_DEFAULT_PCB_VERSION,
) -> KitBatteryReading:
    """Create a Freenove/base battery reading from raw ADS7830 byte."""

    normalized_pcb = normalize_pcb_version(pcb_version)

    adc_voltage_v = ads7830_adc_voltage_from_byte(raw_byte, normalized_pcb)
    battery_voltage_v = ads7830_battery_voltage_from_byte(raw_byte, normalized_pcb)
    soc_pct = estimate_linear_soc_pct(battery_voltage_v)

    return make_kit_battery_reading(
        voltage_v=battery_voltage_v,
        soc_pct=soc_pct,
        adc_voltage_v=adc_voltage_v,
        raw_byte=raw_byte,
        channel=channel,
        pcb_version=normalized_pcb,
    )


def make_kit_battery_error(
    error_message: str,
    *,
    channel: int = c.ADS7830_DEFAULT_CHANNEL,
    pcb_version: str | None = c.ADS7830_DEFAULT_PCB_VERSION,
) -> KitBatteryReading:
    """Create a Freenove/base battery error reading."""

    return KitBatteryReading(
        source=BatterySource.BASE_BATTERY,
        state=PowerState.ERROR,
        ok=False,
        channel=int(channel),
        pcb_version=normalize_pcb_version(pcb_version),
        error_message=str(error_message),
    )


__all__ = [
    "KitBatteryReading",
    "ads7830_adc_voltage_from_byte",
    "ads7830_battery_multiplier",
    "ads7830_battery_voltage_from_byte",
    "ads7830_channel_command",
    "ads7830_reference_voltage",
    "estimate_linear_soc_pct",
    "evaluate_base_battery_state",
    "is_valid_ads7830_channel",
    "is_valid_percentage",
    "make_kit_battery_error",
    "make_kit_battery_reading",
    "make_kit_battery_reading_from_raw",
    "normalize_pcb_version",
]
