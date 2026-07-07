"""Power policy helpers for Robot Savo Python diagnostics and fallbacks."""

from __future__ import annotations

from dataclasses import dataclass

from savo_power import constants as c
from savo_power.models.kit_battery_reading import KitBatteryReading
from savo_power.models.power_status import (
    BatterySource,
    PowerState,
    is_fault_state,
    normalize_battery_source,
)
from savo_power.models.ups_reading import UpsReading


@dataclass(frozen=True)
class PowerPolicyThresholds:
    """Thresholds used to classify Robot Savo power readings."""

    ups_low_voltage_v: float = c.UPS_LOW_VOLTAGE
    ups_critical_voltage_v: float = c.UPS_CRITICAL_VOLTAGE

    base_empty_voltage_v: float = c.BASE_BATTERY_EMPTY_VOLTAGE
    base_low_voltage_v: float = c.BASE_BATTERY_LOW_VOLTAGE
    base_full_voltage_v: float = c.BASE_BATTERY_FULL_VOLTAGE

    base_low_soc_pct: float = c.BASE_BATTERY_LOW_SOC
    base_full_soc_pct: float = c.BASE_BATTERY_FULL_SOC

    full_capacity_pct: float = c.FULL_CAPACITY_PERCENT

    automatic_shutdown_enabled: bool = c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT


def is_valid_percentage(value: float | None) -> bool:
    """Return True when value is a valid 0..100 percentage."""

    if value is None:
        return False

    return 0.0 <= float(value) <= 100.0


def has_driver_error(reading: UpsReading | KitBatteryReading) -> bool:
    """Return True if a reading already reports a driver/error condition."""

    return (
        bool(reading.error_message)
        or reading.state == PowerState.ERROR
        or (not reading.ok and reading.state != PowerState.UNKNOWN)
    )


def evaluate_ups_state(
    reading: UpsReading,
    thresholds: PowerPolicyThresholds | None = None,
) -> PowerState:
    """Evaluate UPS HAT state from a normalized UPS reading."""

    thresholds = thresholds or PowerPolicyThresholds()

    if has_driver_error(reading):
        return PowerState.ERROR

    if reading.voltage_v is None:
        return PowerState.UNKNOWN

    voltage = float(reading.voltage_v)

    if voltage <= thresholds.ups_critical_voltage_v:
        return PowerState.CRITICAL

    if voltage <= thresholds.ups_low_voltage_v:
        return PowerState.LOW

    if (
        reading.capacity_pct is not None
        and is_valid_percentage(reading.capacity_pct)
        and float(reading.capacity_pct) >= thresholds.full_capacity_pct
    ):
        return PowerState.FULL

    return PowerState.OK


def evaluate_base_battery_state(
    reading: KitBatteryReading,
    thresholds: PowerPolicyThresholds | None = None,
) -> PowerState:
    """Evaluate Freenove/base battery state from a normalized reading."""

    thresholds = thresholds or PowerPolicyThresholds()

    if has_driver_error(reading):
        return PowerState.ERROR

    has_voltage = reading.voltage_v is not None
    has_soc = reading.soc_pct is not None and is_valid_percentage(reading.soc_pct)

    if not has_voltage and not has_soc:
        return PowerState.UNKNOWN

    if has_voltage and float(reading.voltage_v) <= thresholds.base_empty_voltage_v:
        return PowerState.CRITICAL

    if has_voltage and float(reading.voltage_v) <= thresholds.base_low_voltage_v:
        return PowerState.LOW

    if has_soc and float(reading.soc_pct) <= thresholds.base_low_soc_pct:
        return PowerState.LOW

    if has_voltage and float(reading.voltage_v) >= thresholds.base_full_voltage_v:
        return PowerState.FULL

    if has_soc and float(reading.soc_pct) >= thresholds.base_full_soc_pct:
        return PowerState.FULL

    return PowerState.OK


def evaluate_power_state(
    reading: UpsReading | KitBatteryReading,
    thresholds: PowerPolicyThresholds | None = None,
) -> PowerState:
    """Evaluate any supported power reading."""

    source = normalize_battery_source(reading.source)

    if source in {BatterySource.CORE_UPS, BatterySource.EDGE_UPS}:
        if not isinstance(reading, UpsReading):
            return PowerState.ERROR
        return evaluate_ups_state(reading, thresholds)

    if source == BatterySource.BASE_BATTERY:
        if not isinstance(reading, KitBatteryReading):
            return PowerState.ERROR
        return evaluate_base_battery_state(reading, thresholds)

    if has_driver_error(reading):
        return PowerState.ERROR

    return PowerState.UNKNOWN


def should_request_shutdown(
    state: str | PowerState,
    thresholds: PowerPolicyThresholds | None = None,
) -> bool:
    """Return True only when shutdown policy is enabled and state is critical."""

    thresholds = thresholds or PowerPolicyThresholds()
    normalized = PowerState(state) if isinstance(state, str) else state

    return (
        thresholds.automatic_shutdown_enabled
        and normalized == PowerState.CRITICAL
    )


def reading_allows_normal_operation(
    reading: UpsReading | KitBatteryReading,
    thresholds: PowerPolicyThresholds | None = None,
) -> bool:
    """Return True when the evaluated reading state is not a fault state."""

    state = evaluate_power_state(reading, thresholds)
    return not is_fault_state(state)


def apply_power_policy(
    reading: UpsReading | KitBatteryReading,
    thresholds: PowerPolicyThresholds | None = None,
) -> UpsReading | KitBatteryReading:
    """Return a copy of reading with state and ok recalculated."""

    state = evaluate_power_state(reading, thresholds)

    ok = state not in {
        PowerState.ERROR,
        PowerState.UNKNOWN,
        PowerState.STALE,
    }

    if isinstance(reading, UpsReading):
        return UpsReading(
            source=reading.source,
            state=state,
            ok=ok,
            voltage_v=reading.voltage_v,
            capacity_pct=reading.capacity_pct,
            raw_voltage_word=reading.raw_voltage_word,
            raw_capacity_word=reading.raw_capacity_word,
            error_message=reading.error_message,
        )

    return KitBatteryReading(
        source=reading.source,
        state=state,
        ok=ok,
        voltage_v=reading.voltage_v,
        soc_pct=reading.soc_pct,
        adc_voltage_v=reading.adc_voltage_v,
        raw_byte=reading.raw_byte,
        channel=reading.channel,
        pcb_version=reading.pcb_version,
        error_message=reading.error_message,
    )


__all__ = [
    "PowerPolicyThresholds",
    "apply_power_policy",
    "evaluate_base_battery_state",
    "evaluate_power_state",
    "evaluate_ups_state",
    "has_driver_error",
    "is_valid_percentage",
    "reading_allows_normal_operation",
    "should_request_shutdown",
]
