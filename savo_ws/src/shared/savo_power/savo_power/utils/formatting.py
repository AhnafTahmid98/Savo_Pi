"""Shared formatting helpers for Robot Savo power Python tools."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
from enum import Enum
from typing import Iterable, Mapping, Sequence

from savo_power.drivers.smbus_adapter import format_i2c_address as _format_i2c_address
from savo_power.models.kit_battery_reading import KitBatteryReading
from savo_power.models.power_health import PowerHealthResult
from savo_power.models.power_status import (
    BatterySource,
    PowerSourceStatus,
    PowerState,
    PowerStatusSummary,
    normalize_battery_source,
    normalize_power_state,
    source_label,
)
from savo_power.models.ups_reading import UpsReading


PowerReading = UpsReading | KitBatteryReading


UNKNOWN_TEXT = "unknown"
OK_TEXT = "OK"
FAIL_TEXT = "FAIL"

DASHBOARD_SEPARATOR = " | "


def safe_text(value: object | None, *, unknown: str = UNKNOWN_TEXT) -> str:
    """Return stable text for optional values."""

    if value is None:
        return unknown

    if isinstance(value, Enum):
        return str(value.value)

    return str(value)


def bool_text(value: bool | None, *, true_text: str = "true", false_text: str = "false") -> str:
    """Format bool or optional bool."""

    if value is None:
        return UNKNOWN_TEXT

    return true_text if bool(value) else false_text


def ok_fail_text(ok: bool | None) -> str:
    """Format an ok/fail value."""

    if ok is None:
        return UNKNOWN_TEXT

    return OK_TEXT if ok else FAIL_TEXT


def power_state_text(state: str | PowerState | None) -> str:
    """Format a power state."""

    return normalize_power_state(state).value


def battery_source_text(source: str | BatterySource | None) -> str:
    """Format a battery source."""

    return normalize_battery_source(source).value


def battery_source_label(source: str | BatterySource | None) -> str:
    """Format battery source as a human label."""

    return source_label(normalize_battery_source(source))


def format_float(
    value: float | int | None,
    *,
    precision: int = 2,
    suffix: str = "",
    unknown: str = UNKNOWN_TEXT,
) -> str:
    """Format a float safely."""

    if value is None:
        return unknown

    return f"{float(value):.{int(precision)}f}{suffix}"


def format_voltage(value: float | int | None) -> str:
    """Format voltage value."""

    return format_float(value, precision=2, suffix=" V")


def format_percentage(value: float | int | None) -> str:
    """Format percentage value."""

    return format_float(value, precision=1, suffix="%")


def format_seconds(value: float | int | None) -> str:
    """Format seconds value."""

    return format_float(value, precision=2, suffix=" s")


def format_rate_hz(value: float | int | None) -> str:
    """Format rate value."""

    return format_float(value, precision=2, suffix=" Hz")


def format_i2c_address(address: int | None) -> str:
    """Format I2C address safely."""

    if address is None:
        return UNKNOWN_TEXT

    return _format_i2c_address(int(address))


def format_raw_word(value: int | None) -> str:
    """Format a 16-bit raw register word."""

    if value is None:
        return UNKNOWN_TEXT

    return f"0x{int(value) & 0xFFFF:04X}"


def format_raw_byte(value: int | None) -> str:
    """Format an 8-bit raw register byte."""

    if value is None:
        return UNKNOWN_TEXT

    return f"0x{int(value) & 0xFF:02X}"


def compact_key_values(
    values: Mapping[str, object] | Iterable[tuple[str, object]],
    *,
    skip_none: bool = False,
) -> str:
    """Format key/value data into one compact line."""

    items = values.items() if isinstance(values, Mapping) else values

    parts: list[str] = []

    for key, value in items:
        if value is None and skip_none:
            continue

        parts.append(f"{key}={safe_text(value)}")

    return " ".join(parts)


def join_non_empty(
    values: Iterable[object | None],
    *,
    separator: str = " ",
) -> str:
    """Join only non-empty text values."""

    parts = [
        str(value)
        for value in values
        if value is not None and str(value) != ""
    ]

    return separator.join(parts)


def truncate_text(text: object, *, max_len: int = 120) -> str:
    """Truncate long text safely."""

    value = safe_text(text)
    limit = max(4, int(max_len))

    if len(value) <= limit:
        return value

    return value[: limit - 3] + "..."


def format_result_line(ok: bool | None, *, prefix: str = "Result") -> str:
    """Format PASS/FAIL result line."""

    return f"{prefix}: {ok_fail_text(ok)}"


def format_heading(title: str, *, underline: str = "=") -> str:
    """Format a small terminal heading."""

    clean = str(title).strip()
    line = str(underline or "=")[0] * len(clean)

    return f"{clean}\n{line}"


def reading_to_text(reading: PowerReading) -> str:
    """Format one power reading."""

    if hasattr(reading, "format_line"):
        return str(reading.format_line())

    return compact_key_values(to_plain_dict(reading))


def readings_to_dashboard_text(readings: Iterable[PowerReading]) -> str:
    """Format several readings as one dashboard line."""

    lines = [reading_to_text(reading) for reading in readings]

    if not lines:
        return "Robot Savo power: no readings"

    return DASHBOARD_SEPARATOR.join(lines)


def ups_reading_brief(reading: UpsReading) -> str:
    """Format compact UPS reading text."""

    return (
        f"{battery_source_label(reading.source)} "
        f"{power_state_text(reading.state)}: "
        f"{format_voltage(reading.voltage_v)}, "
        f"capacity {format_percentage(reading.capacity_pct)}"
    )


def kit_battery_reading_brief(reading: KitBatteryReading) -> str:
    """Format compact base battery reading text."""

    return (
        f"{battery_source_label(reading.source)} "
        f"{power_state_text(reading.state)}: "
        f"{format_voltage(reading.voltage_v)}, "
        f"SoC {format_percentage(reading.soc_pct)}"
    )


def source_status_to_text(status: PowerSourceStatus) -> str:
    """Format one source status."""

    expected_text = "expected" if status.expected else "not_expected"
    seen_text = "seen" if status.seen else "missing"

    return compact_key_values(
        {
            "source": battery_source_text(status.source),
            "state": power_state_text(status.state),
            "ok": bool_text(status.ok),
            "expected": expected_text,
            "seen": seen_text,
            "age_s": format_seconds(status.age_s),
        }
    )


def status_summary_to_text(summary: PowerStatusSummary) -> str:
    """Format aggregated power status summary."""

    if hasattr(summary, "format_line"):
        return str(summary.format_line())

    return compact_key_values(to_plain_dict(summary))


def health_result_to_text(result: PowerHealthResult) -> str:
    """Format power health result."""

    if getattr(result, "text", ""):
        return str(result.text)

    return compact_key_values(to_plain_dict(result))


def to_plain_dict(value: object) -> dict[str, object]:
    """Convert common dataclass-like objects into a plain dictionary."""

    if hasattr(value, "to_dict"):
        data = value.to_dict()
        if isinstance(data, dict):
            return dict(data)

    if is_dataclass(value) and not isinstance(value, type):
        return {
            str(key): _plain_value(item)
            for key, item in asdict(value).items()
        }

    if isinstance(value, Mapping):
        return {
            str(key): _plain_value(item)
            for key, item in value.items()
        }

    return {"value": _plain_value(value)}


def _plain_value(value: object) -> object:
    if isinstance(value, Enum):
        return value.value

    if is_dataclass(value) and not isinstance(value, type):
        return {
            str(key): _plain_value(item)
            for key, item in asdict(value).items()
        }

    if isinstance(value, Mapping):
        return {
            str(key): _plain_value(item)
            for key, item in value.items()
        }

    if isinstance(value, (tuple, list)):
        return [_plain_value(item) for item in value]

    if isinstance(value, set):
        return sorted(_plain_value(item) for item in value)

    return value


def format_table(
    rows: Sequence[Sequence[object]],
    *,
    headers: Sequence[object] = (),
) -> str:
    """Format a small plain-text table."""

    all_rows: list[list[str]] = []

    if headers:
        all_rows.append([safe_text(item) for item in headers])

    for row in rows:
        all_rows.append([safe_text(item) for item in row])

    if not all_rows:
        return ""

    column_count = max(len(row) for row in all_rows)
    widths = [0] * column_count

    for row in all_rows:
        for index in range(column_count):
            cell = row[index] if index < len(row) else ""
            widths[index] = max(widths[index], len(cell))

    def render_row(row: Sequence[str]) -> str:
        cells = []

        for index in range(column_count):
            cell = row[index] if index < len(row) else ""
            cells.append(cell.ljust(widths[index]))

        return "  ".join(cells).rstrip()

    rendered = [render_row(row) for row in all_rows]

    if headers:
        separator = "  ".join("-" * width for width in widths).rstrip()
        rendered.insert(1, separator)

    return "\n".join(rendered)


__all__ = [
    "DASHBOARD_SEPARATOR",
    "FAIL_TEXT",
    "OK_TEXT",
    "PowerReading",
    "UNKNOWN_TEXT",
    "battery_source_label",
    "battery_source_text",
    "bool_text",
    "compact_key_values",
    "format_float",
    "format_heading",
    "format_i2c_address",
    "format_percentage",
    "format_rate_hz",
    "format_raw_byte",
    "format_raw_word",
    "format_result_line",
    "format_seconds",
    "format_table",
    "format_voltage",
    "health_result_to_text",
    "join_non_empty",
    "kit_battery_reading_brief",
    "ok_fail_text",
    "power_state_text",
    "reading_to_text",
    "readings_to_dashboard_text",
    "safe_text",
    "source_status_to_text",
    "status_summary_to_text",
    "to_plain_dict",
    "truncate_text",
    "ups_reading_brief",
]
