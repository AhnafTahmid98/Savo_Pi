"""ROS adapter helpers for Robot Savo Python power fallback nodes."""

from __future__ import annotations

import importlib
import json
from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
from typing import Any, Iterable, Mapping

from savo_power.models.kit_battery_reading import KitBatteryReading
from savo_power.models.power_health import PowerHealthResult
from savo_power.models.power_status import (
    PowerSourceStatus,
    PowerStatusSummary,
)
from savo_power.models.ups_reading import UpsReading


PowerReading = UpsReading | KitBatteryReading


@dataclass
class StringMessageFallback:
    """Fallback replacement for std_msgs.msg.String."""

    data: str = ""


@dataclass
class BoolMessageFallback:
    """Fallback replacement for std_msgs.msg.Bool."""

    data: bool = False


@dataclass
class Float32MessageFallback:
    """Fallback replacement for std_msgs.msg.Float32."""

    data: float = 0.0


@dataclass
class Int32MessageFallback:
    """Fallback replacement for std_msgs.msg.Int32."""

    data: int = 0


def import_message_class(
    module_name: str,
    class_name: str,
    fallback_class: type[Any],
) -> type[Any]:
    """Import a ROS message class, or return fallback if unavailable."""

    try:
        module = importlib.import_module(module_name)
    except ImportError:
        return fallback_class

    return getattr(module, class_name, fallback_class)


def string_message_class() -> type[Any]:
    """Return std_msgs.msg.String or fallback."""

    return import_message_class(
        "std_msgs.msg",
        "String",
        StringMessageFallback,
    )


def bool_message_class() -> type[Any]:
    """Return std_msgs.msg.Bool or fallback."""

    return import_message_class(
        "std_msgs.msg",
        "Bool",
        BoolMessageFallback,
    )


def float32_message_class() -> type[Any]:
    """Return std_msgs.msg.Float32 or fallback."""

    return import_message_class(
        "std_msgs.msg",
        "Float32",
        Float32MessageFallback,
    )


def int32_message_class() -> type[Any]:
    """Return std_msgs.msg.Int32 or fallback."""

    return import_message_class(
        "std_msgs.msg",
        "Int32",
        Int32MessageFallback,
    )


def make_string_message(data: object = "") -> Any:
    """Create String message or fallback."""

    msg = string_message_class()()
    msg.data = str(data)
    return msg


def make_bool_message(data: bool = False) -> Any:
    """Create Bool message or fallback."""

    msg = bool_message_class()()
    msg.data = bool(data)
    return msg


def make_float32_message(data: float = 0.0) -> Any:
    """Create Float32 message or fallback."""

    msg = float32_message_class()()
    msg.data = float(data)
    return msg


def make_int32_message(data: int = 0) -> Any:
    """Create Int32 message or fallback."""

    msg = int32_message_class()()
    msg.data = int(data)
    return msg


def to_jsonable(value: object) -> object:
    """Convert dataclasses/enums/containers into JSON-safe data."""

    if isinstance(value, Enum):
        return value.value

    if is_dataclass(value) and not isinstance(value, type):
        return to_jsonable(asdict(value))

    if isinstance(value, Mapping):
        return {
            str(key): to_jsonable(item)
            for key, item in value.items()
        }

    if isinstance(value, tuple | list):
        return [
            to_jsonable(item)
            for item in value
        ]

    if isinstance(value, set):
        return sorted(to_jsonable(item) for item in value)

    return value


def to_json_text(
    value: object,
    *,
    indent: int | None = None,
    sort_keys: bool = True,
) -> str:
    """Convert a value into stable JSON text."""

    return json.dumps(
        to_jsonable(value),
        indent=indent,
        sort_keys=sort_keys,
    )


def reading_to_dict(reading: PowerReading) -> dict[str, object]:
    """Convert a UPS/base battery reading to a dictionary."""

    if hasattr(reading, "to_dict"):
        return dict(reading.to_dict())

    return dict(to_jsonable(reading))  # type: ignore[arg-type]


def reading_to_text(reading: PowerReading) -> str:
    """Convert a UPS/base battery reading to compact human text."""

    if hasattr(reading, "format_line"):
        return str(reading.format_line())

    data = reading_to_dict(reading)
    return " ".join(f"{key}={value}" for key, value in data.items())


def reading_to_json_text(
    reading: PowerReading,
    *,
    indent: int | None = None,
) -> str:
    """Convert one power reading to JSON text."""

    return to_json_text(
        reading_to_dict(reading),
        indent=indent,
    )


def readings_to_json_text(
    readings: Iterable[PowerReading],
    *,
    indent: int | None = None,
) -> str:
    """Convert many power readings to JSON text."""

    return to_json_text(
        [reading_to_dict(reading) for reading in readings],
        indent=indent,
    )


def readings_to_dashboard_text(readings: Iterable[PowerReading]) -> str:
    """Convert power readings to one dashboard text line."""

    lines = [
        reading_to_text(reading)
        for reading in readings
    ]

    if not lines:
        return "Robot Savo power: no readings"

    return " | ".join(lines)


def status_source_to_dict(status: PowerSourceStatus) -> dict[str, object]:
    """Convert one source status to dictionary."""

    if hasattr(status, "to_dict"):
        return dict(status.to_dict())

    return dict(to_jsonable(status))  # type: ignore[arg-type]


def status_summary_to_dict(summary: PowerStatusSummary) -> dict[str, object]:
    """Convert power status summary to dictionary."""

    if hasattr(summary, "to_dict"):
        return dict(summary.to_dict())

    return dict(to_jsonable(summary))  # type: ignore[arg-type]


def status_summary_to_text(summary: PowerStatusSummary) -> str:
    """Convert power status summary to compact text."""

    if hasattr(summary, "format_line"):
        return str(summary.format_line())

    data = status_summary_to_dict(summary)
    return " ".join(f"{key}={value}" for key, value in data.items())


def health_result_to_dict(result: PowerHealthResult) -> dict[str, object]:
    """Convert power health result to dictionary."""

    if hasattr(result, "to_dict"):
        return dict(result.to_dict())

    return dict(to_jsonable(result))  # type: ignore[arg-type]


def health_result_to_text(result: PowerHealthResult) -> str:
    """Convert power health result to compact text."""

    text = getattr(result, "text", "")

    if text:
        return str(text)

    data = health_result_to_dict(result)
    return " ".join(f"{key}={value}" for key, value in data.items())


def make_reading_text_message(reading: PowerReading) -> Any:
    """Create String message containing one reading as text."""

    return make_string_message(reading_to_text(reading))


def make_reading_json_message(
    reading: PowerReading,
    *,
    indent: int | None = None,
) -> Any:
    """Create String message containing one reading as JSON."""

    return make_string_message(
        reading_to_json_text(
            reading,
            indent=indent,
        )
    )


def make_readings_dashboard_message(readings: Iterable[PowerReading]) -> Any:
    """Create String message containing dashboard text for many readings."""

    return make_string_message(readings_to_dashboard_text(readings))


def make_readings_json_message(
    readings: Iterable[PowerReading],
    *,
    indent: int | None = None,
) -> Any:
    """Create String message containing many readings as JSON."""

    return make_string_message(
        readings_to_json_text(
            readings,
            indent=indent,
        )
    )


def make_status_summary_text_message(summary: PowerStatusSummary) -> Any:
    """Create String message containing power status summary text."""

    return make_string_message(status_summary_to_text(summary))


def make_status_summary_json_message(
    summary: PowerStatusSummary,
    *,
    indent: int | None = None,
) -> Any:
    """Create String message containing power status summary JSON."""

    return make_string_message(
        to_json_text(
            status_summary_to_dict(summary),
            indent=indent,
        )
    )


def make_health_text_message(result: PowerHealthResult) -> Any:
    """Create String message containing power health text."""

    return make_string_message(health_result_to_text(result))


def make_health_json_message(
    result: PowerHealthResult,
    *,
    indent: int | None = None,
) -> Any:
    """Create String message containing power health JSON."""

    return make_string_message(
        to_json_text(
            health_result_to_dict(result),
            indent=indent,
        )
    )


def make_ok_message(ok: bool) -> Any:
    """Create Bool message for an ok/fail state."""

    return make_bool_message(ok)


def make_voltage_message(voltage_v: float | int | None) -> Any:
    """Create Float32 voltage message."""

    return make_float32_message(0.0 if voltage_v is None else float(voltage_v))


def make_percentage_message(value: float | int | None) -> Any:
    """Create Float32 percentage message."""

    return make_float32_message(0.0 if value is None else float(value))


__all__ = [
    "BoolMessageFallback",
    "Float32MessageFallback",
    "Int32MessageFallback",
    "PowerReading",
    "StringMessageFallback",
    "bool_message_class",
    "float32_message_class",
    "health_result_to_dict",
    "health_result_to_text",
    "import_message_class",
    "int32_message_class",
    "make_bool_message",
    "make_float32_message",
    "make_health_json_message",
    "make_health_text_message",
    "make_int32_message",
    "make_ok_message",
    "make_percentage_message",
    "make_reading_json_message",
    "make_reading_text_message",
    "make_readings_dashboard_message",
    "make_readings_json_message",
    "make_status_summary_json_message",
    "make_status_summary_text_message",
    "make_string_message",
    "make_voltage_message",
    "reading_to_dict",
    "reading_to_json_text",
    "reading_to_text",
    "readings_to_dashboard_text",
    "readings_to_json_text",
    "status_source_to_dict",
    "status_summary_to_dict",
    "status_summary_to_text",
    "string_message_class",
    "to_json_text",
    "to_jsonable",
]
