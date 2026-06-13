#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Report formatting helpers for localization diagnostics. No ROS imports."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import is_dataclass, asdict
from typing import Any

from savo_localization.constants import (
    PACKAGE_NAME,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
)


STATUS_LABELS = {
    STATUS_OK: "OK",
    STATUS_WARN: "WARN",
    STATUS_ERROR: "ERROR",
    STATUS_STALE: "STALE",
    STATUS_UNKNOWN: "UNKNOWN",
}


def format_report_title(title: str) -> str:
    title = str(title).strip() or "Localization Report"
    return f"{title}\n{'=' * len(title)}"


def format_status_line(
    *,
    name: str,
    status: str,
    message: str = "",
    ready: bool | None = None,
    usable: bool | None = None,
) -> str:
    parts = [
        f"{name}: {STATUS_LABELS.get(status, status)}",
    ]

    if message:
        parts.append(str(message))

    flags: list[str] = []
    if ready is not None:
        flags.append(f"ready={_bool_text(ready)}")

    if usable is not None:
        flags.append(f"usable={_bool_text(usable)}")

    if flags:
        parts.append(", ".join(flags))

    return " | ".join(parts)


def format_reasons(reasons: Sequence[str] | None, *, indent: str = "  - ") -> str:
    if not reasons:
        return ""

    return "\n".join(f"{indent}{reason}" for reason in reasons)


def format_key_values(
    values: Mapping[str, Any],
    *,
    indent: str = "",
    skip_none: bool = True,
) -> str:
    lines: list[str] = []

    for key, value in values.items():
        if value is None and skip_none:
            continue

        lines.append(f"{indent}{key}: {_format_value(value)}")

    return "\n".join(lines)


def format_json_report(
    payload: Mapping[str, Any] | Any,
    *,
    compact: bool = False,
    sort_keys: bool = True,
) -> str:
    separators = (",", ":") if compact else None

    return json.dumps(
        to_plain_dict(payload),
        indent=None if compact else 2,
        separators=separators,
        sort_keys=sort_keys,
    )


def format_component_block(
    *,
    name: str,
    status: str,
    message: str = "",
    ready: bool | None = None,
    usable: bool | None = None,
    reasons: Sequence[str] | None = None,
    details: Mapping[str, Any] | None = None,
) -> str:
    lines = [
        format_status_line(
            name=name,
            status=status,
            message=message,
            ready=ready,
            usable=usable,
        )
    ]

    if reasons:
        lines.append("Reasons:")
        lines.append(format_reasons(reasons))

    if details:
        lines.append("Details:")
        lines.append(format_key_values(details, indent="  "))

    return "\n".join(lines)


def format_localization_summary(payload: Mapping[str, Any] | Any) -> str:
    data = to_plain_dict(payload)

    lines = [
        format_report_title("Robot Savo Localization Summary"),
        "",
        format_status_line(
            name=str(data.get("package_name", PACKAGE_NAME)),
            status=str(data.get("status", STATUS_UNKNOWN)),
            message=str(data.get("message", "")),
            ready=_optional_bool(data.get("ready")),
            usable=_optional_bool(data.get("usable")),
        ),
    ]

    reasons = data.get("reasons", [])
    if reasons:
        lines.append("")
        lines.append("Reasons:")
        lines.append(format_reasons(_string_list(reasons)))

    components = data.get("components", {})
    if isinstance(components, Mapping) and components:
        lines.append("")
        lines.append("Components:")
        for component_name, component_data in components.items():
            if not isinstance(component_data, Mapping):
                continue

            lines.append(
                "  "
                + format_status_line(
                    name=str(component_name),
                    status=str(component_data.get("status", STATUS_UNKNOWN)),
                    message=str(component_data.get("message", "")),
                    ready=_optional_bool(component_data.get("ready")),
                    usable=_optional_bool(component_data.get("usable")),
                )
            )

    return "\n".join(lines)


def format_imu_report(payload: Mapping[str, Any] | Any) -> str:
    data = to_plain_dict(payload)
    health = _mapping(data.get("health"))

    details = {
        "model": data.get("model"),
        "frame_id": data.get("frame_id"),
        "i2c_bus": data.get("i2c_bus"),
        "i2c_address": data.get("i2c_address_hex", data.get("i2c_address")),
        "mode": data.get("mode"),
        "sample_count": data.get("sample_count"),
        "chip_ok": data.get("chip_ok"),
        "sys_status": data.get("sys_status"),
        "sys_error": data.get("sys_error"),
    }

    calibration = _mapping(data.get("calibration"))
    if calibration:
        details["calibration"] = (
            f"sys={calibration.get('system')}/3 "
            f"gyro={calibration.get('gyro')}/3 "
            f"accel={calibration.get('accel')}/3 "
            f"mag={calibration.get('mag')}/3"
        )

    return format_component_block(
        name="IMU",
        status=str(health.get("status", data.get("status", STATUS_UNKNOWN))),
        message=str(health.get("message", "")),
        ready=_optional_bool(data.get("ready")),
        usable=_status_usable(str(health.get("status", STATUS_UNKNOWN))),
        reasons=_string_list(health.get("reasons", [])),
        details=details,
    )


def format_encoder_report(payload: Mapping[str, Any] | Any) -> str:
    data = to_plain_dict(payload)
    health = _mapping(data.get("health"))

    details = {
        "model": data.get("model"),
        "sample_count": data.get("sample_count"),
        "last_sample_age_s": data.get("last_sample_age_s"),
        "active_wheel_count": health.get("active_wheel_count"),
        "total_illegal_transitions": health.get("total_illegal_transitions"),
        "min_counts_per_second": health.get("min_counts_per_second"),
        "max_counts_per_second": health.get("max_counts_per_second"),
    }

    lines = [
        format_component_block(
            name="Encoders",
            status=str(health.get("status", STATUS_UNKNOWN)),
            message=str(health.get("message", "")),
            ready=_optional_bool(data.get("ready")),
            usable=_status_usable(str(health.get("status", STATUS_UNKNOWN))),
            reasons=_string_list(health.get("reasons", [])),
            details=details,
        )
    ]

    wheels = _mapping(data.get("wheels"))
    if wheels:
        lines.append("")
        lines.append("Wheels:")
        for wheel_name, wheel_data in wheels.items():
            if not isinstance(wheel_data, Mapping):
                continue

            lines.append(
                "  "
                + format_key_values(
                    {
                        "wheel": wheel_name,
                        "count": wheel_data.get("count"),
                        "delta": wheel_data.get("delta_count"),
                        "rate_cps": wheel_data.get("counts_per_second"),
                        "speed_mps": wheel_data.get("speed_mps"),
                        "dir": wheel_data.get("direction_symbol"),
                        "illegal": wheel_data.get("illegal_transitions"),
                    },
                    skip_none=True,
                ).replace("\n", " | ")
            )

    return "\n".join(lines)


def format_wheel_odom_report(payload: Mapping[str, Any] | Any) -> str:
    data = to_plain_dict(payload)
    health = _mapping(data.get("health"))
    pose = _mapping(data.get("pose"))
    twist = _mapping(data.get("twist"))

    details = {
        "topic": data.get("wheel_odom_topic"),
        "frames": f"{data.get('odom_frame_id')} -> {data.get('base_frame_id')}",
        "sample_count": data.get("sample_count"),
        "last_sample_age_s": data.get("last_sample_age_s"),
        "x_m": pose.get("x_m"),
        "y_m": pose.get("y_m"),
        "yaw_rad": pose.get("yaw_rad"),
        "vx_mps": twist.get("vx_mps"),
        "vy_mps": twist.get("vy_mps"),
        "omega_rad_s": twist.get("omega_rad_s"),
        "linear_speed_mps": data.get("linear_speed_mps"),
        "angular_speed_rad_s": data.get("angular_speed_rad_s"),
        "total_distance_m": data.get("total_distance_m"),
        "total_rotation_rad": data.get("total_rotation_rad"),
    }

    return format_component_block(
        name="Wheel Odometry",
        status=str(health.get("status", STATUS_UNKNOWN)),
        message=str(health.get("message", "")),
        ready=_optional_bool(data.get("ready")),
        usable=_status_usable(str(health.get("status", STATUS_UNKNOWN))),
        reasons=_string_list(health.get("reasons", [])),
        details=details,
    )


def format_ekf_report(payload: Mapping[str, Any] | Any) -> str:
    data = to_plain_dict(payload)

    details = {
        "output_topic": data.get("output_topic"),
        "expected_rate_hz": data.get("expected_rate_hz"),
        "sensor_timeout_s": data.get("sensor_timeout_s"),
        "active_inputs": ", ".join(_string_list(data.get("active_inputs", []))),
    }

    lines = [
        format_component_block(
            name="EKF",
            status=str(data.get("status", STATUS_UNKNOWN)),
            message=str(data.get("message", "")),
            ready=_optional_bool(data.get("ready")),
            usable=_optional_bool(data.get("usable")),
            reasons=_string_list(data.get("reasons", [])),
            details=details,
        )
    ]

    for input_name in ("wheel_odom", "imu", "vo_odom"):
        input_data = _mapping(data.get(input_name))
        if not input_data:
            continue

        lines.append(
            "  "
            + format_status_line(
                name=input_name,
                status=str(input_data.get("status", STATUS_UNKNOWN)),
                message=str(input_data.get("message", "")),
                ready=_optional_bool(input_data.get("ok")),
            )
        )

    output = _mapping(data.get("output"))
    if output:
        lines.append(
            "  "
            + format_status_line(
                name="output",
                status=str(output.get("status", STATUS_UNKNOWN)),
                message=str(output.get("message", "")),
                ready=_optional_bool(output.get("ok")),
            )
        )

    tf_data = _mapping(data.get("tf"))
    if tf_data:
        lines.append(
            "  "
            + format_status_line(
                name="tf",
                status=str(tf_data.get("status", STATUS_UNKNOWN)),
                message=str(tf_data.get("message", "")),
                ready=_optional_bool(tf_data.get("ok")),
            )
        )

    return "\n".join(lines)


def format_full_localization_report(payload: Mapping[str, Any] | Any) -> str:
    data = to_plain_dict(payload)

    blocks = [
        format_localization_summary(data),
    ]

    if "imu" in data:
        blocks.append(format_imu_report(data["imu"]))

    if "encoders" in data:
        blocks.append(format_encoder_report(data["encoders"]))

    if "wheel_odom" in data:
        blocks.append(format_wheel_odom_report(data["wheel_odom"]))

    if "ekf" in data:
        blocks.append(format_ekf_report(data["ekf"]))

    return "\n\n".join(blocks)


def format_markdown_report(
    payload: Mapping[str, Any] | Any,
    *,
    title: str = "Robot Savo Localization Report",
) -> str:
    data = to_plain_dict(payload)

    lines = [
        f"# {title}",
        "",
        "| Field | Value |",
        "|---|---|",
        f"| Package | `{data.get('package_name', PACKAGE_NAME)}` |",
        f"| Status | `{data.get('status', STATUS_UNKNOWN)}` |",
        f"| Message | {data.get('message', '')} |",
        f"| Ready | `{_bool_text(data.get('ready', False))}` |",
        f"| Usable | `{_bool_text(data.get('usable', False))}` |",
    ]

    reasons = _string_list(data.get("reasons", []))
    if reasons:
        lines.extend(["", "## Reasons", ""])
        lines.extend(f"- {reason}" for reason in reasons)

    components = _mapping(data.get("components"))
    if components:
        lines.extend(
            [
                "",
                "## Components",
                "",
                "| Component | Status | Ready | Usable | Message |",
                "|---|---|---|---|---|",
            ]
        )

        for name, component in components.items():
            if not isinstance(component, Mapping):
                continue

            lines.append(
                "| "
                f"`{name}` | "
                f"`{component.get('status', STATUS_UNKNOWN)}` | "
                f"`{_bool_text(component.get('ready', False))}` | "
                f"`{_bool_text(component.get('usable', False))}` | "
                f"{component.get('message', '')} |"
            )

    return "\n".join(lines)


def to_plain_dict(value: Any) -> Any:
    if value is None:
        return None

    if hasattr(value, "to_dict") and callable(value.to_dict):
        return to_plain_dict(value.to_dict())

    if is_dataclass(value):
        return to_plain_dict(asdict(value))

    if isinstance(value, Mapping):
        return {
            str(key): to_plain_dict(item)
            for key, item in value.items()
        }

    if isinstance(value, tuple):
        return [to_plain_dict(item) for item in value]

    if isinstance(value, list):
        return [to_plain_dict(item) for item in value]

    return value


def compact_status(status: str) -> str:
    return STATUS_LABELS.get(str(status), str(status))


def _mapping(value: Any) -> dict[str, Any]:
    value = to_plain_dict(value)

    if isinstance(value, Mapping):
        return dict(value)

    return {}


def _string_list(value: Any) -> list[str]:
    if value is None:
        return []

    if isinstance(value, str):
        return [value] if value else []

    if isinstance(value, Sequence):
        return [str(item) for item in value]

    return [str(value)]


def _optional_bool(value: Any) -> bool | None:
    if value is None:
        return None

    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "on"):
            return True
        if text in ("0", "false", "no", "off"):
            return False

    return bool(value)


def _status_usable(status: str) -> bool:
    return status in (STATUS_OK, STATUS_WARN)


def _bool_text(value: Any) -> str:
    return "true" if bool(value) else "false"


def _format_value(value: Any) -> str:
    value = to_plain_dict(value)

    if isinstance(value, bool):
        return _bool_text(value)

    if isinstance(value, float):
        return f"{value:.3f}"

    if isinstance(value, Mapping):
        return json.dumps(dict(value), separators=(",", ":"), sort_keys=True)

    if isinstance(value, list):
        if not value:
            return "none"

        return ", ".join(_format_value(item) for item in value)

    return str(value)