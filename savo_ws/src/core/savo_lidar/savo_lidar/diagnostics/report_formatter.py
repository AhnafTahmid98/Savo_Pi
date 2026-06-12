"""Formatting utilities for LiDAR diagnostic output."""

from __future__ import annotations

import json
from collections.abc import Mapping
from typing import Any


def format_bool(value: bool) -> str:
    return "yes" if bool(value) else "no"


def format_optional_float(value: float | None, *, digits: int = 3, suffix: str = "") -> str:
    if value is None:
        return "n/a"

    return f"{float(value):.{digits}f}{suffix}"


def format_key_value_report(title: str, values: Mapping[str, Any]) -> str:
    lines = [str(title), "-" * len(str(title))]

    for key, value in values.items():
        lines.append(f"{key}: {_format_value(value)}")

    return "\n".join(lines)


def format_json_report(values: Mapping[str, Any]) -> str:
    return json.dumps(dict(values), indent=2, sort_keys=True)


def compact_status_line(
    *,
    name: str,
    ok: bool,
    message: str,
    **items: Any,
) -> str:
    status = "OK" if ok else "FAIL"
    parts = [f"{name}: {status}", str(message)]

    for key, value in items.items():
        if value is None:
            continue
        parts.append(f"{key}={_format_value(value)}")

    return " | ".join(parts)


def _format_value(value: Any) -> str:
    if isinstance(value, bool):
        return format_bool(value)

    if isinstance(value, float):
        return f"{value:.3f}"

    if isinstance(value, (list, tuple)):
        return ", ".join(str(item) for item in value) if value else "none"

    if value is None:
        return "n/a"

    return str(value)