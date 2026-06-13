#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Small logging-format helpers. No ROS imports."""

from __future__ import annotations

import json
from collections.abc import Mapping
from typing import Any


def format_kv(**items: Any) -> str:
    parts: list[str] = []

    for key, value in items.items():
        if value is None:
            continue

        parts.append(f"{key}={_format_value(value)}")

    return " | ".join(parts)


def node_start_message(node_name: str, **items: Any) -> str:
    details = format_kv(**items)

    if details:
        return f"{node_name} started | {details}"

    return f"{node_name} started"


def node_stop_message(node_name: str, **items: Any) -> str:
    details = format_kv(**items)

    if details:
        return f"{node_name} stopped | {details}"

    return f"{node_name} stopped"


def node_ready_message(node_name: str, **items: Any) -> str:
    details = format_kv(**items)

    if details:
        return f"{node_name} ready | {details}"

    return f"{node_name} ready"


def status_line(
    *,
    name: str,
    status: str,
    message: str = "",
    **items: Any,
) -> str:
    parts = [f"{name}: {status}"]

    if message:
        parts.append(str(message))

    details = format_kv(**items)
    if details:
        parts.append(details)

    return " | ".join(parts)


def format_json(
    payload: Mapping[str, Any],
    *,
    compact: bool = True,
    sort_keys: bool = True,
) -> str:
    separators = (",", ":") if compact else None

    return json.dumps(
        dict(payload),
        separators=separators,
        sort_keys=sort_keys,
    )


def safe_str(value: Any, default: str = "") -> str:
    if value is None:
        return default

    try:
        return str(value)
    except Exception:
        return default


def _format_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"

    if isinstance(value, float):
        return f"{value:.3f}"

    if isinstance(value, (list, tuple, set)):
        values = list(value)
        if not values:
            return "none"

        return ",".join(_format_value(item) for item in values)

    if isinstance(value, dict):
        return format_json(value, compact=True, sort_keys=True)

    return safe_str(value)
