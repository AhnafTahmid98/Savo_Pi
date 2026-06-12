"""Small log formatting helpers for Robot Savo LiDAR nodes."""

from __future__ import annotations

from typing import Any


def kv(**items: Any) -> str:
    parts: list[str] = []

    for key, value in items.items():
        if value is None:
            continue
        parts.append(f"{key}={value}")

    return " | ".join(parts)


def node_start_message(node_name: str, **items: Any) -> str:
    details = kv(**items)

    if details:
        return f"{node_name} started | {details}"

    return f"{node_name} started"


def node_stop_message(node_name: str, reason: str = "shutdown", **items: Any) -> str:
    details = kv(reason=reason, **items)

    if details:
        return f"{node_name} stopped | {details}"

    return f"{node_name} stopped"


def hardware_message(device: str, **items: Any) -> str:
    details = kv(**items)

    if details:
        return f"{device} | {details}"

    return str(device)


def status_message(status: str, **items: Any) -> str:
    details = kv(**items)

    if details:
        return f"status={status} | {details}"

    return f"status={status}"


def fault_message(fault: str, **items: Any) -> str:
    details = kv(**items)

    if details:
        return f"fault={fault} | {details}"

    return f"fault={fault}"