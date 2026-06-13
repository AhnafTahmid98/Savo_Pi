# -*- coding: utf-8 -*-
"""Small log-message helpers."""

from __future__ import annotations

from typing import Any


def kv(**items: Any) -> str:
    parts: list[str] = []

    for key, value in items.items():
        if value is None:
            continue

        parts.append(f"{key}={value}")

    return " ".join(parts)


def status_message(component: str, status: str, message: str = "", **items: Any) -> str:
    base = f"[{component}] {status}"

    if message:
        base = f"{base}: {message}"

    extra = kv(**items)
    if extra:
        return f"{base} | {extra}"

    return base


def node_start_message(node_name: str, **items: Any) -> str:
    return status_message(node_name, "START", "node started", **items)


def node_stop_message(node_name: str, **items: Any) -> str:
    return status_message(node_name, "STOP", "node stopped", **items)


def hardware_message(component: str, message: str, **items: Any) -> str:
    return status_message(component, "HW", message, **items)


def fault_message(component: str, message: str, **items: Any) -> str:
    return status_message(component, "FAULT", message, **items)


__all__ = [
    "fault_message",
    "hardware_message",
    "kv",
    "node_start_message",
    "node_stop_message",
    "status_message",
]
