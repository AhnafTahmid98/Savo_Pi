"""Helpers for small JSON-friendly diagnostic payloads."""

from __future__ import annotations

import json
from typing import Any


def make_status_payload(
    *,
    node: str,
    status: str,
    message: str = "",
    **items: Any,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "node": str(node),
        "status": str(status),
        "message": str(message),
    }

    for key, value in items.items():
        if value is None:
            continue
        payload[str(key)] = value

    return payload


def make_health_payload(
    *,
    node: str,
    status: str,
    hardware_ok: bool,
    scan_ok: bool,
    message: str = "",
    **items: Any,
) -> dict[str, Any]:
    return make_status_payload(
        node=node,
        status=status,
        message=message,
        hardware_ok=bool(hardware_ok),
        scan_ok=bool(scan_ok),
        **items,
    )


def payload_to_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def json_status(
    *,
    node: str,
    status: str,
    message: str = "",
    **items: Any,
) -> str:
    return payload_to_json(
        make_status_payload(
            node=node,
            status=status,
            message=message,
            **items,
        )
    )


def json_health(
    *,
    node: str,
    status: str,
    hardware_ok: bool,
    scan_ok: bool,
    message: str = "",
    **items: Any,
) -> str:
    return payload_to_json(
        make_health_payload(
            node=node,
            status=status,
            hardware_ok=hardware_ok,
            scan_ok=scan_ok,
            message=message,
            **items,
        )
    )