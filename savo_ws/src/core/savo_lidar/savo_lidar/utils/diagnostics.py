# -*- coding: utf-8 -*-
"""JSON payload helpers for LiDAR status and health output."""

from __future__ import annotations

import json
import time
from typing import Any, Mapping

from savo_lidar.constants import PACKAGE_NAME, ROBOT_NAME


def make_status_payload(
    *,
    component: str,
    status: str,
    message: str = "",
    **items: Any,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "robot": ROBOT_NAME,
        "package": PACKAGE_NAME,
        "component": str(component),
        "status": str(status),
        "message": str(message),
        "stamp_s": time.time(),
    }

    payload.update({key: value for key, value in items.items() if value is not None})
    return payload


def make_health_payload(
    *,
    component: str,
    ok: bool,
    status: str,
    message: str = "",
    **items: Any,
) -> dict[str, Any]:
    payload = make_status_payload(
        component=component,
        status=status,
        message=message,
        ok=bool(ok),
    )

    payload.update({key: value for key, value in items.items() if value is not None})
    return payload


def payload_to_json(payload: Mapping[str, Any]) -> str:
    return json.dumps(_json_safe(dict(payload)), sort_keys=True)


def json_status(
    *,
    component: str,
    status: str,
    message: str = "",
    **items: Any,
) -> str:
    return payload_to_json(
        make_status_payload(
            component=component,
            status=status,
            message=message,
            **items,
        )
    )


def json_health(
    *,
    component: str,
    ok: bool,
    status: str,
    message: str = "",
    **items: Any,
) -> str:
    return payload_to_json(
        make_health_payload(
            component=component,
            ok=ok,
            status=status,
            message=message,
            **items,
        )
    )


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}

    if isinstance(value, (list, tuple, set)):
        return [_json_safe(item) for item in value]

    if hasattr(value, "to_dict") and callable(value.to_dict):
        return _json_safe(value.to_dict())

    return value


__all__ = [
    "json_health",
    "json_status",
    "make_health_payload",
    "make_status_payload",
    "payload_to_json",
]
