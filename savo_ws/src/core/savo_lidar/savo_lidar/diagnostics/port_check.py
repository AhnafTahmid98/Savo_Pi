# -*- coding: utf-8 -*-
"""Serial-port check for the RPLIDAR A1 connection."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_lidar.constants import DEFAULT_SERIAL_PORT
from savo_lidar.drivers.serial_port import (
    available_serial_ports,
    inspect_serial_port,
)
from savo_lidar.utils.process_utils import user_in_group


@dataclass(frozen=True)
class PortCheckResult:
    preferred_port: str
    available_ports: list[str]
    selected_port: str | None
    preferred_exists: bool
    preferred_usable: bool
    user_in_dialout: bool
    ok: bool
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def check_lidar_port(preferred_port: str = DEFAULT_SERIAL_PORT) -> PortCheckResult:
    preferred_port = str(preferred_port).strip() or DEFAULT_SERIAL_PORT

    ports = available_serial_ports()
    preferred_info = inspect_serial_port(preferred_port)
    in_dialout = user_in_group("dialout")

    selected_port: str | None = None
    if preferred_info.usable:
        selected_port = preferred_port
    elif ports:
        selected_port = ports[0]

    if not ports:
        return PortCheckResult(
            preferred_port=preferred_port,
            available_ports=[],
            selected_port=None,
            preferred_exists=preferred_info.exists,
            preferred_usable=preferred_info.usable,
            user_in_dialout=in_dialout,
            ok=False,
            message="no serial ports found",
        )

    if not in_dialout:
        return PortCheckResult(
            preferred_port=preferred_port,
            available_ports=ports,
            selected_port=selected_port,
            preferred_exists=preferred_info.exists,
            preferred_usable=preferred_info.usable,
            user_in_dialout=False,
            ok=False,
            message="user is not in dialout group",
        )

    if preferred_info.usable:
        return PortCheckResult(
            preferred_port=preferred_port,
            available_ports=ports,
            selected_port=preferred_port,
            preferred_exists=True,
            preferred_usable=True,
            user_in_dialout=True,
            ok=True,
            message="preferred LiDAR serial port is usable",
        )

    return PortCheckResult(
        preferred_port=preferred_port,
        available_ports=ports,
        selected_port=selected_port,
        preferred_exists=preferred_info.exists,
        preferred_usable=preferred_info.usable,
        user_in_dialout=True,
        ok=True,
        message="preferred port not usable, fallback serial port available",
    )


__all__ = [
    "PortCheckResult",
    "check_lidar_port",
]
