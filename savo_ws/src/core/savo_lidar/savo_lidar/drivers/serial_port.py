# -*- coding: utf-8 -*-
"""Serial-port helpers for RPLIDAR USB detection."""

from __future__ import annotations

import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from savo_lidar.constants import DEFAULT_SERIAL_PORT
from savo_lidar.utils.process_utils import list_serial_devices


@dataclass(frozen=True)
class SerialPortInfo:
    path: str
    exists: bool
    is_char_device: bool
    readable: bool
    writable: bool

    @property
    def usable(self) -> bool:
        return self.exists and self.is_char_device and self.readable and self.writable

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["usable"] = self.usable
        return data


def inspect_serial_port(port: str = DEFAULT_SERIAL_PORT) -> SerialPortInfo:
    path = Path(str(port).strip() or DEFAULT_SERIAL_PORT).expanduser()
    exists = path.exists()
    is_char_device = exists and path.is_char_device()

    return SerialPortInfo(
        path=str(path),
        exists=exists,
        is_char_device=is_char_device,
        readable=exists and os.access(path, os.R_OK),
        writable=exists and os.access(path, os.W_OK),
    )


def available_serial_ports() -> list[str]:
    return list_serial_devices()


def choose_serial_port(
    preferred_port: str = DEFAULT_SERIAL_PORT,
    *,
    allow_fallback: bool = True,
) -> str | None:
    preferred = str(preferred_port).strip() or DEFAULT_SERIAL_PORT

    if inspect_serial_port(preferred).usable:
        return preferred

    if not allow_fallback:
        return None

    for port in available_serial_ports():
        if inspect_serial_port(port).usable:
            return port

    return None


def require_serial_port(
    preferred_port: str = DEFAULT_SERIAL_PORT,
    *,
    allow_fallback: bool = True,
) -> str:
    selected = choose_serial_port(
        preferred_port,
        allow_fallback=allow_fallback,
    )

    if selected is None:
        raise FileNotFoundError(
            f"No usable LiDAR serial port found. Preferred port: {preferred_port}"
        )

    return selected


__all__ = [
    "SerialPortInfo",
    "available_serial_ports",
    "choose_serial_port",
    "inspect_serial_port",
    "require_serial_port",
]
