"""Serial-port helpers for RPLIDAR USB detection."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from savo_lidar.utils.process_utils import list_serial_devices


@dataclass(frozen=True)
class SerialPortInfo:
    path: str
    exists: bool
    readable: bool
    writable: bool

    @property
    def usable(self) -> bool:
        return self.exists and self.readable and self.writable

    def to_dict(self) -> dict[str, object]:
        return {
            "path": self.path,
            "exists": self.exists,
            "readable": self.readable,
            "writable": self.writable,
            "usable": self.usable,
        }


def inspect_serial_port(port: str) -> SerialPortInfo:
    path = Path(str(port)).expanduser()

    return SerialPortInfo(
        path=str(path),
        exists=path.exists(),
        readable=path.exists() and path.is_char_device(),
        writable=path.exists() and path.is_char_device(),
    )


def available_serial_ports() -> list[str]:
    return list_serial_devices()


def choose_serial_port(
    preferred_port: str,
    *,
    allow_fallback: bool = True,
) -> str | None:
    preferred = str(preferred_port).strip()

    if preferred and inspect_serial_port(preferred).usable:
        return preferred

    if not allow_fallback:
        return None

    ports = available_serial_ports()
    if not ports:
        return None

    return ports[0]


def require_serial_port(
    preferred_port: str,
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