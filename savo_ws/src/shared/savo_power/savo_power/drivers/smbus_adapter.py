"""Small SMBus adapter for Robot Savo power diagnostics."""

from __future__ import annotations

import importlib
from typing import Any, Callable

from savo_power import constants as c


class I2cBusError(RuntimeError):
    """Raised when an I2C/SMBus operation fails."""


def make_linux_i2c_device_path(bus_id: int) -> str:
    """Return Linux I2C device path for a bus id."""

    return f"/dev/i2c-{int(bus_id)}"


def format_i2c_address(address: int) -> str:
    """Format an I2C address as hex text."""

    return f"0x{int(address) & 0x7F:02X}"


def is_valid_7bit_i2c_address(address: int) -> bool:
    """Return True when address fits inside 7-bit I2C range."""

    return 0 <= int(address) <= 0x7F


def is_reserved_7bit_i2c_address(address: int) -> bool:
    """Return True when address is inside the reserved 7-bit range."""

    value = int(address)
    return value < 0x08 or value > 0x77


def is_usable_7bit_i2c_address(address: int) -> bool:
    """Return True when address is valid and not reserved."""

    return (
        is_valid_7bit_i2c_address(address)
        and not is_reserved_7bit_i2c_address(address)
    )


def checked_i2c_address(address: int) -> int:
    """Validate and return a usable 7-bit I2C address."""

    value = int(address)

    if not is_valid_7bit_i2c_address(value):
        raise ValueError(f"I2C address out of 7-bit range: {value}")

    if is_reserved_7bit_i2c_address(value):
        raise ValueError(f"I2C address is reserved: {format_i2c_address(value)}")

    return value


def checked_register(register: int) -> int:
    """Validate and return an 8-bit register value."""

    value = int(register)

    if value < 0 or value > 0xFF:
        raise ValueError(f"I2C register must be 0..255, got {value}")

    return value


def checked_byte(value: int) -> int:
    """Validate and return an 8-bit byte value."""

    byte_value = int(value)

    if byte_value < 0 or byte_value > 0xFF:
        raise ValueError(f"I2C byte value must be 0..255, got {byte_value}")

    return byte_value


def load_smbus_backend_class() -> type[Any]:
    """Load SMBus backend class from smbus2 or smbus."""

    for module_name in ("smbus2", "smbus"):
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue

        backend_class = getattr(module, "SMBus", None)

        if backend_class is not None:
            return backend_class

    raise I2cBusError(
        "No SMBus backend found. Install python3-smbus or smbus2."
    )


class SmbusAdapter:
    """Minimal SMBus adapter used by Python diagnostics."""

    def __init__(
        self,
        bus_id: int = c.DEFAULT_I2C_BUS,
        *,
        backend_factory: Callable[[int], Any] | None = None,
        auto_open: bool = True,
    ) -> None:
        self._bus_id = int(bus_id)
        self._backend_factory = backend_factory
        self._backend: Any | None = None

        if auto_open:
            self.open()

    @property
    def bus_id(self) -> int:
        return self._bus_id

    @property
    def device_path(self) -> str:
        return make_linux_i2c_device_path(self._bus_id)

    @property
    def is_open(self) -> bool:
        return self._backend is not None

    def open(self) -> None:
        """Open the SMBus backend if needed."""

        if self._backend is not None:
            return

        try:
            backend_factory = self._backend_factory or load_smbus_backend_class()
            self._backend = backend_factory(self._bus_id)
        except Exception as exc:  # noqa: BLE001 - diagnostics should preserve root error
            raise I2cBusError(
                f"Failed to open {self.device_path}: {exc}"
            ) from exc

    def close(self) -> None:
        """Close the SMBus backend if it supports close()."""

        backend = self._backend
        self._backend = None

        if backend is None:
            return

        close = getattr(backend, "close", None)

        if callable(close):
            close()

    def __enter__(self) -> "SmbusAdapter":
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def _require_backend(self) -> Any:
        if self._backend is None:
            self.open()

        if self._backend is None:
            raise I2cBusError(f"{self.device_path} is not open")

        return self._backend

    def read_word_data(self, address: int, register: int) -> int:
        """Read a 16-bit SMBus word from device/register."""

        addr = checked_i2c_address(address)
        reg = checked_register(register)
        backend = self._require_backend()

        try:
            return int(backend.read_word_data(addr, reg)) & 0xFFFF
        except Exception as exc:  # noqa: BLE001
            raise I2cBusError(
                f"Failed read_word_data addr={format_i2c_address(addr)} "
                f"reg=0x{reg:02X}: {exc}"
            ) from exc

    def read_byte(self, address: int) -> int:
        """Read one byte from a device."""

        addr = checked_i2c_address(address)
        backend = self._require_backend()

        try:
            return int(backend.read_byte(addr)) & 0xFF
        except Exception as exc:  # noqa: BLE001
            raise I2cBusError(
                f"Failed read_byte addr={format_i2c_address(addr)}: {exc}"
            ) from exc

    def write_byte(self, address: int, value: int) -> None:
        """Write one byte to a device."""

        addr = checked_i2c_address(address)
        byte_value = checked_byte(value)
        backend = self._require_backend()

        try:
            backend.write_byte(addr, byte_value)
        except Exception as exc:  # noqa: BLE001
            raise I2cBusError(
                f"Failed write_byte addr={format_i2c_address(addr)} "
                f"value=0x{byte_value:02X}: {exc}"
            ) from exc

    def probe_address(self, address: int) -> bool:
        """Return True if read_byte succeeds for an address."""

        try:
            self.read_byte(address)
            return True
        except I2cBusError:
            return False

    def scan(
        self,
        *,
        start: int = 0x08,
        stop: int = 0x77,
    ) -> list[int]:
        """Scan usable I2C address range with read_byte probes."""

        found: list[int] = []

        for address in range(int(start), int(stop) + 1):
            if not is_usable_7bit_i2c_address(address):
                continue

            if self.probe_address(address):
                found.append(address)

        return found


__all__ = [
    "I2cBusError",
    "SmbusAdapter",
    "checked_byte",
    "checked_i2c_address",
    "checked_register",
    "format_i2c_address",
    "is_reserved_7bit_i2c_address",
    "is_usable_7bit_i2c_address",
    "is_valid_7bit_i2c_address",
    "load_smbus_backend_class",
    "make_linux_i2c_device_path",
]
