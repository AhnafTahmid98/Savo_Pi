"""Python UPS HAT diagnostic driver for Robot Savo."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from savo_power import constants as c
from savo_power.drivers.smbus_adapter import (
    I2cBusError,
    SmbusAdapter,
    checked_i2c_address,
    checked_register,
    format_i2c_address,
)
from savo_power.models.power_status import (
    BatterySource,
    normalize_battery_source,
)
from savo_power.models.ups_reading import (
    UpsReading,
    is_ups_source,
    make_ups_error,
    make_ups_reading_from_raw,
    ups_capacity_from_raw_word,
    ups_voltage_from_raw_word,
)


class WordReadableBus(Protocol):
    """Small protocol needed by the UPS HAT driver."""

    def read_word_data(self, address: int, register: int) -> int:
        """Read one SMBus word from address/register."""


@dataclass(frozen=True)
class UpsHatConfig:
    """Configuration for one UPS HAT reader."""

    source: BatterySource
    address: int = c.UPS_HAT_DEFAULT_ADDRESS

    voltage_register: int = c.UPS_HAT_VOLTAGE_REGISTER
    capacity_register: int = c.UPS_HAT_CAPACITY_REGISTER

    @property
    def address_text(self) -> str:
        return format_i2c_address(self.address)


class UpsHatDriver:
    """Read one UPS HAT using a small SMBus-compatible bus object."""

    def __init__(
        self,
        bus: WordReadableBus,
        config: UpsHatConfig,
    ) -> None:
        source = normalize_battery_source(config.source)

        if not is_ups_source(source):
            raise ValueError("UPS HAT source must be core_ups or edge_ups")

        self._bus = bus
        self._config = UpsHatConfig(
            source=source,
            address=checked_i2c_address(config.address),
            voltage_register=checked_register(config.voltage_register),
            capacity_register=checked_register(config.capacity_register),
        )

    @property
    def config(self) -> UpsHatConfig:
        return self._config

    @property
    def source(self) -> BatterySource:
        return self._config.source

    @property
    def address(self) -> int:
        return self._config.address

    @property
    def address_text(self) -> str:
        return self._config.address_text

    def read_raw_voltage_word(self) -> int:
        """Read raw voltage word from UPS HAT."""

        return int(
            self._bus.read_word_data(
                self._config.address,
                self._config.voltage_register,
            )
        ) & 0xFFFF

    def read_raw_capacity_word(self) -> int:
        """Read raw capacity word from UPS HAT."""

        return int(
            self._bus.read_word_data(
                self._config.address,
                self._config.capacity_register,
            )
        ) & 0xFFFF

    def read_voltage_v(self) -> float:
        """Read and convert UPS voltage."""

        return ups_voltage_from_raw_word(self.read_raw_voltage_word())

    def read_capacity_pct(self) -> float:
        """Read and convert UPS capacity percentage."""

        return ups_capacity_from_raw_word(self.read_raw_capacity_word())

    def read(self) -> UpsReading:
        """Read UPS HAT and return normalized reading."""

        try:
            raw_voltage_word = self.read_raw_voltage_word()
            raw_capacity_word = self.read_raw_capacity_word()

            return make_ups_reading_from_raw(
                self._config.source,
                raw_voltage_word=raw_voltage_word,
                raw_capacity_word=raw_capacity_word,
            )
        except Exception as exc:  # noqa: BLE001 - diagnostic driver should preserve root error
            return make_ups_error(
                self._config.source,
                str(exc),
            )


def make_ups_hat_driver(
    *,
    source: str | BatterySource,
    bus_id: int = c.DEFAULT_I2C_BUS,
    address: int = c.UPS_HAT_DEFAULT_ADDRESS,
) -> UpsHatDriver:
    """Create a UPS HAT driver using a real SmbusAdapter."""

    bus = SmbusAdapter(bus_id)

    return UpsHatDriver(
        bus=bus,
        config=UpsHatConfig(
            source=normalize_battery_source(source),
            address=address,
        ),
    )


def make_core_ups_driver(
    *,
    bus_id: int = c.DEFAULT_I2C_BUS,
    address: int = c.UPS_HAT_DEFAULT_ADDRESS,
) -> UpsHatDriver:
    """Create a driver for the core Pi UPS HAT."""

    return make_ups_hat_driver(
        source=BatterySource.CORE_UPS,
        bus_id=bus_id,
        address=address,
    )


def make_edge_ups_driver(
    *,
    bus_id: int = c.DEFAULT_I2C_BUS,
    address: int = c.UPS_HAT_DEFAULT_ADDRESS,
) -> UpsHatDriver:
    """Create a driver for the edge Pi UPS HAT."""

    return make_ups_hat_driver(
        source=BatterySource.EDGE_UPS,
        bus_id=bus_id,
        address=address,
    )


def read_ups_once(
    *,
    source: str | BatterySource,
    bus_id: int = c.DEFAULT_I2C_BUS,
    address: int = c.UPS_HAT_DEFAULT_ADDRESS,
) -> UpsReading:
    """Open SMBus, read one UPS HAT sample, and close the bus."""

    bus = SmbusAdapter(bus_id)

    try:
        driver = UpsHatDriver(
            bus=bus,
            config=UpsHatConfig(
                source=normalize_battery_source(source),
                address=address,
            ),
        )
        return driver.read()
    except I2cBusError as exc:
        return make_ups_error(source, str(exc))
    finally:
        bus.close()


__all__ = [
    "UpsHatConfig",
    "UpsHatDriver",
    "WordReadableBus",
    "make_core_ups_driver",
    "make_edge_ups_driver",
    "make_ups_hat_driver",
    "read_ups_once",
]
