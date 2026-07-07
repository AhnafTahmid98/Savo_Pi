"""Python ADS7830 diagnostic driver for Robot Savo base battery."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from savo_power import constants as c
from savo_power.drivers.smbus_adapter import (
    I2cBusError,
    SmbusAdapter,
    checked_i2c_address,
    format_i2c_address,
)
from savo_power.models.kit_battery_reading import (
    KitBatteryReading,
    ads7830_adc_voltage_from_byte,
    ads7830_battery_voltage_from_byte,
    ads7830_channel_command,
    estimate_linear_soc_pct,
    is_valid_ads7830_channel,
    make_kit_battery_error,
    make_kit_battery_reading,
    normalize_pcb_version,
)


class ByteReadableWritableBus(Protocol):
    """Small protocol needed by the ADS7830 driver."""

    def write_byte(self, address: int, value: int) -> None:
        """Write one byte to an I2C address."""

    def read_byte(self, address: int) -> int:
        """Read one byte from an I2C address."""


@dataclass(frozen=True)
class Ads7830Config:
    """Configuration for one ADS7830 battery reader."""

    address: int = c.ADS7830_DEFAULT_ADDRESS
    channel: int = c.ADS7830_DEFAULT_CHANNEL
    pcb_version: str = c.ADS7830_DEFAULT_PCB_VERSION

    @property
    def address_text(self) -> str:
        return format_i2c_address(self.address)

    @property
    def command(self) -> int:
        return ads7830_channel_command(self.channel)


class Ads7830Driver:
    """Read the Freenove/base battery through ADS7830."""

    def __init__(
        self,
        bus: ByteReadableWritableBus,
        config: Ads7830Config | None = None,
    ) -> None:
        config = config or Ads7830Config()

        if not is_valid_ads7830_channel(config.channel):
            raise ValueError("ADS7830 channel must be in range 0..7")

        self._bus = bus
        self._config = Ads7830Config(
            address=checked_i2c_address(config.address),
            channel=int(config.channel),
            pcb_version=normalize_pcb_version(config.pcb_version),
        )

    @property
    def config(self) -> Ads7830Config:
        return self._config

    @property
    def address(self) -> int:
        return self._config.address

    @property
    def address_text(self) -> str:
        return self._config.address_text

    @property
    def channel(self) -> int:
        return self._config.channel

    @property
    def pcb_version(self) -> str:
        return self._config.pcb_version

    @property
    def command(self) -> int:
        return self._config.command

    def read_raw_byte(self) -> int:
        """Write channel command and read one raw byte."""

        self._bus.write_byte(self._config.address, self.command)
        return int(self._bus.read_byte(self._config.address)) & 0xFF

    def read_stable_byte(self) -> int:
        """Discard first ADS7830 sample and return the next sample."""

        self.read_raw_byte()
        return self.read_raw_byte()

    def read_adc_voltage_v(self) -> float:
        """Read and convert ADS7830 ADC input voltage."""

        return ads7830_adc_voltage_from_byte(
            self.read_stable_byte(),
            self._config.pcb_version,
        )

    def read_battery_voltage_v(self) -> float:
        """Read and convert estimated battery voltage."""

        return ads7830_battery_voltage_from_byte(
            self.read_stable_byte(),
            self._config.pcb_version,
        )

    def estimate_soc_pct(self, voltage_v: float) -> float:
        """Estimate battery state-of-charge."""

        return estimate_linear_soc_pct(voltage_v)

    def read(self) -> KitBatteryReading:
        """Read ADS7830 and return normalized base battery reading."""

        try:
            raw_byte = self.read_stable_byte()

            adc_voltage_v = ads7830_adc_voltage_from_byte(
                raw_byte,
                self._config.pcb_version,
            )
            battery_voltage_v = ads7830_battery_voltage_from_byte(
                raw_byte,
                self._config.pcb_version,
            )
            soc_pct = self.estimate_soc_pct(battery_voltage_v)

            return make_kit_battery_reading(
                voltage_v=battery_voltage_v,
                soc_pct=soc_pct,
                adc_voltage_v=adc_voltage_v,
                raw_byte=raw_byte,
                channel=self._config.channel,
                pcb_version=self._config.pcb_version,
            )
        except Exception as exc:  # noqa: BLE001 - diagnostic driver should preserve root error
            return make_kit_battery_error(
                str(exc),
                channel=self._config.channel,
                pcb_version=self._config.pcb_version,
            )


def make_ads7830_driver(
    *,
    bus_id: int = c.DEFAULT_I2C_BUS,
    address: int = c.ADS7830_DEFAULT_ADDRESS,
    channel: int = c.ADS7830_DEFAULT_CHANNEL,
    pcb_version: str = c.ADS7830_DEFAULT_PCB_VERSION,
) -> Ads7830Driver:
    """Create an ADS7830 driver using a real SmbusAdapter."""

    bus = SmbusAdapter(bus_id)

    return Ads7830Driver(
        bus=bus,
        config=Ads7830Config(
            address=address,
            channel=channel,
            pcb_version=pcb_version,
        ),
    )


def read_kit_battery_once(
    *,
    bus_id: int = c.DEFAULT_I2C_BUS,
    address: int = c.ADS7830_DEFAULT_ADDRESS,
    channel: int = c.ADS7830_DEFAULT_CHANNEL,
    pcb_version: str = c.ADS7830_DEFAULT_PCB_VERSION,
) -> KitBatteryReading:
    """Open SMBus, read one ADS7830/base battery sample, and close the bus."""

    bus = SmbusAdapter(bus_id)

    try:
        driver = Ads7830Driver(
            bus=bus,
            config=Ads7830Config(
                address=address,
                channel=channel,
                pcb_version=pcb_version,
            ),
        )
        return driver.read()
    except I2cBusError as exc:
        return make_kit_battery_error(
            str(exc),
            channel=channel,
            pcb_version=pcb_version,
        )
    finally:
        bus.close()


__all__ = [
    "Ads7830Config",
    "Ads7830Driver",
    "ByteReadableWritableBus",
    "make_ads7830_driver",
    "read_kit_battery_once",
]
