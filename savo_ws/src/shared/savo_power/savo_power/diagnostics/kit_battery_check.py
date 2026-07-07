"""Freenove/base battery diagnostic for Robot Savo."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass

from savo_power import constants as c
from savo_power.drivers.ads7830 import (
    Ads7830Config,
    Ads7830Driver,
)
from savo_power.drivers.smbus_adapter import (
    I2cBusError,
    SmbusAdapter,
    format_i2c_address,
    make_linux_i2c_device_path,
)
from savo_power.models.kit_battery_reading import (
    KitBatteryReading,
    normalize_pcb_version,
)


@dataclass(frozen=True)
class KitBatteryCheckSummary:
    """Summary for one Freenove/base battery diagnostic run."""

    bus_id: int
    address: int
    channel: int
    pcb_version: str
    readings: tuple[KitBatteryReading, ...]

    @property
    def device_path(self) -> str:
        return make_linux_i2c_device_path(self.bus_id)

    @property
    def address_text(self) -> str:
        return format_i2c_address(self.address)

    @property
    def sample_count(self) -> int:
        return len(self.readings)

    @property
    def ok(self) -> bool:
        return bool(self.readings) and all(reading.ok for reading in self.readings)

    @property
    def error_count(self) -> int:
        return sum(1 for reading in self.readings if not reading.ok)

    def to_dict(self) -> dict[str, object]:
        return {
            "source": c.BASE_BATTERY_SOURCE,
            "bus_id": self.bus_id,
            "device_path": self.device_path,
            "address": self.address,
            "address_text": self.address_text,
            "channel": self.channel,
            "pcb_version": self.pcb_version,
            "sample_count": self.sample_count,
            "ok": self.ok,
            "error_count": self.error_count,
            "readings": [reading.to_dict() for reading in self.readings],
        }

    def format_text(self) -> str:
        lines = [
            (
                "Robot Savo kit/base battery check: "
                f"bus={self.device_path} "
                f"address={self.address_text} "
                f"channel={self.channel} "
                f"pcb={self.pcb_version}"
            )
        ]

        for index, reading in enumerate(self.readings, start=1):
            lines.append(f"[{index}] {reading.format_line()}")

        if self.ok:
            lines.append("Result: PASS")
        else:
            lines.append(
                f"Result: FAIL samples={self.sample_count} errors={self.error_count}"
            )

        return "\n".join(lines)


def read_kit_battery_samples(
    driver: Ads7830Driver,
    *,
    samples: int = 1,
    interval_s: float = 0.25,
) -> tuple[KitBatteryReading, ...]:
    """Read one or more ADS7830/base battery samples."""

    sample_count = max(1, int(samples))
    delay = max(0.0, float(interval_s))

    readings: list[KitBatteryReading] = []

    for index in range(sample_count):
        readings.append(driver.read())

        if index + 1 < sample_count and delay > 0.0:
            time.sleep(delay)

    return tuple(readings)


def run_kit_battery_check(
    bus: object,
    *,
    bus_id: int = c.DEFAULT_I2C_BUS,
    address: int = c.ADS7830_DEFAULT_ADDRESS,
    channel: int = c.ADS7830_DEFAULT_CHANNEL,
    pcb_version: str = c.ADS7830_DEFAULT_PCB_VERSION,
    samples: int = 1,
    interval_s: float = 0.25,
) -> KitBatteryCheckSummary:
    """Run Freenove/base battery diagnostic using an already-open bus-like object."""

    normalized_pcb = normalize_pcb_version(pcb_version)

    driver = Ads7830Driver(
        bus=bus,
        config=Ads7830Config(
            address=address,
            channel=channel,
            pcb_version=normalized_pcb,
        ),
    )

    return KitBatteryCheckSummary(
        bus_id=int(bus_id),
        address=int(address),
        channel=int(channel),
        pcb_version=normalized_pcb,
        readings=read_kit_battery_samples(
            driver,
            samples=samples,
            interval_s=interval_s,
        ),
    )


def make_arg_parser() -> argparse.ArgumentParser:
    """Create CLI parser."""

    parser = argparse.ArgumentParser(
        description="Check Robot Savo Freenove/base battery through ADS7830.",
    )

    parser.add_argument(
        "--bus",
        type=int,
        default=c.DEFAULT_I2C_BUS,
        help="Linux I2C bus id. Default: 1",
    )
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=c.ADS7830_DEFAULT_ADDRESS,
        help="ADS7830 I2C address. Default: 0x48",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=c.ADS7830_DEFAULT_CHANNEL,
        help="ADS7830 battery channel. Default: 2",
    )
    parser.add_argument(
        "--pcb-version",
        default=c.ADS7830_DEFAULT_PCB_VERSION,
        help="Freenove battery ADC PCB version: v1 or v2. Default: v2",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=1,
        help="Number of samples to read. Default: 1",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.25,
        help="Delay between samples in seconds. Default: 0.25",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print JSON instead of text.",
    )

    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint."""

    parser = make_arg_parser()
    args = parser.parse_args(argv)

    bus = SmbusAdapter(args.bus)

    try:
        summary = run_kit_battery_check(
            bus,
            bus_id=args.bus,
            address=args.address,
            channel=args.channel,
            pcb_version=args.pcb_version,
            samples=args.samples,
            interval_s=args.interval,
        )
    except (I2cBusError, ValueError) as exc:
        print(f"Robot Savo kit/base battery check failed: {exc}", file=sys.stderr)
        return 2
    finally:
        bus.close()

    if args.json:
        print(json.dumps(summary.to_dict(), indent=2, sort_keys=True))
    else:
        print(summary.format_text())

    return 0 if summary.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "KitBatteryCheckSummary",
    "main",
    "make_arg_parser",
    "read_kit_battery_samples",
    "run_kit_battery_check",
]
