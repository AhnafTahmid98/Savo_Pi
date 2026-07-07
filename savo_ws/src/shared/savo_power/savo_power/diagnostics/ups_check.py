"""UPS HAT diagnostic for Robot Savo."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass

from savo_power import constants as c
from savo_power.drivers.smbus_adapter import (
    I2cBusError,
    SmbusAdapter,
    format_i2c_address,
    make_linux_i2c_device_path,
)
from savo_power.drivers.ups_hat import (
    UpsHatConfig,
    UpsHatDriver,
)
from savo_power.models.power_status import (
    BatterySource,
    normalize_battery_source,
)
from savo_power.models.ups_reading import UpsReading


@dataclass(frozen=True)
class UpsCheckSummary:
    """Summary for one UPS HAT diagnostic run."""

    source: BatterySource
    bus_id: int
    address: int
    readings: tuple[UpsReading, ...]

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
            "source": self.source.value,
            "bus_id": self.bus_id,
            "device_path": self.device_path,
            "address": self.address,
            "address_text": self.address_text,
            "sample_count": self.sample_count,
            "ok": self.ok,
            "error_count": self.error_count,
            "readings": [reading.to_dict() for reading in self.readings],
        }

    def format_text(self) -> str:
        lines = [
            (
                "Robot Savo UPS HAT check: "
                f"source={self.source.value} "
                f"bus={self.device_path} "
                f"address={self.address_text}"
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


def read_ups_samples(
    driver: UpsHatDriver,
    *,
    samples: int = 1,
    interval_s: float = 0.25,
) -> tuple[UpsReading, ...]:
    """Read one or more UPS HAT samples."""

    sample_count = max(1, int(samples))
    delay = max(0.0, float(interval_s))

    readings: list[UpsReading] = []

    for index in range(sample_count):
        readings.append(driver.read())

        if index + 1 < sample_count and delay > 0.0:
            time.sleep(delay)

    return tuple(readings)


def run_ups_check(
    bus: object,
    *,
    source: str | BatterySource,
    bus_id: int = c.DEFAULT_I2C_BUS,
    address: int = c.UPS_HAT_DEFAULT_ADDRESS,
    samples: int = 1,
    interval_s: float = 0.25,
) -> UpsCheckSummary:
    """Run UPS HAT diagnostic using an already-open bus-like object."""

    normalized_source = normalize_battery_source(source)

    if normalized_source not in {
        BatterySource.CORE_UPS,
        BatterySource.EDGE_UPS,
    }:
        raise ValueError("UPS source must be core_ups or edge_ups")

    driver = UpsHatDriver(
        bus=bus,
        config=UpsHatConfig(
            source=normalized_source,
            address=address,
        ),
    )

    return UpsCheckSummary(
        source=normalized_source,
        bus_id=int(bus_id),
        address=int(address),
        readings=read_ups_samples(
            driver,
            samples=samples,
            interval_s=interval_s,
        ),
    )


def make_arg_parser() -> argparse.ArgumentParser:
    """Create CLI parser."""

    parser = argparse.ArgumentParser(
        description="Check one Robot Savo UPS HAT.",
    )

    parser.add_argument(
        "--source",
        choices=(c.CORE_UPS_SOURCE, c.EDGE_UPS_SOURCE),
        default=c.CORE_UPS_SOURCE,
        help="UPS source label. Use core_ups on core Pi, edge_ups on edge Pi.",
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
        default=c.UPS_HAT_DEFAULT_ADDRESS,
        help="UPS HAT I2C address. Default: 0x36",
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
        summary = run_ups_check(
            bus,
            source=args.source,
            bus_id=args.bus,
            address=args.address,
            samples=args.samples,
            interval_s=args.interval,
        )
    except (I2cBusError, ValueError) as exc:
        print(f"Robot Savo UPS HAT check failed: {exc}", file=sys.stderr)
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
    "UpsCheckSummary",
    "main",
    "make_arg_parser",
    "read_ups_samples",
    "run_ups_check",
]
