"""I2C power device diagnostic for Robot Savo."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from typing import Protocol

from savo_power import constants as c
from savo_power.drivers.smbus_adapter import (
    I2cBusError,
    SmbusAdapter,
    checked_i2c_address,
    format_i2c_address,
    make_linux_i2c_device_path,
)
from savo_power.models.kit_battery_reading import (
    ads7830_battery_voltage_from_byte,
    ads7830_channel_command,
    normalize_pcb_version,
)
from savo_power.models.ups_reading import (
    ups_capacity_from_raw_word,
    ups_voltage_from_raw_word,
)


class PowerI2cBus(Protocol):
    """Small protocol used by this diagnostic."""

    def read_word_data(self, address: int, register: int) -> int:
        """Read SMBus word."""

    def write_byte(self, address: int, value: int) -> None:
        """Write SMBus byte."""

    def read_byte(self, address: int) -> int:
        """Read SMBus byte."""


@dataclass(frozen=True)
class I2cDeviceCheck:
    """Result for one I2C device check."""

    name: str
    address: int
    present: bool
    ok: bool
    detail: str = ""

    @property
    def address_text(self) -> str:
        return format_i2c_address(self.address)

    @property
    def status_text(self) -> str:
        if self.ok:
            return "OK"

        if self.present:
            return "ERROR"

        return "MISSING"

    def to_dict(self) -> dict[str, object]:
        data = asdict(self)
        data["address_text"] = self.address_text
        data["status"] = self.status_text
        return data

    def format_line(self) -> str:
        suffix = f" — {self.detail}" if self.detail else ""
        return f"[{self.status_text}] {self.name} {self.address_text}{suffix}"


@dataclass(frozen=True)
class I2cPowerCheckSummary:
    """Summary for local power I2C checks."""

    bus_id: int
    device_path: str
    checks: tuple[I2cDeviceCheck, ...]

    @property
    def ok(self) -> bool:
        return all(check.ok for check in self.checks)

    @property
    def missing_count(self) -> int:
        return sum(1 for check in self.checks if not check.present)

    @property
    def error_count(self) -> int:
        return sum(1 for check in self.checks if check.present and not check.ok)

    def to_dict(self) -> dict[str, object]:
        return {
            "bus_id": self.bus_id,
            "device_path": self.device_path,
            "ok": self.ok,
            "missing_count": self.missing_count,
            "error_count": self.error_count,
            "checks": [check.to_dict() for check in self.checks],
        }

    def format_text(self) -> str:
        lines = [
            f"Robot Savo power I2C check: {self.device_path}",
        ]

        for check in self.checks:
            lines.append(check.format_line())

        if self.ok:
            lines.append("Result: PASS")
        else:
            lines.append(
                f"Result: FAIL missing={self.missing_count} errors={self.error_count}"
            )

        return "\n".join(lines)


def check_ups_hat(
    bus: PowerI2cBus,
    *,
    address: int = c.UPS_HAT_DEFAULT_ADDRESS,
) -> I2cDeviceCheck:
    """Check UPS HAT by reading voltage and capacity registers."""

    address = checked_i2c_address(address)

    try:
        raw_voltage = bus.read_word_data(
            address,
            c.UPS_HAT_VOLTAGE_REGISTER,
        )
        raw_capacity = bus.read_word_data(
            address,
            c.UPS_HAT_CAPACITY_REGISTER,
        )

        voltage_v = ups_voltage_from_raw_word(raw_voltage)
        capacity_pct = ups_capacity_from_raw_word(raw_capacity)

        return I2cDeviceCheck(
            name="UPS HAT",
            address=address,
            present=True,
            ok=True,
            detail=(
                f"voltage={voltage_v:.2f} V "
                f"capacity={capacity_pct:.1f}% "
                f"raw_voltage=0x{raw_voltage & 0xFFFF:04X} "
                f"raw_capacity=0x{raw_capacity & 0xFFFF:04X}"
            ),
        )
    except Exception as exc:  # noqa: BLE001 - diagnostic should preserve root error
        return I2cDeviceCheck(
            name="UPS HAT",
            address=address,
            present=False,
            ok=False,
            detail=str(exc),
        )


def check_ads7830(
    bus: PowerI2cBus,
    *,
    address: int = c.ADS7830_DEFAULT_ADDRESS,
    channel: int = c.ADS7830_DEFAULT_CHANNEL,
    pcb_version: str = c.ADS7830_DEFAULT_PCB_VERSION,
) -> I2cDeviceCheck:
    """Check ADS7830 by writing channel command and reading one byte."""

    address = checked_i2c_address(address)
    pcb_version = normalize_pcb_version(pcb_version)

    try:
        command = ads7830_channel_command(channel)
        bus.write_byte(address, command)
        raw_byte = int(bus.read_byte(address)) & 0xFF

        voltage_v = ads7830_battery_voltage_from_byte(
            raw_byte,
            pcb_version,
        )

        return I2cDeviceCheck(
            name="ADS7830 base battery ADC",
            address=address,
            present=True,
            ok=True,
            detail=(
                f"channel={channel} command=0x{command:02X} "
                f"raw={raw_byte} estimated_battery={voltage_v:.2f} V "
                f"pcb={pcb_version}"
            ),
        )
    except Exception as exc:  # noqa: BLE001
        return I2cDeviceCheck(
            name="ADS7830 base battery ADC",
            address=address,
            present=False,
            ok=False,
            detail=str(exc),
        )


def run_power_i2c_check(
    bus: PowerI2cBus,
    *,
    bus_id: int = c.DEFAULT_I2C_BUS,
    ups_address: int = c.UPS_HAT_DEFAULT_ADDRESS,
    ads7830_address: int = c.ADS7830_DEFAULT_ADDRESS,
    ads7830_channel: int = c.ADS7830_DEFAULT_CHANNEL,
    pcb_version: str = c.ADS7830_DEFAULT_PCB_VERSION,
) -> I2cPowerCheckSummary:
    """Run all local power I2C checks."""

    checks = (
        check_ups_hat(
            bus,
            address=ups_address,
        ),
        check_ads7830(
            bus,
            address=ads7830_address,
            channel=ads7830_channel,
            pcb_version=pcb_version,
        ),
    )

    return I2cPowerCheckSummary(
        bus_id=int(bus_id),
        device_path=make_linux_i2c_device_path(bus_id),
        checks=checks,
    )


def make_arg_parser() -> argparse.ArgumentParser:
    """Create CLI parser."""

    parser = argparse.ArgumentParser(
        description="Check Robot Savo local power I2C devices.",
    )

    parser.add_argument(
        "--bus",
        type=int,
        default=c.DEFAULT_I2C_BUS,
        help="Linux I2C bus id. Default: 1",
    )
    parser.add_argument(
        "--ups-address",
        type=lambda value: int(value, 0),
        default=c.UPS_HAT_DEFAULT_ADDRESS,
        help="UPS HAT I2C address. Default: 0x36",
    )
    parser.add_argument(
        "--ads7830-address",
        type=lambda value: int(value, 0),
        default=c.ADS7830_DEFAULT_ADDRESS,
        help="ADS7830 I2C address. Default: 0x48",
    )
    parser.add_argument(
        "--ads7830-channel",
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
        summary = run_power_i2c_check(
            bus,
            bus_id=args.bus,
            ups_address=args.ups_address,
            ads7830_address=args.ads7830_address,
            ads7830_channel=args.ads7830_channel,
            pcb_version=args.pcb_version,
        )
    except I2cBusError as exc:
        print(f"Robot Savo power I2C check failed: {exc}", file=sys.stderr)
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
    "I2cDeviceCheck",
    "I2cPowerCheckSummary",
    "check_ads7830",
    "check_ups_hat",
    "main",
    "make_arg_parser",
    "run_power_i2c_check",
]
