#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Scan TCA9548A mux channels for VL53L1X sensors."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Optional

from savo_perception.constants import (
    I2C_BUS_DEFAULT,
    TCA9548A_ADDR_DEFAULT,
    VL53_LEFT_CHANNEL_DEFAULT,
    VL53_RIGHT_CHANNEL_DEFAULT,
    VL53L1X_ADDR_DEFAULT,
)
from savo_perception.drivers.vl53_mux_driver import import_smbus


@dataclass(frozen=True)
class ChannelScanResult:
    channel: int
    present: bool
    error: str = ""


class TcaScanner:
    def __init__(self, *, bus: int, tca_addr: int, vl53_addr: int, settle_s: float) -> None:
        SMBus = import_smbus()
        self.bus_num = int(bus)
        self.tca_addr = int(tca_addr)
        self.vl53_addr = int(vl53_addr)
        self.settle_s = float(settle_s)
        self.bus = SMBus(self.bus_num)

    def select_channel(self, channel: int) -> None:
        ch = int(channel)
        if not 0 <= ch <= 7:
            raise ValueError("TCA9548A channel must be 0..7")
        self.bus.write_byte(self.tca_addr, 1 << ch)
        time.sleep(self.settle_s)

    def disable_all(self) -> None:
        self.bus.write_byte(self.tca_addr, 0x00)

    def probe_vl53(self, channel: int) -> ChannelScanResult:
        try:
            self.select_channel(channel)
            self.bus.read_byte(self.vl53_addr)
            return ChannelScanResult(channel=channel, present=True)
        except Exception as exc:
            return ChannelScanResult(
                channel=channel,
                present=False,
                error=type(exc).__name__,
            )

    def close(self) -> None:
        try:
            self.disable_all()
        finally:
            close = getattr(self.bus, "close", None)
            if callable(close):
                close()


def expected_label(channel: int, *, right_ch: int, left_ch: int) -> str:
    if channel == right_ch:
        return "RIGHT expected"
    if channel == left_ch:
        return "LEFT expected"
    return "-"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Scan Robot Savo TCA9548A mux channels for VL53L1X sensors"
    )

    parser.add_argument("--bus", type=int, default=I2C_BUS_DEFAULT)
    parser.add_argument("--tca-addr", type=lambda s: int(s, 0), default=TCA9548A_ADDR_DEFAULT)
    parser.add_argument("--vl53-addr", type=lambda s: int(s, 0), default=VL53L1X_ADDR_DEFAULT)

    parser.add_argument("--right-ch", type=int, default=VL53_RIGHT_CHANNEL_DEFAULT)
    parser.add_argument("--left-ch", type=int, default=VL53_LEFT_CHANNEL_DEFAULT)

    parser.add_argument("--start-ch", type=int, default=0)
    parser.add_argument("--end-ch", type=int, default=7)
    parser.add_argument("--settle-s", type=float, default=0.01)

    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    start_ch = max(0, int(args.start_ch))
    end_ch = min(7, int(args.end_ch))

    if start_ch > end_ch:
        raise SystemExit("--start-ch must be <= --end-ch")

    scanner: Optional[TcaScanner] = None

    print(f"[TCA9548A] bus=i2c-{args.bus} address=0x{args.tca_addr:02X}")
    print(f"[VL53L1X] probe address=0x{args.vl53_addr:02X}")
    print(f"[Expected] RIGHT=channel {args.right_ch}  LEFT=channel {args.left_ch}")
    print()
    print(" channel | present | expected       | note")
    print("-" * 52)

    try:
        scanner = TcaScanner(
            bus=args.bus,
            tca_addr=args.tca_addr,
            vl53_addr=args.vl53_addr,
            settle_s=args.settle_s,
        )

        found_channels: list[int] = []

        for channel in range(start_ch, end_ch + 1):
            result = scanner.probe_vl53(channel)
            label = expected_label(channel, right_ch=args.right_ch, left_ch=args.left_ch)

            if result.present:
                found_channels.append(channel)
                present = "YES"
                note = "VL53 responded"
            else:
                present = "NO"
                note = result.error or "-"

            print(f"{channel:8d} | {present:<7s} | {label:<14s} | {note}")

        print("-" * 52)
        print(f"Found channels: {found_channels}")

        expected = {int(args.right_ch), int(args.left_ch)}
        found = set(found_channels)

        if expected.issubset(found):
            print("Result: PASS - expected Robot Savo ToF channels found.")
            return 0

        print("Result: WARN - expected ToF channels were not both found.")
        return 1

    finally:
        if scanner is not None:
            scanner.close()


if __name__ == "__main__":
    raise SystemExit(main())