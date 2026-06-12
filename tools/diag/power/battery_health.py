#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Battery health diagnostic for the UPS HAT and robot kit pack."""

import argparse
import csv
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus  # type: ignore
    except ImportError as exc:  # pragma: no cover - environment specific
        print(
            "ERROR: Neither 'smbus2' nor 'smbus' is available. "
            "Install with: sudo apt install python3-smbus2",
            file=sys.stderr,
        )
        raise


UPS_I2C_ADDR = 0x36
ADC_I2C_ADDR = 0x48
ADS7830_CMD_BASE = 0x84


@dataclass
class UpsReading:
    voltage: Optional[float]
    capacity: Optional[float]
    ok: bool
    error: Optional[str] = None


@dataclass
class KitReading:
    voltage: Optional[float]
    soc: Optional[float] = None
    ok: bool = False
    error: Optional[str] = None


class UpsHat:
    """UPS Shield helper; vendor voltage formula is byte-swapped raw * 1.25 / 1000 / 16."""

    def __init__(self, bus_id: int = 1, address: int = UPS_I2C_ADDR) -> None:
        self.address = address
        self.bus = SMBus(bus_id)

    def read_voltage(self) -> float:
        read = self.bus.read_word_data(self.address, 2)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        voltage = swapped * 1.25 / 1000.0 / 16.0
        return float(f"{voltage:.3f}")  # round to 3 decimals

    def read_capacity(self) -> float:
        read = self.bus.read_word_data(self.address, 4)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        capacity = swapped / 256.0
        return float(f"{capacity:.1f}")  # round to 0.1%

    def safe_read(self) -> UpsReading:
        try:
            v = self.read_voltage()
            c = self.read_capacity()
            return UpsReading(voltage=v, capacity=c, ok=True)
        except OSError as exc:
            return UpsReading(voltage=None, capacity=None, ok=False, error=str(exc))

    def close(self) -> None:
        try:
            self.bus.close()
        except Exception:
            pass


class KitBattery:
    """ADS7830 kit battery reader; v1 uses 3.3*3, v2 uses 5.2*2."""

    def __init__(
        self,
        bus_id: int = 1,
        address: int = ADC_I2C_ADDR,
        channel: int = 2,
        pcb_version: int = 2,
        v_empty: float = 6.4,
        v_full: float = 8.4,
    ) -> None:
        if channel < 0 or channel > 7:
            raise ValueError("ADS7830 channel must be 0..7")
        if pcb_version not in (1, 2):
            raise ValueError("pcb_version must be 1 or 2")
        if v_full <= v_empty:
            raise ValueError("v_full must be greater than v_empty for SoC calculation")

        self.address = address
        self.channel = channel
        self.pcb_version = pcb_version
        self.adc_voltage_coefficient = 3.3 if pcb_version == 1 else 5.2
        self.v_empty = float(v_empty)
        self.v_full = float(v_full)
        self.bus = SMBus(bus_id)

    def _read_stable_byte(self) -> int:
        while True:
            value1 = self.bus.read_byte(self.address)
            value2 = self.bus.read_byte(self.address)
            if value1 == value2:
                return value1

    def _read_adc_channel(self) -> float:
        cmd = ADS7830_CMD_BASE | (
            (((self.channel << 2) | (self.channel >> 1)) & 0x07) << 4
        )
        self.bus.write_byte(self.address, cmd)
        value = self._read_stable_byte()
        voltage = value / 255.0 * self.adc_voltage_coefficient
        return voltage

    def read_battery_voltage(self) -> float:
        adc_voltage = self._read_adc_channel()
        if self.pcb_version == 1:
            battery_voltage = adc_voltage * 3.0
        else:
            battery_voltage = adc_voltage * 2.0
        return float(f"{battery_voltage:.3f}")

    def estimate_soc(self, voltage: float) -> float:
        """Simple linear SoC estimate for 2S pack between v_empty and v_full."""
        if voltage <= self.v_empty:
            soc = 0.0
        elif voltage >= self.v_full:
            soc = 100.0
        else:
            soc = (voltage - self.v_empty) / (self.v_full - self.v_empty) * 100.0
        return float(f"{soc:.1f}")

    def safe_read(self) -> KitReading:
        try:
            v = self.read_battery_voltage()
            soc = self.estimate_soc(v)
            return KitReading(voltage=v, soc=soc, ok=True)
        except OSError as exc:
            return KitReading(voltage=None, soc=None, ok=False, error=str(exc))

    def close(self) -> None:
        try:
            self.bus.close()
        except Exception:
            pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Robot Savo battery health diagnostic (UPS HAT + robot kit pack).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--ups-bus", type=int, default=1, help="I²C bus for UPS HAT (0x36)")
    parser.add_argument("--adc-bus", type=int, default=1, help="I²C bus for ADS7830 (0x48)")
    parser.add_argument(
        "--adc-channel", type=int, default=2, help="ADS7830 channel used for battery sense (0-7)"
    )
    parser.add_argument(
        "--pcb-version",
        type=int,
        choices=[1, 2],
        default=2,
        help="Robot kit PCB version (affects ADC scaling)",
    )

    parser.add_argument(
        "--kit-v-empty",
        type=float,
        default=6.4,
        help="Kit battery voltage mapped to 0% SoC (2S pack, linear model)",
    )
    parser.add_argument(
        "--kit-v-full",
        type=float,
        default=8.4,
        help="Kit battery voltage mapped to 100% SoC (2S pack, linear model)",
    )

    parser.add_argument("--interval", type=float, default=2.0, help="Sampling interval in seconds")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Total duration in seconds (0 = run until Ctrl+C)",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Optional CSV log file to append battery samples",
    )

    parser.add_argument(
        "--no-ups",
        action="store_true",
        help="Disable UPS HAT readings (skip 0x36)",
    )
    parser.add_argument(
        "--no-kit",
        action="store_true",
        help="Disable robot kit battery readings (skip 0x48)",
    )

    parser.add_argument(
        "--ups-low-v",
        type=float,
        default=3.40,
        help="UPS low voltage warning threshold (V)",
    )
    parser.add_argument(
        "--ups-shutdown-v",
        type=float,
        default=3.20,
        help="UPS emergency shutdown threshold (V)",
    )

    parser.add_argument(
        "--kit-low-v",
        type=float,
        default=7.20,
        help="Robot kit pack low voltage warning threshold (V)",
    )
    parser.add_argument(
        "--kit-low-soc",
        type=float,
        default=20.0,
        help="Kit SoC warning threshold (percent, linear estimate)",
    )
    parser.add_argument(
        "--kit-full-soc",
        type=float,
        default=95.0,
        help="Kit SoC 'full' threshold (percent, linear estimate)",
    )

    parser.add_argument(
        "--allow-shutdown",
        action="store_true",
        help="Actually call 'sudo shutdown -h now' when UPS voltage is below ups-shutdown-v",
    )

    return parser.parse_args(argv)


def maybe_shutdown(ups_v: float, args) -> None:
    """Request shutdown if enabled and UPS voltage is critically low."""
    if ups_v <= args.ups_shutdown_v and args.allow_shutdown:
        print(
            ">>> UPS voltage below shutdown threshold "
            f"({ups_v:.3f} V <= {args.ups_shutdown_v:.3f} V)."
        )
        print(">>> Requesting system shutdown NOW...")
        import subprocess

        try:
            subprocess.Popen(["sudo", "shutdown", "-h", "now"])
        except Exception as exc:
            print(f"ERROR: Failed to execute shutdown: {exc}", file=sys.stderr)


def main(argv=None) -> int:
    args = parse_args(argv)

    if args.no_ups and args.no_kit:
        print(
            "Nothing to do: both UPS and robot kit readings are disabled.",
            file=sys.stderr,
        )
        return 1

    ups = None
    kit = None

    if not args.no_ups:
        try:
            ups = UpsHat(bus_id=args.ups_bus)
        except Exception as exc:
            print(
                f"WARNING: Failed to initialise UPS HAT on bus {args.ups_bus}: {exc}",
                file=sys.stderr,
            )

    if not args.no_kit:
        try:
            kit = KitBattery(
                bus_id=args.adc_bus,
                channel=args.adc_channel,
                pcb_version=args.pcb_version,
                v_empty=args.kit_v_empty,
                v_full=args.kit_v_full,
            )
        except Exception as exc:
            print(
                f"WARNING: Failed to initialise ADS7830 on bus {args.adc_bus}: {exc}",
                file=sys.stderr,
            )

    if ups is None and kit is None:
        print(
            "ERROR: Neither UPS HAT nor robot kit battery monitor could be initialised.",
            file=sys.stderr,
        )
        return 1

    csv_file = None
    csv_writer = None
    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        csv_file = args.csv.open("a", newline="")
        csv_writer = csv.writer(csv_file)
        if args.csv.stat().st_size == 0:
            csv_writer.writerow(
                [
                    "timestamp",
                    "ups_voltage_V",
                    "ups_capacity_pct",
                    "kit_voltage_V",
                    "kit_soc_pct",
                    "ups_state",
                    "kit_state",
                ]
            )

    print("-" * 80)
    print("Robot Savo — Battery Health Diagnostic")
    if args.no_ups or ups is None:
        ups_line = "disabled"
    else:
        ups_line = f"I2C-{args.ups_bus} @ 0x{UPS_I2C_ADDR:02X}"
    print(f"UPS HAT    : {ups_line}")
    if args.no_kit or kit is None:
        kit_line = "disabled"
    else:
        kit_line = (
            f"I2C-{args.adc_bus} @ 0x{ADC_I2C_ADDR:02X}, "
            f"ch={args.adc_channel}, PCB v{args.pcb_version}, "
            f"SoC range {args.kit_v_empty:.2f}–{args.kit_v_full:.2f} V -> 0–100%"
        )
    print(f"Kit battery: {kit_line}")

    duration_str = "infinite" if args.duration <= 0 else f"{args.duration:.1f} s"
    print(f"Interval   : {args.interval:.2f} s   Duration: {duration_str}")

    if args.allow_shutdown and not args.no_ups:
        print(f"Shutdown   : ENABLED below {args.ups_shutdown_v:.3f} V (UPS HAT)")
    else:
        print("Shutdown   : disabled (no automatic power-off)")

    print("-" * 80)
    print(" time       UPS_V   UPS_%   KIT_V   KIT_%   Notes")
    print("--------------------------------------------------------------------------")

    start_time = time.time()
    try:
        while True:
            now = datetime.now().strftime("%H:%M:%S")

            ups_read = UpsReading(voltage=None, capacity=None, ok=False)
            kit_read = KitReading(voltage=None, soc=None, ok=False)
            ups_state = "ok"
            kit_state = "ok"
            if ups is not None:
                ups_read = ups.safe_read()
                if ups_read.ok and ups_read.voltage is not None:
                    if ups_read.voltage <= args.ups_shutdown_v:
                        ups_state = "critical"
                    elif ups_read.voltage <= args.ups_low_v:
                        ups_state = "low"
                    else:
                        ups_state = "ok"
                else:
                    ups_state = "error"
            if kit is not None:
                kit_read = kit.safe_read()
                if kit_read.ok and kit_read.voltage is not None:
                    low_by_v = kit_read.voltage <= args.kit_low_v
                    low_by_soc = (
                        kit_read.soc is not None
                        and kit_read.soc <= args.kit_low_soc
                    )
                    full_by_soc = (
                        kit_read.soc is not None
                        and kit_read.soc >= args.kit_full_soc
                    )
                    if full_by_soc:
                        kit_state = "full"
                    elif low_by_v or low_by_soc:
                        kit_state = "low"
                    else:
                        kit_state = "ok"
                else:
                    kit_state = "error"
            if ups_state == "error" and kit_state == "error":
                notes = "UPS and kit error"
            elif ups_state == "error":
                notes = "UPS error"
            elif kit_state == "error":
                notes = "Kit error"
            else:
                ups_low_flag = ups_state in ("low", "critical")
                kit_low_flag = kit_state == "low"

                if ups_low_flag and kit_low_flag:
                    notes = "Both low – needs charging"
                elif ups_low_flag:
                    notes = "UPS low"
                elif kit_low_flag:
                    notes = "Kit low – needs charging"
                else:
                    notes = "Good condition"

            ups_v_str = f"{ups_read.voltage:5.2f}" if ups_read.voltage is not None else "  n/a"
            ups_c_str = f"{ups_read.capacity:5.1f}" if ups_read.capacity is not None else "  n/a"
            kit_v_str = f"{kit_read.voltage:5.2f}" if kit_read.voltage is not None else "  n/a"
            kit_soc_str = f"{kit_read.soc:5.1f}" if kit_read.soc is not None else "  n/a"

            print(f"{now}  {ups_v_str}  {ups_c_str}  {kit_v_str}  {kit_soc_str}   {notes}")
            if csv_writer is not None:
                csv_writer.writerow(
                    [
                        datetime.now().isoformat(),
                        ups_read.voltage if ups_read.voltage is not None else "",
                        ups_read.capacity if ups_read.capacity is not None else "",
                        kit_read.voltage if kit_read.voltage is not None else "",
                        kit_read.soc if kit_read.soc is not None else "",
                        ups_state,
                        kit_state,
                    ]
                )
                csv_file.flush()
            if ups is not None and ups_read.ok and ups_read.voltage is not None:
                maybe_shutdown(ups_read.voltage, args)

            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                print("Duration reached, exiting.")
                break

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nInterrupted by user, exiting...")

    finally:
        if ups is not None:
            ups.close()
        if kit is not None:
            kit.close()
        if csv_file is not None:
            csv_file.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
