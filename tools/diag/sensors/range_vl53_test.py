#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — VL53L1X Range Tester (dual I²C buses)
--------------------------------------------------
- Tests one or two VL53L1X ToF sensors both at address 0x29 by placing
  them on different I²C controllers (i2c-0 and i2c-1) on the Raspberry Pi.
- Uses the Pimoroni "vl53l1x" driver which allows selecting i2c_bus=0/1.

Author: Savo Copilot
"""

import argparse
import csv
import sys
import time
from typing import Dict, Optional

# Prefer Pimoroni driver (lets us choose i2c bus explicitly)
try:
    from vl53l1x import VL53L1X  # pip install vl53l1x
except Exception as e:
    print("ERROR: Python module 'vl53l1x' not found.", file=sys.stderr)
    print("Install with: pip3 install vl53l1x", file=sys.stderr)
    raise

I2C_ADDRESS = 0x29  # fixed per sensor; we separate by I²C controller

def human_mode_name(mode: str) -> str:
    m = mode.strip().lower()
    if m in ("short", "s"):
        return "short"
    if m in ("medium", "med", "m"):
        return "medium"
    if m in ("long", "l"):
        return "long"
    return "short"

def setup_sensor(bus: int, address: int, mode: str, timing_ms: int, inter_measure_ms: Optional[int]) -> VL53L1X:
    sensor = VL53L1X(i2c_bus=bus, i2c_address=address)
    # Some driver versions require .open() before config; try if present.
    if hasattr(sensor, "open"):
        sensor.open()

    # Distance mode (SHORT/MEDIUM/LONG)
    mode = human_mode_name(mode)
    try:
        # Newer driver API
        sensor.set_distance_mode(mode)
    except Exception:
        # Older API fallbacks
        try:
            sensor.distance_mode = mode
        except Exception:
            pass  # ignore if not supported

    # Timing budget / inter-measurement period
    # Common stable choices: 50ms / 70ms, 100ms / 120ms, 200ms / 220ms
    try:
        if hasattr(sensor, "set_timing"):
            sensor.set_timing(timing_ms, inter_measure_ms or (timing_ms + 20))
        else:
            # Some variants expose properties
            if hasattr(sensor, "timing_budget"):
                sensor.timing_budget = timing_ms
            if inter_measure_ms and hasattr(sensor, "inter_measurement"):
                sensor.inter_measurement = inter_measure_ms
    except Exception:
        pass  # not fatal; continue with defaults

    # Start ranging
    if hasattr(sensor, "start_ranging"):
        sensor.start_ranging()
    elif hasattr(sensor, "start"):
        sensor.start()
    return sensor

def read_distance_mm(sensor: VL53L1X) -> Optional[int]:
    """Return distance in millimeters or None if invalid/out-of-range."""
    try:
        # Pimoroni API: get_distance() -> mm (int)
        if hasattr(sensor, "get_distance"):
            d = sensor.get_distance()
        elif hasattr(sensor, "distance"):
            d = sensor.distance  # property in some forks
        else:
            return None
        # sanitize
        if d is None:
            return None
        if isinstance(d, float):
            d = int(round(d))
        if d <= 0 or d >= 4000:  # VL53L1X typical usable 40mm..4000mm
            return None
        return d
    except Exception:
        return None

def main():
    ap = argparse.ArgumentParser(description="VL53L1X range test on dual I²C buses")
    ap.add_argument("--buses", type=str, default="1,0",
                    help="Comma-separated list of I²C buses to probe (e.g. '1' or '1,0'). Default: 1,0")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="Sampling rate in Hz (default: 20.0)")
    ap.add_argument("--samples", type=int, default=0,
                    help="Number of samples to capture (0 = run forever)")
    ap.add_argument("--mode", type=str, default="short",
                    help="Distance mode: short|medium|long (default: short)")
    ap.add_argument("--timing", type=int, default=50,
                    help="Timing budget in ms (default: 50)")
    ap.add_argument("--inter", type=int, default=0,
                    help="Inter-measurement period in ms (0 = auto)")
    ap.add_argument("--csv", action="store_true",
                    help="Write CSV to --out (default stdout table only)")
    ap.add_argument("--out", type=str, default="vl53l1x_log.csv",
                    help="CSV output filename (used only with --csv)")
    args = ap.parse_args()

    try:
        buses = [int(x.strip()) for x in args.buses.split(",") if x.strip() != ""]
    except ValueError:
        print("ERROR: --buses must be a comma-separated list of integers (e.g. '1' or '1,0').", file=sys.stderr)
        sys.exit(2)

    if args.rate <= 0:
        print("ERROR: --rate must be > 0", file=sys.stderr)
        sys.exit(2)

    period = 1.0 / args.rate
    sensors: Dict[int, VL53L1X] = {}

    print(f"[VL53L1X] Address=0x{I2C_ADDRESS:02X}  Buses={buses}  Mode={human_mode_name(args.mode).upper()}  "
          f"Timing={args.timing}ms  Inter={args.inter if args.inter>0 else 'auto'}ms  Rate={args.rate:.2f} Hz")

    # Bring up sensors per bus
    for bus in buses:
        try:
            s = setup_sensor(bus, I2C_ADDRESS, args.mode, args.timing, args.inter if args.inter > 0 else None)
            # quick warm-up read
            time.sleep(0.05)
            _ = read_distance_mm(s)
            sensors[bus] = s
            print(f"  - OK: i2c-{bus} online")
        except Exception as e:
            print(f"  - FAIL: i2c-{bus} init error: {e}", file=sys.stderr)

    if not sensors:
        print("ERROR: No sensors initialized. Check wiring and that i2c-0/i2c-1 show 0x29 with i2cdetect.", file=sys.stderr)
        sys.exit(1)

    # CSV setup
    csv_writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.out, "w", newline="")
        csv_writer = csv.writer(csv_file)
        header = ["t_sec"] + [f"bus{bus}_mm" for bus in buses]
        csv_writer.writerow(header)
        print(f"[CSV] Logging to {args.out}")

    # Pretty header
    cols = " | ".join([f"bus{bus:>1}(mm)" for bus in buses])
    print("\n t(s)  | " + cols)
    print("-" * (8 + 3 + len(cols)))

    t0 = time.perf_counter()
    count = 0
    try:
        while True:
            now = time.perf_counter()
            t = now - t0

            row_mm = []
            for bus in buses:
                d = read_distance_mm(sensors[bus])
                row_mm.append(d if d is not None else -1)

            # Print
            values = " | ".join([f"{mm:>8d}" if mm >= 0 else "   ---  " for mm in row_mm])
            print(f"{t:6.2f} | {values}")

            # CSV
            if csv_writer:
                csv_writer.writerow([f"{t:.3f}"] + row_mm)

            count += 1
            if args.samples > 0 and count >= args.samples:
                break

            # Rate control
            elapsed = time.perf_counter() - now
            sleep_s = period - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop sensors
        for bus, s in sensors.items():
            try:
                if hasattr(s, "stop_ranging"):
                    s.stop_ranging()
                elif hasattr(s, "stop"):
                    s.stop()
            except Exception:
                pass
        if csv_file:
            csv_file.close()

        print("\nDone.")
        if args.csv:
            print(f"CSV saved: {args.out}")

if __name__ == "__main__":
    main()
