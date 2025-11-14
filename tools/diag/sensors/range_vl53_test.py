#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — VL53L1X Range Tester (dual I²C buses, pro features)
-----------------------------------------------------------------
- Reads one or two VL53L1X sensors both at address 0x29 by placing them on
  different I²C controllers (i2c-1 and i2c-0) on Raspberry Pi.
- Clean SIGINT (Ctrl+C), CSV logging, rolling median filter, threshold flag.

Physical mapping (LOCKED for Robot Savo front bumper):
  - i2c-1 (bus 1) -> Front-Right  (FR)
  - i2c-0 (bus 0) -> Front-Left   (FL)

Author: Robot Savo
"""

import argparse
import csv
import sys
import time
import signal
from collections import deque
from typing import Dict, Optional, List

# --- Robust import (handles both module name casings) ------------------------
try:
    from VL53L1X import VL53L1X
except ImportError:
    try:
        from vl53l1x import VL53L1X
    except Exception:
        print("ERROR: Python module 'VL53L1X/vl53l1x' not found.", file=sys.stderr)
        print("Install with: sudo -H python3 -m pip install --break-system-packages --upgrade vl53l1x", file=sys.stderr)
        sys.exit(1)

I2C_ADDRESS = 0x29

# Locked roles:
#   bus 1 = Front-Right (FR)
#   bus 0 = Front-Left  (FL)
def bus_role(bus: int) -> str:
    if bus == 1:
        return "FR"
    if bus == 0:
        return "FL"
    return f"bus{bus}"

def bus_desc(bus: int) -> str:
    """Long label with role + I²C bus number, e.g. 'FR (i2c-1)'."""
    return f"{bus_role(bus)} (i2c-{bus})"

def col_base(bus: int) -> str:
    """Base column label, e.g. 'FR[bus1]'."""
    return f"{bus_role(bus)}[bus{bus}]"

# --- Ctrl+C handling ---------------------------------------------------------
_stop = False
def _sigint_handler(signum, frame):
    global _stop
    if not _stop:
        _stop = True
        print("\n^C  Stopping cleanly (closing sensors & files)...", flush=True)

signal.signal(signal.SIGINT, _sigint_handler)

# --- Utilities ---------------------------------------------------------------
def _mode_name(mode: str) -> str:
    m = mode.strip().lower()
    if m in ("short", "s"):
        return "short"
    if m in ("medium", "med", "m"):
        return "medium"
    if m in ("long", "l"):
        return "long"
    return "short"

def _median(vals: List[int]) -> int:
    if not vals:
        return -1
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    return s[mid] if (n % 2) else (s[mid - 1] + s[mid]) // 2

def setup_sensor(bus: int, address: int, mode: str, timing_ms: int, inter_ms: Optional[int]) -> VL53L1X:
    s = VL53L1X(i2c_bus=bus, i2c_address=address)
    if hasattr(s, "open"):
        s.open()

    mode = _mode_name(mode)
    try:
        s.set_distance_mode(mode)
    except Exception:
        try:
            s.distance_mode = mode
        except Exception:
            pass

    try:
        if hasattr(s, "set_timing"):
            s.set_timing(timing_ms, inter_ms or (timing_ms + 20))
        else:
            if hasattr(s, "timing_budget"):
                s.timing_budget = timing_ms
            if inter_ms and hasattr(s, "inter_measurement"):
                s.inter_measurement = inter_ms
    except Exception:
        pass

    if hasattr(s, "start_ranging"):
        s.start_ranging()
    elif hasattr(s, "start"):
        s.start()
    return s

def read_distance_mm(sensor: VL53L1X) -> Optional[int]:
    try:
        d = sensor.get_distance() if hasattr(sensor, "get_distance") else getattr(sensor, "distance", None)
        if d is None:
            return None
        if isinstance(d, float):
            d = int(round(d))
        if d <= 0 or d >= 4000:
            return None
        return int(d)
    except Exception:
        return None

# --- Main --------------------------------------------------------------------
def main():
    global _stop
    ap = argparse.ArgumentParser(description="VL53L1X range test on dual I²C buses (output in cm)")
    ap.add_argument("--buses", type=str, default="1,0", help="Comma-separated I²C buses, e.g. '1' or '1,0'. (1=FR, 0=FL)")
    ap.add_argument("--rate", type=float, default=20.0, help="Sample rate in Hz (default: 20.0)")
    ap.add_argument("--samples", type=int, default=0, help="Number of samples (0 = run until Ctrl+C)")
    ap.add_argument("--mode", type=str, default="short", help="Distance mode: short|medium|long (default: short)")
    ap.add_argument("--timing", type=int, default=50, help="Timing budget in ms (default: 50)")
    ap.add_argument("--inter", type=int, default=0, help="Inter-measurement in ms (0 = auto)")
    ap.add_argument("--median", type=int, default=3, help="Rolling median window (odd int ≥1). 1 = no filter. (default: 3)")
    ap.add_argument("--threshold", type=float, default=28.0, help="Near-field alert threshold in cm (default: 28.0)")
    ap.add_argument("--csv", action="store_true", help="Write CSV to --out")
    ap.add_argument("--out", type=str, default="vl53l1x_log.csv", help="CSV filename")
    args = ap.parse_args()

    # Parse bus list
    try:
        buses = [int(x.strip()) for x in args.buses.split(",") if x.strip()]
    except ValueError:
        print("ERROR: --buses must be a comma-separated list of integers.", file=sys.stderr)
        return 2
    if args.rate <= 0:
        print("ERROR: --rate must be > 0", file=sys.stderr)
        return 2
    if args.median < 1 or (args.median % 2) == 0:
        print("ERROR: --median must be an odd integer ≥ 1 (e.g., 1,3,5).", file=sys.stderr)
        return 2

    period = 1.0 / args.rate
    sensors: Dict[int, VL53L1X] = {}
    hist: Dict[int, deque] = {}

    # Info line with mapping
    label_str = ", ".join([bus_desc(bus) for bus in buses])
    print(
        f"[VL53L1X] Addr=0x{I2C_ADDRESS:02X}  Buses={buses} [{label_str}]  "
        f"Mode={_mode_name(args.mode).upper()}  "
        f"Timing={args.timing}ms  Inter={args.inter if args.inter>0 else 'auto'}ms  "
        f"Rate={args.rate:.2f} Hz  Median={args.median}  Threshold={args.threshold:.1f} cm"
    )

    # Init sensors
    for bus in buses:
        hist[bus] = deque(maxlen=args.median)
        if _stop:
            break
        try:
            s = setup_sensor(bus, I2C_ADDRESS, args.mode, args.timing, args.inter if args.inter > 0 else None)
            time.sleep(0.05)
            _ = read_distance_mm(s)
            sensors[bus] = s
            print(f"  - OK: {bus_desc(bus)} online")
        except Exception as e:
            print(f"  - FAIL: {bus_desc(bus)} init error: {e}", file=sys.stderr)

    if not sensors:
        print("ERROR: No sensors initialized. Check wiring and i2cdetect output.", file=sys.stderr)
        return 1

    active_buses = sorted(sensors.keys())

    # CSV setup
    csv_writer = None
    csv_file = None
    try:
        if args.csv:
            csv_file = open(args.out, "w", newline="")
            csv_writer = csv.writer(csv_file)
            header = ["t_sec"] \
                     + [f"{col_base(bus)}_cm" for bus in active_buses] \
                     + [f"{col_base(bus)}_f_cm" for bus in active_buses]
            csv_writer.writerow(header)
            print(f"[CSV] Logging to {args.out}")

        # Printed header line
        cols = " | ".join(
            [f"{col_base(bus)}(cm)" for bus in active_buses]
            + [f"{col_base(bus)}_f(cm)" for bus in active_buses]
        )
        print("\n t(s)  | " + cols + " | alert")
        print("-" * (12 + len(cols) + 8))

        t0 = time.perf_counter()
        count = 0
        while not _stop:
            loop_start = time.perf_counter()
            t = loop_start - t0

            raw_vals_cm, filt_vals_cm = [], []
            for bus in active_buses:
                d_mm = read_distance_mm(sensors[bus])
                d_cm = round(d_mm / 10.0, 1) if d_mm is not None else -1
                raw_vals_cm.append(d_cm)

                if d_cm > 0:
                    hist[bus].append(d_cm)

                if args.median > 1 and len(hist[bus]) > 0:
                    fv = _median(list(hist[bus]))
                else:
                    fv = hist[bus][-1] if len(hist[bus]) else -1
                filt_vals_cm.append(fv)

            alert = any((v >= 0 and v < args.threshold) for v in filt_vals_cm)

            def fmt(v: float) -> str:
                return f"{v:>6.1f}" if v >= 0 else "  --- "

            line = " | ".join([fmt(v) for v in raw_vals_cm] + [fmt(v) for v in filt_vals_cm])
            print(f"{t:6.2f} | " + line + f" | {'!' if alert else '-'}")

            if csv_writer:
                csv_writer.writerow([f"{t:.3f}"] + raw_vals_cm + filt_vals_cm)

            count += 1
            if args.samples > 0 and count >= args.samples:
                break

            sleep_s = period - (time.perf_counter() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        _stop = True
        print("\n^C  KeyboardInterrupt caught. Cleaning up...", flush=True)
    finally:
        for s in sensors.values():
            try:
                (s.stop_ranging() if hasattr(s, "stop_ranging") else s.stop())
            except Exception:
                pass
        if csv_file:
            try:
                csv_file.flush()
                csv_file.close()
            except Exception:
                pass
        print("Done." + (" (Exited by user Ctrl+C)" if _stop else ""))
        return 130 if _stop else 0

if __name__ == "__main__":
    sys.exit(main())
