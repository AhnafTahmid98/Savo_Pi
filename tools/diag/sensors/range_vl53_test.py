#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — VL53L1X Simple Dual-Bus Range Tester
-------------------------------------------------
- Reads VL53L1X sensors at address 0x29 on any I²C buses you pass (e.g. 1,0).
- NO FR/FL labels here, only explicit bus numbers so we can't get confused.
"""

import argparse
import sys
import time
import signal
from collections import deque
from typing import Dict, Optional, List

try:
    from VL53L1X import VL53L1X
except ImportError:
    try:
        from vl53l1x import VL53L1X
    except Exception:
        print("ERROR: Python module 'VL53L1X/vl53l1x' not found.", file=sys.stderr)
        sys.exit(1)

I2C_ADDRESS = 0x29
_stop = False

def sigint_handler(signum, frame):
    global _stop
    if not _stop:
        _stop = True
        print("\n^C  Stopping cleanly...", flush=True)

signal.signal(signal.SIGINT, sigint_handler)

def median(vals: List[float]) -> float:
    if not vals:
        return -1.0
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    return s[mid] if (n % 2) else (s[mid - 1] + s[mid]) / 2.0

def setup_sensor(bus: int, mode: str, timing_ms: int, inter_ms: Optional[int]) -> VL53L1X:
    s = VL53L1X(i2c_bus=bus, i2c_address=I2C_ADDRESS)
    if hasattr(s, "open"):
        s.open()

    mode = mode.strip().lower()
    if mode not in ("short", "medium", "long"):
        mode = "short"

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
    except Exception:
        pass

    if hasattr(s, "start_ranging"):
        s.start_ranging()
    elif hasattr(s, "start"):
        s.start()
    return s

def read_mm(sensor: VL53L1X) -> Optional[int]:
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

def main():
    global _stop
    ap = argparse.ArgumentParser(description="Simple dual-bus VL53L1X test (cm)")
    ap.add_argument("--buses", type=str, default="1,0", help="Comma-separated I²C buses, e.g. '1,0'.")
    ap.add_argument("--rate", type=float, default=10.0, help="Hz (default: 10.0)")
    ap.add_argument("--median", type=int, default=3, help="Median window (odd ≥1, default: 3)")
    ap.add_argument("--threshold", type=float, default=28.0, help="Alert threshold in cm")
    args = ap.parse_args()

    try:
        buses = [int(x.strip()) for x in args.buses.split(",") if x.strip()]
    except ValueError:
        print("ERROR: --buses must be a comma-separated list of integers.", file=sys.stderr)
        return 2

    if args.rate <= 0:
        print("ERROR: --rate must be > 0", file=sys.stderr)
        return 2
    if args.median < 1 or (args.median % 2) == 0:
        print("ERROR: --median must be odd ≥1", file=sys.stderr)
        return 2

    period = 1.0 / args.rate
    sensors: Dict[int, VL53L1X] = {}
    hist: Dict[int, deque] = {}

    print(f"[VL53L1X SIMPLE] Addr=0x{I2C_ADDRESS:02X}  Buses={buses}  Rate={args.rate:.2f} Hz  Median={args.median}")

    # Init
    for bus in buses:
        if _stop:
            break
        try:
            s = setup_sensor(bus, "short", 50, None)
            time.sleep(0.05)
            _ = read_mm(s)
            sensors[bus] = s
            hist[bus] = deque(maxlen=args.median)
            print(f"  - OK: bus{bus} online")
        except Exception as e:
            print(f"  - FAIL: bus{bus} init error: {e}", file=sys.stderr)

    if not sensors:
        print("ERROR: No sensors initialized.", file=sys.stderr)
        return 1

    active_buses = [b for b in buses if b in sensors]

    # Header
    cols = " | ".join([f"bus{b}_cm" for b in active_buses] +
                      [f"bus{b}_f_cm" for b in active_buses])
    print("\n t(s)  | " + cols + " | alert")
    print("-" * (12 + len(cols) + 8))

    t0 = time.perf_counter()
    while not _stop:
        t = time.perf_counter() - t0
        raw_vals, filt_vals = [], []

        for bus in active_buses:
            d_mm = read_mm(sensors[bus])
            d_cm = round(d_mm / 10.0, 1) if d_mm is not None else -1.0
            raw_vals.append(d_cm)

            if d_cm > 0:
                hist[bus].append(d_cm)
            fv = median(list(hist[bus])) if (args.median > 1 and hist[bus]) else (hist[bus][-1] if hist[bus] else -1.0)
            filt_vals.append(fv)

        alert = any((v >= 0 and v < args.threshold) for v in filt_vals)

        def fmt(v: float) -> str:
            return f"{v:>6.1f}" if v >= 0 else "  --- "

        line = " | ".join([fmt(v) for v in raw_vals] + [fmt(v) for v in filt_vals])
        print(f"{t:6.2f} | " + line + f" | {'!' if alert else '-'}")

        sleep_s = period - (time.perf_counter() - (t0 + t))
        if sleep_s > 0:
            time.sleep(sleep_s)

    # Cleanup
    for s in sensors.values():
        try:
            (s.stop_ranging() if hasattr(s, "stop_ranging") else s.stop())
        except Exception:
            pass
    print("Done.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
