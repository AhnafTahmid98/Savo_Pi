#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — VL53L1X Range Tester (dual I²C buses, pro features)
-----------------------------------------------------------------
- Reads one or two VL53L1X sensors both at address 0x29 by placing them on
  different I²C controllers (i2c-1 and i2c-0) on Raspberry Pi.
- Clean SIGINT (Ctrl+C), CSV logging, rolling median filter, threshold flag,
 
Author: Savo Copilot

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
    from VL53L1X import VL53L1X   # typical wheel exposes this
except ImportError:
    try:
        from vl53l1x import VL53L1X  # some forks use lowercase package
    except Exception:
        print("ERROR: Python module 'VL53L1X/vl53l1x' not found.", file=sys.stderr)
        print("Install with: sudo -H python3 -m pip install --break-system-packages --upgrade vl53l1x", file=sys.stderr)
        sys.exit(1)

# Optional buzzer (gpiozero). We fail gracefully if missing.
BUZZER_AVAILABLE = False
Buzzer = None
try:
    from gpiozero import Buzzer as _Buzzer
    Buzzer = _Buzzer
    BUZZER_AVAILABLE = True
except Exception:
    BUZZER_AVAILABLE = False

I2C_ADDRESS = 0x29  # fixed address; we separate sensors by bus id

# --- SIGINT / Ctrl+C handling ------------------------------------------------
_stop = False
def _sigint_handler(signum, frame):
    global _stop
    if not _stop:
        _stop = True
        print("\n^C  Stopping cleanly (closing sensors & files)...", flush=True)
signal.signal(signal.SIGINT, _sigint_handler)

# --- Utils -------------------------------------------------------------------
def _mode_name(mode: str) -> str:
    m = mode.strip().lower()
    if m in ("short", "s"): return "short"
    if m in ("medium", "med", "m"): return "medium"
    if m in ("long", "l"): return "long"
    return "short"

def _median(vals: List[int]) -> int:
    if not vals:
        return -1
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    if n % 2:
        return s[mid]
    return (s[mid - 1] + s[mid]) // 2

def setup_sensor(bus: int, address: int, mode: str, timing_ms: int, inter_ms: Optional[int]) -> VL53L1X:
    s = VL53L1X(i2c_bus=bus, i2c_address=address)
    if hasattr(s, "open"): s.open()

    mode = _mode_name(mode)
    # distance mode
    try:
        s.set_distance_mode(mode)
    except Exception:
        try:
            s.distance_mode = mode
        except Exception:
            pass

    # timing budget / inter-measurement
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

    # start ranging
    if hasattr(s, "start_ranging"):
        s.start_ranging()
    elif hasattr(s, "start"):
        s.start()
    return s

def read_distance_mm(sensor: VL53L1X) -> Optional[int]:
    """Return mm or None if invalid/out-of-range."""
    try:
        d = sensor.get_distance() if hasattr(sensor, "get_distance") else getattr(sensor, "distance", None)
        if d is None:
            return None
        if isinstance(d, float):
            d = int(round(d))
        # Typical usable 40..4000 mm
        if d <= 0 or d >= 4000:
            return None
        return int(d)
    except Exception:
        return None

def main():
    global _stop
    ap = argparse.ArgumentParser(description="VL53L1X range test on dual I²C buses")
    ap.add_argument("--buses", type=str, default="1,0",
                    help="Comma-separated list of I²C buses (e.g. '1' or '1,0'). Default: 1,0")
    ap.add_argument("--rate", type=float, default=20.0, help="Sample rate in Hz (default: 20.0)")
    ap.add_argument("--samples", type=int, default=0, help="Number of samples to capture (0 = run until Ctrl+C)")
    ap.add_argument("--mode", type=str, default="short", help="Distance mode: short|medium|long (default: short)")
    ap.add_argument("--timing", type=int, default=50, help="Timing budget in ms (default: 50)")
    ap.add_argument("--inter", type=int, default=0, help="Inter-measurement in ms (0 = auto)")
    ap.add_argument("--median", type=int, default=3, help="Rolling median window (odd int ≥1). 1 = no filtering. (default: 3)")
    ap.add_argument("--threshold", type=int, default=280, help="Near-field alert threshold in mm (default: 280)")
    ap.add_argument("--beep", action="store_true", help="Enable buzzer on GPIO17 when under threshold")
    ap.add_argument("--buzzer-pin", type=int, default=17, help="BCM pin for buzzer (default: 17)")
    ap.add_argument("--csv", action="store_true", help="Write CSV to --out")
    ap.add_argument("--out", type=str, default="vl53l1x_log.csv", help="CSV filename")
    args = ap.parse_args()

    # Parse buses
    try:
        buses = [int(x.strip()) for x in args.buses.split(",") if x.strip() != ""]
    except ValueError:
        print("ERROR: --buses must be a comma-separated list of integers (e.g. '1' or '1,0').", file=sys.stderr)
        return 2
    if args.rate <= 0:
        print("ERROR: --rate must be > 0", file=sys.stderr)
        return 2
    if args.median < 1 or (args.median % 2) == 0:
        print("ERROR: --median must be an odd integer ≥ 1 (e.g., 1,3,5).", file=sys.stderr)
        return 2

    period = 1.0 / args.rate
    sensors: Dict[int, VL53L1X] = {}
    hist: Dict[int, deque] = {bus: deque(maxlen=args.median) for bus in buses}

    # Optional buzzer
    buzzer = None
    if args.beep:
        if not BUZZER_AVAILABLE:
            print("WARN: gpiozero not available; --beep ignored.", file=sys.stderr)
        else:
            try:
                buzzer = Buzzer(args.buzzer_pin, active_high=True)
            except Exception as e:
                print(f"WARN: Buzzer init failed on GPIO{args.buzzer_pin}: {e}", file=sys.stderr)
                buzzer = None

    print(f"[VL53L1X] Addr=0x{I2C_ADDRESS:02X}  Buses={buses}  Mode={_mode_name(args.mode).upper()}  "
          f"Timing={args.timing}ms  Inter={args.inter if args.inter>0 else 'auto'}ms  "
          f"Rate={args.rate:.2f} Hz  Median={args.median}  Threshold={args.threshold}mm  "
          f"Buzzer={'ON' if buzzer else 'OFF'}")

    # Bring up sensors per bus
    for bus in buses:
        if _stop: break
        try:
            s = setup_sensor(bus, I2C_ADDRESS, args.mode, args.timing, args.inter if args.inter > 0 else None)
            time.sleep(0.05)
            _ = read_distance_mm(s)
            sensors[bus] = s
            print(f"  - OK: i2c-{bus} online")
        except Exception as e:
            print(f"  - FAIL: i2c-{bus} init error: {e}", file=sys.stderr)

    if _stop:
        pass
    elif not sensors:
        print("ERROR: No sensors initialized. Check wiring and that i2c-0/i2c-1 show 0x29 with i2cdetect.", file=sys.stderr)
        return 1

    # CSV setup
    csv_writer = None
    csv_file = None
    try:
        if args.csv:
            csv_file = open(args.out, "w", newline="")
            csv_writer = csv.writer(csv_file)
            header = ["t_sec"] + [f"bus{bus}_mm" for bus in buses] + [f"bus{bus}_filt_mm" for bus in buses]
            csv_writer.writerow(header)
            print(f"[CSV] Logging to {args.out}")

        # Pretty header
        cols = " | ".join([f"bus{bus}(mm)" for bus in buses] + [f"bus{bus}_f(mm)" for bus in buses])
        print("\n t(s)  | " + cols + " | alert")
        print("-" * (12 + len(cols) + 8))

        t0 = time.perf_counter()
        count = 0
        while not _stop:
            loop_start = time.perf_counter()
            t = loop_start - t0

            raw_vals = []
            filt_vals = []
            for bus in buses:
                d = read_distance_mm(sensors[bus])
                raw_vals.append(d if d is not None else -1)
                # push into history
                if d is not None:
                    hist[bus].append(d)
                # compute median (or pass through if window=1)
                if len(hist[bus]) > 0:
                    fv = _median(list(hist[bus])) if args.median > 1 else hist[bus][-1]
                    filt_vals.append(fv)
                else:
                    filt_vals.append(-1)

            # Determine alert (any filtered bus under threshold and valid)
            alert = any((v >= 0 and v < args.threshold) for v in filt_vals)
            if buzzer:
                try:
                    buzzer.on() if alert else buzzer.off()
                except Exception:
                    pass

            # Print line
            def fmt(v):  return f"{v:>6d}" if v >= 0 else "  --- "
            line = " | ".join([fmt(v) for v in raw_vals] + [fmt(v) for v in filt_vals])
            print(f"{t:6.2f} | {line} | {'!' if alert else '-'}")

            # CSV
            if csv_writer:
                csv_writer.writerow([f"{t:.3f}"] + raw_vals + filt_vals)

            count += 1
            if args.samples > 0 and count >= args.samples:
                break

            # rate control
            elapsed = time.perf_counter() - loop_start
            sleep_s = period - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        _stop = True
        print("\n^C  KeyboardInterrupt caught. Cleaning up...", flush=True)
    finally:
        # Stop sensors and buzzer
        for bus, s in sensors.items():
            try:
                (s.stop_ranging() if hasattr(s, "stop_ranging") else s.stop())
            except Exception:
                pass
        if csv_file:
            try:
                csv_file.flush(); csv_file.close()
            except Exception:
                pass
        if buzzer:
            try:
                buzzer.off()
            except Exception:
                pass
        if _stop:
            print("Done. (Exited by user Ctrl+C)")
            return 130
        else:
            print("Done.")
            return 0

if __name__ == "__main__":
    sys.exit(main())
