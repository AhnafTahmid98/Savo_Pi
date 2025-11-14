#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — VL53L1X Range Tester (single I²C bus)
--------------------------------------------------
- Reads one VL53L1X sensor at address 0x29 on a chosen I²C bus.
- Clean SIGINT (Ctrl+C), rolling median filter, threshold flag.

IMPORTANT LIMITATION:
  The Python 'VL53L1X/vl53l1x' module uses GLOBAL I²C state internally.
  That means using multiple buses (e.g. 1 and 0) in the SAME PROCESS
  is unreliable: the last-initialized bus "wins" and both reads come
  from that bus.

So this tool is intentionally SINGLE-BUS.
To see both front sensors, run it twice:

  # Front-Right (FR) on bus 1
  python3 range_vl53_test.py --bus 1 --rate 10

  # Front-Left (FL) on bus 0
  python3 range_vl53_test.py --bus 0 --rate 10

Physical mapping (LOCKED for Robot Savo front bumper):
  - i2c-1 (bus 1) -> Front-Right  (FR)
  - i2c-0 (bus 0) -> Front-Left   (FL)
"""

import argparse
import sys
import time
import signal
from collections import deque
from typing import Optional, List

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


def role_for_bus(bus: int) -> str:
    """Human label for this bus."""
    if bus == 1:
        return "FR"  # Front-Right
    if bus == 0:
        return "FL"  # Front-Left
    return f"bus{bus}"


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
    ap = argparse.ArgumentParser(description="Single-bus VL53L1X test (cm)")
    ap.add_argument("--bus", type=int, default=1,
                    help="I²C bus number (default: 1). Use 1=FR, 0=FL.")
    ap.add_argument("--rate", type=float, default=10.0, help="Hz (default: 10.0)")
    ap.add_argument("--mode", type=str, default="short", help="short|medium|long (default: short)")
    ap.add_argument("--timing", type=int, default=50, help="Timing budget ms (default: 50)")
    ap.add_argument("--median", type=int, default=3, help="Median window (odd ≥1, default: 3)")
    ap.add_argument("--threshold", type=float, default=28.0, help="Alert threshold in cm (default: 28.0)")
    args = ap.parse_args()

    if args.rate <= 0:
        print("ERROR: --rate must be > 0", file=sys.stderr)
        return 2
    if args.median < 1 or (args.median % 2) == 0:
        print("ERROR: --median must be odd ≥1", file=sys.stderr)
        return 2

    role = role_for_bus(args.bus)
    period = 1.0 / args.rate

    print(
        f"[VL53L1X SINGLE] Addr=0x{I2C_ADDRESS:02X}  Bus={args.bus} ({role})  "
        f"Mode={args.mode.upper()}  Timing={args.timing}ms  "
        f"Rate={args.rate:.2f} Hz  Median={args.median}  Threshold={args.threshold:.1f} cm"
    )

    try:
        sensor = setup_sensor(args.bus, args.mode, args.timing, None)
        print(f"  - OK: {role} (i2c-{args.bus}) online")
    except Exception as e:
        print(f"  - FAIL: Could not init VL53L1X on bus {args.bus}: {e}", file=sys.stderr)
        return 1

    hist = deque(maxlen=args.median)

    print(f"\n t(s)  | {role}[bus{args.bus}](cm) | {role}[bus{args.bus}]_f(cm) | alert")
    print("-" * 57)

    t0 = time.perf_counter()
    while not _stop:
        loop_start = time.perf_counter()
        t = loop_start - t0

        d_mm = read_mm(sensor)
        d_cm = round(d_mm / 10.0, 1) if d_mm is not None else -1.0

        if d_cm > 0:
            hist.append(d_cm)

        if args.median > 1 and len(hist) > 0:
            fv = median(list(hist))
        else:
            fv = hist[-1] if hist else -1.0

        alert = (fv >= 0 and fv < args.threshold)

        def fmt(v: float) -> str:
            return f"{v:>6.1f}" if v >= 0 else "  --- "

        print(f"{t:6.2f} | {fmt(d_cm)} | {fmt(fv)} | {'!' if alert else '-'}")

        sleep_s = period - (time.perf_counter() - loop_start)
        if sleep_s > 0:
            time.sleep(sleep_s)

    try:
        if hasattr(sensor, "stop_ranging"):
            sensor.stop_ranging()
        elif hasattr(sensor, "stop"):
            sensor.stop()
    except Exception:
        pass

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
