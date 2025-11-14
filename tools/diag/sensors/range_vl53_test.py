#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Dual VL53L1X Integrator (Python only, no ROS2)
-----------------------------------------------------------
- Reads TWO VL53L1X sensors at 0x29:
    * bus 1 -> Front-Right (FR)
    * bus 0 -> Front-Left  (FL)
- Uses multiprocessing: ONE process per sensor (because the VL53L1X
  Python driver uses global I²C state and cannot reliably handle
  bus 0 + bus 1 in the same process).
- Main process integrates both readings and prints:

      t(s) | FR_raw | FL_raw | FR_filt | FL_filt | alert

  where alert = "!" if either filtered distance < threshold (e.g. 28 cm).

Run example:
  python3 tools/diag/sensors/range_vl53_integrated.py --rate 10 --threshold 28
"""

import argparse
import sys
import time
import signal
import queue
from collections import deque
from typing import Optional
from multiprocessing import Process, Queue

I2C_ADDRESS = 0x29
_stop = False


# ---------------------------------------------------------------------------
# Ctrl+C handling (main process)
# ---------------------------------------------------------------------------
def _sigint_handler(signum, frame):
    global _stop
    if not _stop:
        _stop = True
        print("\n^C  Stopping cleanly...", flush=True)


signal.signal(signal.SIGINT, _sigint_handler)


# ---------------------------------------------------------------------------
# Worker-side helpers (each process has its own VL53L1X instance)
# ---------------------------------------------------------------------------
def _import_vl53():
    """Import VL53L1X module inside worker."""
    try:
        from VL53L1X import VL53L1X  # type: ignore
        return VL53L1X
    except ImportError:
        try:
            from vl53l1x import VL53L1X  # type: ignore
            return VL53L1X
        except Exception:
            print(
                "ERROR (worker): Python module 'VL53L1X/vl53l1x' not found.",
                file=sys.stderr,
            )
            sys.exit(1)


def worker_read_mm(sensor) -> Optional[int]:
    """Read distance in mm from VL53L1X instance."""
    try:
        if hasattr(sensor, "get_distance"):
            d = sensor.get_distance()
        else:
            d = getattr(sensor, "distance", None)
        if d is None:
            return None
        if isinstance(d, float):
            d = int(round(d))
        if d <= 0 or d >= 4000:
            return None
        return int(d)
    except Exception:
        return None


def tof_worker(bus: int, role: str, rate_hz: float, median_n: int, out_q: Queue):
    """
    Worker process:
      - Initializes VL53L1X on given bus.
      - Loops at rate_hz, reads distance, applies median filter.
      - Sends (kind, ...) messages to main:
          ("init_ok", bus, role, "")
          ("init_error", bus, role, err_msg)
          ("data", bus, role, raw_cm, filt_cm, t_now)
          ("done", bus, role, "")
    """
    VL53L1X = _import_vl53()

    period = 1.0 / rate_hz
    hist = deque(maxlen=median_n)

    try:
        sensor = VL53L1X(i2c_bus=bus, i2c_address=I2C_ADDRESS)
        if hasattr(sensor, "open"):
            sensor.open()

        mode = "short"
        try:
            sensor.set_distance_mode(mode)
        except Exception:
            try:
                sensor.distance_mode = mode
            except Exception:
                pass

        try:
            if hasattr(sensor, "set_timing"):
                sensor.set_timing(50, 70)  # 50 ms budget, ~70 ms inter
        except Exception:
            pass

        if hasattr(sensor, "start_ranging"):
            sensor.start_ranging()
        elif hasattr(sensor, "start"):
            sensor.start()
    except Exception as e:
        out_q.put(("init_error", bus, role, str(e)))
        return

    out_q.put(("init_ok", bus, role, ""))

    try:
        while True:
            t_now = time.perf_counter()
            d_mm = worker_read_mm(sensor)
            d_cm = round(d_mm / 10.0, 1) if d_mm is not None else -1.0

            if d_cm > 0:
                hist.append(d_cm)

            if median_n > 1 and len(hist) > 0:
                s = sorted(hist)
                n = len(s)
                mid = n // 2
                if n % 2:
                    fv = float(s[mid])
                else:
                    fv = 0.5 * (s[mid - 1] + s[mid])
            else:
                fv = float(hist[-1]) if hist else -1.0

            out_q.put(("data", bus, role, d_cm, fv, t_now))

            # Sleep to maintain rate
            dt = period - (time.perf_counter() - t_now)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if hasattr(sensor, "stop_ranging"):
                sensor.stop_ranging()
            elif hasattr(sensor, "stop"):
                sensor.stop()
        except Exception:
            pass
        out_q.put(("done", bus, role, ""))


# ---------------------------------------------------------------------------
# Main process: integrate FR + FL
# ---------------------------------------------------------------------------
def main():
    global _stop

    ap = argparse.ArgumentParser(description="Robot Savo — Dual VL53L1X integrator (cm)")
    ap.add_argument("--rate", type=float, default=10.0, help="Sample rate in Hz (default: 10.0)")
    ap.add_argument("--median", type=int, default=3, help="Median window (odd ≥1, default: 3)")
    ap.add_argument(
        "--threshold",
        type=float,
        default=28.0,
        help="Near-field alert threshold in cm (default: 28.0)",
    )
    args = ap.parse_args()

    if args.rate <= 0:
        print("ERROR: --rate must be > 0", file=sys.stderr)
        return 2
    if args.median < 1 or (args.median % 2) == 0:
        print("ERROR: --median must be odd ≥1", file=sys.stderr)
        return 2

    # Locked mapping: bus 1 = FR, bus 0 = FL
    workers_cfg = [
        (1, "FR"),
        (0, "FL"),
    ]

    q: Queue = Queue()

    procs = []
    for bus, role in workers_cfg:
        p = Process(target=tof_worker, args=(bus, role, args.rate, args.median, q), daemon=True)
        p.start()
        procs.append(p)

    print(
        f"[DUAL TOF] VL53L1X @0x{I2C_ADDRESS:02X}  Workers: "
        f"{', '.join([f'{r}(bus{b})' for b, r in workers_cfg])}  "
        f"Rate={args.rate:.2f} Hz  Median={args.median}  Th={args.threshold:.1f} cm"
    )

    # Wait for init messages
    init_ok = set()
    init_fail = False
    start_time = time.perf_counter()

    while len(init_ok) < len(workers_cfg) and not init_fail:
        try:
            msg = q.get(timeout=2.0)
        except queue.Empty:
            break

        kind = msg[0]
        if kind == "init_ok":
            _, bus, role, _ = msg
            print(f"  - OK: {role} (bus{bus}) online")
            init_ok.add((bus, role))
        elif kind == "init_error":
            _, bus, role, err = msg
            print(f"  - FAIL: {role} (bus{bus}) init error: {err}", file=sys.stderr)
            init_fail = True

    if init_fail or not init_ok:
        print("ERROR: One or more ToF sensors failed to init.", file=sys.stderr)
        _stop = True

    print("\n t(s)  | FR_raw(cm) | FL_raw(cm) | FR_filt(cm) | FL_filt(cm) | alert")
    print("-" * 72)

    latest = {
        "FR": {"raw": -1.0, "filt": -1.0},
        "FL": {"raw": -1.0, "filt": -1.0},
    }

    try:
        while not _stop:
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue

            kind = msg[0]
            if kind == "data":
                _, bus, role, raw_cm, filt_cm, t_now = msg
                latest[role]["raw"] = raw_cm
                latest[role]["filt"] = filt_cm

                vals = [v["filt"] for v in latest.values() if v["filt"] >= 0.0]
                alert = any(v < args.threshold for v in vals)

                t_rel = t_now - start_time

                def fmt(v: float) -> str:
                    return f"{v:>7.1f}" if v >= 0 else "   --- "

                line = (
                    f"{t_rel:6.2f} | "
                    f"{fmt(latest['FR']['raw'])} | "
                    f"{fmt(latest['FL']['raw'])} | "
                    f"{fmt(latest['FR']['filt'])} | "
                    f"{fmt(latest['FL']['filt'])} | "
                    f"{'!' if alert else '-'}"
                )
                print(line)

            elif kind == "done":
                _, bus, role, _ = msg
                print(f"[INFO] Worker {role}(bus{bus}) exited.", file=sys.stderr)

    except KeyboardInterrupt:
        _stop = True
    finally:
        for p in procs:
            if p.is_alive():
                p.terminate()
        for p in procs:
            p.join(timeout=1.0)
        print("Done.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
