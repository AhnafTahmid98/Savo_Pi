#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Dual VL53L1X ToF API (no ROS2)
-------------------------------------------
High-level helper for drive_automode.py and other Python-only scripts.

- Reads TWO VL53L1X sensors at 0x29:
    * bus 1 -> Front-Right (FR)
    * bus 0 -> Front-Left  (FL)
- Uses multiprocessing: ONE worker process per sensor, because the Python
  VL53L1X driver does not behave well when a single process talks to
  both i2c-1 and i2c-0 at the same time.
- Main process provides a clean API:

    from vl53_dual_api import DualToF

    tof = DualToF(rate_hz=20.0, median=3, threshold_cm=28.0)
    fr_raw, fr_filt, fl_raw, fl_filt, alert_str, any_near = tof.read()

Where:
  * *_raw   are the latest unfiltered distances in cm (or -1.0 if none)
  * *_filt  are median-filtered distances in cm (or -1.0 if none yet)
  * alert_str is:
        "-"                         = no near-field obstacle
        "FR 5.4cm"                  = FR is closer than threshold
        "FL 3.2cm"                  = FL is closer than threshold
        "FR 6.1cm, FL 4.3cm"        = both are closer than threshold
  * any_near is a boolean (True if either side is closer than threshold)

IMPORTANT:
- This module is intended for *Python-only* use (no ROS2).
- For ROS2, we'll create one node per sensor using a simpler single-sensor API.

You can also run this file directly as a demo:

    python3 vl53_dual_api.py --rate 10 --threshold 28 --median 3
"""

import sys
import time
import signal
import queue
from collections import deque
from typing import Optional, Dict, Tuple
from multiprocessing import Process, Queue

I2C_ADDRESS = 0x29


# ---------------------------------------------------------------------------
# Worker-side helpers (run in separate processes)
# ---------------------------------------------------------------------------
def _import_vl53():
    """Import VL53L1X module inside a worker process."""
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


def _worker_read_mm(sensor) -> Optional[int]:
    """Read distance in mm from a VL53L1X instance."""
    try:
        if hasattr(sensor, "get_distance"):
            d = sensor.get_distance()
        else:
            d = getattr(sensor, "distance", None)
    except Exception:
        return None

    if d is None:
        return None
    if isinstance(d, float):
        d = int(round(d))
    if d <= 0 or d >= 4000:
        return None
    return int(d)


def _tof_worker(bus: int, role: str, rate_hz: float, median_n: int, out_q: Queue):
    """
    Worker process for ONE ToF sensor.

    Sends messages to main via out_q:
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
                # 50 ms timing budget, ~70 ms inter-measurement
                sensor.set_timing(50, 70)
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
            d_mm = _worker_read_mm(sensor)
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
# Public API class
# ---------------------------------------------------------------------------
class DualToF:
    """
    Dual VL53L1X helper for Python-only code (e.g. drive_automode.py).

    Usage:
        from vl53_dual_api import DualToF

        tof = DualToF(rate_hz=20.0, median=3, threshold_cm=28.0)

        try:
            while True:
                fr_raw, fr_filt, fl_raw, fl_filt, alert_str, any_near = tof.read()
                if any_near:
                    print("ALERT:", alert_str)
                time.sleep(0.05)
        finally:
            tof.close()
    """

    def __init__(self, rate_hz: float = 10.0, median: int = 3, threshold_cm: float = 28.0):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        if median < 1 or (median % 2) == 0:
            raise ValueError("median must be an odd integer >= 1")

        self.rate_hz = rate_hz
        self.median = median
        self.threshold_cm = threshold_cm

        # workers: bus 1 = FR, bus 0 = FL
        self._workers_cfg = [
            (1, "FR"),
            (0, "FL"),
        ]

        self._q: Queue = Queue()
        self._procs = []
        self._latest: Dict[str, Dict[str, float]] = {
            "FR": {"raw": -1.0, "filt": -1.0},
            "FL": {"raw": -1.0, "filt": -1.0},
        }
        self._inited = False
        self._init_fail = False

        # For convenience, install a Ctrl+C handler for scripts that only use this API
        signal.signal(signal.SIGINT, self._sigint_handler)

        self._start_workers()
        self._wait_for_init(timeout_s=3.0)

        print(
            f"[DualToF] Initialized: FR(bus1), FL(bus0), "
            f"rate={self.rate_hz:.1f} Hz, median={self.median}, "
            f"threshold={self.threshold_cm:.1f} cm"
        )

    # ---------------------- internal setup / teardown ----------------------
    def _sigint_handler(self, signum, frame):
        # Ensure we stop cleanly when Ctrl+C is pressed in main script
        self.close()
        raise KeyboardInterrupt

    def _start_workers(self):
        for bus, role in self._workers_cfg:
            p = Process(
                target=_tof_worker,
                args=(bus, role, self.rate_hz, self.median, self._q),
                daemon=True,
            )
            p.start()
            self._procs.append(p)

    def _wait_for_init(self, timeout_s: float = 3.0):
        """Wait for workers to report init status."""
        start = time.perf_counter()
        ok = set()

        while time.perf_counter() - start < timeout_s and len(ok) < len(self._workers_cfg):
            try:
                msg = self._q.get(timeout=0.5)
            except queue.Empty:
                continue

            kind = msg[0]
            if kind == "init_ok":
                _, bus, role, _ = msg
                print(f"[DualToF] OK: {role}(bus{bus}) online")
                ok.add((bus, role))
            elif kind == "init_error":
                _, bus, role, err = msg
                sys.stderr.write(f"[DualToF] INIT FAIL {role}(bus{bus}): {err}\n")
                self._init_fail = True
                break

        if self._init_fail or not ok:
            self.close()
            raise RuntimeError("DualToF: one or more sensors failed to initialize.")

        self._inited = True

    # ---------------------- public API methods -----------------------------
    def read(self, max_wait_s: float = 0.05) -> Tuple[float, float, float, float, str, bool]:
        """
        Pull new data from workers (non-blocking-ish) and return:

            (fr_raw_cm, fr_filt_cm, fl_raw_cm, fl_filt_cm, alert_str, any_near)

        - max_wait_s: how long we are willing to wait for at least one message.

        raw/filt are -1.0 if no valid reading yet.
        """
        if not self._inited:
            raise RuntimeError("DualToF: read() called before successful initialization.")

        deadline = time.perf_counter() + max_wait_s

        # Drain queue until timeout
        while time.perf_counter() < deadline:
            try:
                msg = self._q.get(timeout=0.005)
            except queue.Empty:
                break

            kind = msg[0]

            if kind == "data":
                _, bus, role, raw_cm, filt_cm, _t = msg
                self._latest[role]["raw"] = float(raw_cm)
                self._latest[role]["filt"] = float(filt_cm)
            elif kind == "done":
                _, bus, role, _ = msg
                sys.stderr.write(f"[DualToF] worker {role}(bus{bus}) exited\n")

        # Build alert string based on latest filtered values
        fr_f = self._latest["FR"]["filt"]
        fl_f = self._latest["FL"]["filt"]

        alerts = []
        if fr_f >= 0.0 and fr_f < self.threshold_cm:
            alerts.append(f"FR {fr_f:.1f}cm")
        if fl_f >= 0.0 and fl_f < self.threshold_cm:
            alerts.append(f"FL {fl_f:.1f}cm")

        alert_str = ", ".join(alerts) if alerts else "-"
        any_near = bool(alerts)

        fr_raw = self._latest["FR"]["raw"]
        fl_raw = self._latest["FL"]["raw"]

        return fr_raw, fr_f, fl_raw, fl_f, alert_str, any_near

    def close(self):
        """Terminate worker processes and clean up."""
        for p in self._procs:
            if p.is_alive():
                p.terminate()
        for p in self._procs:
            p.join(timeout=1.0)
        self._procs.clear()


# ---------------------------------------------------------------------------
# Simple demo when run directly
# ---------------------------------------------------------------------------
def _demo():
    import argparse

    ap = argparse.ArgumentParser(description="Robot Savo — Dual ToF demo using DualToF API")
    ap.add_argument("--rate", type=float, default=10.0, help="Sample rate in Hz (default: 10.0)")
    ap.add_argument("--median", type=int, default=3, help="Median window (odd ≥1, default: 3)")
    ap.add_argument(
        "--threshold",
        type=float,
        default=28.0,
        help="Near-field alert threshold in cm (default: 28.0)",
    )
    args = ap.parse_args()

    tof = DualToF(rate_hz=args.rate, median=args.median, threshold_cm=args.threshold)

    print(
        "\n t(s)  | FR_raw(cm) | FR_filt(cm) | FL_raw(cm) | FL_filt(cm) | Alert"
    )
    print("-" * 78)

    t0 = time.perf_counter()

    try:
        period = 1.0 / args.rate
        while True:
            loop_start = time.perf_counter()
            fr_raw, fr_f, fl_raw, fl_f, alert_str, any_near = tof.read()

            t_rel = loop_start - t0

            def fmt(v: float) -> str:
                return f"{v:>7.1f}" if v >= 0.0 else "   --- "

            line = (
                f"{t_rel:6.2f} | "
                f"{fmt(fr_raw)} | {fmt(fr_f)} | "
                f"{fmt(fl_raw)} | {fmt(fl_f)} | {alert_str}"
            )
            print(line)

            dt = period - (time.perf_counter() - loop_start)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[DualToF] Demo interrupted by user.")
    finally:
        tof.close()
        print("[DualToF] Clean shutdown.")


if __name__ == "__main__":
    _demo()
