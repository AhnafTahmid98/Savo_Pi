#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Dual VL53L1X ToF API (Python-only, no ROS)
-------------------------------------------------------
Why multiprocessing:
- Many VL53L1X Python drivers keep global I2C state / are not safe across buses.
- Running each sensor in its own process is the most reliable way.

Locked mapping (project standard):
- bus 1 @ 0x29 -> RIGHT  (FR in your older naming)
- bus 0 @ 0x29 -> LEFT   (FL in your older naming)

Public API:
- DualVL53() : starts workers + provides get_latest()
- get_latest() returns distances in meters + health/stale info
"""

from __future__ import annotations

import sys
import time
import queue
from dataclasses import dataclass
from collections import deque
from typing import Optional, Dict, Tuple
from multiprocessing import Process, Queue, Event


I2C_ADDRESS = 0x29

# Locked physical mapping
BUS_RIGHT = 1  # "FR"
BUS_LEFT  = 0  # "FL"


# ----------------------------- Data models -----------------------------

@dataclass
class SensorSample:
    raw_m: Optional[float]   # None if invalid
    filt_m: Optional[float]  # None if invalid
    t_mono: float            # monotonic timestamp


@dataclass
class DualVL53State:
    right: SensorSample
    left: SensorSample
    right_ok: bool
    left_ok: bool
    right_stale: bool
    left_stale: bool


# ----------------------------- Worker helpers -----------------------------

def _import_vl53_driver():
    """
    Import inside worker. Support common driver module names.
    """
    try:
        from VL53L1X import VL53L1X  # type: ignore
        return VL53L1X
    except Exception:
        try:
            from vl53l1x import VL53L1X  # type: ignore
            return VL53L1X
        except Exception as e:
            raise ImportError("VL53L1X Python driver not found (VL53L1X or vl53l1x).") from e


def _read_mm(sensor) -> Optional[int]:
    """
    Read distance in mm. Return None if invalid.
    """
    try:
        if hasattr(sensor, "get_distance"):
            d = sensor.get_distance()
        else:
            d = getattr(sensor, "distance", None)
        if d is None:
            return None
        if isinstance(d, float):
            d = int(round(d))
        d = int(d)
        if d <= 0 or d >= 4000:
            return None
        return d
    except Exception:
        return None


def _median(hist: deque) -> Optional[float]:
    if not hist:
        return None
    s = sorted(hist)
    n = len(s)
    mid = n // 2
    if n % 2:
        return float(s[mid])
    return 0.5 * (s[mid - 1] + s[mid])


def _tof_worker(bus: int, role: str, rate_hz: float, median_n: int, stop_evt: Event, out_q: Queue):
    """
    Worker process:
    - init VL53L1X on bus
    - loop: read, filter, push ("data", role, raw_m, filt_m, t_mono)
    - sends ("init_ok"/"init_error")
    """
    try:
        VL53L1X = _import_vl53_driver()
        sensor = VL53L1X(i2c_bus=bus, i2c_address=I2C_ADDRESS)

        if hasattr(sensor, "open"):
            sensor.open()

        # Best-effort config (driver differences tolerated)
        try:
            sensor.set_distance_mode("short")
        except Exception:
            try:
                sensor.distance_mode = "short"
            except Exception:
                pass

        try:
            if hasattr(sensor, "set_timing"):
                sensor.set_timing(50, 70)
        except Exception:
            pass

        if hasattr(sensor, "start_ranging"):
            sensor.start_ranging()
        elif hasattr(sensor, "start"):
            sensor.start()

        out_q.put(("init_ok", role, ""))
    except Exception as e:
        out_q.put(("init_error", role, str(e)))
        return

    period = 1.0 / max(rate_hz, 0.1)
    hist = deque(maxlen=median_n)

    try:
        while not stop_evt.is_set():
            t0 = time.perf_counter()
            d_mm = _read_mm(sensor)
            raw_m = (d_mm / 1000.0) if d_mm is not None else None

            if raw_m is not None:
                hist.append(raw_m)

            filt_m = _median(hist) if median_n > 1 else (hist[-1] if hist else None)

            out_q.put(("data", role, raw_m, filt_m, t0))

            dt = period - (time.perf_counter() - t0)
            if dt > 0:
                time.sleep(dt)
    finally:
        try:
            if hasattr(sensor, "stop_ranging"):
                sensor.stop_ranging()
            elif hasattr(sensor, "stop"):
                sensor.stop()
        except Exception:
            pass
        out_q.put(("done", role, ""))


# ----------------------------- Public API -----------------------------

class DualVL53:
    """
    Dual VL53L1X ToF reader (RIGHT + LEFT) using one process per sensor.
    """

    def __init__(
        self,
        rate_hz: float = 25.0,
        median_n: int = 5,
        stale_timeout_s: float = 0.30,
        start_immediately: bool = True,
    ):
        if median_n < 1 or (median_n % 2) == 0:
            raise ValueError("median_n must be odd and >= 1")

        self.rate_hz = rate_hz
        self.median_n = median_n
        self.stale_timeout_s = stale_timeout_s

        self._q: Queue = Queue()
        self._stop_evt: Event = Event()
        self._procs: Dict[str, Process] = {}

        now = time.perf_counter()
        self._latest: Dict[str, SensorSample] = {
            "RIGHT": SensorSample(None, None, now),
            "LEFT":  SensorSample(None, None, now),
        }
        self._ok: Dict[str, bool] = {"RIGHT": False, "LEFT": False}

        if start_immediately:
            self.start()

    def start(self) -> None:
        if self._procs:
            return

        cfg = [
            (BUS_RIGHT, "RIGHT"),
            (BUS_LEFT,  "LEFT"),
        ]
        for bus, role in cfg:
            p = Process(
                target=_tof_worker,
                args=(bus, role, self.rate_hz, self.median_n, self._stop_evt, self._q),
                daemon=True,
            )
            p.start()
            self._procs[role] = p

        # Wait briefly for init results
        deadline = time.perf_counter() + 2.0
        while time.perf_counter() < deadline and not all(self._ok.values()):
            try:
                msg = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            kind = msg[0]
            if kind == "init_ok":
                _, role, _ = msg
                self._ok[role] = True
            elif kind == "init_error":
                _, role, err = msg
                self._ok[role] = False
                raise RuntimeError(f"VL53 init failed for {role}: {err}")

    def stop(self) -> None:
        self._stop_evt.set()
        for p in self._procs.values():
            p.join(timeout=1.0)
        # If something is stuck, terminate last resort
        for p in self._procs.values():
            if p.is_alive():
                p.terminate()
        self._procs.clear()

    def pump(self, max_msgs: int = 10) -> None:
        """
        Drain queue (non-blocking) and update latest samples.
        Call this regularly in your main loop.
        """
        n = 0
        while n < max_msgs:
            try:
                msg = self._q.get_nowait()
            except queue.Empty:
                break
            kind = msg[0]
            if kind == "data":
                _, role, raw_m, filt_m, t_mono = msg
                self._latest[role] = SensorSample(raw_m, filt_m, t_mono)
            n += 1

    def get_latest(self) -> DualVL53State:
        """
        Returns latest samples + health flags.
        """
        self.pump()

        now = time.perf_counter()
        r = self._latest["RIGHT"]
        l = self._latest["LEFT"]

        right_stale = (now - r.t_mono) > self.stale_timeout_s
        left_stale  = (now - l.t_mono) > self.stale_timeout_s

        return DualVL53State(
            right=r,
            left=l,
            right_ok=self._ok["RIGHT"],
            left_ok=self._ok["LEFT"],
            right_stale=right_stale,
            left_stale=left_stale,
        )


# ----------------------------- CLI test (viewer) -----------------------------

def _fmt_m(v: Optional[float]) -> str:
    return f"{v:6.3f}" if v is not None else "  --- "

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=25.0)
    ap.add_argument("--median", type=int, default=5)
    ap.add_argument("--stale", type=float, default=0.30)
    ap.add_argument("--threshold", type=float, default=0.20, help="alert threshold in meters")
    args = ap.parse_args()

    tof = DualVL53(rate_hz=args.rate, median_n=args.median, stale_timeout_s=args.stale)
    t0 = time.perf_counter()

    print(" t(s) | R_raw  R_filt | L_raw  L_filt | Alert")
    print("-" * 60)
    try:
        while True:
            st = tof.get_latest()
            tr = time.perf_counter() - t0

            alerts = []
            if st.right.filt_m is not None and st.right.filt_m < args.threshold:
                alerts.append(f"RIGHT {st.right.filt_m:.3f}m")
            if st.left.filt_m is not None and st.left.filt_m < args.threshold:
                alerts.append(f"LEFT {st.left.filt_m:.3f}m")

            if st.right_stale: alerts.append("RIGHT STALE")
            if st.left_stale:  alerts.append("LEFT STALE")

            alert = ", ".join(alerts) if alerts else "-"

            print(
                f"{tr:5.1f} | {_fmt_m(st.right.raw_m)} {_fmt_m(st.right.filt_m)} | "
                f"{_fmt_m(st.left.raw_m)} {_fmt_m(st.left.filt_m)} | {alert}"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        tof.stop()

if __name__ == "__main__":
    sys.exit(main())
