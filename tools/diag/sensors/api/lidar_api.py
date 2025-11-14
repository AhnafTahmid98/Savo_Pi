#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — RPLIDAR A1 Safety API (Python only, no ROS2)
---------------------------------------------------------
Thin wrapper around rplidar for near-field safety & diagnostics.

Features:
- Robust connect() with reset + motor start (similar to lidar_test.py).
- Per-scan API: poll() → LidarReading (min distance, angle, scan_hz, obstacle flag).
- Front-sector gating via angle_min/angle_max + range_min/range_max.
- Debounced obstacle detection with hysteresis (same idea as lidar_test.py).
- Simple CLI test: prints one debug line per scan.

Typical usage in another script (e.g. drive_automode.py):

    from lidar_api import LidarSafetyAPI

    api = LidarSafetyAPI(
        port="/dev/ttyUSB0",
        angle_min=-35.0,
        angle_max=35.0,
        range_min=0.05,
        range_max=2.0,
        obst_threshold=0.28,
        debounce=3,
        hyst=0.03,
        verbose=False,   # or True for prints
    )
    api.start()

    try:
        while True:
            reading = api.poll()   # blocks until next scan
            if reading.obstacle:
                # trigger stop / reverse / safety behavior
                print("Safety STOP, LiDAR says obstacle!", reading)
            time.sleep(0.01)
    finally:
        api.stop()

Author: Robot Savo
"""

import os
import sys
import time
import math
import glob
import argparse
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

# Local pip target (no system-wide install required):
#   python3 -m pip install --target ~/Savo_Pi/.pylibs rplidar
sys.path.insert(0, os.path.expanduser('~/Savo_Pi/.pylibs'))

try:
    from rplidar import RPLidar, RPLidarException
    _IMPORT_ERROR: Optional[BaseException] = None
except Exception as e:
    # Degrade gracefully if imported as a module without rplidar installed.
    RPLidar = None  # type: ignore
    class RPLidarException(Exception):
        pass
    _IMPORT_ERROR = e


# ---------------- basics ----------------
def now() -> float:
    return time.time()


def clamp(v: int, a: int, b: int) -> int:
    return max(a, min(b, v))


# ---------------- dataclasses ----------------
@dataclass
class LidarReading:
    """
    Single "front-sector" summary for one LiDAR rotation.
    """
    t_epoch_s: float
    scan_id: int
    n_points: int
    scan_hz: float
    pts_per_sec: float
    min_dist_m: Optional[float]
    min_angle_deg: Optional[float]
    obstacle: bool


# ---------------- filters ----------------
def gate(points: Iterable[Tuple[int, float, int]],
         a_min: float, a_max: float,
         r_min: float, r_max: float) -> List[Tuple[int, float, float]]:
    """
    (quality, angle_deg, dist_mm) → filtered list of (q, angle_deg, dist_m).
    Same idea as in lidar_test.py but very small & focused.
    """
    out: List[Tuple[int, float, float]] = []
    for q, ang, d_mm in points:
        if d_mm <= 0:
            continue
        d_m = d_mm / 1000.0
        if a_min <= ang <= a_max and r_min <= d_m <= r_max:
            out.append((q, ang, d_m))
    return out


# ---------------- alert utils ----------------
class AlertState:
    """
    Debounced obstacle / clear state (scan-based).
    """
    def __init__(self, obst: Optional[float], debounce: int, hyst: float):
        self.obst = obst
        self.debounce = max(1, debounce)
        self.hyst = max(0.0, hyst)
        self.hit_cnt = 0
        self.on = False

    def update(self, d_min: Optional[float]) -> bool:
        """
        Update internal state for this scan.
        Returns True if the alert state CHANGED (OFF→ON or ON→OFF), False otherwise.
        """
        if self.obst is None or d_min is None:
            # No threshold or no measurement; don't auto-clear if already on.
            return False

        if not self.on:
            # Waiting to trigger.
            self.hit_cnt = self.hit_cnt + 1 if d_min <= self.obst else 0
            if self.hit_cnt >= self.debounce:
                self.on = True
                return True
        else:
            # Active; require clear above obst + hyst.
            if d_min > self.obst + self.hyst:
                self.on = False
                self.hit_cnt = 0
                return True

        return False


# ---------------- connect/init ----------------
def connect_open(port: str,
                 baud: int,
                 timeout: float,
                 motor_pwm: Optional[int]) -> "RPLidar":
    """
    Open RPLidar on given serial port, reset, flush, and start motor+scan.

    This is a trimmed version of lidar_test.py:connect_open().
    """
    if RPLidar is None:
        raise RPLidarException(
            f"'rplidar' import failed: {_IMPORT_ERROR!r}\n"
            "Install with:\n"
            "  python3 -m pip install --target ~/Savo_Pi/.pylibs rplidar"
        )

    # Resolve glob if by-id wildcard passed.
    if "*" in port:
        matches = glob.glob(port)
        port = matches[0] if matches else "/dev/ttyUSB0"

    print(f"[LiDAR-API] Connecting: port='{port}' baud={baud} timeout={timeout:.1f}s ...")
    lidar = RPLidar(port=port, baudrate=baud, timeout=timeout)

    # Clear any stale state.
    for fn in (lidar.stop, lidar.stop_motor):
        try:
            fn()
        except Exception:
            pass

    # Info & health.
    info = lidar.get_info()
    fw = info.get("firmware", (0, 0))
    print(
        "[LiDAR-API] Info: "
        f"model={info.get('model','?')} fw={fw[0]}.{fw[1]} "
        f"hw={info.get('hardware',0)} serial={info.get('serialnumber','n/a')}"
    )
    status, err = lidar.get_health()
    print(f"[LiDAR-API] Health: status={status} error_code={err}")
    if str(status).lower() != "good":
        raise RPLidarException(f"Health not GOOD: {status} (err={err})")

    # Reset & flush a bit.
    try:
        lidar.reset()
        time.sleep(0.25)
    except Exception:
        pass

    try:
        lidar.clear_input()
    except Exception:
        pass

    # Motor control.
    if motor_pwm is not None:
        pwm = clamp(motor_pwm, 0, 1023)
        print(f"[LiDAR-API] Starting motor with PWM={pwm} ...")
        lidar.set_pwm(pwm)
    else:
        print("[LiDAR-API] Starting motor (default speed) ...")
        lidar.start_motor()

    time.sleep(0.5)

    # Kick scan; some forks require this.
    try:
        lidar.start_scan()
    except Exception:
        pass

    print("[LiDAR-API] Connected, motor running.")
    return lidar


def safe_cleanup(lidar: Optional["RPLidar"]) -> None:
    if not lidar:
        return
    for fn in (lidar.stop, lidar.stop_motor, lidar.disconnect):
        try:
            fn()
        except Exception:
            pass


# ---------------- main API class ----------------
class LidarSafetyAPI:
    """
    High-level LiDAR front-sector safety helper.

    - Call start() once.
    - Then call poll() repeatedly; each call blocks until one scan is ready.
    - poll() returns LidarReading with front min distance and obstacle flag.
    """

    def __init__(self,
                 port: str = "/dev/ttyUSB0",
                 baudrate: int = 115200,
                 timeout: float = 2.0,
                 motor_pwm: Optional[int] = None,
                 angle_min: float = -35.0,
                 angle_max: float = 35.0,
                 range_min: float = 0.05,
                 range_max: float = 2.0,
                 obst_threshold: Optional[float] = 0.28,
                 debounce: int = 3,
                 hyst: float = 0.03,
                 min_len: int = 120,
                 verbose: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.motor_pwm = motor_pwm

        self.angle_min = angle_min
        self.angle_max = angle_max
        self.range_min = range_min
        self.range_max = range_max

        self.min_len = min_len
        self.verbose = verbose

        self._lidar: Optional["RPLidar"] = None
        self._iter = None
        self._scan_id = 0
        self._last_scan_wall: Optional[float] = None

        self._alert = AlertState(obst_threshold, debounce, hyst)

    # ----- lifecycle -----
    def start(self) -> None:
        """
        Open serial, start motor, and start scan iterator.
        """
        if self._lidar is not None:
            return
        self._lidar = connect_open(
            port=self.port,
            baud=self.baudrate,
            timeout=self.timeout,
            motor_pwm=self.motor_pwm,
        )
        # Use iter_scans with a small buffer & min_len for stable rotations.
        self._iter = self._lidar.iter_scans(max_buf_meas=2048, min_len=self.min_len)
        self._scan_id = 0
        self._last_scan_wall = None
        if self.verbose:
            print("[LiDAR-API] Ready. Polling scans...")

    def stop(self) -> None:
        """
        Stop motor and disconnect.
        """
        if self.verbose:
            print("[LiDAR-API] Stopping and disconnecting...")
        safe_cleanup(self._lidar)
        self._lidar = None
        self._iter = None

    # ----- core reading -----
    def poll(self) -> LidarReading:
        """
        Block until one full LiDAR rotation is available, then:
        - Filter to configured sector.
        - Compute min distance/angle.
        - Update debounced obstacle state.
        - Return LidarReading.

        Raises RPLidarException if something goes badly wrong.
        """
        if self._iter is None:
            raise RPLidarException("LidarSafetyAPI.poll() called before start().")

        raw_scan = next(self._iter)  # may raise RPLidarException
        t_now = now()
        self._scan_id += 1

        # Scan frequency estimate (Hz).
        if self._last_scan_wall is None:
            scan_hz = float("nan")
        else:
            period = max(1e-3, t_now - self._last_scan_wall)
            scan_hz = 1.0 / period
        self._last_scan_wall = t_now

        # Filter + convert mm→m.
        if (self.angle_min > -180.0 or self.angle_max < 180.0 or
                self.range_min > 0.0 or self.range_max < 40.0):
            filt = gate(raw_scan, self.angle_min, self.angle_max,
                        self.range_min, self.range_max)
        else:
            # No gating; just convert.
            filt = [
                (q, ang, d_mm / 1000.0)
                for (q, ang, d_mm) in raw_scan
                if d_mm > 0
            ]

        n = len(filt)
        pts_per_sec = (n * scan_hz) if math.isfinite(scan_hz) else float("nan")

        if n:
            _, ang_min, d_min = min(filt, key=lambda x: x[2])
        else:
            d_min, ang_min = None, None

        # Update debounced obstacle state.
        state_changed = self._alert.update(d_min)
        obstacle = self._alert.on

        reading = LidarReading(
            t_epoch_s=t_now,
            scan_id=self._scan_id,
            n_points=n,
            scan_hz=scan_hz,
            pts_per_sec=pts_per_sec,
            min_dist_m=d_min,
            min_angle_deg=ang_min,
            obstacle=obstacle,
        )

        if self.verbose:
            self._print_reading(reading, state_changed)

        return reading

    # ----- printing -----
    def _print_reading(self, r: LidarReading, state_changed: bool) -> None:
        """
        Human-friendly one-line debug print for testing.
        """
        hz_txt = (
            f"{min(r.scan_hz, 50.0):5.2f} Hz"
            if math.isfinite(r.scan_hz) else "  n/a "
        )
        pps_txt = (
            f"{int(min(r.pts_per_sec, 50000)):5d}/s"
            if math.isfinite(r.pts_per_sec) else "  n/a "
        )
        if r.min_dist_m is not None and r.min_angle_deg is not None:
            md = f"{r.min_dist_m:.3f} m @ {r.min_angle_deg:.1f}°"
        else:
            md = "—"

        suffix = "  **OBSTACLE**" if r.obstacle else ""
        print(
            f"[LiDAR-API] scan={r.scan_id:05d}  {hz_txt}  "
            f"pts/scan={r.n_points:4d}  pts/s={pps_txt}  min={md}{suffix}"
        )

        if state_changed:
            if r.obstacle:
                print(
                    f"[LiDAR-API] ALERT: obstacle within threshold "
                    f"(min_dist={r.min_dist_m:.3f} m)"
                    if r.min_dist_m is not None else
                    "[LiDAR-API] ALERT: obstacle (no distance estimate)"
                )
            else:
                print("[LiDAR-API] CLEAR: obstacle region free again.")


# ---------------- CLI test harness ----------------
def main() -> None:
    ap = argparse.ArgumentParser(
        description="Robot Savo — RPLIDAR A1 Safety API quick test"
    )
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="Serial device (default /dev/ttyUSB0). By-id glob supported.")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=2.0,
                    help="Serial read timeout (s). Use ≥2.0 on CP2102.")
    ap.add_argument("--motor-pwm", type=int, default=None,
                    help="Optional motor PWM 0..1023 (else default speed).")

    ap.add_argument("--angle-min", type=float, default=-35.0)
    ap.add_argument("--angle-max", type=float, default=35.0)
    ap.add_argument("--range-min", type=float, default=0.05)
    ap.add_argument("--range-max", type=float, default=2.0)
    ap.add_argument("--obst", type=float, default=0.28,
                    help="Obstacle threshold (m). Use e.g. 0.28 for Robot Savo bumper.")
    ap.add_argument("--debounce", type=int, default=3,
                    help="Consecutive scans under threshold before ALERT (default: 3).")
    ap.add_argument("--hyst", type=float, default=0.03,
                    help="Hysteresis (m): CLEAR when d > obst + hyst.")
    ap.add_argument("--min-len", type=int, default=120,
                    help="Min points per rotation; try 80–200.")

    ap.add_argument("--duration", type=float, default=None,
                    help="Stop after N seconds (optional).")
    ap.add_argument("--sleep", type=float, default=0.0,
                    help="Extra sleep after each poll (s).")
    ap.add_argument("--quiet", action="store_true",
                    help="Disable per-scan prints (only ALERT/CLEAR).")

    args = ap.parse_args()

    api = LidarSafetyAPI(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        motor_pwm=args.motor_pwm,
        angle_min=args.angle_min,
        angle_max=args.angle_max,
        range_min=args.range_min,
        range_max=args.range_max,
        obst_threshold=args.obst,
        debounce=args.debounce,
        hyst=args.hyst,
        min_len=args.min_len,
        verbose=not args.quiet,
    )

    api.start()
    t0 = now()
    try:
        while True:
            r = api.poll()
            if args.duration is not None and (now() - t0) >= args.duration:
                print("[LiDAR-API] Duration reached; stopping.")
                break
            if args.sleep > 0.0:
                time.sleep(args.sleep)
    except KeyboardInterrupt:
        print("\n[LiDAR-API] Ctrl+C — stopping.")
    finally:
        api.stop()


if __name__ == "__main__":
    main()
