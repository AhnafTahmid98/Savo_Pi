#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — RPLIDAR A1 Safety API (Python only, no ROS2)
---------------------------------------------------------
Thin wrapper around rplidar for near-field safety & diagnostics.

Design:
- Robust init pattern:
    stop → reset → drain_input → start_motor → start_scan → drain_input.
- 360° scan, plus derived sectors:
    * FRONT  : around 0°   (|angle_wrapped| <= 45°)
    * RIGHT  : -135°..-45°
    * BACK   : |angle_wrapped| >= 135°
    * LEFT   :  45°..135°
- Debounced obstacle detection (using the PRIMARY sector only):
    * PRIMARY sector is defined by angle_min..angle_max in the API ctor.
    * For drive_automode.py we still use a FRONT window (≈−35°..+35°).
- CLI test (this file as a script):
    python3 tools/diag/sensors/api/lidar_api.py
    → by default uses FULL 360° as PRIMARY sector (0..360°) and prints:
        - global min
        - per-sector mins (front/right/back/left)
        - ALERT/CLEAR messages when within 'obst' threshold.

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
    RPLidar = None  # type: ignore

    class RPLidarException(Exception):
        pass

    _IMPORT_ERROR = e


# ---------------- basics ----------------
def now() -> float:
    return time.time()


def clamp(v: int, a: int, b: int) -> int:
    return max(a, min(b, v))


def wrap_deg(a: float) -> float:
    """
    Wrap angle to [-180, 180) degrees.
    """
    while a >= 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


# ---------------- dataclasses ----------------
@dataclass
class LidarReading:
    """
    Summary for one LiDAR rotation.
    PRIMARY sector = [angle_min, angle_max] from LidarSafetyAPI ctor.
    """
    t_epoch_s: float
    scan_id: int
    n_points: int          # points in PRIMARY sector
    scan_hz: float
    pts_per_sec: float

    # PRIMARY sector (used for safety thresholds in drive_automode.py)
    min_dist_m: Optional[float]
    min_angle_deg: Optional[float]
    obstacle: bool

    # Global 360° minimum
    global_min_m: Optional[float]
    global_min_angle_deg: Optional[float]

    # Sector mins (360°) – for diagnostics / debug
    front_min_m: Optional[float]
    front_min_angle_deg: Optional[float]
    right_min_m: Optional[float]
    right_min_angle_deg: Optional[float]
    back_min_m: Optional[float]
    back_min_angle_deg: Optional[float]
    left_min_m: Optional[float]
    left_min_angle_deg: Optional[float]


# ---------------- alert utils ----------------
class AlertState:
    """
    Debounced obstacle / clear state (scan-based).
    Uses PRIMARY sector min distance only.
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


# ---------------- io helpers ----------------
def drain_input(lidar: "RPLidar", sec: float = 0.25) -> None:
    """
    Aggressively flush stale bytes from the serial buffer
    for 'sec' seconds.
    """
    t0 = now()
    while now() - t0 < sec:
        try:
            lidar.clear_input()
        except Exception:
            pass
        time.sleep(0.02)


def safe_cleanup(lidar: Optional["RPLidar"]) -> None:
    if not lidar:
        return
    for fn in (lidar.stop, lidar.stop_motor, lidar.disconnect):
        try:
            fn()
        except Exception:
            pass


# ---------------- connect/init ----------------
def connect_open(port: str,
                 baud: int,
                 timeout: float,
                 motor_pwm: Optional[int]) -> "RPLidar":
    """
    Open RPLidar on given serial port, reset, drain, start motor + scan, drain.
    """
    if RPLidar is None:
        raise RPLidarException(
            f"'rplidar' import failed: {_IMPORT_ERROR!r}\n"
            "Install with:\n"
            "  python3 -m pip install --target ~/Savo_Pi/.pylibs rplidar"
        )

    # resolve glob if by-id wildcard passed
    if "*" in port:
        matches = glob.glob(port)
        port = matches[0] if matches else "/dev/ttyUSB0"

    print(f"[LiDAR-API] Connecting: port='{port}' baud={baud} timeout={timeout:.1f}s ...")
    lidar = RPLidar(port=port, baudrate=baud, timeout=timeout)

    # clear any stale state
    for fn in (lidar.stop, lidar.stop_motor):
        try:
            fn()
        except Exception:
            pass

    # info & health
    info = lidar.get_info()
    fw = info.get('firmware', (0, 0))
    print(
        "[LiDAR-API] Info: "
        f"model={info.get('model','?')} fw={fw[0]}.{fw[1]} "
        f"hw={info.get('hardware',0)} serial={info.get('serialnumber','n/a')}"
    )
    status, err = lidar.get_health()
    print(f"[LiDAR-API] Health: status={status} error_code={err}")
    if str(status).lower() != "good":
        raise RPLidarException(f"Health not GOOD: {status} (err={err})")

    # reset & flush
    try:
        lidar.reset()
        time.sleep(0.25)
    except Exception:
        pass
    drain_input(lidar, 0.25)

    # motor
    if motor_pwm is not None:
        pwm = clamp(motor_pwm, 0, 1023)
        print(f"[LiDAR-API] Starting motor with PWM={pwm} ...")
        lidar.set_pwm(pwm)
    else:
        print("[LiDAR-API] Starting motor (default speed) ...")
        lidar.start_motor()

    time.sleep(0.50)
    drain_input(lidar, 0.30)

    # explicitly kick scan; some forks require this
    try:
        lidar.start_scan()
    except Exception:
        pass
    drain_input(lidar, 0.10)

    print("[LiDAR-API] Connected, motor running.")
    return lidar


# ---------------- main API class ----------------
class LidarSafetyAPI:
    """
    High-level LiDAR safety helper.

    - PRIMARY sector is [angle_min, angle_max] (deg).
      * For drive_automode.py we pass something like -35..+35 (front).
      * For CLI default we use 0..360 (full circle).
    - Call start() once, then poll() in a loop.
    - poll() returns LidarReading with:
        * PRIMARY sector min distance + obstacle flag.
        * global 360° min.
        * per-sector mins (front/right/back/left) for debug.
    """

    def __init__(self,
                 port: str = "/dev/ttyUSB0",
                 baudrate: int = 115200,
                 timeout: float = 2.0,
                 motor_pwm: Optional[int] = None,
                 angle_min: float = 0.0,
                 angle_max: float = 360.0,
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

        # PRIMARY sector window
        self.angle_min = angle_min
        self.angle_max = angle_max

        # Distance window
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
    def poll(self, _retry_header: bool = True) -> LidarReading:
        """
        Block until one full LiDAR rotation is available, then:
        - Build 360° point list (within range_min..range_max).
        - Compute PRIMARY sector min distance for safety logic.
        - Compute global + sector mins for diagnostics.
        - Update debounced obstacle state.
        - Return LidarReading.
        """
        if self._iter is None:
            raise RPLidarException("LidarSafetyAPI.poll() called before start().")

        try:
            raw_scan = next(self._iter)  # may raise RPLidarException
        except RPLidarException as e:
            msg = str(e)
            header_keys = ("Wrong body size", "descriptor", "Incorrect descriptor", "length mismatch")
            if _retry_header and any(k in msg for k in header_keys):
                if self.verbose:
                    print(f"[LiDAR-API] Header/desync in poll(): {msg} → draining input & rebuilding iterator.")
                if self._lidar is None:
                    raise

                # Recovery: flush & restart scan.
                try:
                    drain_input(self._lidar, 0.30)
                except Exception:
                    pass
                try:
                    self._lidar.stop()
                except Exception:
                    pass
                time.sleep(0.1)
                try:
                    self._lidar.start_scan()
                except Exception:
                    pass
                drain_input(self._lidar, 0.10)

                # Rebuild iterator; retry once.
                self._iter = self._lidar.iter_scans(max_buf_meas=2048, min_len=self.min_len)
                self._last_scan_wall = None
                return self.poll(_retry_header=False)
            else:
                raise

        t_now = now()
        self._scan_id += 1

        # Scan frequency estimate (Hz).
        if self._last_scan_wall is None:
            scan_hz = float("nan")
        else:
            period = max(1e-3, t_now - self._last_scan_wall)
            scan_hz = 1.0 / period
        self._last_scan_wall = t_now

        # 360° list within [range_min, range_max]
        full_pts: List[Tuple[int, float, float]] = []
        for q, ang_deg, d_mm in raw_scan:
            if d_mm <= 0:
                continue
            d_m = d_mm / 1000.0
            if self.range_min <= d_m <= self.range_max:
                full_pts.append((q, ang_deg, d_m))

        # PRIMARY sector subset (angle_min..angle_max)
        primary_pts: List[Tuple[int, float, float]] = [
            (q, ang, d) for (q, ang, d) in full_pts
            if self.angle_min <= ang <= self.angle_max
        ]

        n_primary = len(primary_pts)
        pts_per_sec = (n_primary * scan_hz) if math.isfinite(scan_hz) else float("nan")

        # PRIMARY min
        if n_primary:
            _, p_ang_min, p_d_min = min(primary_pts, key=lambda x: x[2])
        else:
            p_d_min, p_ang_min = None, None

        # Debounced safety state uses PRIMARY sector only
        state_changed = self._alert.update(p_d_min)
        obstacle = self._alert.on

        # Global 360° min
        if full_pts:
            _, g_ang_min, g_d_min = min(full_pts, key=lambda x: x[2])
        else:
            g_d_min, g_ang_min = None, None

        # Sector mins over 360° (use wrapped angle)
        front_min = right_min = back_min = left_min = None
        front_ang = right_ang = back_ang = left_ang = None

        for _, ang, d in full_pts:
            w = wrap_deg(ang)  # [-180, 180)
            # FRONT: |w| ≤ 45°
            if abs(w) <= 45.0:
                if front_min is None or d < front_min:
                    front_min, front_ang = d, ang
            # RIGHT: -135° < w < -45°
            elif -135.0 < w < -45.0:
                if right_min is None or d < right_min:
                    right_min, right_ang = d, ang
            # LEFT: 45° < w < 135°
            elif 45.0 < w < 135.0:
                if left_min is None or d < left_min:
                    left_min, left_ang = d, ang
            # BACK: else (|w| ≥ 135°)
            else:
                if back_min is None or d < back_min:
                    back_min, back_ang = d, ang

        reading = LidarReading(
            t_epoch_s=t_now,
            scan_id=self._scan_id,
            n_points=n_primary,
            scan_hz=scan_hz,
            pts_per_sec=pts_per_sec,
            min_dist_m=p_d_min,
            min_angle_deg=p_ang_min,
            obstacle=obstacle,
            global_min_m=g_d_min,
            global_min_angle_deg=g_ang_min,
            front_min_m=front_min,
            front_min_angle_deg=front_ang,
            right_min_m=right_min,
            right_min_angle_deg=right_ang,
            back_min_m=back_min,
            back_min_angle_deg=back_ang,
            left_min_m=left_min,
            left_min_angle_deg=left_ang,
        )

        if self.verbose:
            self._print_reading(reading, state_changed)

        return reading

    # ----- printing -----
    def _print_reading(self, r: LidarReading, state_changed: bool) -> None:
        """
        Human-friendly one-line debug print for testing.
        Shows PRIMARY sector + 360° sectors.
        """
        hz_txt = (
            f"{min(r.scan_hz, 50.0):5.2f} Hz"
            if math.isfinite(r.scan_hz) else "  n/a "
        )
        pps_txt = (
            f"{int(min(r.pts_per_sec, 50000)):5d}/s"
            if math.isfinite(r.pts_per_sec) else "  n/a "
        )

        def fmt_md(d: Optional[float], a: Optional[float]) -> str:
            if d is None or a is None:
                return "   —   "
            return f"{d:4.3f} m @ {a:6.1f}°"

        prim_txt = fmt_md(r.min_dist_m, r.min_angle_deg)
        glob_txt = fmt_md(r.global_min_m, r.global_min_angle_deg)
        f_txt = fmt_md(r.front_min_m, r.front_min_angle_deg)
        r_txt = fmt_md(r.right_min_m, r.right_min_angle_deg)
        b_txt = fmt_md(r.back_min_m, r.back_min_angle_deg)
        l_txt = fmt_md(r.left_min_m, r.left_min_angle_deg)

        obst_txt = " **OBSTACLE**" if r.obstacle else ""

        print(
            f"[LiDAR-API] scan={r.scan_id:05d}  {hz_txt}  pts/PRIMARY={r.n_points:4d}  pts/s={pps_txt}\n"
            f"             PRIMARY[{self.angle_min:.1f}..{self.angle_max:.1f}]  : {prim_txt}{obst_txt}\n"
            f"             GLOBAL(0..360)             : {glob_txt}\n"
            f"             FRONT  (±45°)             : {f_txt}\n"
            f"             RIGHT  (-135°..-45°)      : {r_txt}\n"
            f"             BACK   (|angle| ≥135°)    : {b_txt}\n"
            f"             LEFT   (45°..135°)        : {l_txt}"
        )

        if state_changed:
            if r.obstacle:
                if r.min_dist_m is not None:
                    print(
                        f"[LiDAR-API] ALERT: obstacle in PRIMARY sector "
                        f"(min_dist={r.min_dist_m:.3f} m)"
                    )
                else:
                    print("[LiDAR-API] ALERT: obstacle (no distance estimate)")
            else:
                print("[LiDAR-API] CLEAR: PRIMARY sector free again.")


# ---------------- CLI test harness ----------------
def main() -> None:
    ap = argparse.ArgumentParser(
        description="Robot Savo — RPLIDAR A1 Safety API quick test (360° + sectors)"
    )
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="Serial device (default /dev/ttyUSB0). By-id glob supported.")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=2.0,
                    help="Serial read timeout (s). Use ≥2.0 on CP2102.")
    ap.add_argument("--motor-pwm", type=int, default=None,
                    help="Optional motor PWM 0..1023 (else default speed).")

    # IMPORTANT: defaults are 0..360 → PRIMARY = full circle
    ap.add_argument("--angle-min", type=float, default=0.0,
                    help="PRIMARY sector min angle (deg, default 0).")
    ap.add_argument("--angle-max", type=float, default=360.0,
                    help="PRIMARY sector max angle (deg, default 360).")

    ap.add_argument("--range-min", type=float, default=0.05)
    ap.add_argument("--range-max", type=float, default=2.0)
    ap.add_argument("--obst", type=float, default=0.28,
                    help="Obstacle threshold (m) for PRIMARY sector.")
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
            _ = api.poll()
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
