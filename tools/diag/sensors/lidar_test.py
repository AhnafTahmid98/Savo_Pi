#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — RPLIDAR A1 Diagnostic (expert edition)
---------------------------------------------------
- Robust init: stop → reset → flush → start_motor → start_scan → drain.
- SCAN mode (per-rotation stats via iter_scans) with *correct* scan Hz calc.
- RAW mode (iter_measurments) for flaky links; auto-fallback on header/desync.
- Sector & range gates, near-field obstacle alert, CSV logging (scans/points).
- Stable defaults for CP2102 bridges: /dev/ttyUSB0, timeout=2.0s.
- Safe cleanup (idempotent); controlled retries.

Usage
-----
# General sanity (defaults)
python3 tools/diag/sensors/lidar_test.py --duration 8

# Forward sector gate + obstacle alert at 0.28 m
python3 tools/diag/sensors/lidar_test.py --angle-min -60 --angle-max 60 --obst 0.28 --duration 10

# Force RAW (robust) with explicit port/timeout
python3 tools/diag/sensors/lidar_test.py --port /dev/ttyUSB0 --timeout 2.0 --raw --duration 8

# Log per-scan summary
python3 tools/diag/sensors/lidar_test.py --csv scans.csv --csv-mode scans --duration 12
"""

import os
import sys
import csv
import time
import math
import glob
import argparse
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

# Project-local pip target (no system-wide install required):
#   python3 -m pip install --target ~/Savo_Pi/.pylibs rplidar
sys.path.insert(0, os.path.expanduser('~/Savo_Pi/.pylibs'))

try:
    from rplidar import RPLidar, RPLidarException
except Exception:
    print("ERROR: 'rplidar' not found.\n"
          "Install with:\n"
          "  python3 -m pip install --target ~/Savo_Pi/.pylibs rplidar",
          file=sys.stderr)
    sys.exit(1)

# ---------------- basics ----------------
def now() -> float: return time.time()
def clamp(v: int, a: int, b: int) -> int: return max(a, min(b, v))

@dataclass
class ScanStats:
    t: float
    scan_id: int
    n_points: int
    scan_hz: float
    pts_per_sec: float
    min_dist_m: Optional[float]
    min_angle_deg: Optional[float]

# ---------------- filters ----------------
def gate(points: Iterable[Tuple[int, float, int]],
         a_min: float, a_max: float,
         r_min: float, r_max: float) -> List[Tuple[int, float, float]]:
    """(q, angle_deg, dist_mm) → filtered list of (q, angle_deg, dist_m)"""
    out: List[Tuple[int, float, float]] = []
    for q, ang, d_mm in points:
        if d_mm <= 0:
            continue
        d_m = d_mm / 1000.0
        if a_min <= ang <= a_max and r_min <= d_m <= r_max:
            out.append((q, ang, d_m))
    return out

# ---------------- io helpers ----------------
def drain_input(lidar: "RPLidar", sec: float = 0.25) -> None:
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
def connect_open(port: str, baud: int, timeout: float, motor_pwm: Optional[int]) -> "RPLidar":
    # resolve glob if by-id wildcard passed
    if "*" in port:
        matches = glob.glob(port)
        port = matches[0] if matches else "/dev/ttyUSB0"

    print(f"[LiDAR] Connecting: port='{port}' baud={baud} timeout={timeout:.1f}s ...")
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
    print(f"[LiDAR] Info: model={info.get('model','?')} fw={fw[0]}.{fw[1]} hw={info.get('hardware',0)} serial={info.get('serialnumber','n/a')}")
    status, err = lidar.get_health()
    print(f"[LiDAR] Health: status={status} error_code={err}")
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
        print(f"[LiDAR] Starting motor with PWM={pwm} ...")
        lidar.set_pwm(pwm)
    else:
        print("[LiDAR] Starting motor (default speed) ...")
        lidar.start_motor()

    time.sleep(0.50)
    drain_input(lidar, 0.30)

    # explicitly kick scan; some forks require this
    try:
        lidar.start_scan()
    except Exception:
        pass
    drain_input(lidar, 0.10)

    return lidar

# ---------------- printing ----------------
def print_scan_line(s: ScanStats, obstacle: bool) -> None:
    md = f"{s.min_dist_m:.3f} m @ {s.min_angle_deg:.1f}°" if s.min_dist_m is not None else "—"
    hz_txt = f"{min(s.scan_hz, 50.0):5.2f} Hz" if math.isfinite(s.scan_hz) else "  n/a "
    pps = s.pts_per_sec if math.isfinite(s.pts_per_sec) else float("nan")
    pps_txt = f"{int(min(pps, 50000)):5d}/s" if math.isfinite(pps) else "  n/a "
    print(f"[scan {s.scan_id:05d}] {hz_txt}  pts/scan={s.n_points:4d}  pts/s={pps_txt}  min={md}{'  **OBSTACLE**' if obstacle else ''}")

# ---------------- run modes ----------------
def run_scan(lidar: "RPLidar", args) -> None:
    a_min, a_max = args.angle_min, args.angle_max
    r_min, r_max = args.range_min, args.range_max
    obst = args.obst
    t_start = now()
    scan_id = 0
    last_scan_wall: Optional[float] = None

    writer, csvf = None, None
    if args.csv:
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        csvf = open(args.csv, "w", newline="")
        writer = csv.writer(csvf)
        if args.csv_mode == "scans":
            writer.writerow(["t_epoch_s","scan_id","n_points","scan_hz","pts_per_sec","min_dist_m","min_angle_deg"])
        else:
            writer.writerow(["t_epoch_s","scan_id","quality","angle_deg","distance_m"])

    try:
        for raw_scan in lidar.iter_scans(max_buf_meas=2048, min_len=args.min_len):
            scan_id += 1
            t_now = now()

            # time between successive scans → scan_hz
            if last_scan_wall is None:
                scan_hz = float("nan")
            else:
                period = max(1e-3, t_now - last_scan_wall)  # ≥1 ms to avoid spikes
                scan_hz = 1.0 / period
            last_scan_wall = t_now

            # filtering + mm→m
            if (a_min > -180.0) or (a_max < 180.0) or (r_min > 0.0) or (r_max < 40.0):
                filt = gate(raw_scan, a_min, a_max, r_min, r_max)
            else:
                filt = [(q, ang, d / 1000.0) for (q, ang, d) in raw_scan if d > 0]

            n = len(filt)
            pts_per_sec = (n * scan_hz) if math.isfinite(scan_hz) else float("nan")

            if n:
                _, ang_min, d_min = min(filt, key=lambda x: x[2])
            else:
                d_min, ang_min = None, None

            obstacle = bool(obst is not None and d_min is not None and d_min <= obst)

            # print
            s = ScanStats(t=t_now, scan_id=scan_id, n_points=n, scan_hz=scan_hz,
                          pts_per_sec=pts_per_sec, min_dist_m=d_min, min_angle_deg=ang_min)
            print_scan_line(s, obstacle)

            # csv
            if writer:
                if args.csv_mode == "scans":
                    writer.writerow([
                        f"{t_now:.6f}", scan_id, n,
                        f"{scan_hz:.4f}" if math.isfinite(scan_hz) else "",
                        f"{pts_per_sec:.1f}" if math.isfinite(pts_per_sec) else "",
                        f"{d_min:.4f}" if d_min is not None else "",
                        f"{ang_min:.2f}" if ang_min is not None else "",
                    ])
                else:
                    for q, ang, dm in filt:
                        writer.writerow([f"{t_now:.6f}", scan_id, q, f"{ang:.2f}", f"{dm:.4f}"])

            # stopping conditions
            if args.duration is not None and (t_now - t_start) >= args.duration:
                print("[LiDAR] Duration reached; stopping.")
                break
            if args.max_scans is not None and scan_id >= args.max_scans:
                print("[LiDAR] Max scans reached; stopping.")
                break
    finally:
        if csvf:
            try:
                csvf.flush(); os.fsync(csvf.fileno()); csvf.close()
            except Exception:
                pass

def run_raw(lidar: "RPLidar", args) -> None:
    a_min, a_max = args.angle_min, args.angle_max
    r_min, r_max = args.range_min, args.range_max
    obst = args.obst
    t_start = now()
    t0 = now()
    last = 0.0
    n = 0
    min_d = None
    min_ang = None

    print("[LiDAR] RAW mode: streaming measurements (robust).")
    for _, q, ang, d_mm in lidar.iter_measurments(max_buf_meas=4096):
        n += 1
        if d_mm > 0:
            d = d_mm / 1000.0
            if a_min <= ang <= a_max and r_min <= d <= r_max:
                if (min_d is None) or (d < min_d):
                    min_d, min_ang = d, ang

        t = now()
        if (t - last) >= 0.5:
            rate = int(n / max(1e-6, (t - t0)))
            md = f"{min_d:.3f} m @ {min_ang:.1f}°" if min_d is not None else "—"
            alert = "  **OBSTACLE**" if (obst is not None and min_d is not None and min_d <= obst) else ""
            print(f"[RAW] {rate} meas/s   min={md}{alert}")
            last = t

        if args.duration is not None and (t - t_start) >= args.duration:
            print("[LiDAR] Duration reached; stopping.")
            break

# ---------------- main ----------------
def main():
    ap = argparse.ArgumentParser(description="Robot Savo — RPLIDAR A1 Diagnostic (expert)")
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="Serial device (default /dev/ttyUSB0). You can pass a by-id glob if wanted.")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=2.0, help="Serial read timeout (s). Use ≥2.0 on CP2102.")
    ap.add_argument("--motor-pwm", type=int, default=None, help="Optional motor PWM 0..1023.")

    ap.add_argument("--angle-min", type=float, default=-180.0)
    ap.add_argument("--angle-max", type=float, default=180.0)
    ap.add_argument("--range-min", type=float, default=0.05)
    ap.add_argument("--range-max", type=float, default=8.0)
    ap.add_argument("--obst", type=float, default=None, help="Obstacle threshold (m).")

    ap.add_argument("--duration", type=float, default=None, help="Stop after N seconds.")
    ap.add_argument("--max-scans", type=int, default=None, help="Stop after N scans (SCAN mode).")
    ap.add_argument("--min-len", type=int, default=120, help="Min points per rotation; try 80–200.")

    ap.add_argument("--csv", type=str, default=None, help="CSV path (optional).")
    ap.add_argument("--csv-mode", choices=["scans", "points"], default="scans")

    ap.add_argument("--raw", action="store_true", help="Force RAW mode.")
    ap.add_argument("--attempts", type=int, default=3, help="Reconnect attempts on errors.")
    args = ap.parse_args()

    for attempt in range(1, args.attempts + 1):
        lidar = None
        try:
            lidar = connect_open(args.port, args.baudrate, args.timeout, args.motor_pwm)
            print("[LiDAR] Streaming...  (Ctrl+C to stop)")

            try:
                if args.raw:
                    run_raw(lidar, args)
                else:
                    run_scan(lidar, args)
            except RPLidarException as e:
                msg = str(e)
                if any(key in msg for key in ("Wrong body size", "descriptor", "Incorrect descriptor", "length mismatch")):
                    print(f"[LiDAR] Header/desync (attempt {attempt}) → RAW fallback.")
                    drain_input(lidar, 0.30)
                    run_raw(lidar, args)
                else:
                    raise
            break  # success
        except KeyboardInterrupt:
            print("\n[LiDAR] Ctrl+C — stopping.")
            break
        except RPLidarException as e:
            print(f"[LiDAR] RPLidarException on attempt {attempt}: {e}")
            time.sleep(0.6)
            if attempt == args.attempts:
                sys.exit(4)
        except Exception as e:
            print(f"[LiDAR] ERROR (attempt {attempt}): {e}")
            time.sleep(0.6)
            if attempt == args.attempts:
                sys.exit(5)
        finally:
            safe_cleanup(lidar)
            print("[LiDAR] Stopped motor and disconnected.")

if __name__ == "__main__":
    main()
