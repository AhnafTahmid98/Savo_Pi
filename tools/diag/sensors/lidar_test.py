#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — RPLIDAR A1 Diagnostic
----------------------------------
Expert-grade, safe-by-default scanner for field testing your Slamtec RPLIDAR.

Features
- Prints model/firmware/serial + HEALTH (fails fast if not 'Good').
- Live metrics per FULL rotation: scan_hz, pts/scan, pts/sec (correct).
- Min-distance tracker with bearing and **obstacle alert** (threshold).
- Angle/range gating (e.g., forward sector -60..+60 deg).
- CSV logging (per-scan stats or raw points).
- Clean shutdown: stop() + stop_motor() + disconnect() on every exit path.

Author: Robot Savo

"""

import os, sys
sys.path.insert(0, os.path.expanduser('~/Savo_Pi/.pylibs'))

import argparse
import csv
import math
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

# -------------------------- RPLIDAR import --------------------------
try:
    from rplidar import RPLidar, RPLidarException  # type: ignore
except Exception:
    print("ERROR: 'rplidar' library not found. Install it (project-local is fine):")
    print("  python3 -m pip install --target ~/Savo_Pi/.pylibs rplidar")
    sys.exit(1)

# -------------------------- Helpers --------------------------
def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def now_s() -> float:
    return time.time()

@dataclass
class ScanStats:
    t: float                  # epoch seconds (scan timestamp)
    scan_id: int
    n_points: int
    hz_estimate: float        # rotations per second
    pts_per_sec: float
    min_dist_m: Optional[float]
    min_angle_deg: Optional[float]

def gate_by_angle_and_range(points: Iterable[Tuple[int, float, int]],
                            angle_min: float,
                            angle_max: float,
                            r_min_m: float,
                            r_max_m: float) -> List[Tuple[int, float, float]]:
    """
    Select a sector/range from tuples (quality, angle_deg, distance_mm)
    -> returns [(quality, angle_deg, distance_m), ...]
    """
    out: List[Tuple[int, float, float]] = []
    for q, ang, d_mm in points:
        d_m = d_mm / 1000.0
        if angle_min <= ang <= angle_max and r_min_m <= d_m <= r_max_m:
            out.append((q, ang, d_m))
    return out

def compute_scan_stats(scan_id: int,
                       t_scan: float,
                       n_points: int,
                       period_s: Optional[float],
                       min_pair: Optional[Tuple[int, float, float]]) -> ScanStats:
    if period_s and period_s > 0:
        hz = 1.0 / period_s
        pts_per_sec = n_points / period_s
    else:
        hz = float('nan')
        pts_per_sec = float('nan')

    if min_pair is not None:
        _, ang_min, d_min = min_pair
        return ScanStats(t=t_scan, scan_id=scan_id, n_points=n_points,
                         hz_estimate=hz, pts_per_sec=pts_per_sec,
                         min_dist_m=d_min, min_angle_deg=ang_min)
    else:
        return ScanStats(t=t_scan, scan_id=scan_id, n_points=n_points,
                         hz_estimate=hz, pts_per_sec=pts_per_sec,
                         min_dist_m=None, min_angle_deg=None)

# -------------------------- Main Diagnostic --------------------------
def run(args):
    port = args.port
    baudrate = args.baudrate
    motor_pwm = args.motor_pwm
    angle_min = args.angle_min
    angle_max = args.angle_max
    r_min_m   = args.range_min
    r_max_m   = args.range_max
    obst_th   = args.obst
    duration  = args.duration
    max_scans = args.max_scans
    csv_path  = args.csv
    csv_mode  = args.csv_mode

    if angle_min > angle_max:
        print("ERROR: --angle-min must be <= --angle-max")
        sys.exit(2)
    if r_min_m >= r_max_m:
        print("ERROR: --range-min must be < --range-max")
        sys.exit(2)

    # CSV setup (lazy open to avoid partial files on early failure)
    csv_file = None
    csv_writer = None

    lidar: Optional[RPLidar] = None
    scan_count = 0
    t_start = now_s()
    prev_scan_ts: Optional[float] = None
    ret = 0

    try:
        # Short timeout so Ctrl+C breaks read quickly
        print(f"[LiDAR] Connecting: port='{port}' baud={baudrate} ...")
        lidar = RPLidar(port=port, baudrate=baudrate, timeout=0.2)

        info = lidar.get_info()
        model = info.get('model', 'unknown')
        fw = info.get('firmware', (0, 0))
        hw = info.get('hardware', 0)
        ser = info.get('serialnumber', 'n/a')
        print(f"[LiDAR] Info: model={model}  fw={fw[0]}.{fw[1]}  hw={hw}  serial={ser}")

        health = lidar.get_health()
        status = str(health[0])
        err_code = health[1] if len(health) > 1 else 0
        print(f"[LiDAR] Health: status={status}  error_code={err_code}")
        if status.lower() != 'good':
            print("ERROR: LiDAR health is not GOOD. Power-cycle the sensor and recheck.")
            try:
                lidar.stop(); lidar.stop_motor()
            except Exception:
                pass
            sys.exit(3)

        # Start motor
        if motor_pwm is not None:
            pwm = clamp(motor_pwm, 0, 1023)
            print(f"[LiDAR] Starting motor with PWM={pwm} ...")
            lidar.set_pwm(pwm)
        else:
            print("[LiDAR] Starting motor (default speed) ...")
            lidar.start_motor()

        time.sleep(0.4)  # stabilize rotor

        # Prepare CSV if requested
        if csv_path:
            dirname = os.path.dirname(csv_path)
            if dirname and not os.path.exists(dirname):
                os.makedirs(dirname, exist_ok=True)
            csv_file = open(csv_path, "w", newline="")
            csv_writer = csv.writer(csv_file)
            if csv_mode == 'scans':
                csv_writer.writerow([
                    "t_epoch_s", "scan_id", "n_points",
                    "scan_hz", "pts_per_sec", "min_dist_m", "min_angle_deg"
                ])
            else:  # points
                csv_writer.writerow([
                    "t_epoch_s", "scan_id", "quality", "angle_deg", "distance_m"
                ])

        print("[LiDAR] Streaming...  (Ctrl+C to stop)")
        last_print = 0.0

        # Enforce *full-rotation* scans (min_len high; large buffer)
        try:
            for scan in lidar.iter_scans(max_buf_meas=8192, min_len=args.min_len):
                t_scan = now_s()
                scan_count += 1

                # Sector/range gating
                if (angle_min != -180.0) or (angle_max != 180.0) or (r_min_m > 0.0) or (r_max_m < 40.0):
                    filtered = gate_by_angle_and_range(scan, angle_min, angle_max, r_min_m, r_max_m)
                else:
                    filtered = [(q, ang, d_mm/1000.0) for (q, ang, d_mm) in scan]

                # Compute period from consecutive scan timestamps (realistic Hz)
                period = (t_scan - prev_scan_ts) if prev_scan_ts is not None else None
                prev_scan_ts = t_scan

                # Min distance/angle in filtered set
                min_pair = min(filtered, key=lambda x: x[2]) if filtered else None

                stats = compute_scan_stats(
                    scan_id=scan_count, t_scan=t_scan,
                    n_points=len(filtered), period_s=period,
                    min_pair=min_pair
                )

                # Obstacle alert check
                obstacle_hit = bool(
                    obst_th is not None and
                    stats.min_dist_m is not None and
                    stats.min_dist_m <= obst_th
                )

                # Console line (rate-limited to ~10 Hz)
                if (t_scan - last_print) >= 0.1:
                    last_print = t_scan
                    md_txt = (
                        f"{stats.min_dist_m:.3f} m @ {stats.min_angle_deg:.1f}°"
                        if stats.min_dist_m is not None else "—"
                    )
                    hz_txt = f"{stats.hz_estimate:4.2f} Hz" if math.isfinite(stats.hz_estimate) else " n/a "
                    pps_txt = f"{int(round(stats.pts_per_sec))}/s" if math.isfinite(stats.pts_per_sec) else "n/a"
                    alert = "  **OBSTACLE**" if obstacle_hit else ""
                    print(f"[scan {stats.scan_id:05d}] {hz_txt}  pts/scan={stats.n_points:4d}  pts/s={pps_txt}  min={md_txt}{alert}")

                # CSV logging
                if csv_writer:
                    if csv_mode == 'scans':
                        csv_writer.writerow([
                            f"{stats.t:.6f}", stats.scan_id, stats.n_points,
                            f"{stats.hz_estimate:.4f}" if math.isfinite(stats.hz_estimate) else "",
                            f"{stats.pts_per_sec:.1f}" if math.isfinite(stats.pts_per_sec) else "",
                            f"{stats.min_dist_m:.4f}" if stats.min_dist_m is not None else "",
                            f"{stats.min_angle_deg:.2f}" if stats.min_angle_deg is not None else "",
                        ])
                    else:  # points
                        for q, ang, d_m in filtered:
                            csv_writer.writerow([f"{t_scan:.6f}", stats.scan_id, q, f"{ang:.2f}", f"{d_m:.4f}"])

                # Stop conditions
                if duration is not None and (now_s() - t_start) >= duration:
                    print("[LiDAR] Duration reached; stopping.")
                    break
                if max_scans is not None and scan_count >= max_scans:
                    print("[LiDAR] Max scans reached; stopping.")
                    break

        except KeyboardInterrupt:
            print("\n[LiDAR] Ctrl+C received — stopping.")

    except RPLidarException as e:
        print(f"RPLidarException: {e}")
        print("Tip: Check power, USB cable, port path, and that no other process uses the port.")
        ret = 4
    except Exception as e:
        print(f"ERROR: {e}")
        ret = 5
    finally:
        # Flush CSV
        try:
            if csv_file:
                csv_file.flush()
                os.fsync(csv_file.fileno())
                csv_file.close()
        except Exception:
            pass
        # Stop lidar safely
        try:
            if lidar is not None:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
                print("[LiDAR] Stopped motor and disconnected.")
        except Exception as e:
            print(f"[LiDAR] Cleanup warning: {e}")
    sys.exit(ret)

# -------------------------- CLI --------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Robot Savo — RPLIDAR A1 Diagnostic (expert edition, Ctrl+C safe, no buzzer)"
    )
    parser.add_argument("--port", type=str,
                        default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*-if00-port0",
                        help="Serial device (prefer /dev/serial/by-id). Glob fallback to ttyUSB0.")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud (default 115200).")
    parser.add_argument("--motor-pwm", type=int, default=None,
                        help="Optional motor PWM (0..1023). If omitted, start_motor() default is used.")

    parser.add_argument("--angle-min", type=float, default=-180.0, help="Min angle (deg) for gating (default -180).")
    parser.add_argument("--angle-max", type=float, default=180.0, help="Max angle (deg) for gating (default +180).")
    parser.add_argument("--range-min", type=float, default=0.05, help="Min distance (m) (default 0.05).")
    parser.add_argument("--range-max", type=float, default=8.0, help="Max distance (m) (default 8.0).")

    parser.add_argument("--obst", type=float, default=None,
                        help="Obstacle threshold (meters). If set, alert when min distance ≤ threshold.")
    parser.add_argument("--duration", type=float, default=None, help="Stop after N seconds (optional).")
    parser.add_argument("--max-scans", type=int, default=None, help="Stop after N scans (optional).")

    parser.add_argument("--csv", type=str, default=None, help="CSV output path (optional).")
    parser.add_argument("--csv-mode", choices=["scans", "points"], default="scans",
                        help="CSV format: per-scan stats or raw per-point rows (default: scans).")

    parser.add_argument("--min-len", type=int, default=200,
                        help="Minimum points per scan (full rotation). If you see no output, try 80–120.")

    args = parser.parse_args()

    # Resolve globbed by-id to first match or fallback to ttyUSB0
    if "*" in args.port:
        import glob
        matches = glob.glob(args.port)
        args.port = matches[0] if matches else "/dev/ttyUSB0"

    run(args)

if __name__ == "__main__":
    main()
