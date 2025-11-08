#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — RPLIDAR A1 Diagnostic
----------------------------------
Expert-grade, safe-by-default scanner for field testing your Slamtec RPLIDAR.

Features
- Prints model/firmware/serial and HEALTH (fails fast if not 'Good').
- Live scan metrics: scan rate (Hz), points/scan, points/sec.
- Min-distance tracker with angle (bearing) and simple obstacle alert.
- Angle/range gating to analyze a sector (e.g. forward  -45..+45 deg).
- CSV logging: choose per-scan stats or raw per-point rows.
- Clean shutdown: stop() + stop_motor() + disconnect() on any exit path.

Author: Robot Savo

"""

import argparse
import csv
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

try:
    from rplidar import RPLidar, RPLidarException
except Exception as e:
    print("ERROR: 'rplidar' library not found. Install with: pip3 install rplidar", file=sys.stderr)
    raise

# -------------------------- Helpers --------------------------

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def now_s() -> float:
    return time.time()

@dataclass
class ScanStats:
    t: float                  # epoch seconds (start of scan)
    scan_id: int
    n_points: int
    hz_estimate: float        # 1 / scan_period (if measurable)
    pts_per_sec: float
    min_dist_m: Optional[float]
    min_angle_deg: Optional[float]


def gate_by_angle_and_range(points: Iterable[Tuple[float, float, int]],
                            angle_min: float,
                            angle_max: float,
                            r_min_m: float,
                            r_max_m: float):
    """
    points: iterable of (quality, angle_deg, distance_mm)
    returns filtered list of (quality, angle_deg, distance_m)
    """
    out = []
    for q, ang, d_mm in points:
        d_m = d_mm / 1000.0
        if angle_min <= ang <= angle_max and r_min_m <= d_m <= r_max_m:
            out.append((q, ang, d_m))
    return out


def compute_scan_stats(scan_id: int,
                       t0: float,
                       t1: float,
                       points_m: List[Tuple[int, float, float]]) -> ScanStats:
    n = len(points_m)
    period = (t1 - t0) if t1 > t0 else float('nan')
    hz = (1.0 / period) if period and period > 0 else float('nan')
    pts_per_sec = (n / period) if period and period > 0 else float('nan')
    if n > 0:
        # min distance and corresponding angle
        q_min, ang_min, d_min = min(points_m, key=lambda x: x[2])
        return ScanStats(t=t0, scan_id=scan_id, n_points=n,
                         hz_estimate=hz, pts_per_sec=pts_per_sec,
                         min_dist_m=d_min, min_angle_deg=ang_min)
    else:
        return ScanStats(t=t0, scan_id=scan_id, n_points=0,
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
        print("ERROR: angle-min must be <= angle-max", file=sys.stderr)
        sys.exit(2)
    if r_min_m >= r_max_m:
        print("ERROR: range-min must be < range-max", file=sys.stderr)
        sys.exit(2)

    # CSV setup (lazy open to avoid partial files on early failure)
    csv_file = None
    csv_writer = None

    # Graceful stop flag
    stop_flag = {"stop": False}
    def _sigint_handler(sig, frame):
        stop_flag["stop"] = True
    signal.signal(signal.SIGINT, _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)

    lidar: Optional[RPLidar] = None
    scan_iter = None
    scan_count = 0
    t_start = now_s()

    try:
        print(f"[LiDAR] Connecting: port='{port}' baud={baudrate} ...")
        lidar = RPLidar(port=port, baudrate=baudrate, timeout=3)

        info = lidar.get_info()
        model = info.get('model', 'unknown')
        fw1, fw2 = info.get('firmware', (0, 0))
        hw = info.get('hardware', 0)
        ser = info.get('serialnumber', 'n/a')
        print(f"[LiDAR] Info: model={model}  fw={fw1}.{fw2}  hw={hw}  serial={ser}")

        health = lidar.get_health()
        status = health[0]
        err_code = health[1] if len(health) > 1 else 0
        print(f"[LiDAR] Health: status={status}  error_code={err_code}")
        if str(status).lower() != 'good':
            print("ERROR: LiDAR health is not GOOD. Power-cycle the sensor and recheck.", file=sys.stderr)
            try:
                lidar.stop()
                lidar.stop_motor()
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

        # Give rotor time to stabilize speed
        time.sleep(0.5)

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

        # Iterate scans
        # Note: RPLidar.iter_scans returns an iterable of scans; each scan is a list of tuples (quality, angle_deg, distance_mm)
        print("[LiDAR] Streaming...  (Ctrl+C to stop)")
        last_print = 0.0
        for scan in lidar.iter_scans(max_buf_meas=500, min_len=5):
            if stop_flag["stop"]:
                break
            t0 = now_s()
            scan_count += 1

            # Optional gating (angle/range)
            if (angle_min != -180.0) or (angle_max != 180.0) or (r_min_m > 0.0) or (r_max_m < 40.0):
                filtered = gate_by_angle_and_range(scan, angle_min, angle_max, r_min_m, r_max_m)
            else:
                # Convert to meters without gating
                filtered = [(q, ang, d_mm/1000.0) for (q, ang, d_mm) in scan]

            t1 = now_s()
            stats = compute_scan_stats(scan_id=scan_count, t0=t0, t1=t1, points_m=filtered)

            # Obstacle alert
            obstacle_hit = False
            if obst_th is not None and stats.min_dist_m is not None:
                if stats.min_dist_m <= obst_th:
                    obstacle_hit = True

            # Periodic console line (10 Hz max to keep it readable)
            if (t1 - last_print) >= 0.1:
                last_print = t1
                md_txt = f"{stats.min_dist_m:.3f} m @ {stats.min_angle_deg:.1f}°" if stats.min_dist_m is not None else "—"
                hz_txt = f"{stats.hz_estimate:5.2f} Hz" if math.isfinite(stats.hz_estimate) else "  n/a  "
                pps_txt = f"{int(stats.pts_per_sec):5d}/s" if math.isfinite(stats.pts_per_sec) else "  n/a "
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
                        csv_writer.writerow([f"{t0:.6f}", stats.scan_id, q, f"{ang:.2f}", f"{d_m:.4f}"])

            # Stop conditions
            if duration is not None and (now_s() - t_start) >= duration:
                print("[LiDAR] Duration reached; stopping.")
                break
            if max_scans is not None and scan_count >= max_scans:
                print("[LiDAR] Max scans reached; stopping.")
                break

    except RPLidarException as e:
        print(f"RPLidarException: {e}", file=sys.stderr)
        print("Tip: Check power, USB cable, and that the port is correct and not in use.", file=sys.stderr)
        ret = 4
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        ret = 5
    else:
        ret = 0
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
            print(f"[LiDAR] Cleanup warning: {e}", file=sys.stderr)

    sys.exit(ret)


def main():
    parser = argparse.ArgumentParser(
        description="Robot Savo — RPLIDAR A1 Diagnostic (expert edition)"
    )
    parser.add_argument("--port", type=str,
                        default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*-if00-port0",
                        help="Serial device (use by-id path if available). Default tries CP2102 by-id glob.")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate (default 115200).")
    parser.add_argument("--motor-pwm", type=int, default=None,
                        help="Optional motor PWM (0..1023). If omitted, start_motor() default speed is used.")
    parser.add_argument("--angle-min", type=float, default=-180.0, help="Min angle (deg) for gating (default -180).")
    parser.add_argument("--angle-max", type=float, default=180.0, help="Max angle (deg) for gating (default +180).")
    parser.add_argument("--range-min", type=float, default=0.05, help="Min distance (meters) (default 0.05 m).")
    parser.add_argument("--range-max", type=float, default=8.0, help="Max distance (meters) (default 8.0 m).")
    parser.add_argument("--obst", type=float, default=None,
                        help="Obstacle threshold (meters). If set, prints alert when min distance ≤ threshold.")
    parser.add_argument("--duration", type=float, default=None, help="Stop after N seconds (default: run until Ctrl+C).")
    parser.add_argument("--max-scans", type=int, default=None, help="Stop after N scans (optional).")

    parser.add_argument("--csv", type=str, default=None, help="CSV output path (optional).")
    parser.add_argument("--csv-mode", choices=["scans", "points"], default="scans",
                        help="CSV format: per-scan stats or raw per-point rows (default: scans).")

    args = parser.parse_args()

    # If user left default by-id glob and it doesn't exist, fall back to ttyUSB0
    if "*" in args.port:
        # simple glob fallback
        import glob
        matches = glob.glob(args.port)
        if matches:
            args.port = matches[0]
        else:
            args.port = "/dev/ttyUSB0"

    run(args)


if __name__ == "__main__":
    main()
