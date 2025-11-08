#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — RPLIDAR A1 Diagnostic (resync-hardened, Ctrl+C safe, no buzzer)
-----------------------------------------------------------------------------

- Prints model/firmware/serial + HEALTH (fails fast if not 'Good').
- Full-rotation metrics: scan_hz, pts/scan, pts/sec (correct).
- Min-distance + bearing + optional obstacle alert (--obst meters).
- Angle/range gating (e.g. -60..+60 deg forward sector).
- CSV logging (--csv scans.csv --csv-mode scans|points).
- Robust to stream desync: auto-resync + retry on common frame errors.
- Clean shutdown on Ctrl+C (stop, stop_motor, disconnect).

Notes: Mapping/SLAM is out of scope; this is a diagnostic streamer.
"""

# --- ensure project-local libs (works even if env.sh wasn't sourced) ---
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
    print("ERROR: 'rplidar' not found. Install locally:")
    print("  python3 -m pip install --target ~/Savo_Pi/.pylibs rplidar")
    sys.exit(1)

# -------------------------- Helpers --------------------------
def clamp(val, lo, hi): return max(lo, min(hi, val))
def now_s() -> float: return time.time()

@dataclass
class ScanStats:
    t: float
    scan_id: int
    n_points: int
    hz_estimate: float
    pts_per_sec: float
    min_dist_m: Optional[float]
    min_angle_deg: Optional[float]

def gate_by_angle_and_range(points: Iterable[Tuple[int, float, int]],
                            angle_min: float, angle_max: float,
                            r_min_m: float, r_max_m: float) -> List[Tuple[int, float, float]]:
    out: List[Tuple[int, float, float]] = []
    for q, ang, d_mm in points:
        d_m = d_mm / 1000.0
        if angle_min <= ang <= angle_max and r_min_m <= d_m <= r_max_m:
            out.append((q, ang, d_m))
    return out

def compute_scan_stats(scan_id: int, t_scan: float, n_points: int,
                       period_s: Optional[float],
                       min_pair: Optional[Tuple[int, float, float]]) -> ScanStats:
    if period_s and period_s > 0:
        hz = 1.0 / period_s
        pps = n_points / period_s
    else:
        hz = float('nan')
        pps = float('nan')
    if min_pair is not None:
        _, ang_min, d_min = min_pair
        return ScanStats(t_scan, scan_id, n_points, hz, pps, d_min, ang_min)
    return ScanStats(t_scan, scan_id, n_points, hz, pps, None, None)

def resync_lidar(lidar: "RPLidar"):
    """Recover from framing errors: stop → stop_motor → restart → flush."""
    try: lidar.stop()
    except Exception: pass
    try: lidar.stop_motor()
    except Exception: pass
    time.sleep(0.5)
    try: lidar.start_motor()
    except Exception: pass
    time.sleep(1.0)
    try: lidar.clear_input()
    except Exception: pass

# -------------------------- Main --------------------------
def run(args):
    port         = args.port
    baudrate     = args.baudrate
    motor_pwm    = args.motor_pwn if hasattr(args, 'motor_pwn') else args.motor_pwm  # compat
    angle_min    = args.angle_min
    angle_max    = args.angle_max
    r_min_m      = args.range_min
    r_max_m      = args.range_max
    obst_th      = args.obst
    duration     = args.duration
    max_scans    = args.max_scans
    csv_path     = args.csv
    csv_mode     = args.csv_mode
    timeout_s    = args.timeout
    max_retries  = args.retries

    if angle_min > angle_max:
        print("ERROR: --angle-min must be <= --angle-max"); sys.exit(2)
    if r_min_m >= r_max_m:
        print("ERROR: --range-min must be < --range-max"); sys.exit(2)

    csv_file = None; csv_writer = None
    lidar: Optional[RPLidar] = None
    scan_count = 0
    t_start = now_s()
    prev_scan_ts: Optional[float] = None
    ret = 0

    try:
        print(f"[LiDAR] Connecting: port='{port}' baud={baudrate} timeout={timeout_s:.1f}s ...")
        lidar = RPLidar(port=port, baudrate=baudrate, timeout=timeout_s)

        info = lidar.get_info()
        model = info.get('model', 'unknown'); fw = info.get('firmware', (0, 0))
        hw = info.get('hardware', 0); ser = info.get('serialnumber', 'n/a')
        print(f"[LiDAR] Info: model={model}  fw={fw[0]}.{fw[1]}  hw={hw}  serial={ser}")

        status, err_code = lidar.get_health()
        print(f"[LiDAR] Health: status={status}  error_code={err_code}")
        if str(status).lower() != 'good':
            print("ERROR: LiDAR health is not GOOD. Power-cycle and retry.")
            try: lidar.stop(); lidar.stop_motor()
            except Exception: pass
            sys.exit(3)

        # Start motor
        if motor_pwm is not None:
            pwm = clamp(motor_pwm, 0, 1023)
            print(f"[LiDAR] Starting motor with PWM={pwm} ...")
            lidar.set_pwm(pwm)
        else:
            print("[LiDAR] Starting motor (default speed) ...")
            lidar.start_motor()

        # Stabilize + flush any garbage bytes
        time.sleep(1.0)
        try: lidar.clear_input()
        except Exception: pass

        # CSV init
        if csv_path:
            os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
            csv_file = open(csv_path, "w", newline="")
            csv_writer = csv.writer(csv_file)
            if csv_mode == 'scans':
                csv_writer.writerow(["t_epoch_s","scan_id","n_points","scan_hz","pts_per_sec","min_dist_m","min_angle_deg"])
            else:
                csv_writer.writerow(["t_epoch_s","scan_id","quality","angle_deg","distance_m"])

        print("[LiDAR] Streaming...  (Ctrl+C to stop)")
        last_print = 0.0
        retries = 0

        try:
            while True:
                try:
                    # iterate full-rotation scans
                    for scan in lidar.iter_scans(max_buf_meas=8192, min_len=args.min_len):
                        t_scan = now_s()
                        scan_count += 1

                        # sector/range gating
                        if (angle_min != -180.0) or (angle_max != 180.0) or (r_min_m > 0.0) or (r_max_m < 40.0):
                            filtered = gate_by_angle_and_range(scan, angle_min, angle_max, r_min_m, r_max_m)
                        else:
                            filtered = [(q, ang, d_mm/1000.0) for (q, ang, d_mm) in scan]

                        # period & stats
                        period = (t_scan - prev_scan_ts) if prev_scan_ts is not None else None
                        prev_scan_ts = t_scan
                        min_pair = min(filtered, key=lambda x: x[2]) if filtered else None
                        stats = compute_scan_stats(scan_count, t_scan, len(filtered), period, min_pair)

                        # obstacle check
                        obstacle_hit = (obst_th is not None and stats.min_dist_m is not None and stats.min_dist_m <= obst_th)

                        # print (<=10 Hz)
                        if (t_scan - last_print) >= 0.1:
                            last_print = t_scan
                            md_txt = f"{stats.min_dist_m:.3f} m @ {stats.min_angle_deg:.1f}°" if stats.min_dist_m is not None else "—"
                            hz_txt = f"{stats.hz_estimate:4.2f} Hz" if math.isfinite(stats.hz_estimate) else " n/a "
                            pps_txt = f"{int(round(stats.pts_per_sec))}/s" if math.isfinite(stats.pts_per_sec) else "n/a"
                            alert = "  **OBSTACLE**" if obstacle_hit else ""
                            print(f"[scan {stats.scan_id:05d}] {hz_txt}  pts/scan={stats.n_points:4d}  pts/s={pps_txt}  min={md_txt}{alert}")

                        # CSV
                        if csv_writer:
                            if csv_mode == 'scans':
                                csv_writer.writerow([
                                    f"{stats.t:.6f}", stats.scan_id, stats.n_points,
                                    f"{stats.hz_estimate:.4f}" if math.isfinite(stats.hz_estimate) else "",
                                    f"{stats.pts_per_sec:.1f}" if math.isfinite(stats.pts_per_sec) else "",
                                    f"{stats.min_dist_m:.4f}" if stats.min_dist_m is not None else "",
                                    f"{stats.min_angle_deg:.2f}" if stats.min_angle_deg is not None else "",
                                ])
                            else:
                                for q, ang, d_m in filtered:
                                    csv_writer.writerow([f"{t_scan:.6f}", stats.scan_id, q, f"{ang:.2f}", f"{d_m:.4f}"])

                        # stop conditions
                        if duration is not None and (now_s() - t_start) >= duration:
                            print("[LiDAR] Duration reached; stopping."); raise KeyboardInterrupt
                        if max_scans is not None and scan_count >= max_scans:
                            print("[LiDAR] Max scans reached; stopping."); raise KeyboardInterrupt

                except RPLidarException as e:
                    msg = str(e)
                    if ("Wrong body size" in msg) or ("Incorrect descriptor starting bytes" in msg):
                        retries += 1
                        if retries > max_retries:
                            raise
                        print(f"[LiDAR] Stream desync ({msg}). Resync {retries}/{max_retries} ...")
                        resync_lidar(lidar)
                        continue  # restart while loop
                    else:
                        raise
                break  # clean exit of while True

        except KeyboardInterrupt:
            print("\n[LiDAR] Ctrl+C received — stopping.")

    except RPLidarException as e:
        print(f"RPLidarException: {e}")
        print("Tip: Check power, USB cable/port, and that no other process uses the device.")
        ret = 4
    except Exception as e:
        print(f"ERROR: {e}")
        ret = 5
    finally:
        try:
            if csv_file:
                csv_file.flush(); os.fsync(csv_file.fileno()); csv_file.close()
        except Exception: pass
        try:
            if lidar is not None:
                lidar.stop(); lidar.stop_motor(); lidar.disconnect()
                print("[LiDAR] Stopped motor and disconnected.")
        except Exception as e:
            print(f"[LiDAR] Cleanup warning: {e}")
    sys.exit(ret)

# -------------------------- CLI --------------------------
def main():
    p = argparse.ArgumentParser(description="Robot Savo — RPLIDAR A1 Diagnostic (resync-hardened)")
    p.add_argument("--port", type=str,
                   default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*-if00-port0",
                   help="Serial device (prefer /dev/serial/by-id). Glob fallback to /dev/ttyUSB0.")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout (s). 0.5–1.0 is robust.")
    p.add_argument("--retries", type=int, default=3, help="Max resync retries on stream desync.")

    p.add_argument("--motor-pwm", type=int, default=None, help="Motor PWM 0..1023. If omitted, default speed.")
    p.add_argument("--angle-min", type=float, default=-180.0)
    p.add_argument("--angle-max", type=float, default=180.0)
    p.add_argument("--range-min", type=float, default=0.05)
    p.add_argument("--range-max", type=float, default=8.0)

    p.add_argument("--obst", type=float, default=None, help="Obstacle threshold (meters).")
    p.add_argument("--duration", type=float, default=None, help="Stop after N seconds.")
    p.add_argument("--max-scans", type=int, default=None, help="Stop after N scans.")

    p.add_argument("--csv", type=str, default=None, help="CSV output path.")
    p.add_argument("--csv-mode", choices=["scans","points"], default="scans")
    p.add_argument("--min-len", type=int, default=200, help="Min points per full rotation. Try 80–120 if sparse.")

    args = p.parse_args()

    # Resolve glob to first match or fallback
    if "*" in args.port:
        import glob
        m = glob.glob(args.port)
        args.port = m[0] if m else "/dev/ttyUSB0"

    run(args)

if __name__ == "__main__":
    main()
