#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — RPLIDAR A1 Diagnostic (stable-lite)
- Keeps the original, tolerant behavior that worked on your A1.
- No buzzer, has angle/range gating + obstacle alert.
- Clean Ctrl+C handling.
- Project-local libs path so it runs without env.sh.

NOTE: This is intentionally "loose" to avoid frame-size trips on flaky streams.
"""

# project-local libs (no venv required)
import os, sys
sys.path.insert(0, os.path.expanduser('~/Savo_Pi/.pylibs'))

import argparse, csv, math, time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

try:
    from rplidar import RPLidar, RPLidarException  # v0.9.2
except Exception:
    print("ERROR: 'rplidar' not found. Install:")
    print("  python3 -m pip install --target ~/Savo_Pi/.pylibs rplidar")
    sys.exit(1)

def clamp(v,a,b): return max(a, min(b, v))
def now_s()->float: return time.time()

@dataclass
class ScanStats:
    t: float
    scan_id: int
    n_points: int
    hz_estimate: float
    pts_per_sec: float
    min_dist_m: Optional[float]
    min_angle_deg: Optional[float]

def gate(points: Iterable[Tuple[int,float,int]],
         a_min: float, a_max: float, r_min: float, r_max: float):
    out=[]
    for q, ang, d_mm in points:
        d = d_mm/1000.0
        if a_min <= ang <= a_max and r_min <= d <= r_max:
            out.append((q, ang, d))
    return out

def compute_stats(scan_id:int, t0:float, t1:float, pts:List[Tuple[int,float,float]])->ScanStats:
    n = len(pts)
    per = (t1 - t0) if t1>t0 else float('nan')
    hz  = (1.0/per) if per and per>0 else float('nan')
    pps = (n/per)   if per and per>0 else float('nan')
    if n:
        _, ang_min, d_min = min(pts, key=lambda x:x[2])
        return ScanStats(t0, scan_id, n, hz, pps, d_min, ang_min)
    return ScanStats(t0, scan_id, 0, hz, pps, None, None)

def run(args):
    port       = args.port
    baudrate   = args.baudrate
    motor_pwm  = args.motor_pwm
    a_min      = args.angle_min
    a_max      = args.angle_max
    r_min      = args.range_min
    r_max      = args.range_max
    obst_th    = args.obst
    duration   = args.duration
    max_scans  = args.max_scans
    csv_path   = args.csv
    csv_mode   = args.csv_mode

    if a_min > a_max: sys.exit("ERROR: --angle-min must be <= --angle-max")
    if r_min >= r_max: sys.exit("ERROR: --range-min must be < --range-max")

    csvf = None; w = None
    lidar = None
    scan_id = 0
    t_start = now_s()

    try:
        # Keep a moderate timeout; tolerant on slow/dirty reads
        print(f"[LiDAR] Connecting: port='{port}' baud={baudrate} timeout=0.5s ...")
        lidar = RPLidar(port=port, baudrate=baudrate, timeout=0.5)

        info = lidar.get_info()
        model = info.get('model','?'); fw = info.get('firmware',(0,0))
        hw = info.get('hardware',0);   ser= info.get('serialnumber','n/a')
        print(f"[LiDAR] Info: model={model} fw={fw[0]}.{fw[1]} hw={hw} serial={ser}")

        status, err = lidar.get_health()
        print(f"[LiDAR] Health: status={status} error_code={err}")
        if str(status).lower() != 'good':
            print("ERROR: Health not GOOD. Power-cycle and retry."); sys.exit(3)

        if motor_pwm is not None:
            pwm = clamp(motor_pwm,0,1023)
            print(f"[LiDAR] Starting motor with PWM={pwm} ...")
            lidar.set_pwm(pwm)
        else:
            print("[LiDAR] Starting motor (default speed) ...")
            lidar.start_motor()

        time.sleep(0.4)  # small settle

        # CSV
        if csv_path:
            os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
            csvf = open(csv_path, "w", newline="")
            w = csv.writer(csvf)
            if csv_mode == 'scans':
                w.writerow(["t_epoch_s","scan_id","n_points","scan_hz","pts_per_sec","min_dist_m","min_angle_deg"])
            else:
                w.writerow(["t_epoch_s","scan_id","quality","angle_deg","distance_m"])

        print("[LiDAR] Streaming...  (Ctrl+C to stop)")
        last_print = 0.0

        try:
            # Tolerant iterator: small buffer + tiny min_len (what worked for you)
            for scan in lidar.iter_scans(max_buf_meas=500, min_len=5):
                t0 = now_s()
                scan_id += 1

                if (a_min != -180.0) or (a_max != 180.0) or (r_min > 0.0) or (r_max < 40.0):
                    filt = gate(scan, a_min, a_max, r_min, r_max)
                else:
                    filt = [(q, ang, d_mm/1000.0) for (q, ang, d_mm) in scan]

                t1 = now_s()
                stats = compute_stats(scan_id, t0, t1, filt)

                obstacle = (obst_th is not None and stats.min_dist_m is not None and stats.min_dist_m <= obst_th)

                if (t1 - last_print) >= 0.1:
                    last_print = t1
                    md = f"{stats.min_dist_m:.3f} m @ {stats.min_angle_deg:.1f}°" if stats.min_dist_m is not None else "—"
                    hz = f"{stats.hz_estimate:5.2f} Hz" if math.isfinite(stats.hz_estimate) else "  n/a  "
                    pps = f"{int(stats.pts_per_sec):5d}/s" if math.isfinite(stats.pts_per_sec) else "  n/a "
                    alert = "  **OBSTACLE**" if obstacle else ""
                    print(f"[scan {stats.scan_id:05d}] {hz}  pts/scan={stats.n_points:4d}  pts/s={pps}  min={md}{alert}")

                if w:
                    if csv_mode == 'scans':
                        w.writerow([
                            f"{stats.t:.6f}", stats.scan_id, stats.n_points,
                            f"{stats.hz_estimate:.4f}" if math.isfinite(stats.hz_estimate) else "",
                            f"{stats.pts_per_sec:.1f}" if math.isfinite(stats.pts_per_sec) else "",
                            f"{stats.min_dist_m:.4f}" if stats.min_dist_m is not None else "",
                            f"{stats.min_angle_deg:.2f}" if stats.min_angle_deg is not None else "",
                        ])
                    else:
                        for q, ang, d_m in filt:
                            w.writerow([f"{t0:.6f}", stats.scan_id, q, f"{ang:.2f}", f"{d_m:.4f}"])

                # stop conditions
                if duration is not None and (now_s() - t_start) >= duration:
                    print("[LiDAR] Duration reached; stopping.")
                    break
                if max_scans is not None and scan_id >= max_scans:
                    print("[LiDAR] Max scans reached; stopping.")
                    break

        except KeyboardInterrupt:
            print("\n[LiDAR] Ctrl+C received — stopping.")

    except RPLidarException as e:
        print(f"RPLidarException: {e}")
        print("Tip: Check power/USB/port, and that no other process uses the device.")
        sys.exit(4)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(5)
    finally:
        try:
            if csvf: csvf.flush(); os.fsync(csvf.fileno()); csvf.close()
        except Exception: pass
        try:
            if lidar is not None:
                lidar.stop(); lidar.stop_motor(); lidar.disconnect()
                print("[LiDAR] Stopped motor and disconnected.")
        except Exception as e:
            print(f"[LiDAR] Cleanup warning: {e}")

def main():
    ap = argparse.ArgumentParser(description="Robot Savo — RPLIDAR A1 Diagnostic (stable-lite)")
    ap.add_argument("--port", type=str,
                    default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*-if00-port0",
                    help="Serial device (prefer /dev/serial/by-id). Glob fallback to /dev/ttyUSB0.")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--motor-pwm", type=int, default=None, help="Motor PWM 0..1023 (optional).")

    ap.add_argument("--angle-min", type=float, default=-180.0)
    ap.add_argument("--angle-max", type=float, default=180.0)
    ap.add_argument("--range-min", type=float, default=0.05)
    ap.add_argument("--range-max", type=float, default=8.0)

    ap.add_argument("--obst", type=float, default=None, help="Obstacle threshold (m).")
    ap.add_argument("--duration", type=float, default=None, help="Stop after N seconds.")
    ap.add_argument("--max-scans", type=int, default=None, help="Stop after N scans.")
    ap.add_argument("--csv", type=str, default=None)
    ap.add_argument("--csv-mode", choices=["scans","points"], default="scans")
    ap.add_argument("--min-len", type=int, default=5, help="(kept for compatibility; unused in this stable-lite)")

    args = ap.parse_args()
    if "*" in args.port:
        import glob
        m = glob.glob(args.port)
        args.port = m[0] if m else "/dev/ttyUSB0"
    run(args)

if __name__ == "__main__":
    main()
