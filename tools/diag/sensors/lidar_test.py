#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — RPLIDAR A1 Diagnostic (RAW fallback, Ctrl+C safe)
- SCAN mode (default): per-rotation stats, gating, obstacle alert.
- RAW mode (--raw or auto-fallback): robust iter_measurments streamer.
"""
import os, sys, argparse, csv, math, time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

# project-local libs
sys.path.insert(0, os.path.expanduser('~/Savo_Pi/.pylibs'))
try:
    from rplidar import RPLidar, RPLidarException
except Exception:
    print("ERROR: 'rplidar' not found. Install:\n  python3 -m pip install --target ~/Savo_Pi/.pylibs rplidar")
    sys.exit(1)

def clamp(v,a,b): return max(a, min(b, v))
def now_s()->float: return time.time()

@dataclass
class ScanStats:
    t: float; scan_id: int; n_points: int; hz_estimate: float; pts_per_sec: float
    min_dist_m: Optional[float]; min_angle_deg: Optional[float]

def gate(points: Iterable[Tuple[int,float,int]], a_min: float, a_max: float, r_min: float, r_max: float):
    out=[]
    for q, ang, d_mm in points:
        d = d_mm/1000.0
        if a_min <= ang <= a_max and r_min <= d <= r_max:
            out.append((q, ang, d))
    return out

def compute_stats(scan_id:int, t0:float, t1:float, pts:List[Tuple[int,float,float]])->ScanStats:
    n=len(pts); per=(t1-t0) if t1>t0 else float('nan')
    hz=(1.0/per) if per and per>0 else float('nan')
    pps=(n/per) if per and per>0 else float('nan')
    if n:
        _, ang_min, d_min = min(pts, key=lambda x:x[2])
        return ScanStats(t0, scan_id, n, hz, pps, d_min, ang_min)
    return ScanStats(t0, scan_id, 0, hz, pps, None, None)

def print_scan_line(stats: ScanStats, obstacle: bool):
    md = f"{stats.min_dist_m:.3f} m @ {stats.min_angle_deg:.1f}°" if stats.min_dist_m is not None else "—"
    hz = f"{stats.hz_estimate:5.2f} Hz" if math.isfinite(stats.hz_estimate) else "  n/a  "
    pps = f"{int(stats.pts_per_sec):5d}/s" if math.isfinite(stats.pts_per_sec) else "  n/a "
    alert = "  **OBSTACLE**" if obstacle else ""
    print(f"[scan {stats.scan_id:05d}] {hz}  pts/scan={stats.n_points:4d}  pts/s={pps}  min={md}{alert}")

def run_scan_mode(lidar: "RPLidar", args) -> None:
    a_min,a_max,r_min,r_max = args.angle_min, args.angle_max, args.range_min, args.range_max
    obst_th, duration, max_scans = args.obst, args.duration, args.max_scans
    csv_path, csv_mode = args.csv, args.csv_mode
    csvf=None; w=None
    if csv_path:
        os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
        csvf=open(csv_path,"w",newline=""); w=csv.writer(csvf)
        if csv_mode=="scans":
            w.writerow(["t_epoch_s","scan_id","n_points","scan_hz","pts_per_sec","min_dist_m","min_angle_deg"])
        else:
            w.writerow(["t_epoch_s","scan_id","quality","angle_deg","distance_m"])
    t_start=now_s(); last_print=0.0; scan_id=0
    try:
        for scan in lidar.iter_scans(max_buf_meas=500, min_len=5):
            t0=now_s(); scan_id+=1
            if (a_min!=-180.0) or (a_max!=180.0) or (r_min>0.0) or (r_max<40.0):
                filt=gate(scan,a_min,a_max,r_min,r_max)
            else:
                filt=[(q,ang,d_mm/1000.0) for (q,ang,d_mm) in scan]
            t1=now_s(); stats=compute_stats(scan_id,t0,t1,filt)
            obstacle=(obst_th is not None and stats.min_dist_m is not None and stats.min_dist_m<=obst_th)
            if (t1-last_print)>=0.1:
                last_print=t1; print_scan_line(stats,obstacle)
            if w:
                if csv_mode=='scans':
                    w.writerow([f"{stats.t:.6f}",stats.scan_id,stats.n_points,
                                f"{stats.hz_estimate:.4f}" if math.isfinite(stats.hz_estimate) else "",
                                f"{stats.pts_per_sec:.1f}" if math.isfinite(stats.pts_per_sec) else "",
                                f"{stats.min_dist_m:.4f}" if stats.min_dist_m is not None else "",
                                f"{stats.min_angle_deg:.2f}" if stats.min_angle_deg is not None else ""])
                else:
                    for q,ang,d_m in filt:
                        w.writerow([f"{t0:.6f}",stats.scan_id,q,f"{ang:.2f}",f"{d_m:.4f}"])
            if duration is not None and (now_s()-t_start)>=duration: print("[LiDAR] Duration reached; stopping."); break
            if max_scans is not None and scan_id>=max_scans: print("[LiDAR] Max scans reached; stopping."); break
    finally:
        try:
            if csvf: csvf.flush(); os.fsync(csvf.fileno()); csvf.close()
        except Exception: pass

def run_raw_mode(lidar: "RPLidar", args) -> None:
    a_min,a_max,r_min,r_max = args.angle_min, args.angle_max, args.range_min, args.range_max
    obst_th, duration = args.obst, args.duration
    print("[LiDAR] RAW mode: streaming measurements (robust).")
    t_start=now_s(); t0=now_s(); last=0.0; n=0; min_d=None; min_ang=None
    for i,(new_scan,q,ang,d_mm) in enumerate(lidar.iter_measurments(max_buf_meas=4096)):
        n+=1
        if d_mm>0:
            d=d_mm/1000.0
            if a_min<=ang<=a_max and r_min<=d<=r_max:
                if (min_d is None) or (d<min_d):
                    min_d, min_ang = d, ang
        t=now_s()
        if (t-last)>=0.5:
            rate=int(n/(t-t0)) if (t-t0)>0 else 0
            md=f"{min_d:.3f} m @ {min_ang:.1f}°" if min_d is not None else "—"
            alert="  **OBSTACLE**" if (obst_th and min_d and min_d<=obst_th) else ""
            print(f"[RAW] {rate} meas/s   min={md}{alert}")
            last=t
        if duration is not None and (t-t_start)>=duration:
            print("[LiDAR] Duration reached; stopping."); break

def main():
    ap=argparse.ArgumentParser(description="Robot Savo — RPLIDAR A1 Diagnostic (RAW fallback)")
    ap.add_argument("--port", type=str,
        default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*-if00-port0",
        help="Serial device (prefer by-id). Glob fallback to /dev/ttyUSB0.")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.0)
    ap.add_argument("--motor-pwm", type=int, default=None)
    ap.add_argument("--angle-min", type=float, default=-180.0)
    ap.add_argument("--angle-max", type=float, default=180.0)
    ap.add_argument("--range-min", type=float, default=0.05)
    ap.add_argument("--range-max", type=float, default=8.0)
    ap.add_argument("--obst", type=float, default=None)
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--max-scans", type=int, default=None)
    ap.add_argument("--csv", type=str, default=None)
    ap.add_argument("--csv-mode", choices=["scans","points"], default="scans")
    ap.add_argument("--raw", action="store_true", help="Force RAW mode (iter_measurments).")
    args=ap.parse_args()

    if "*" in args.port:
        import glob
        m=glob.glob(args.port); args.port=m[0] if m else "/dev/ttyUSB0"

    lidar=None
    try:
        print(f"[LiDAR] Connecting: port='{args.port}' baud={args.baudrate} timeout={args.timeout}s ...")
        lidar=RPLidar(port=args.port, baudrate=args.baudrate, timeout=args.timeout)
        info=lidar.get_info()
        print(f"[LiDAR] Info: model={info.get('model','?')} fw={info.get('firmware',(0,0))[0]}.{info.get('firmware',(0,0))[1]} hw={info.get('hardware',0)} serial={info.get('serialnumber','n/a')}")
        status, err = lidar.get_health()
        print(f"[LiDAR] Health: status={status} error_code={err}")
        if str(status).lower()!="good":
            print("ERROR: Health not GOOD. Power-cycle and retry."); sys.exit(3)
        if args.motor_pwm is not None:
            pwm=clamp(args.motor_pwm,0,1023); print(f"[LiDAR] Starting motor with PWM={pwm} ..."); lidar.set_pwm(pwm)
        else:
            print("[LiDAR] Starting motor (default speed) ..."); lidar.start_motor()
        time.sleep(0.6)
        try: lidar.clear_input()
        except Exception: pass
        print("[LiDAR] Streaming...  (Ctrl+C to stop)")
        if args.raw:
            run_raw_mode(lidar, args)
        else:
            try:
                run_scan_mode(lidar, args)
            except RPLidarException as e:
                msg=str(e)
                if "Wrong body size" in msg or "Incorrect descriptor starting bytes" in msg:
                    print("[LiDAR] Scan mode desync → switching to RAW mode.")
                    run_raw_mode(lidar, args)
                else:
                    raise
    except KeyboardInterrupt:
        print("\n[LiDAR] Ctrl+C received — stopping.")
    except RPLidarException as e:
        print(f"RPLidarException: {e}\nTip: Swap USB cable/port or use a powered hub; RAW mode is more tolerant.")
        sys.exit(4)
    except Exception as e:
        print(f"ERROR: {e}"); sys.exit(5)
    finally:
        try:
            if lidar is not None:
                lidar.stop(); lidar.stop_motor(); lidar.disconnect()
                print("[LiDAR] Stopped motor and disconnected.")
        except Exception as ee:
            print(f"[LiDAR] Cleanup warning: {ee}")

if __name__ == "__main__":
    main()
