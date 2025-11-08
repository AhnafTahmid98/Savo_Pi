#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Camera Diagnostic (Ubuntu 24.04 + Weston/Wayland, Expert Edition)
-----------------------------------------------------------------------------
Designed for **Ubuntu 24.04 on Raspberry Pi 5** using **libcamera** with:
- **Wayland/Weston** full‑screen preview on the 7" DSI (waylandsink)
- **libcamera-vid/libcamera-still** for video/still with AE/AWB/exposure/gain controls
- **Per‑second metadata → CSV** from libcamera JSON metadata
- Clean exit and timestamped outputs in ./log

Install once
  sudo apt update
  sudo apt install -y libcamera-tools libcamera-apps gstreamer1.0-libcamera \
      gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good weston jq

Start Weston (if not already running on the DSI tty)
  weston --backend=drm-backend.so --tty=1 --idle-time=0 &

Examples
  # Full-screen live preview on DSI via Wayland
  python3 tools/diag/sensors/camera_test.py --mode preview --res 1280x720 --fps 30 --sink wayland

  # 10 s 1080p30 with fixed shutter (1/100s) and gain 2.0, with CSV metadata
  python3 tools/diag/sensors/camera_test.py --mode video --res 1920x1080 --fps 30 \
    --duration 10 --exposure-us 10000 --gain 2.0 --awb daylight --metering matrix --csv

  # Single full-res still with custom AWB
  python3 tools/diag/sensors/camera_test.py --mode still --res 3280x2464 --awb cloudy --csv

Notes
- Preview uses GStreamer (libcamerasrc → waylandsink) for robust Weston output.
- Video/Still use libcamera-apps for reliable control + metadata JSON.
- If preview fails under Wayland, confirm Weston is running and WAYLAND_DISPLAY exists.
"""

import argparse
import json
import os
import shlex
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from typing import Dict, Tuple

# ---------------------------- helpers ----------------------------

def parse_res(s: str) -> Tuple[int, int]:
    try:
        w, h = s.lower().split("x")
        return int(w), int(h)
    except Exception as e:
        raise argparse.ArgumentTypeError("Resolution must be like 1280x720") from e


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def ts_name(prefix: str, ext: str) -> str:
    return f"{prefix}_{datetime.now().strftime('%Y%m%d-%H%M%S')}.{ext}"


def run(cmd: str) -> int:
    print(f"[RUN] {cmd}")
    return subprocess.call(cmd, shell=True)


def capture_cam_list() -> str:
    try:
        return subprocess.check_output(["cam", "-l"], text=True)
    except Exception as e:
        return f"cam -l failed: {e}"


def write_info_log(outdir: str, label: str, extra: str = "") -> None:
    ensure_dir(outdir)
    fname = os.path.join(outdir, ts_name(f"{label}_info", "txt"))
    with open(fname, "w") as f:
        f.write("time: " + datetime.now().isoformat() + "\n"
")
        if extra:
            f.write(extra + "
")
        f.write("
# cam -l
")
        f.write(capture_cam_list())
    print(f"[INFO] Wrote {fname}")

# ---------------------------- pipelines ----------------------------

AWB_CHOICES = [
    "auto", "incandescent", "tungsten", "fluorescent", "indoor", "daylight", "cloudy",
]
METERING_CHOICES = ["centre", "center", "spot", "matrix"]


def do_preview(args) -> int:
    w, h = args.res
    if args.sink == "wayland":
        sink = "waylandsink fullscreen=true"
    elif args.sink == "drm":
        sink = "kmssink"
    else:
        sink = "autovideosink"
    pipeline = (
        f"libcamerasrc ! video/x-raw,width={w},height={h},framerate={args.fps}/1 "
        f"! videoconvert ! {sink}"
    )
    return run(f"gst-launch-1.0 -v {pipeline}")


# libcamera CLI mappers

def map_metering(m: str) -> str:
    m = m.lower()
    return {"center": "centre", "centre": "centre", "spot": "spot", "matrix": "matrix"}.get(m, "centre")


def build_libcamera_common(args) -> str:
    w, h = args.res
    parts = [f"--width {w}", f"--height {h}", f"--framerate {args.fps}"]
    if args.awb and args.awb in AWB_CHOICES:
        parts.append(f"--awb {shlex.quote(args.awb)}")
    if args.metering:
        parts.append(f"--metering {shlex.quote(map_metering(args.metering))}")
    if args.exposure_us is not None:
        parts.append(f"--shutter {int(args.exposure_us)}")  # microseconds
    if args.gain is not None:
        parts.append(f"--gain {float(args.gain):.3f}")
    return " ".join(parts)


def parse_metadata_json_to_csv(meta_json_path: str, csv_path: str) -> None:
    """Convert libcamera metadata JSON (one JSON object per frame) into ~1 Hz CSV."""
    import csv

    try:
        with open(meta_json_path, "r") as f:
            lines = [json.loads(line) for line in f if line.strip()]
    except Exception as e:
        print(f"[WARN] Failed to parse metadata JSON: {e}")
        return

    if not lines:
        print("[WARN] Empty metadata JSON; no CSV produced")
        return

    # Aggregate by whole seconds from Timestamp (microseconds)
    buckets: Dict[int, Dict[str, float]] = {}
    for rec in lines:
        ts_us = rec.get("Timestamp", 0)
        sec = int(float(ts_us) / 1_000_000)
        ag = rec.get("AnalogueGain") or rec.get("AgcAnalogueGain") or 0.0
        sh = rec.get("ExposureTime") or rec.get("ShutterSpeed") or 0
        ct = rec.get("ColourTemperature", 0)
        lux = rec.get("Lux", 0.0)
        fd = rec.get("FrameDuration", 0)
        b = buckets.setdefault(sec, {"count": 0, "gain": 0.0, "exp": 0.0, "ct": 0.0, "lux": 0.0, "fd": 0.0})
        b["count"] += 1
        b["gain"] += float(ag)
        b["exp"] += int(sh)
        b["ct"] += int(ct)
        b["lux"] += float(lux)
        b["fd"] += int(fd)

    t0 = min(buckets.keys())
    rows = []
    for sec in sorted(buckets.keys()):
        b = buckets[sec]
        n = max(1, b["count"])
        gain_avg = b["gain"] / n
        row = {
            "t": f"{sec - t0:.3f}",
            "exposure_us": int(b["exp"] / n),
            "gain": f"{gain_avg:.3f}",
            "iso_like": int(100 * gain_avg),
            "ct_k": int(b["ct"] / n),
            "lux": f"{(b["lux"] / n):.2f}",
            "frame_duration_us": int(b["fd"] / n),
        }
        rows.append(row)

    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "t", "exposure_us", "gain", "iso_like", "ct_k", "lux", "frame_duration_us"
        ])
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"[Meta] CSV written → {csv_path}")


def do_video(args) -> int:(args) -> int:
    ensure_dir(args.outdir)
    mp4 = os.path.join(args.outdir, ts_name("cam_video", "mp4"))
    meta_json = os.path.join(args.outdir, ts_name("cam_video_meta", "json"))
    common = build_libcamera_common(args)
    timeout_ms = max(0, int(args.duration * 1000)) if args.duration > 0 else 0

    cmd = (
        f"libcamera-vid -o {shlex.quote(mp4)} {common} --codec h264 --profile high "
        f"--bitrate {args.bitrate*1000} --inline --flush "+
        (f"--timeout {timeout_ms} " if timeout_ms else "")+
        ("--metadata " + shlex.quote(meta_json) + " " if args.csv else "")
    ).strip()

    write_info_log(args.outdir, "video", extra=f"cmd={cmd}")
    ret = run(cmd)

    if ret == 0 and args.csv:
        csv_path = os.path.join(args.outdir, ts_name("cam_video_meta", "csv"))
        parse_metadata_json_to_csv(meta_json, csv_path)

    print(f"[Video] Saved → {mp4}")
    return ret


def do_still(args) -> int:
    ensure_dir(args.outdir)
    jpg = os.path.join(args.outdir, ts_name("cam_still", "jpg"))
    meta_json = os.path.join(args.outdir, ts_name("cam_still_meta", "json"))
    w, h = args.res
    common = build_libcamera_common(args)

    cmd = (
        f"libcamera-still -o {shlex.quote(jpg)} {common} --immediate --autofocus-mode manual "
        f"--metadata {shlex.quote(meta_json)}"
    )

    write_info_log(args.outdir, "still", extra=f"cmd={cmd}")
    ret = run(cmd)

    if ret == 0 and args.csv:
        # Convert single-record JSON to one-row CSV
        try:
            with open(meta_json, "r") as f:
                rec = json.load(f)
            import csv
            csv_path = os.path.join(args.outdir, ts_name("cam_still_meta", "csv"))
            with open(csv_path, "w", newline="") as cf:
                wcsv = csv.DictWriter(cf, fieldnames=[
                    "t", "exposure_us", "gain", "iso_like", "ct_k", "lux", "frame_duration_us"
                ])
                wcsv.writeheader()
                ag = rec.get("AnalogueGain") or rec.get("AgcAnalogueGain") or 0.0
                lux = rec.get("Lux", 0.0)
                row = {
                    "t": "0.000",
                    "exposure_us": rec.get("ExposureTime") or rec.get("ShutterSpeed") or 0,
                    "gain": f"{float(ag):.3f}",
                    "iso_like": int(100 * float(ag)),
                    "ct_k": rec.get("ColourTemperature", 0),
                    "lux": f"{float(lux):.2f}",
                    "frame_duration_us": rec.get("FrameDuration", 0),
                }
                wcsv.writerow(row)
            print(f"[Meta] CSV written → {csv_path}")
        except Exception as e:
            print(f"[WARN] Failed to convert still metadata to CSV: {e}")

    print(f"[Still] Saved → {jpg}")
    return ret

# ---------------------------- main ----------------------------

def main() -> None:
    p = argparse.ArgumentParser(description="Robot Savo Camera Diagnostic (Ubuntu + Weston, Expert)")
    p.add_argument("--mode", choices=["preview", "video", "still"], default="preview")
    p.add_argument("--res", type=parse_res, default="1280x720", help="WxH (default 1280x720)")
    p.add_argument("--fps", type=int, default=30, help="Frames per second (default 30)")
    p.add_argument("--duration", type=float, default=5.0, help="Seconds for video; 0 = until Ctrl+C")
    p.add_argument("--bitrate", type=int, default=6000, help="Video bitrate (kbps) for H.264")
    p.add_argument("--sink", choices=["wayland", "drm", "auto"], default="wayland",
                   help="Preview sink: wayland=Weston full-screen, drm=raw KMS, auto=desktop window")
    p.add_argument("--awb", choices=AWB_CHOICES, default="auto", help="AWB mode")
    p.add_argument("--metering", choices=METERING_CHOICES, default="centre", help="AE metering mode")
    p.add_argument("--exposure-us", type=int, default=None, help="Lock exposure time (µs); sets --shutter")
    p.add_argument("--gain", type=float, default=None, help="Lock analogue gain (e.g., 2.0 ≈ ISO200)")
    p.add_argument("--csv", action="store_true", help="Write per-second metadata CSV (video) or one-row CSV (still)")
    p.add_argument("--outdir", default="log", help="Output directory for files and logs")

    args = p.parse_args()

    if args.mode == "preview":
        # Ensure Weston is active for wayland sink
        if args.sink == "wayland" and not os.environ.get("WAYLAND_DISPLAY"):
            print("[WARN] WAYLAND_DISPLAY not set. Start Weston: 
  weston --backend=drm-backend.so --tty=1 --idle-time=0 &",
                  file=sys.stderr)
        sys.exit(do_preview(args))

    elif args.mode == "video":
        # If duration=0, user can Ctrl+C to stop; libcamera-vid will run until killed
        ret = do_video(args)
        sys.exit(ret)

    else:  # still
        ret = do_still(args)
        sys.exit(ret)


if __name__ == "__main__":
    main()
