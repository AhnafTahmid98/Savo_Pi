#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Camera Tester (expert, Ubuntu 24.04 / Pi 5)
========================================================
- Zero-guessing camera bring-up for IMX219 + DSI 7".
- Previews:
    --preview wayland   → waylandsink (your letterboxed 800x480)
    --preview kms       → kmssink (DRM/KMS to the DSI panel)
    --preview window    → autovideosink (X11/Wayland window if available)
- Stills:
    --snap PATH         → Picamera2 or libcamera-jpeg
    --snap-gst PATH     → One-buffer GStreamer: libcamerasrc num-buffers=1 → jpegenc
- Video:
    --record PATH       → MP4/H.264 with --bitrate, width/height/fps (Picamera2/libcamera-vid/GStreamer)
- Logging:
    --log BASEPATH      → Writes BASEPATH.txt (human) + BASEPATH.csv (machine) with timestamps + config

Environment (for Wayland preview)
---------------------------------
export XDG_RUNTIME_DIR=/run/user/1000
export WAYLAND_DISPLAY=wayland-1
export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:/usr/local/lib

Examples
--------
# List cameras
python3 camera_test.py --list

# Still via libcamera-jpeg (no Picamera2 needed)
python3 camera_test.py --snap /tmp/cam.jpg --width 1280 --height 720

# Still via one-buffer GStreamer
python3 camera_test.py --snap-gst /tmp/cam_gst.jpg --width 1280 --height 720

# Record 8s MP4 at 1920x1080/30, 10 Mbit/s, with logs
python3 camera_test.py --record /tmp/cam.mp4 --width 1920 --height 1080 --fps 30 --bitrate 10000000 --duration 8 --log /tmp/cam_record

# Live preview
python3 camera_test.py --preview wayland
python3 camera_test.py --preview kms
python3 camera_test.py --preview window
"""

import argparse
import csv
import datetime as dt
import os
import shlex
import shutil
import subprocess
import sys
import time
from typing import Optional, Tuple, Dict, Any

# ---------------- Utilities ----------------

def which(cmd: str) -> bool:
    return shutil.which(cmd) is not None

def run(cmd, *, capture=False, check=False, env=None) -> Tuple[int, str]:
    try:
        if capture:
            out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True, env=env)
            return 0, out
        rc = subprocess.call(cmd, env=env)
        if check and rc != 0:
            raise subprocess.CalledProcessError(rc, cmd)
        return rc, ""
    except subprocess.CalledProcessError as e:
        return e.returncode, getattr(e, "output", str(e))
    except FileNotFoundError:
        return 127, f"{cmd[0]} not found"
    except Exception as e:
        return 1, f"{type(e).__name__}: {e}"

def require(cmd: str, friendly: Optional[str] = None):
    if not which(cmd):
        print(f"ERROR: {friendly or cmd} not found in PATH.", file=sys.stderr)
        sys.exit(127)

def ensure_dir_for(path: str):
    d = os.path.dirname(os.path.abspath(path))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)

def hint_video_group():
    print("Hint: If access fails, add user to 'video':\n"
          "  sudo usermod -aG video $USER && newgrp video", file=sys.stderr)

def now_iso():
    return dt.datetime.now().isoformat(timespec="seconds")

# ---------------- Logging ----------------

def start_logs(log_base: Optional[str], header: Dict[str, Any]) -> Optional[Tuple[str, str]]:
    if not log_base:
        return None
    txt = f"{log_base}.txt"
    csvp = f"{log_base}.csv"
    ensure_dir_for(txt)
    # human text
    with open(txt, "a", encoding="utf-8") as f:
        f.write(f"[{now_iso()}] camera_test start\n")
        for k, v in header.items():
            f.write(f"  {k}: {v}\n")
    # csv header (append if not exists)
    new_file = not os.path.exists(csvp)
    with open(csvp, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["timestamp"] + list(header.keys()))
        w.writerow([now_iso()] + [header[k] for k in header.keys()])
    return txt, csvp

def append_log(log_paths: Optional[Tuple[str, str]], row: Dict[str, Any]):
    if not log_paths:
        return
    txt, csvp = log_paths
    with open(txt, "a", encoding="utf-8") as f:
        f.write(f"[{now_iso()}] " + " ".join(f"{k}={v}" for k, v in row.items()) + "\n")
    with open(csvp, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        # keep column order stable
        keys = list(row.keys())
        w.writerow([now_iso()] + [row[k] for k in keys])

# ---------------- Picamera2 detection ----------------

def have_picamera2() -> bool:
    try:
        import picamera2  # noqa: F401
        return True
    except Exception:
        return False

# ---------------- List cameras ----------------

def list_picamera2():
    from picamera2 import Picamera2
    cams = Picamera2.global_camera_info()
    if not cams:
        print("No cameras detected (Picamera2).")
        hint_video_group()
        return
    print("Detected cameras (Picamera2):")
    for i, ci in enumerate(cams):
        model = ci.get("Model", "Unknown")
        sensor = ci.get("Sensor", "Unknown")
        location = ci.get("Location", "Unknown")
        print(f"  [{i}] {model} | sensor={sensor} | location={location}")

def list_cli():
    require("libcamera-hello", "libcamera-apps")
    rc, out = run(["libcamera-hello", "--list-cameras"], capture=True)
    if rc == 0 and out.strip():
        print(out.strip())
    else:
        print("No cameras detected (libcamera-hello).")
        if out:
            print(out.strip())
        hint_video_group()

# ---------------- Stills ----------------

def snap_picamera2(path: str, w: int, h: int, exposure_us: Optional[int], iso: Optional[int], awb: Optional[str]):
    from picamera2 import Picamera2
    cam = Picamera2()
    cfg = cam.create_still_configuration(main={"size": (w, h)})
    cam.configure(cfg)
    controls = {}
    if exposure_us is not None:
        controls["ExposureTime"] = int(max(100, exposure_us))
    if iso is not None:
        controls["AnalogueGain"] = float(max(1.0, iso/100.0))
    if awb:
        controls["AwbMode"] = awb
    cam.start(); time.sleep(0.5)
    if controls:
        cam.set_controls(controls); time.sleep(0.3)
    cam.capture_file(path)
    cam.stop()
    print(f"Saved still: {path}")

def snap_cli(path: str, w: int, h: int, exposure_us: Optional[int], iso: Optional[int], awb: Optional[str], cam_index: Optional[int]):
    require("libcamera-jpeg", "libcamera-apps")
    cmd = ["libcamera-jpeg", "-o", path, "--width", str(w), "--height", str(h), "-n"]
    if cam_index is not None: cmd += ["--camera", str(cam_index)]
    if awb: cmd += ["--awb", awb]
    if iso: cmd += ["--gain", f"{max(1.0, float(iso)/100.0):.2f}"]
    if exposure_us: cmd += ["--shutter", str(int(max(100, exposure_us)))]
    print("Running:", " ".join(shlex.quote(x) for x in cmd))
    rc, out = run(cmd, capture=True)
    if rc != 0:
        print(out, file=sys.stderr); hint_video_group(); sys.exit(rc)
    print(f"Saved still: {path}")

def snap_gst(path: str, w: int, h: int):
    """One-buffer GStreamer capture → JPEG."""
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    ensure_dir_for(path)
    pipeline = [
        "gst-launch-1.0", "-e",
        "libcamerasrc", "num-buffers=1", "!",
        f"video/x-raw,format=I420,width={w},height={h},framerate=30/1", "!",
        "videoconvert", "!", "jpegenc", "!",
        "filesink", f"location={path}"
    ]
    print("Running:", " ".join(shlex.quote(x) for x in pipeline))
    rc, out = run(pipeline, capture=True)
    if rc != 0:
        print(out, file=sys.stderr); hint_video_group(); sys.exit(rc)
    print(f"Saved still (gst): {path}")

# ---------------- Video record ----------------

def record_picamera2(path: str, w: int, h: int, fps: int, duration: float, bitrate: int,
                     exposure_us: Optional[int], iso: Optional[int], awb: Optional[str]):
    from picamera2 import Picamera2
    from picamera2.encoders import H264Encoder, Quality
    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={"size": (w, h)},
        controls={"FrameDurationLimits": (int(1e6/fps), int(1e6/fps))}
    )
    cam.configure(cfg)
    controls = {}
    if exposure_us is not None: controls["ExposureTime"] = int(max(100, exposure_us))
    if iso is not None: controls["AnalogueGain"] = float(max(1.0, iso/100.0))
    if awb: controls["AwbMode"] = awb
    enc = H264Encoder(bitrate=max(100_000, int(bitrate)))
    cam.start_recording(enc, path, quality=Quality.MEDIUM)
    time.sleep(0.5)
    if controls: cam.set_controls(controls)
    print(f"Recording {duration:.1f}s → {path} @ {w}x{h} {fps}fps, {bitrate} bps")
    time.sleep(max(0.0, duration))
    cam.stop_recording()
    print("Done.")

def record_cli(path: str, w: int, h: int, fps: int, duration: float, bitrate: int, cam_index: Optional[int]):
    require("libcamera-vid", "libcamera-apps")
    cmd = [
        "libcamera-vid", "-t", str(int(duration*1000)),
        "--width", str(w), "--height", str(h),
        "--framerate", str(int(fps)),
        "--bitrate", str(int(bitrate)),
        "-o", path, "-n"
    ]
    if cam_index is not None:
        cmd += ["--camera", str(cam_index)]
    print("Running:", " ".join(shlex.quote(x) for x in cmd))
    rc, out = run(cmd, capture=True)
    if rc != 0:
        print(out, file=sys.stderr); hint_video_group(); sys.exit(rc)
    print(f"Saved video: {path}")

def record_gst(path: str, w: int, h: int, fps: int, bitrate: int, duration: float):
    """Pure GStreamer MP4/H.264 path."""
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    ensure_dir_for(path)
    # Needs x264enc (gstreamer1.0-plugins-ugly) and mp4mux (gstreamer1.0-plugins-good)
    pipeline = [
        "gst-launch-1.0", "-e",
        "libcamerasrc", "!",
        f"video/x-raw,format=I420,width={w},height={h},framerate={fps}/1", "!",
        "videoconvert", "!",
        "x264enc", f"bitrate={int(bitrate/1000)}", "speed-preset=superfast", "tune=zerolatency", "!",
        "h264parse", "!",
        "mp4mux", "!",
        "filesink", f"location={path}"
    ]
    print("Running:", " ".join(shlex.quote(x) for x in pipeline))
    t0 = time.time()
    proc = subprocess.Popen(pipeline)
    try:
        while proc.poll() is None and (time.time() - t0) < duration:
            time.sleep(0.2)
        if proc.poll() is None:
            proc.terminate()
            try: proc.wait(timeout=2)
            except Exception: proc.kill()
    except KeyboardInterrupt:
        proc.terminate()
        try: proc.wait(timeout=2)
        except Exception: proc.kill()
        raise
    print(f"Saved video (gst): {path}")

# ---------------- Previews ----------------

def check_wayland_env():
    missing = [k for k in ("XDG_RUNTIME_DIR", "WAYLAND_DISPLAY", "GST_PLUGIN_PATH", "LD_LIBRARY_PATH")
               if not os.environ.get(k)]
    if missing:
        print("ERROR: Missing env:", ", ".join(missing), file=sys.stderr)
        print("Export the Wayland environment (see header).", file=sys.stderr)
        sys.exit(2)

def preview_wayland(src_w: int, src_h: int, fps: int, panel_w: int = 800, panel_h: int = 480):
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    check_wayland_env()
    target_w = panel_w
    target_h = int(round(panel_w * 9 / 16))  # 450
    pad = panel_h - target_h                 # 30 → 15/15
    cmd = [
        "gst-launch-1.0", "-v",
        "libcamerasrc", "!",
        f"video/x-raw,format=I420,width={src_w},height={src_h},framerate={fps}/1", "!",
        "videoscale", "!", f"video/x-raw,width={target_w},height={target_h}", "!",
        "videobox", f"border-alpha=1", f"top={pad//2}", f"bottom={pad - pad//2}", "left=0", "right=0", "!",
        "videoscale", "!", f"video/x-raw,width={panel_w},height={panel_h}", "!",
        "videoconvert", "!",
        f"waylandsink", f"display={os.environ.get('WAYLAND_DISPLAY','wayland-1')}"
    ]
    print("Running (Ctrl+C to quit):", " ".join(shlex.quote(x) for x in cmd))
    os.execvp(cmd[0], cmd)

def preview_kms(src_w: int, src_h: int, fps: int, conn_id: Optional[int] = None):
    """
    DRM/KMS direct-to-panel preview (no compositor).
    Optionally pass connector id with --kms-conn.
    """
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    sink = ["kmssink"]
    if conn_id is not None:
        sink += [f"connector-id={conn_id}"]
    cmd = [
        "gst-launch-1.0", "-v",
        "libcamerasrc", "!",
        f"video/x-raw,format=I420,width={src_w},height={src_h},framerate={fps}/1", "!",
        "videoconvert", "!",
    ] + sink
    print("Running (Ctrl+C to quit):", " ".join(shlex.quote(x) for x in cmd))
    os.execvp(cmd[0], cmd)

def preview_window(src_w: int, src_h: int, fps: int):
    """Windowed preview (X11/Wayland desktop)."""
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    cmd = [
        "gst-launch-1.0", "-v",
        "libcamerasrc", "!",
        f"video/x-raw,format=I420,width={src_w},height={src_h},framerate={fps}/1", "!",
        "videoconvert", "!",
        "autovideosink"
    ]
    print("Running (Ctrl+C to quit):", " ".join(shlex.quote(x) for x in cmd))
    os.execvp(cmd[0], cmd)

# ---------------- Main ----------------

def main():
    ap = argparse.ArgumentParser(description="Robot Savo — Camera Tester")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--list", action="store_true", help="List cameras")
    g.add_argument("--snap", metavar="PATH", help="Capture JPEG to PATH (Picamera2 or libcamera-jpeg)")
    g.add_argument("--snap-gst", metavar="PATH", help="Capture JPEG via one-buffer GStreamer pipeline")
    g.add_argument("--record", metavar="PATH", help="Record MP4/H.264 to PATH")
    g.add_argument("--preview", choices=["wayland", "kms", "window"], help="Live preview sink")

    ap.add_argument("--width", type=int, default=1280, help="Frame width (snap/record/src)")
    ap.add_argument("--height", type=int, default=720, help="Frame height (snap/record/src)")
    ap.add_argument("--fps", type=int, default=30, help="FPS for record/preview")
    ap.add_argument("--duration", type=float, default=5.0, help="Record duration (sec)")
    ap.add_argument("--bitrate", type=int, default=8_000_000, help="Video bitrate (bps)")
    ap.add_argument("--camera", type=int, default=None, help="Camera index (CLI fallback)")
    ap.add_argument("--awb", type=str, default=None, help="AWB mode (auto/daylight/cloudy/fluorescent/etc.)")
    ap.add_argument("--iso", type=int, default=None, help="Approx ISO (Picamera2 via AnalogueGain)")
    ap.add_argument("--exposure", type=int, default=None, help="Exposure time (microseconds)")
    ap.add_argument("--kms-conn", type=int, default=None, help="DRM/KMS connector id (for --preview kms)")
    ap.add_argument("--log", type=str, default=None, help="Base path for logs (writes .txt and .csv)")
    args = ap.parse_args()

    # Begin logging
    header = {
        "action": ("list" if args.list else
                   "snap-gst" if args.snap_gst else
                   "snap" if args.snap else
                   "record" if args.record else
                   f"preview:{args.preview}"),
        "width": args.width, "height": args.height, "fps": args.fps,
        "bitrate": args.bitrate, "duration": args.duration,
    }
    logs = start_logs(args.log, header)

    use_p2 = have_picamera2()

    if args.list:
        (list_picamera2 if use_p2 else list_cli)()
        append_log(logs, {"result": "listed"})
        return

    if args.snap_gst:
        snap_gst(args.snap_gst, args.width, args.height)
        append_log(logs, {"result": "snap_gst_ok", "path": args.snap_gst})
        return

    if args.snap:
        ensure_dir_for(args.snap)
        if use_p2:
            snap_picamera2(args.snap, args.width, args.height, args.exposure, args.iso, args.awb)
        else:
            snap_cli(args.snap, args.width, args.height, args.exposure, args.iso, args.awb, args.camera)
        append_log(logs, {"result": "snap_ok", "path": args.snap})
        return

    if args.record:
        ensure_dir_for(args.record)
        # Prefer libcamera-vid on headless systems; Picamera2 if available; else GStreamer
        if use_p2:
            record_picamera2(args.record, args.width, args.height, args.fps, args.duration, args.bitrate,
                             args.exposure, args.iso, args.awb)
        elif which("libcamera-vid"):
            record_cli(args.record, args.width, args.height, args.fps, args.duration, args.bitrate, args.camera)
        else:
            record_gst(args.record, args.width, args.height, args.fps, args.bitrate, args.duration)
        append_log(logs, {"result": "record_ok", "path": args.record})
        return

    if args.preview:
        if args.preview == "wayland":
            preview_wayland(args.width, args.height, args.fps)
        elif args.preview == "kms":
            preview_kms(args.width, args.height, args.fps, args.kms_conn)
        else:
            preview_window(args.width, args.height, args.fps)
        # execvp replaces process; if it returns, it's an error
        append_log(logs, {"result": "preview_exited"})
        return

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
