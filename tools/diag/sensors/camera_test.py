#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Camera Diagnostic (Ubuntu 24.04 on Raspberry Pi 5)
----------------------------------------------------------------
- List cameras (libcamera 'cam -l' or gst-device-monitor).
- Live preview to 7" DSI: Wayland, X11, or DRM/KMS (kmssink).
- Record MP4 (H.264) with valid moov (graceful EOS), GOP control, faststart.
- Capture a JPEG still using a minimal GStreamer pipeline.
- Optional CSV logging of config + results.
- No Picamera2 required; works with libcamera + GStreamer.


Wayland env (for DSI preview):
  export XDG_RUNTIME_DIR=/run/user/1000
  export WAYLAND_DISPLAY=wayland-1
  # If you compiled GStreamer/libcamera to /usr/local:
  export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0
  export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:/usr/local/lib

  
"""

import argparse
import os
import sys
import shutil
import subprocess
import signal
import time
import tempfile
import glob
import csv
import json
import shlex
from datetime import datetime
from typing import Optional, List, Tuple

# -------- Utilities --------

def require(bin_name: str, human: str):
    if shutil.which(bin_name) is None:
        print(f"ERROR: {human} not found in PATH (binary: {bin_name}).", file=sys.stderr)
        hint_video_group()
        sys.exit(1)

def run(cmd: List[str], capture: bool = False, env: Optional[dict] = None) -> Tuple[int, str]:
    if capture:
        try:
            out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, env=env)
            return (0, out.decode(errors="replace"))
        except subprocess.CalledProcessError as e:
            return (e.returncode, e.output.decode(errors="replace"))
    else:
        try:
            rc = subprocess.call(cmd, env=env)
            return (rc, "")
        except FileNotFoundError:
            return (127, "")

def ensure_dir_for(path: str):
    d = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(d, exist_ok=True)

def hint_video_group():
    print("\nHint: If camera access fails, add user to 'video' group then relogin:", file=sys.stderr)
    print("  sudo usermod -aG video $USER && newgrp video\n", file=sys.stderr)

def pick_encoder(enc: str) -> str:
    enc = (enc or "x264").lower()
    if enc not in ("x264", "openh264"):
        print(f"WARNING: Unknown encoder '{enc}', falling back to x264.")
        enc = "x264"
    # Presence hints (no hard fail)
    if shutil.which("gst-inspect-1.0"):
        rc_x264, _ = run(["gst-inspect-1.0", "x264"], capture=True)
        rc_oh, _  = run(["gst-inspect-1.0", "openh264"], capture=True)
        if enc == "x264" and rc_x264 != 0:
            print("WARNING: 'x264enc' plugin not found; install gstreamer1.0-plugins-ugly (Ubuntu may bundle it elsewhere).", file=sys.stderr)
        if enc == "openh264" and rc_oh != 0:
            print("WARNING: 'openh264enc' plugin not found; prefer --encoder x264 or install OpenH264.", file=sys.stderr)
    return enc

def run_with_optional_timeout(cmd_list: List[str], timeout_int: Optional[float]):
    """Wrap gst-launch in `timeout --signal=INT N ...` so mp4mux receives EOS."""
    if timeout_int and shutil.which("timeout"):
        return subprocess.Popen(["timeout", "--signal=INT", str(timeout_int), *cmd_list])
    return subprocess.Popen(cmd_list)

def size_bytes(path: str) -> Optional[int]:
    try:
        return os.path.getsize(path)
    except Exception:
        return None

def write_csv_row(csv_path: str, data: dict):
    if not csv_path:
        return
    ensure_dir_for(csv_path)
    new_file = not os.path.exists(csv_path)
    keys = ["ts","mode","width","height","fps","encoder","bitrate","gop",
            "duration","faststart","path","size_bytes","extra"]
    with open(csv_path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        if new_file:
            w.writeheader()
        row = {k: data.get(k, "") for k in keys}
        w.writerow(row)

def print_config(namespace: argparse.Namespace):
    cfg = vars(namespace).copy()
    # Hide long/unused fields if any later
    print("# Config:")
    print(json.dumps(cfg, indent=2, sort_keys=True))

# -------- Listing --------

def list_devices():
    # Prefer libcamera 'cam -l' (from libcamera-tools)
    cam_bin = shutil.which("cam") or "/usr/local/bin/cam"
    if os.path.exists(cam_bin):
        rc, out = run([cam_bin, "-l"], capture=True)
        if out:
            print(out.strip())
            return
    # Fallback: gst-device-monitor
    mon = shutil.which("gst-device-monitor-1.0")
    if mon:
        env = os.environ.copy()
        env.setdefault("GST_DEBUG", "0")
        rc, out = run([mon, "Video/Source"], capture=True, env=env)
        if out:
            print("Detected video sources (GStreamer):")
            print(out.strip())
            return
    print("No camera listers available.")
    print("Install (if missing):")
    print("  sudo apt install libcamera-tools")
    print("  sudo apt install gstreamer1.0-plugins-base-apps")

# -------- Still capture --------

def snap_gst(path: str, w: int, h: int, fps: int, hflip: bool, vflip: bool):
    """Take a single JPEG using GStreamer; save to `path`."""
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    ensure_dir_for(path)
    tmpdir = tempfile.mkdtemp(prefix="savo_cam_snap_")
    pattern = os.path.join(tmpdir, "frame-%06d.jpg")

    flips = []
    # Some builds expose a 'videoflip' element; safest is to request transforms via capsfilters
    # but videoflip is simpler if present. Here we try videoflip; if absent, caps ignores it.
    if hflip or vflip:
        mode = "horizontal-flip" if hflip and not vflip else \
               "vertical-flip" if vflip and not hflip else \
               "rotate-180"
        flips = ["!", "videoflip", f"video-direction={mode}"]

    pipeline = [
        "gst-launch-1.0", "-e",
        "libcamerasrc", "!",
        f"video/x-raw,format=I420,width={w},height={h},framerate={fps}/1", "!",
        "videoconvert", *flips, "!",
        "jpegenc", "!",
        "multifilesink", f"location={pattern}", "post-messages=true"
    ]
    print("Running:", " ".join(shlex.quote(x) for x in pipeline))
    rc, _ = run(pipeline, capture=False)

    # pick last frame
    try:
        files = sorted(glob.glob(os.path.join(tmpdir, "frame-*.jpg")))
        if not files:
            raise FileNotFoundError("No JPEG frames produced.")
        last = files[-1]
        shutil.copy2(last, path)
    finally:
        try:
            for f in glob.glob(os.path.join(tmpdir, "frame-*.jpg")):
                os.remove(f)
            os.rmdir(tmpdir)
        except Exception:
            pass
    print(f"Saved still (gst): {path}")

# -------- Recording --------

def record_gst(path: str, w: int, h: int, fps: int, bitrate: int,
               duration: float, encoder: str = "x264", gop: Optional[int] = 60,
               timeout_int: Optional[float] = None, faststart: bool = True,
               hflip: bool = False, vflip: bool = False) -> dict:
    """
    Record MP4/H.264 with EOS finalization:
    - External timeout: `timeout --signal=INT <N>` wraps gst-launch
    - Else internal SIGINT after <duration>
    Returns a small result dict for logging.
    """
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    ensure_dir_for(path)
    enc = pick_encoder(encoder)

    flips = []
    if hflip or vflip:
        mode = "horizontal-flip" if hflip and not vflip else \
               "vertical-flip" if vflip and not hflip else \
               "rotate-180"
        flips = ["!", "videoflip", f"video-direction={mode}"]

    if enc == "openh264":
        enc_chain = [
            "videoconvert", *flips, "!",
            "openh264enc", f"bitrate={int(bitrate/1000)}", "!",
            "h264parse"
        ]
    else:
        enc_chain = [
            "videoconvert", *flips, "!",
            "x264enc",
            f"bitrate={int(bitrate/1000)}",
            "speed-preset=superfast",
            "tune=zerolatency",
        ]
        if gop and gop > 0:
            enc_chain += [f"key-int-max={gop}"]
        enc_chain += ["!", "h264parse"]

    mux_opts = ["mp4mux"]
    if faststart:
        mux_opts += ["faststart=true"]

    pipeline = [
        "gst-launch-1.0", "-e",
        "libcamerasrc", "!",
        f"video/x-raw,format=I420,width={w},height={h},framerate={fps}/1", "!",
        *enc_chain, "!",
        *mux_opts, "!",
        "filesink", f"location={path}"
    ]
    print("Running:", " ".join(shlex.quote(x) for x in pipeline))

    t0 = time.time()
    proc = run_with_optional_timeout(pipeline, timeout_int=timeout_int)

    try:
        if not timeout_int:
            t_end = t0 + max(0.0, duration)
            while proc.poll() is None and time.time() < t_end:
                time.sleep(0.1)
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)  # graceful EOS
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=2.0)
        else:
            proc.wait()
    except KeyboardInterrupt:
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5.0)
        except Exception:
            proc.terminate()

    t1 = time.time()
    out_size = size_bytes(path)
    print(f"Saved video (gst/{enc}): {path}")
    return {
        "path": path,
        "elapsed": round(t1 - t0, 3),
        "size_bytes": out_size,
        "encoder": enc
    }

# -------- Preview --------

def preview_wayland(w: int, h: int, fps: int, latency: str, hflip: bool, vflip: bool):
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    display = os.environ.get("WAYLAND_DISPLAY", "wayland-1")
    flips = []
    if hflip or vflip:
        mode = "horizontal-flip" if hflip and not vflip else \
               "vertical-flip" if vflip and not hflip else \
               "rotate-180"
        flips = ["!", "videoflip", f"video-direction={mode}"]

    if latency == "low":
        pipeline = [
            "gst-launch-1.0", "-v",
            "libcamerasrc", "!", "queue", "max-size-buffers=3", "leaky=downstream", "!",
            f"video/x-raw,format=I420,width={w},height={h},framerate={fps}/1", "!",
            "videoscale", "!", "video/x-raw,width=800,height=450", "!",
            "videobox", "border-alpha=1", "top=15", "bottom=15", "left=0", "right=0", "!",
            "videoscale", "!", "video/x-raw,width=800,height=480", "!",
            "videoconvert", *flips, "!", "queue", "max-size-buffers=3", "leaky=downstream", "!",
            "waylandsink", f"display={display}", "sync=false"
        ]
    else:
        pipeline = [
            "gst-launch-1.0", "-v",
            "libcamerasrc", "!",
            f"video/x-raw,format=I420,width={w},height={h},framerate={fps}/1", "!",
            "videoscale", "!", "video/x-raw,width=800,height=450", "!",
            "videobox", "border-alpha=1", "top=15", "bottom=15", "left=0", "right=0", "!",
            "videoscale", "!", "video/x-raw,width=800,height=480", "!",
            "videoconvert", *flips, "!",
            "waylandsink", f"display={display}"
        ]
    print("Running (Ctrl+C to quit):", " ".join(shlex.quote(x) for x in pipeline))
    os.execvp(pipeline[0], pipeline)

def preview_x11(w: int, h: int, fps: int, hflip: bool, vflip: bool):
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    flips = []
    if hflip or vflip:
        mode = "horizontal-flip" if hflip and not vflip else \
               "vertical-flip" if vflip and not hflip else \
               "rotate-180"
        flips = ["!", "videoflip", f"video-direction={mode}"]
    pipeline = [
        "gst-launch-1.0", "-v",
        "libcamerasrc", "!",
        f"video/x-raw,format=I420,width={w},height={h},framerate={fps}/1", "!",
        "videoconvert", *flips, "!",
        "autovideosink"
    ]
    print("Running (Ctrl+C to quit):", " ".join(shlex.quote(x) for x in pipeline))
    os.execvp(pipeline[0], pipeline)

def preview_drm(w: int, h: int, fps: int, hflip: bool, vflip: bool):
    """Direct DRM/KMS preview via kmssink (console); may need tty & KMS console."""
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    # We keep it simple; connector/plane selection left to defaults.
    flips = []
    if hflip or vflip:
        mode = "horizontal-flip" if hflip and not vflip else \
               "vertical-flip" if vflip and not hflip else \
               "rotate-180"
        flips = ["!", "videoflip", f"video-direction={mode}"]
    pipeline = [
        "gst-launch-1.0", "-v",
        "libcamerasrc", "!",
        f"video/x-raw,format=I420,width={w},height={h},framerate={fps}/1", "!",
        "videoconvert", *flips, "!",
        "kmssink"
    ]
    print("Running (Ctrl+C to quit):", " ".join(shlex.quote(x) for x in pipeline))
    os.execvp(pipeline[0], pipeline)

# -------- Main --------

def main():
    ap = argparse.ArgumentParser(description="Robot Savo Camera Diagnostic")
    ap.add_argument("--list", action="store_true", help="List detected cameras.")
    ap.add_argument("--preview", choices=["wayland", "x11", "drm"], help="Live preview mode.")
    ap.add_argument("--latency", choices=["normal", "low"], default="normal",
                    help="Wayland preview latency preset; 'low' adds queues + sync=false.")
    ap.add_argument("--record", type=str, help="Record MP4 path.")
    ap.add_argument("--encoder", choices=["x264", "openh264"], default="x264",
                    help="H.264 encoder (x264 recommended).")
    ap.add_argument("--gop", type=int, default=60, help="GOP size (key-int-max) for x264.")
    ap.add_argument("--faststart", action="store_true", default=True,
                    help="Enable mp4 'faststart' (moov at head). Default: on.")
    ap.add_argument("--no-faststart", dest="faststart", action="store_false",
                    help="Disable mp4 faststart.")
    ap.add_argument("--snap-gst", type=str, help="Capture a JPEG to this path via GStreamer.")
    ap.add_argument("--width", type=int, default=1280, help="Frame width.")
    ap.add_argument("--height", type=int, default=720, help="Frame height.")
    ap.add_argument("--fps", type=int, default=30, help="Frames per second.")
    ap.add_argument("--bitrate", type=int, default=8_000_000, help="Target bitrate (bits/s).")
    ap.add_argument("--duration", type=float, default=6.0, help="Record duration (seconds).")
    ap.add_argument("--timeout-int", type=float, default=None,
                    help="Wrap gst-launch with `timeout --signal=INT N` to force EOS finalization.")
    ap.add_argument("--log", type=str, default=None, help="Append a CSV log row to this path.")
    ap.add_argument("--print-config", action="store_true", help="Print config JSON before running.")
    ap.add_argument("--hflip", action="store_true", help="Horizontal flip (preview/record/snap).")
    ap.add_argument("--vflip", action="store_true", help="Vertical flip (preview/record/snap).")

    args = ap.parse_args()

    # Sanity
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        print("ERROR: width/height/fps must be positive.", file=sys.stderr)
        sys.exit(2)

    if args.print_config:
        print_config(args)

    # Actions
    if args.list:
        list_devices()
        return

    did_something = False

    if args.snap_gst:
        snap_gst(args.snap_gst, args.width, args.height, args.fps, args.hflip, args.vflip)
        write_csv_row(args.log, {
            "ts": datetime.utcnow().isoformat(),
            "mode": "snap",
            "width": args.width, "height": args.height, "fps": args.fps,
            "encoder": "", "bitrate": "", "gop": "",
            "duration": "", "faststart": "",
            "path": args.snap_gst, "size_bytes": size_bytes(args.snap_gst),
            "extra": ""
        })
        did_something = True

    if args.record:
        res = record_gst(args.record, args.width, args.height, args.fps,
                         args.bitrate, args.duration, args.encoder, args.gop,
                         args.timeout_int, args.faststart, args.hflip, args.vflip)
        write_csv_row(args.log, {
            "ts": datetime.utcnow().isoformat(),
            "mode": "record",
            "width": args.width, "height": args.height, "fps": args.fps,
            "encoder": res.get("encoder"), "bitrate": args.bitrate, "gop": args.gop,
            "duration": args.duration, "faststart": args.faststart,
            "path": args.record, "size_bytes": res.get("size_bytes", ""),
            "extra": ""
        })
        did_something = True

    if args.preview:
        # Execs (replaces process)
        if args.preview == "wayland":
            preview_wayland(args.width, args.height, args.fps, args.latency, args.hflip, args.vflip)
        elif args.preview == "x11":
            preview_x11(args.width, args.height, args.fps, args.hflip, args.vflip)
        elif args.preview == "drm":
            preview_drm(args.width, args.height, args.fps, args.hflip, args.vflip)
        did_something = True

    if not did_something:
        print("Nothing to do. Try one of:")
        print("  --list")
        print("  --preview wayland|x11|drm  [--latency low] [--hflip/--vflip]")
        print("  --record /tmp/cam.mp4 --width 1920 --height 1080 --fps 30 --bitrate 10000000 --duration 6 [--timeout-int 6] [--log /tmp/cam.csv]")
        print("  --snap-gst /tmp/cam.jpg --width 1280 --height 720")
        print("Add --print-config to dump the current settings.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
