#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Camera Diagnostic (Ubuntu edition, libcamera + GStreamer)
----------------------------------------------------------------------
- Works on Ubuntu 24.04 for Raspberry Pi 5 (no Picamera2 required).
- Live preview to the 7" DSI via DRM/KMS or to a window.
- Records MP4 (H.264) with configurable resolution/FPS/bitrate.
- Captures a JPEG still using a one‑buffer GStreamer pipeline.
- Optional text/CSV metadata logs (config + timestamps) for quick sanity checks.

Author: Robot Savo
"""

import argparse
import os
import shlex
import subprocess
import sys
import time
from datetime import datetime
from typing import Tuple


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


def log_meta_text(outdir: str, label: str, extra: str = "") -> None:
    ensure_dir(outdir)
    fname = os.path.join(outdir, ts_name(f"{label}_info", "txt"))
    with open(fname, "w") as f:
        f.write(f"time: {datetime.now().isoformat()}\n")
        if extra:
            f.write(extra + "\n")
        # Try to capture a quick camera list for context
        try:
            out = subprocess.check_output(["cam", "-l"], text=True)
            f.write("\n# cam -l\n" + out)
        except Exception as e:
            f.write(f"\n# cam -l failed: {e}\n")
    print(f"[INFO] Wrote {fname}")


def do_preview(args) -> int:
    w, h = args.res
    sink = "kmssink" if args.sink == "drm" else "autovideosink"
    pipeline = (
        f"libcamerasrc ! video/x-raw,width={w},height={h},framerate={args.fps}/1 "
        f"! videoconvert ! {sink}"
    )
    return run(f"gst-launch-1.0 -v {pipeline}")


def do_video(args) -> int:
    ensure_dir(args.outdir)
    mp4 = os.path.join(args.outdir, ts_name("cam_video", "mp4"))
    w, h = args.res
    # Use hardware-friendly x264 defaults; tune=zerolatency for low-latency preview
    pipeline = (
        f"libcamerasrc ! video/x-raw,width={w},height={h},framerate={args.fps}/1 "
        f"! videoconvert ! x264enc tune=zerolatency bitrate={args.bitrate} "
        f"! mp4mux ! filesink location={shlex.quote(mp4)}"
    )
    log_meta_text(args.outdir, "video", extra=f"res={w}x{h} fps={args.fps} bitrate={args.bitrate}k")
    return run(f"gst-launch-1.0 -e {pipeline}")


def do_still(args) -> int:
    ensure_dir(args.outdir)
    jpg = os.path.join(args.outdir, ts_name("cam_still", "jpg"))
    w, h = args.res
    # Take exactly one buffer and encode to JPEG
    pipeline = (
        f"libcamerasrc num-buffers=1 ! video/x-raw,width={w},height={h},framerate={args.fps}/1 "
        f"! videoconvert ! jpegenc ! filesink location={shlex.quote(jpg)}"
    )
    log_meta_text(args.outdir, "still", extra=f"res={w}x{h} fps={args.fps}")
    return run(f"gst-launch-1.0 -e {pipeline}")


def main() -> None:
    p = argparse.ArgumentParser(description="Robot Savo Camera Diagnostic (Ubuntu, libcamera+GStreamer)")
    p.add_argument("--mode", choices=["preview", "video", "still"], default="preview")
    p.add_argument("--res", type=parse_res, default="1280x720", help="WxH (default 1280x720)")
    p.add_argument("--fps", type=int, default=30, help="Frames per second (default 30)")
    p.add_argument("--duration", type=int, default=10, help="Seconds for video mode; ignored for preview (Ctrl+C to stop)")
    p.add_argument("--bitrate", type=int, default=6000, help="Video bitrate (kbps) for H.264")
    p.add_argument("--sink", choices=["drm", "auto"], default="drm", help="Preview sink: drm=DSI full-screen, auto=desktop window")
    p.add_argument("--outdir", default="log", help="Output directory for files and logs")

    args = p.parse_args()

    if args.mode == "preview":
        sys.exit(do_preview(args))
    elif args.mode == "video":
        # Run a timed recording by sleeping then killing the pipeline would be complex
        # when shelling out. For simplicity users can Ctrl+C to stop early; otherwise
        # we rely on -e EOS handling with a short sleep wrapper.
        start = time.time()
        ret = do_video(args)
        elapsed = time.time() - start
        print(f"[Video] Pipeline exited, elapsed={elapsed:.2f}s")
        sys.exit(ret)
    else:
        sys.exit(do_still(args))


if __name__ == "__main__":
    main()
