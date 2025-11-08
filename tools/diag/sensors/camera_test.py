#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Camera Tester (Ubuntu 24.04 / Pi 5)
================================================
- Ubuntu-first: works without Picamera2; uses libcamera core tools + GStreamer. Picamera2 is optional.
- Listing: tries in order → cam -l (libcamera-tools), libcamera-hello, libcamera-ctl, gst-device-monitor.
- Preview:
    --preview wayland  → waylandsink (letterboxed 1280x720 → 800x480) + queues + sync=false
    --preview kms      → kmssink (DRM/KMS, no compositor) [optional connector id]
    --preview window   → autovideosink (X11/Wayland window)
- Stills:
    --snap PATH        → libcamera-jpeg (if present) or Picamera2 (if present)
    --snap-gst PATH    → one-buffer GStreamer: libcamerasrc → jpegenc → filesink
- Video:
    --record PATH      → MP4/H.264 via Picamera2 or libcamera-vid or pure GStreamer
    --bitrate BPS      → set desired bitrate (default 8 Mbps)
- Logging (optional):
    --log BASE         → writes BASE.txt (human) & BASE.csv (machine) with timestamps + config

Wayland preview env (example):
    export XDG_RUNTIME_DIR=/run/user/1000
    export WAYLAND_DISPLAY=wayland-1
    export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0
    export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:/usr/local/lib
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
    with open(txt, "a", encoding="utf-8") as f:
        f.write(f"[{now_iso()}] camera_test start\n")
        for k, v in header.items():
            f.write(f"  {k}: {v}\n")
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
        keys = list(row.keys())
        w.writerow([now_iso()] + [row[k] for k in keys])

# ---------------- Optional Picamera2 ----------------

def have_picamera2() -> bool:
    try:
        import picamera2  # noqa: F401
        return True
    except Exception:
        return False

# ---------------- Listing (robust) ----------------

def list_devices():
    """
    Robust camera lister for Ubuntu/RPi:
    1) cam -l (libcamera-tools) at common paths
    2) libcamera-hello --list-cameras (if available)
    3) libcamera-ctl --list-cameras (if available)
    4) gst-device-monitor-1.0 Video/Source (fallback, muted warnings)
    """
    def try_cmd(cmd, env=None, label=None):
        rc, out = run(cmd, capture=True, env=env)
        if rc == 0 and out and out.strip():
            print(out.strip() if not label else f"{label}\n{out.strip()}")
            return True
        if out:
            print(out.strip())
        return False

    # 1) cam -l (explicit common locations)
    cam_candidates = [
        shutil.which("cam"),
        "/usr/bin/cam",
        "/usr/local/bin/cam",
    ]
    for c in cam_candidates:
        if c and os.path.exists(c):
            if try_cmd([c, "-l"]):
                return

    # 2) libcamera-hello --list-cameras
    hello = shutil.which("libcamera-hello")
    if hello and try_cmd([hello, "--list-cameras"]):
        return

    # 3) libcamera-ctl --list-cameras
    ctl = shutil.which("libcamera-ctl")
    if ctl and try_cmd([ctl, "--list-cameras"]):
        return

    # 4) GStreamer device monitor (suppress noisy warnings)
    mon = shutil.which("gst-device-monitor-1.0")
    if mon:
        env = os.environ.copy()
        env.setdefault("GST_DEBUG", "0")
        if try_cmd([mon, "Video/Source"], env=env, label="Detected video sources (GStreamer):"):
            return

    print("No camera listers found or returned no devices.")
    print("Install (if missing):")
    print("  sudo apt install libcamera-tools")
    print("  sudo apt install gstreamer1.0-plugins-base-apps")

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
    if not which("libcamera-jpeg"):
        print("libcamera-jpeg not found; use --snap-gst or install libcamera-apps.", file=sys.stderr)
        sys.exit(127)
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
    """Robust still capture via GStreamer without libcamera-apps.
    Strategy: run libcamerasrc → jpegenc → multifilesink, wait for first frame, then stop.
    """
    import tempfile, glob, shutil as _shutil, time as _time, signal

    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    ensure_dir_for(path)

    tmpdir = tempfile.mkdtemp(prefix="savo_cam_snap_")
    pattern = os.path.join(tmpdir, "frame-%06d.jpg")

    pipeline = [
        "gst-launch-1.0", "-e",
        "libcamerasrc", "!",
        f"video/x-raw,format=I420,width={w},height={h},framerate=30/1", "!",
        "videoconvert", "!", "jpegenc", "!",
        "multifilesink", f"location={pattern}", "post-messages=true"
    ]
    print("Running:", " ".join(shlex.quote(x) for x in pipeline))
    proc = subprocess.Popen(pipeline, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    first_path = None
    t0 = _time.time()
    try:
        # Wait up to ~2.0s for the first frame to appear
        while (first_path is None) and (_time.time() - t0) < 2.0:
            files = sorted(glob.glob(os.path.join(tmpdir, "frame-*.jpg")))
            if files:
                first_path = files[0]
                break
            _time.sleep(0.05)

        if first_path is None:
            # Read a bit of stderr for debug and exit
            try:
                _, err = proc.communicate(timeout=0.2)
            except Exception:
                err = ""
            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except Exception:
                proc.kill()
            if err:
                print(err.strip(), file=sys.stderr)
            print("ERROR: No frame produced. Check camera access and GStreamer.", file=sys.stderr)
            hint_video_group()
            sys.exit(1)

        # We have a frame → stop the pipeline cleanly
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=1.0)
        except Exception:
            proc.kill()

        # Move the first frame to the requested path
        _shutil.move(first_path, path)
        print(f"Saved still (gst): {path}")

    finally:
        # Clean temp dir
        try:
            _shutil.rmtree(tmpdir, ignore_errors=True)
        except Exception:
            pass


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
    if not which("libcamera-vid"):
        print("libcamera-vid not found; falling back to pure GStreamer.", file=sys.stderr)
        record_gst(path, w, h, fps, bitrate, duration)
        return
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
    """Pure GStreamer MP4/H.264 path (needs x264enc + mp4mux)."""
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    ensure_dir_for(path)
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
    target_h = int(round(panel_w * 9 / 16))  # 450 for 800 wide
    pad = panel_h - target_h                 # 30 → 15/15
    cmd = [
        "gst-launch-1.0", "-v",
        "libcamerasrc", "!", "queue", "!",
        f"video/x-raw,format=I420,width={src_w},height={src_h},framerate={fps}/1", "!",
        "videoscale", "!", f"video/x-raw,width={target_w},height={target_h}", "!",
        "videobox", f"border-alpha=1", f"top={pad//2}", f"bottom={pad - pad//2}", "left=0", "right=0", "!",
        "videoscale", "!", f"video/x-raw,width={panel_w},height={panel_h}", "!",
        "videoconvert", "!", "queue", "!",
        "waylandsink", f"display={os.environ.get('WAYLAND_DISPLAY','wayland-1')}", "sync=false"
    ]
    print("Running (Ctrl+C to quit):", " ".join(shlex.quote(x) for x in cmd))
    os.execvp(cmd[0], cmd)

def preview_kms(src_w: int, src_h: int, fps: int, conn_id: Optional[int] = None):
    """DRM/KMS direct-to-panel preview (no compositor)."""
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    sink = ["kmssink"]
    if conn_id is not None:
        sink += [f"connector-id={conn_id}"]
    cmd = [
        "gst-launch-1.0", "-v",
        "libcamerasrc", "!", "queue", "!",
        f"video/x-raw,format=I420,width={src_w},height={src_h},framerate={fps}/1", "!",
        "videoconvert", "!", "queue", "!",
    ] + sink
    print("Running (Ctrl+C to quit):", " ".join(shlex.quote(x) for x in cmd))
    os.execvp(cmd[0], cmd)

def preview_window(src_w: int, src_h: int, fps: int):
    """Windowed preview (X11/Wayland desktop)."""
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")
    cmd = [
        "gst-launch-1.0", "-v",
        "libcamerasrc", "!", "queue", "!",
        f"video/x-raw,format=I420,width={src_w},height={src_h},framerate={fps}/1", "!",
        "videoconvert", "!", "queue", "!",
        "autovideosink"
    ]
    print("Running (Ctrl+C to quit):", " ".join(shlex.quote(x) for x in cmd))
    os.execvp(cmd[0], cmd)

# ---------------- Main ----------------

def main():
    ap = argparse.ArgumentParser(description="Robot Savo — Camera Tester")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--list", action="store_true", help="List cameras (cam -l / hello / ctl / gst-device-monitor)")
    g.add_argument("--snap", metavar="PATH", help="Capture JPEG to PATH (libcamera-jpeg or Picamera2)")
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
        list_devices()
        append_log(logs, {"result": "listed"})
        return

    if args.snap_gst:
        snap_gst(args.snap_gst, args.width, args.height)
        append_log(logs, {"result": "snap_gst_ok", "path": args.snap_gst})
        return

    if args.snap:
        ensure_dir_for(args.snap)
        if which("libcamera-jpeg"):
            snap_cli(args.snap, args.width, args.height, args.exposure, args.iso, args.awb, args.camera)
        elif use_p2:
            snap_picamera2(args.snap, args.width, args.height, args.exposure, args.iso, args.awb)
        else:
            print("Neither libcamera-jpeg nor Picamera2 available. Use --snap-gst.", file=sys.stderr)
            sys.exit(127)
        append_log(logs, {"result": "snap_ok", "path": args.snap})
        return

    if args.record:
        ensure_dir_for(args.record)
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
        append_log(logs, {"result": "preview_exited"})
        return

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
