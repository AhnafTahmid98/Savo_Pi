#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Camera Live Stream (Pi → Laptop, no recording)
-----------------------------------------------------------
- Uses libcamera via GStreamer to stream H.264 over UDP.
- No files are saved; it's purely a live preview.
- Designed to be simple and robust for testing and for drive_automode.py.

Author: Robot Savo
"""

import argparse
import subprocess
import sys
import shutil


def build_sender_pipeline(host: str,
                          port: int,
                          width: int,
                          height: int,
                          fps: int,
                          bitrate: int) -> str:
    """
    Build the GStreamer pipeline for sending H.264 video over UDP.

    We use:
      libcamerasrc → video/x-raw → videoconvert → x264enc (low latency) → H.264 → RTP → UDP

    No local display, no file saving.
    """
    pipeline = (
        "libcamerasrc ! "
        f"video/x-raw,width={width},height={height},framerate={fps}/1 ! "
        "videoconvert ! "
        "x264enc tune=zerolatency speed-preset=superfast "
        f"bitrate={bitrate} key-int-max=30 ! "
        "video/x-h264,profile=baseline ! "
        "rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host={host} port={port} sync=false async=false"
    )
    return pipeline


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Robot Savo — Pi camera live stream to laptop (UDP, no recording)"
    )
    ap.add_argument(
        "--host",
        required=True,
        help="Laptop/PC IP address (receiver). Example: 192.168.1.50",
    )
    ap.add_argument(
        "--port",
        type=int,
        default=5000,
        help="UDP port on receiver (default: 5000).",
    )
    ap.add_argument(
        "--width",
        type=int,
        default=640,
        help="Video width (default: 640).",
    )
    ap.add_argument(
        "--height",
        type=int,
        default=480,
        help="Video height (default: 480).",
    )
    ap.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Frame rate (default: 30).",
    )
    ap.add_argument(
        "--bitrate",
        type=int,
        default=1500,
        help="Approx. video bitrate in kbit/s (default: 1500).",
    )
    ap.add_argument(
        "--print-only",
        action="store_true",
        help="Only print the gst-launch command and exit (no streaming).",
    )

    args = ap.parse_args()

    # Check that gst-launch-1.0 exists on the Pi
    if shutil.which("gst-launch-1.0") is None:
        print("[Camera-Stream] ERROR: 'gst-launch-1.0' not found in PATH.", file=sys.stderr)
        print("  Install GStreamer (gst-launch-1.0) before using this script.", file=sys.stderr)
        sys.exit(1)

    pipeline = build_sender_pipeline(
        host=args.host,
        port=args.port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=args.bitrate,
    )

    cmd = ["gst-launch-1.0", "-v"] + pipeline.split()

    print("\n[Camera-Stream] Robot Savo — Pi → Laptop Live Stream")
    print("----------------------------------------------------")
    print(f"[Camera-Stream] Host       : {args.host}")
    print(f"[Camera-Stream] Port       : {args.port}")
    print(f"[Camera-Stream] Resolution : {args.width}x{args.height}")
    print(f"[Camera-Stream] FPS        : {args.fps}")
    print(f"[Camera-Stream] Bitrate    : {args.bitrate} kbit/s\n")
    print("[Camera-Stream] Sender pipeline (Pi):")
    print("  " + " ".join(cmd))
    print("\n[Camera-Stream] On your LAPTOP, run:")
    print(f"  gst-launch-1.0 -v \\")
    print(f"    udpsrc port={args.port} caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! \\")
    print("    rtph264depay ! avdec_h264 ! videoconvert ! autovideosink\n")

    # FIX: argparse stores --print-only as args.print_only
    if args.print_only:
        print("[Camera-Stream] --print-only set, not starting pipeline.")
        return

    print("[Camera-Stream] Starting GStreamer sender...  (Ctrl+C to stop)\n")

    proc = None
    try:
        proc = subprocess.Popen(cmd)
        proc.wait()
    except KeyboardInterrupt:
        print("\n[Camera-Stream] Ctrl+C received, terminating pipeline...")
        if proc is not None:
            try:
                proc.terminate()
            except Exception:
                pass
    finally:
        print("[Camera-Stream] Done.")


if __name__ == "__main__":
    main()
