#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Camera Live Stream (Pi → Laptop, no recording)
-----------------------------------------------------------
- Uses libcamera-vid to stream H.264 over UDP.
- No files are saved; it's purely a live preview.
- Avoids gst-libcamera negotiation issues we saw with libcamerasrc.

Author: Robot Savo
"""

import argparse
import shutil
import subprocess
import sys


def build_libcamera_vid_cmd(host: str,
                            port: int,
                            width: int,
                            height: int,
                            fps: int,
                            bitrate_kbps: int) -> list[str]:
    """
    Build the libcamera-vid command for sending H.264 over UDP.

    We use:
      libcamera-vid -t 0 --inline --width W --height H --framerate FPS
                    --codec h264 --profile baseline --bitrate (bits/s)
                    --nopreview
                    -o udp://HOST:PORT
    """
    bitrate_bits = bitrate_kbps * 1000

    cmd = [
        "libcamera-vid",
        "-t", "0",                    # 0 ms == run until Ctrl+C
        "--inline",                   # include SPS/PPS in stream for late join
        "--width", str(width),
        "--height", str(height),
        "--framerate", str(fps),
        "--codec", "h264",
        "--profile", "baseline",
        "--bitrate", str(bitrate_bits),
        "--nopreview",               # no local preview window
        "-o", f"udp://{host}:{port}",
    ]
    return cmd


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Robot Savo — Pi camera live stream to laptop (UDP, libcamera-vid, no recording)"
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
        default=1280,
        help="Video width (default: 1280).",
    )
    ap.add_argument(
        "--height",
        type=int,
        default=720,
        help="Video height (default: 720).",
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
        default=4000,
        help="Approx. video bitrate in kbit/s (default: 4000).",
    )
    ap.add_argument(
        "--print-only",
        action="store_true",
        help="Only print the libcamera-vid command and exit (no streaming).",
    )

    args = ap.parse_args()

    # Check that libcamera-vid exists on the Pi
    if shutil.which("libcamera-vid") is None:
        print("[Camera-Stream] ERROR: 'libcamera-vid' not found in PATH.", file=sys.stderr)
        print("  Install libcamera-apps (libcamera-vid) before using this script.", file=sys.stderr)
        sys.exit(1)

    cmd = build_libcamera_vid_cmd(
        host=args.host,
        port=args.port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate_kbps=args.bitrate,
    )

    print("\n[Camera-Stream] Robot Savo — Pi → Laptop Live Stream (libcamera-vid)")
    print("---------------------------------------------------------------------")
    print(f"[Camera-Stream] Host       : {args.host}")
    print(f"[Camera-Stream] Port       : {args.port}")
    print(f"[Camera-Stream] Resolution : {args.width}x{args.height}")
    print(f"[Camera-Stream] FPS        : {args.fps}")
    print(f"[Camera-Stream] Bitrate    : {args.bitrate} kbit/s\n")
    print("[Camera-Stream] Sender command (Pi):")
    print("  " + " ".join(cmd))
    print("\n[Camera-Stream] On your LAPTOP, you can use for example:\n")
    print("  # Option 1: ffplay (simple)")
    print(f"  ffplay -fflags nobuffer -flags low_delay -framedrop -strict experimental udp://@:{args.port}\n")
    print("  # Option 2: GStreamer")
    print(f"  gst-launch-1.0 -v \\")
    print(f"    udpsrc port={args.port} ! \\")
    print("    application/x-rtp, payload=96 ! \\")
    print("    rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink\n")

    if args.print_only:
        print("[Camera-Stream] --print-only set, not starting libcamera-vid.")
        return

    print("[Camera-Stream] Starting libcamera-vid sender...  (Ctrl+C to stop)\n")

    proc = None
    try:
        proc = subprocess.Popen(cmd)
        proc.wait()
    except KeyboardInterrupt:
        print("\n[Camera-Stream] Ctrl+C received, terminating libcamera-vid...")
        if proc is not None:
            try:
                proc.terminate()
            except Exception:
                pass
    finally:
        print("[Camera-Stream] Done.")


if __name__ == "__main__":
    main()
