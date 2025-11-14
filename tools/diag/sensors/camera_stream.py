#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Camera Live Stream (Pi → Laptop, GStreamer + libcamerasrc)
-----------------------------------------------------------------------
- Uses libcamerasrc + x264enc + rtph264pay to stream H.264 over UDP.
- No files are saved; it's purely a live preview.
- Avoids libcamera-vid completely (better on Ubuntu 24.04).

Author: Robot Savo
"""

import argparse
import shutil
import subprocess
import sys
import shlex


def require(bin_name: str, human: str) -> None:
    if shutil.which(bin_name) is None:
        print(f"[Camera-Stream] ERROR: {human} not found in PATH (binary: {bin_name}).", file=sys.stderr)
        print("  Install the missing package (e.g. gstreamer1.0-* plugins).", file=sys.stderr)
        sys.exit(1)


def build_gst_pipeline(host: str,
                       port: int,
                       width: int,
                       height: int,
                       fps: int,
                       bitrate_kbps: int) -> list[str]:
    """
    Build a GStreamer pipeline:

      libcamerasrc !
        video/x-raw,width=WxH,framerate=FPS/1 !
        videoconvert !
        x264enc tune=zerolatency bitrate=(kbps) speed-preset=superfast key-int-max=60 !
        rtph264pay config-interval=1 pt=96 !
        udpsink host=HOST port=PORT sync=false async=false
    """
    bitrate = int(bitrate_kbps)  # x264enc bitrate is in kbit/s (not bits/s)

    pipeline = [
        "gst-launch-1.0", "-v",
        "libcamerasrc", "!",
        f"video/x-raw,width={width},height={height},framerate={fps}/1", "!",
        "videoconvert", "!",
        "x264enc",
        "tune=zerolatency",
        f"bitrate={bitrate}",
        "speed-preset=superfast",
        "key-int-max=60", "!",
        "rtph264pay",
        "config-interval=1",
        "pt=96", "!",
        "udpsink",
        f"host={host}",
        f"port={port}",
        "sync=false",
        "async=false",
    ]
    return pipeline


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Robot Savo — Pi camera live stream to laptop (UDP/RTP, GStreamer, no recording)"
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
        help="Approx. video bitrate in kbit/s for x264enc (default: 4000).",
    )
    ap.add_argument(
        "--print-only",
        action="store_true",
        help="Only print the GStreamer command and exit (no streaming).",
    )

    args = ap.parse_args()

    # Check that GStreamer is available
    require("gst-launch-1.0", "GStreamer (gst-launch-1.0)")

    cmd = build_gst_pipeline(
        host=args.host,
        port=args.port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate_kbps=args.bitrate,
    )

    print("\n[Camera-Stream] Robot Savo — Pi → Laptop Live Stream (GStreamer + libcamerasrc)")
    print("-------------------------------------------------------------------------------")
    print(f"[Camera-Stream] Host       : {args.host}")
    print(f"[Camera-Stream] Port       : {args.port}")
    print(f"[Camera-Stream] Resolution : {args.width}x{args.height}")
    print(f"[Camera-Stream] FPS        : {args.fps}")
    print(f"[Camera-Stream] Bitrate    : {args.bitrate} kbit/s\n")

    print("[Camera-Stream] Sender pipeline (Pi):")
    print("  " + " ".join(shlex.quote(x) for x in cmd))
    print("\n[Camera-Stream] On your LAPTOP, you can use for example:\n")
    print("  # GStreamer RTP receiver")
    print(f"  gst-launch-1.0 -v \\")
    print(f"    udpsrc port={args.port} caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! \\")
    print("    rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink\n")

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
