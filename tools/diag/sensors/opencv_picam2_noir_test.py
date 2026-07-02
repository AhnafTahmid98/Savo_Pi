#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import cv2


def build_pipeline(width: int, height: int, fps: int) -> str:
    return (
        "libcamerasrc "
        f"! video/x-raw,format=I420,width={width},height={height},framerate={fps}/1 "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink drop=true max-buffers=1 sync=false"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--out", default="/tmp/savo_picam2_noir_opencv.jpg")
    args = parser.parse_args()

    pipeline = build_pipeline(args.width, args.height, args.fps)

    print("OpenCV version:", cv2.__version__)
    print("Using GStreamer pipeline:")
    print(pipeline)

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("ERROR: OpenCV could not open Pi Camera through GStreamer.")
        return 1

    ok_count = 0
    last_frame = None

    start_time = time.monotonic()

    for _ in range(args.frames):
        ok, frame = cap.read()

        if not ok or frame is None:
            print("ERROR: Failed to read frame from Pi Camera.")
            cap.release()
            return 2

        last_frame = frame
        ok_count += 1

    elapsed = time.monotonic() - start_time
    cap.release()

    if last_frame is None:
        print("ERROR: No frame captured.")
        return 3

    out_path = Path(args.out)
    cv2.imwrite(str(out_path), last_frame)

    print("Camera test passed.")
    print(f"Captured frames: {ok_count}")
    print(f"Measured FPS: {ok_count / elapsed:.2f}")
    print(f"Frame shape: {last_frame.shape}")
    print(f"Saved image: {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())